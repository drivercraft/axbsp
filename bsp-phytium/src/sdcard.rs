extern crate alloc;

use phytium_mci::mci_host::err::MCIHostError;
use phytium_mci::sd::SdCard;
pub use phytium_mci::{Kernel, set_impl};

use alloc::{boxed::Box, vec::Vec, sync::Arc};
use log::trace;

use rdif_block::{BlkError, IQueue, Interface, Request, RequestId};
use rdrive::{DriverGeneric, KError};

use spin::Mutex;

use core::{cmp, marker::{Send, Sync}, ptr::NonNull};

// pub use dma_api::{Direction, Impl as DmaImpl};
// pub use dma_api::set_impl as set_dma_impl;

const OFFSET: usize = 0x400_0000;
const BLOCK_SIZE: usize = 512;

pub struct SdCardDriver {
    sd_card: Arc<Mutex<Box<SdCard>>>,
}

impl SdCardDriver {
    pub fn new(sd_addr: NonNull<u8>) -> Self {
        let sd_card = Arc::new(Mutex::new(Box::new(SdCard::new(sd_addr))));
        SdCardDriver { sd_card }
    }
}

unsafe impl Send for SdCardDriver {}
unsafe impl Sync for SdCardDriver {}

unsafe impl Send for SdCardQueue {}
unsafe impl Sync for SdCardQueue {}

impl DriverGeneric for SdCardDriver {
    fn open(&mut self) -> Result<(), KError> {
        Ok(())
    }

    fn close(&mut self) -> Result<(), KError> {
        Ok(())
    }
}

impl Interface for SdCardDriver {
    fn create_queue(&mut self) -> Option<Box<dyn IQueue>> {
        Some(Box::new(SdCardQueue {
            sd_card: Arc::clone(&self.sd_card),
        }))
    }

    fn enable_irq(&mut self) {
        todo!()
    }

    fn disable_irq(&mut self) {
        todo!()
    }

    fn is_irq_enabled(&self) -> bool {
        false
    }

    fn handle_irq(&mut self) -> rdif_block::Event {
        rdif_block::Event::none()
    }
}

pub struct SdCardQueue {
    sd_card: Arc<Mutex<Box<SdCard>>>,
}

impl IQueue for SdCardQueue {
    /// Returns the number of blocks on the SD card.
    fn num_blocks(&self) -> usize {
        self.sd_card.lock().block_count() as usize
    }

    /// Returns the block size in bytes.
    fn block_size(&self) -> usize {
        self.sd_card.lock().block_size() as usize
    }

    fn id(&self) -> usize {
        0
    }

    fn buff_config(&self) -> rdif_block::BuffConfig {
        rdif_block::BuffConfig {
            dma_mask: u64::MAX,
            align: 0x1000,
            size: self.block_size(),
        }
    }

    fn submit_request(&mut self, request: Request<'_>) -> Result<RequestId, BlkError> {
        let actual_block_id = request.block_id + OFFSET / 512;
        
        match request.kind {
            rdif_block::RequestKind::Read(mut buffer) => {
                trace!("read block {}", actual_block_id);

                Self::validate_buffer(&buffer)?;

                let (_, aligned_buf, _) = unsafe { buffer.align_to_mut::<u32>() };
                let mut temp_buf: Vec<u32> = Vec::with_capacity(aligned_buf.len());
                
                self.sd_card.lock()
                    .read_blocks(&mut temp_buf, actual_block_id as u32, 1)
                    .map_err(|err| map_mci_error_to_blk_error(err))?;

                let copy_len = cmp::min(temp_buf.len(), aligned_buf.len());
                aligned_buf[..copy_len].copy_from_slice(&temp_buf[..copy_len]);
                
                Ok(RequestId::new(0))
            }
            rdif_block::RequestKind::Write(buffer) => {
                trace!("write block {}", actual_block_id);

                Self::validate_buffer(&buffer)?;
                
                let (_, aligned_buf, _) = unsafe { buffer.align_to::<u32>() };
                let mut write_buf: Vec<u32> = aligned_buf.to_vec();

                self.sd_card.lock()
                    .write_blocks(&mut write_buf, actual_block_id as u32, 1)
                    .map_err(|err| map_mci_error_to_blk_error(err))?;
                
                Ok(RequestId::new(0))
            }
        }
    }

    fn poll_request(
        &mut self,
        _request: rdif_block::RequestId,
    ) -> Result<(), rdif_block::BlkError> {
        Ok(())
    }
}

impl SdCardQueue {
    fn validate_buffer(buffer: &[u8]) -> Result<(), BlkError> {
        if buffer.len() < BLOCK_SIZE {
            return Err(BlkError::Other(Box::new(
                BufferError::InvalidSize {
                    expected: BLOCK_SIZE,
                    actual: buffer.len(),
                }
            )));
        }

        let (prefix, _, suffix) = unsafe { buffer.align_to::<u32>() };
        if !prefix.is_empty() || !suffix.is_empty() {
            return Err(BlkError::Other(Box::new(
                BufferError::InvalidAlignment
            )));
        }

        Ok(())
    }
}

#[derive(Debug)]
enum BufferError {
    InvalidSize { expected: usize, actual: usize },
    InvalidAlignment,
}

impl core::fmt::Display for BufferError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            BufferError::InvalidSize { expected, actual } => {
                write!(f, "Invalid buffer size: expected at least {}, got {}", expected, actual)
            }
            BufferError::InvalidAlignment => {
                write!(f, "Buffer is not properly aligned for u32 access")
            }
        }
    }
}

impl core::error::Error for BufferError {}

#[derive(Debug)]
struct MCIErrorWrapper(MCIHostError);

impl core::fmt::Display for MCIErrorWrapper {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "MCI Host Error: {:?}", self.0)
    }
}

impl core::error::Error for MCIErrorWrapper {}

// 错误映射函数
fn map_mci_error_to_blk_error(err: MCIHostError) -> BlkError {
    match err {
        MCIHostError::Timeout => BlkError::Retry,

        MCIHostError::OutOfRange | MCIHostError::InvalidArgument => {
            BlkError::Other(Box::new(MCIErrorWrapper(err)))
        }

        MCIHostError::CardDetectFailed | MCIHostError::CardInitFailed => {
            BlkError::NotSupported
        }

        MCIHostError::InvalidVoltage 
        | MCIHostError::SwitchVoltageFail 
        | MCIHostError::SwitchVoltage18VFail33VSuccess => {
            BlkError::NotSupported
        }
        
        MCIHostError::TransferFailed 
        | MCIHostError::StopTransmissionFailed 
        | MCIHostError::WaitWriteCompleteFailed => {
            BlkError::Retry
        }
        
        // 其他所有错误包装为Other
        _ => BlkError::Other(Box::new(MCIErrorWrapper(err))),
    }
}

// impl SdCardDriver {
//     #[inline]
//     fn validate_buffer_alignment(buffer: &[u8]) -> Result<(), io::Error> {
//         if buffer.len() < BLOCK_SIZE {
//             return Err(io::Error {
//                 kind: io::ErrorKind::InvalidParameter { name: "buffer" },
//                 success_pos: 0,
//             });
//         }

//         let (prefix, _, suffix) = unsafe { buffer.align_to::<u32>() };
//         if !prefix.is_empty() || !suffix.is_empty() {
//             return Err(io::Error {
//                 kind: io::ErrorKind::InvalidParameter { name: "buffer" },
//                 success_pos: 0,
//             });
//         }

//         Ok(())
//     }
// }

// impl Interface for SdCardDriver {
//     /// Reads a block of data from the SD card.
//     ///
//     /// # Arguments
//     /// * `block_id` - The block identifier to read from
//     /// * `buffer` - Mutable buffer to store the read data (must be >= BLOCK_SIZE and u32-aligned)
//     ///
//     /// # Returns
//     /// * `Ok(())` on success
//     /// * `Err(io::Error)` if buffer is invalid or read operation fails
//     fn read_block(&mut self, block_id: usize, buffer: &mut [u8]) -> Result<(), io::Error> {
//         trace!("read block {}", block_id + OFFSET / 512);
//         let actual_block_id = block_id + OFFSET / 512;

//         Self::validate_buffer_alignment(buffer)?;

//         let (_, aligned_buf, _) = unsafe { buffer.align_to_mut::<u32>() };

//         let mut temp_buf: Vec<u32> = Vec::with_capacity(aligned_buf.len());

//         let sd_card = unsafe { &mut *self.0.get() };
//         sd_card
//             .read_blocks(&mut temp_buf, actual_block_id as u32, 1)
//             .map_err(deal_mci_host_err)?;

//         let copy_len = cmp::min(temp_buf.len(), aligned_buf.len());
//         aligned_buf[..copy_len].copy_from_slice(&temp_buf[..copy_len]);

//         Ok(())
//     }

//     /// Writes a block of data to the SD card.
//     ///
//     /// # Arguments
//     /// * `block_id` - The block identifier to write to
//     /// * `buffer` - Buffer containing data to write (must be >= BLOCK_SIZE and u32-aligned)
//     ///
//     /// # Returns
//     /// * `Ok(())` on success
//     /// * `Err(io::Error)` if buffer is invalid or write operation fails
//     fn write_block(&mut self, block_id: usize, buffer: &[u8]) -> Result<(), io::Error> {
//         trace!("write block {}", block_id + OFFSET / 512);
//         let actual_block_id = block_id + OFFSET / 512;

//         Self::validate_buffer_alignment(buffer)?;

//         let (_, aligned_buf, _) = unsafe { buffer.align_to::<u32>() };

//         let sd_card = unsafe { &mut *self.0.get() };

//         let mut write_buf: Vec<u32> = aligned_buf.to_vec();
//         sd_card
//             .write_blocks(&mut write_buf, actual_block_id as u32, 1)
//             .map_err(deal_mci_host_err)?;

//         Ok(())
//     }

//     /// flushes any buffered data to the SD card.
//     fn flush(&mut self) -> Result<(), io::Error> {
//         Ok(())
//     }

//     /// Returns the number of blocks on the SD card.
//     #[inline]
//     fn num_blocks(&self) -> usize {
//         let sd_card = unsafe { &*self.0.get() };
//         sd_card.block_count() as usize
//     }

//     /// Returns the block size in bytes.
//     #[inline]
//     fn block_size(&self) -> usize {
//         let sd_card = unsafe { &*self.0.get() };
//         sd_card.block_size() as usize
//     }
// }
