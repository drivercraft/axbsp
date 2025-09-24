extern crate alloc;

use alloc::boxed::Box;
use alloc::format;
use log::info;
use rdrive::{DriverGeneric, driver::block::*};
use rdrive::get_one;

use sdmmc::BLOCK_SIZE;
use sdmmc::emmc::clock::{Clk, ClkError, init_global_clk};
use sdmmc::err::SdError;

pub use sdmmc::emmc::EMmcHost;
pub use sdmmc::{Kernel, set_impl};

const OFFSET: usize = 0x7_A000;

/// Maps `SdError` values from the lower-level driver to generic `DevError`s.
fn deal_emmc_err(err: SdError) -> io::Error {
    let kind = match err {
        SdError::Timeout | SdError::DataTimeout => io::ErrorKind::TimedOut,

        SdError::InvalidResponse | SdError::InvalidResponseType => {
            io::ErrorKind::InvalidParameter { name: "response" }
        }
        SdError::UnsupportedCard => io::ErrorKind::InvalidParameter { name: "card" },

        SdError::TuningFailed | SdError::VoltageSwitchFailed => io::ErrorKind::InvalidParameter {
            name: "configuration",
        },
        SdError::BadMessage | SdError::InvalidArgument => {
            io::ErrorKind::InvalidParameter { name: "parameter" }
        }
        SdError::BusWidth => io::ErrorKind::InvalidParameter { name: "bus_width" },
        SdError::CardError(_, _) => io::ErrorKind::InvalidParameter { name: "card_error" },
        _ => io::ErrorKind::Other(format!("{err}").into()),
    };
    io::Error {
        kind,
        success_pos: 0,
    }
}

/// Driver for the RK3568 eMMC controller.
pub struct EmmcDriver(pub EMmcHost);

impl EmmcDriver {
    /// Creates a new `EmmcDriver` instance.
    pub fn new(base_addr: usize) -> Self {
        EmmcDriver(EMmcHost::new(base_addr))
    }
}

impl DriverGeneric for EmmcDriver {
    fn open(&mut self) -> Result<(), KError> {
        Ok(())
    }

    fn close(&mut self) -> Result<(), KError> {
        Ok(())
    }
}

impl Interface for EmmcDriver {
    /// Reads a single block from the eMMC device into the provided buffer.
    fn read_block(&mut self, block_id: usize, buf: &mut [u8]) -> Result<(), io::Error> {
        let block_id = block_id + OFFSET;
        if buf.len() < BLOCK_SIZE {
            return Err(io::Error {
                kind: io::ErrorKind::InvalidParameter { name: "buffer" },
                success_pos: 0,
            });
        }

        let (prefix, _, suffix) = unsafe { buf.align_to_mut::<u32>() };
        if !prefix.is_empty() || !suffix.is_empty() {
            return Err(io::Error {
                kind: io::ErrorKind::InvalidParameter { name: "buffer" },
                success_pos: 0,
            });
        }

        self.0
            .read_blocks(block_id as u32, 1, buf)
            .map_err(deal_emmc_err)
    }

    /// Writes a single block to the eMMC device from the given buffer.
    fn write_block(&mut self, block_id: usize, buf: &[u8]) -> Result<(), io::Error> {
        let block_id = block_id + OFFSET;
        if buf.len() < BLOCK_SIZE {
            return Err(io::Error {
                kind: io::ErrorKind::InvalidParameter { name: "buffer" },
                success_pos: 0,
            });
        }

        let (prefix, _, suffix) = unsafe { buf.align_to::<u32>() };
        if !prefix.is_empty() || !suffix.is_empty() {
            return Err(io::Error {
                kind: io::ErrorKind::InvalidParameter { name: "buffer" },
                success_pos: 0,
            });
        }

        self.0
            .write_blocks(block_id as u32, 1, buf)
            .map_err(deal_emmc_err)
    }

    /// Flushes any cached writes (no-op for now).
    fn flush(&mut self) -> Result<(), io::Error> {
        Ok(())
    }

    /// Returns the total number of blocks available on the device.
    #[inline]
    fn num_blocks(&self) -> usize {
        self.0.get_block_num() as _
    }

    /// Returns the block size in bytes.
    #[inline]
    fn block_size(&self) -> usize {
        self.0.get_block_size()
    }
}

pub struct EmmcClk {
    pub core_clk_index: usize,
}

impl EmmcClk {
    pub fn new(core_clk_index: usize) -> Self {
        EmmcClk { core_clk_index }
    }
}

impl Clk for EmmcClk {
    fn emmc_get_clk(&self) -> Result<u64, ClkError> {
        let clk_device = get_one::<rdrive::driver::Clk>().unwrap();

        info!(
            "Getting eMMC clock rate using core_clk_index: {}",
            self.core_clk_index
        );
        info!("Found clock device: {:?}", unsafe {
            clk_device.force_use()
        });

        let clk = clk_device.lock().unwrap();
        let rate = clk.get_rate(self.core_clk_index.into()).unwrap();
        Ok(rate)
    }

    fn emmc_set_clk(&self, rate: u64) -> Result<u64, ClkError> {
        let clk_device = get_one::<rdrive::driver::Clk>().unwrap();

        info!(
            "Getting eMMC clock rate using core_clk_index: {}",
            self.core_clk_index
        );
        info!("Found clock device: {:?}", unsafe {
            clk_device.force_use()
        });

        let mut clk = clk_device.lock().unwrap();
        let _rate = clk.set_rate(self.core_clk_index.into(), rate).unwrap();
        Ok(0)
    }
}

pub fn init_clk(core_clk_index: usize) -> Result<(), ClkError> {
    let emmc_clk = EmmcClk::new(core_clk_index);
    let static_clk: &'static dyn Clk = Box::leak(Box::new(emmc_clk));
    init_global_clk(static_clk);
    Ok(())
}
