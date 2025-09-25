extern crate alloc;

use crate::clk::ClkDriver;
use alloc::{boxed::Box, sync::Arc};
use axklib::{mem::iomap, time::busy_wait};
use core::time::Duration;
use log::{debug, info, warn};
use rdif_block::{IQueue, Interface};
use rdif_clk::Interface as _;
use rdrive::{DriverGeneric, KError, get_one};
use rdrive::{PlatformDevice, module_driver, probe::OnProbeError, register::FdtInfo};

use sdmmc::{
    BLOCK_SIZE, Kernel,
    emmc::{
        EMmcHost,
        clock::{Clk, ClkError, init_global_clk},
    },
    err::SdError,
    set_impl,
};

use spin::Mutex;

const OFFSET: usize = 0x7_A000;

/// Driver for the RK3568 eMMC controller.
/// Driver for the RK3568 eMMC controller.

pub struct KernelImpl;

impl Kernel for KernelImpl {
    fn sleep(us: u64) {
        let duration = Duration::from_micros(us);
        busy_wait(duration);
    }
}

set_impl!(KernelImpl);

module_driver!(
    name: "Rockchip SDHCI",
    level: ProbeLevel::PostKernel,
    priority: ProbePriority::DEFAULT,
    probe_kinds: &[
        ProbeKind::Fdt {
            compatibles: &["rockchip,dwcmshc-sdhci"],
            on_probe: probe_mmc
        }
    ],
);

fn probe_mmc(info: FdtInfo<'_>, plat_dev: PlatformDevice) -> Result<(), OnProbeError> {
    info!("Probing Rockchip DWCM SHC SDHCI...");

    let mci_reg = info
        .node
        .reg()
        .and_then(|mut regs| regs.next())
        .ok_or(OnProbeError::other(alloc::format!(
            "[{}] has no reg",
            info.node.name()
        )))?;

    info!(
        "MCI reg: addr={:#x}, size={:#x}",
        mci_reg.address as usize,
        mci_reg.size.unwrap_or(0)
    );

    let mci_reg_base = iomap(
        (mci_reg.address as usize).into(),
        mci_reg.size.unwrap_or(0x10000),
    )
    .expect("Failed to iomap MCI");

    let _ = init_clk(0x7c);

    let mmc_address = mci_reg_base.as_ptr() as usize;

    debug!("mmc address: {:#x}", mmc_address);
    let mut emmc = EMmcHost::new(mmc_address);

    if emmc.init().is_ok() {
        info!("RK3568 eMMC: successfully initialized");
    } else {
        warn!("RK3568 eMMC: init failed");
    }

    let emmc = EmmcDriver::new(emmc);
    let dev = rdif_block::Block::new(emmc);
    plat_dev.register(dev);

    Ok(())
}

pub struct EmmcDriver {
    pub host: Arc<Mutex<EMmcHost>>,
}

impl EmmcDriver {
    /// Creates a new `EmmcDriver` instance.
    pub fn new(emmc_host: EMmcHost) -> Self {
        let host = Arc::new(Mutex::new(emmc_host));
        EmmcDriver { host }
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
    fn create_queue(&mut self) -> Option<alloc::boxed::Box<dyn rdif_block::IQueue>> {
        // 创建新的队列结构体实例
        Some(alloc::boxed::Box::new(EmmcQueue {
            host: Arc::clone(&self.host),
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

/// 专门用于处理I/O队列操作的结构体
pub struct EmmcQueue {
    host: Arc<Mutex<EMmcHost>>,
}

impl IQueue for EmmcQueue {
    /// Returns the total number of blocks available on the device.
    fn num_blocks(&self) -> usize {
        self.host.lock().get_block_num() as _
    }

    /// Returns the block size in bytes.
    fn block_size(&self) -> usize {
        self.host.lock().get_block_size()
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

    fn submit_request(
        &mut self,
        request: rdif_block::Request<'_>,
    ) -> Result<rdif_block::RequestId, rdif_block::BlkError> {
        let id = request.block_id + OFFSET;

        match request.kind {
            rdif_block::RequestKind::Read(mut buffer) => {
                if buffer.len() < BLOCK_SIZE {
                    return Err(rdif_block::BlkError::Other(Box::new(
                        BufferError::InvalidSize {
                            expected: BLOCK_SIZE,
                            actual: buffer.len(),
                        },
                    )));
                }

                let (prefix, _, suffix) = unsafe { buffer.align_to_mut::<u32>() };
                if !prefix.is_empty() || !suffix.is_empty() {
                    return Err(rdif_block::BlkError::Other(Box::new(
                        BufferError::InvalidAlignment,
                    )));
                }

                self.host
                    .lock()
                    .read_blocks(id as u32, 1, &mut buffer)
                    .map_err(|err| map_sd_error_to_blk_error(err))?;

                Ok(rdif_block::RequestId::new(0))
            }
            rdif_block::RequestKind::Write(buffer) => {
                if buffer.len() < BLOCK_SIZE {
                    return Err(rdif_block::BlkError::Other(Box::new(
                        BufferError::InvalidSize {
                            expected: BLOCK_SIZE,
                            actual: buffer.len(),
                        },
                    )));
                }

                let (prefix, _, suffix) = unsafe { buffer.align_to::<u32>() };
                if !prefix.is_empty() || !suffix.is_empty() {
                    return Err(rdif_block::BlkError::Other(Box::new(
                        BufferError::InvalidAlignment,
                    )));
                }

                // 修复：使用正确的 self.host.lock() 而不是 self.0
                self.host
                    .lock()
                    .write_blocks(id as u32, 1, buffer)
                    .map_err(|err| map_sd_error_to_blk_error(err))?;

                Ok(rdif_block::RequestId::new(0))
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
        let clk_device = get_one::<ClkDriver>().unwrap();

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
        let clk_device = get_one::<ClkDriver>().unwrap();

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

#[derive(Debug)]
enum BufferError {
    InvalidSize { expected: usize, actual: usize },
    InvalidAlignment,
}

impl core::fmt::Display for BufferError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            BufferError::InvalidSize { expected, actual } => {
                write!(
                    f,
                    "Invalid buffer size: expected at least {}, got {}",
                    expected, actual
                )
            }
            BufferError::InvalidAlignment => {
                write!(f, "Buffer is not properly aligned for u32 access")
            }
        }
    }
}

impl core::error::Error for BufferError {}
#[derive(Debug)]
struct SdErrorWrapper(SdError);

impl core::fmt::Display for SdErrorWrapper {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "SD/eMMC Error: {:?}", self.0)
    }
}

impl core::error::Error for SdErrorWrapper {}

// 错误映射函数
fn map_sd_error_to_blk_error(err: SdError) -> rdif_block::BlkError {
    match err {
        // 超时错误通常需要重试
        SdError::Timeout | SdError::DataTimeout => rdif_block::BlkError::Retry,

        // 不支持的卡类型
        SdError::UnsupportedCard => rdif_block::BlkError::NotSupported,

        // 其他错误包装为Other，使用自定义包装器
        _ => rdif_block::BlkError::Other(Box::new(SdErrorWrapper(err))),
    }
}
