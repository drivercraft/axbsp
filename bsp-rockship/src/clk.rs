extern crate alloc;

use rdrive::{DriverGeneric, driver::clk::*};
use rk3568_clk::CRU;
use rk3568_clk::cru_clksel_con28_bits::*;

/// 频率常量
const MHZ: u32 = 1_000_000;
const KHZ: u32 = 1_000;

use core::convert::Into;
use core::result::Result::{self, *};
use log::{debug, info, warn};

pub struct ClkDriver(CRU);

pub const EMMC_CLK_ID: usize = 0x7c;

impl ClkDriver {
    pub fn new(cru_address: u64) -> Self {
        ClkDriver(CRU::new(cru_address as *mut _))
    }
}

impl DriverGeneric for ClkDriver {
    fn open(&mut self) -> Result<(), KError> {
        Ok(())
    }

    fn close(&mut self) -> Result<(), KError> {
        Ok(())
    }
}

impl Interface for ClkDriver {
    fn perper_enable(&mut self) {
        debug!("perper_enable");
    }

    fn get_rate(&self, id: ClockId) -> Result<u64, KError> {
        let rate = match id.into() {
            EMMC_CLK_ID => {
                let con = self.0.cru_clksel_get_cclk_emmc();
                con >> CRU_CLKSEL_CCLK_EMMC_POS
            }
            _ => {
                warn!("Unsupported clock ID: {:?}", id);
                Err(KError::InvalidArg { name: "clock_id" })?
            }
        };
        Ok(rate as u64)
    }

    fn set_rate(&mut self, id: ClockId, rate: u64) -> Result<(), KError> {
        match id.into() {
            EMMC_CLK_ID => {
                info!("Setting eMMC clock to {} Hz", rate);
                let src_clk = match rate as u32 {
                    r if r == 24 * MHZ => CRU_CLKSEL_CCLK_EMMC_XIN_SOC0_MUX,
                    r if r == 52 * MHZ || r == 50 * MHZ => CRU_CLKSEL_CCLK_EMMC_CPL_DIV_50M,
                    r if r == 100 * MHZ => CRU_CLKSEL_CCLK_EMMC_CPL_DIV_100M,
                    r if r == 150 * MHZ => CRU_CLKSEL_CCLK_EMMC_GPL_DIV_150M,
                    r if r == 200 * MHZ => CRU_CLKSEL_CCLK_EMMC_GPL_DIV_200M,
                    r if r == 400 * KHZ || r == 375 * KHZ => CRU_CLKSEL_CCLK_EMMC_SOC0_375K,
                    _ => panic!("Unsupported eMMC clock rate: {} Hz", rate),
                };
                self.0.cru_clksel_set_cclk_emmc(src_clk);
            }
            _ => {
                warn!("Unsupported clock ID: {:?}", id);
                return Err(KError::InvalidArg { name: "clock_id" });
            }
        }
        Ok(())
    }
}
