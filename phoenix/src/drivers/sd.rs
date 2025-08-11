use defmt::*;
use embassy_stm32::gpio::Output;
use embassy_stm32::spi::Spi;
use embassy_time::Delay;
use embedded_hal_1::delay::DelayNs;
use embedded_hal_bus::spi::RefCellDevice;
use embedded_sdmmc::{Mode, SdCard, VolumeIdx, VolumeManager};
use core::marker::PhantomData;

/// Minimal time source for embedded-sdmmc
pub struct TimeSink {
    pub(crate) _marker: PhantomData<*const ()>,
}

impl TimeSink {
    pub fn new() -> Self {
        Self { _marker: PhantomData }
    }
}

impl embedded_sdmmc::TimeSource for TimeSink {
    fn get_timestamp(&self) -> embedded_sdmmc::Timestamp {
        embedded_sdmmc::Timestamp {
            year_since_1970: 0,
            zero_indexed_month: 0,
            zero_indexed_day: 0,
            hours: 0,
            minutes: 0,
            seconds: 0,
        }
    }
}

// Initializes the SD card and performs the small demo previously in main.rs.
// Returns Ok(()) if initialization and simple access succeed.
pub fn init_and_demo(
    sd_spi_bus: Spi<'static, embassy_stm32::mode::Blocking>,
    sd_cs: Output<'static>,
) -> Result<(), ()> {
    let sd_spi_bus_ref_cell = core::cell::RefCell::new(sd_spi_bus);
    let sd_spi_device = RefCellDevice::new(&sd_spi_bus_ref_cell, sd_cs, Delay).map_err(|_| ())?;
    let sdcard = SdCard::new(sd_spi_device, Delay);
    info!("Card size is {} bytes", sdcard.num_bytes().map_err(|_| ())?);
    let volume_mgr = VolumeManager::new(
        sdcard,
        TimeSink::new(),
    );
    let _volume0 = volume_mgr.open_volume(VolumeIdx(0)).map_err(|_| ())?;
    // let root_dir = volume0.open_root_dir().map_err(|_| ())?;
    // Example read disabled by default; enable as needed.
    Ok(())
}
