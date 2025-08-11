use defmt::*;
use embassy_stm32::gpio::Output;
use embassy_stm32::spi::Spi;
use embassy_time::Delay;
use embedded_hal_1::delay::DelayNs;
use embedded_hal_bus::spi::RefCellDevice;
use embedded_sdmmc::{Mode, SdCard, VolumeIdx, VolumeManager};

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
        super::super::TimeSink {
            _marker: core::marker::PhantomData,
        },
    );
    let _volume0 = volume_mgr.open_volume(VolumeIdx(0)).map_err(|_| ())?;
    // let root_dir = volume0.open_root_dir().map_err(|_| ())?;
    // Example read disabled by default; enable as needed.
    Ok(())
}
