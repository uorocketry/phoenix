use common_arm::drivers::ms5611::{Ms5611, OversamplingRatio};
use defmt::{error, info};
use embassy_stm32::gpio::Output;
use embassy_stm32::mode::Blocking;
use embassy_stm32::spi::Spi;
use embassy_time::{Delay, Duration, Timer};

use crate::resources::PRESSURE_CHANNEL;

#[embassy_executor::task]
pub async fn baro_reader_task(mut baro: Ms5611<Spi<'static, Blocking>, Output<'static>, Delay>) {
    info!("Barometer reader task started.");

    loop {
        match baro.read_pressure_temperature(OversamplingRatio::Osr4096) {
            Ok(reading) => {
                PRESSURE_CHANNEL.try_send((reading.1, reading.0, 1, embassy_time::Instant::now()));
            }
            Err(e) => {
                error!("Baro: Driver reading failed");
            }
        }
        Timer::after(Duration::from_millis(10)).await;
    }
}
