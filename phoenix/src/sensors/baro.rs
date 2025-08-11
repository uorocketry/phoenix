use common_arm::drivers::ms5611::{Ms5611, OversamplingRatio};
use defmt::info;
use embassy_stm32::gpio::Output;
use embassy_stm32::mode::Blocking;
use embassy_stm32::spi::Spi;
use embassy_time::{Delay, Duration, Timer};

#[embassy_executor::task]
async fn baro_reader_task(mut baro: Ms5611<Spi<'static, Blocking>, Output<'static>, Delay>) {
    info!("Barometer reader task started.");
    loop {
        match baro.read_pressure_temperature(OversamplingRatio::Osr4096) {
            Ok(reading) => {}
            Err(e) => {
                // error!("Baro: Driver reading failed: {:?}", e);
            }
        }
        Timer::after(Duration::from_millis(100)).await;
    }
}
