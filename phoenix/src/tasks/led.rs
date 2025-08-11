use defmt::*;
use embassy_executor::task;
use embassy_stm32::{gpio::Level, gpio::Output, gpio::Speed, peripherals};
use embassy_time::Timer;

#[task]
pub async fn led_blinker_task(pin: peripherals::PB14) {
    let mut led = Output::new(pin, Level::High, Speed::Low);
    info!("LED blinker task started.");
    loop {
        led.set_high();
        Timer::after_millis(500).await;
        led.set_low();
        Timer::after_millis(500).await;
    }
}
