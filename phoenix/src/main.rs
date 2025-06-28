#![no_std]
#![no_main]

mod madgwick_service;
mod sbg_manager;

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex};
use embassy_sync::channel::Channel;
use embassy_time::Timer;
use embassy_time::{Duration, Ticker};
use messages_prost::sensor::sbg::Air;

use {defmt_rtt as _, panic_probe as _};

static SENSOR_CHANNEL: Channel<CriticalSectionRawMutex, Air, 4> = Channel::new();

#[inline(never)]
#[defmt::panic_handler]
fn panic() -> ! {
    // Resets the system if a panic occurs.
    stm32h7xx_hal::pac::SCB::sys_reset()
}

#[embassy_executor::task]
async fn sensor_task() {
    info!("Sensor task started");
    // A Ticker provides a tick at a fixed interval.
    let mut ticker = Ticker::every(Duration::from_secs(1));
    let mut temp_reading = 20;

    loop {
        // Simulate reading temperature
        let reading = Air::default();

        // 3. Send the data.
        //    .send() is async. If the channel is full, this task will yield
        //    and wait until there is space available.
        SENSOR_CHANNEL.send(reading).await;

        temp_reading += 1; // Increment for the next reading

        // Occasionally, let's send a different kind of reading.
        if temp_reading % 5 == 0 {
            let humidity_reading = Air::default();
            SENSOR_CHANNEL.send(humidity_reading).await;
        }

        // Wait for the next tick.
        ticker.next().await;
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(Default::default());
    info!("Hello World!");

    let mut led = Output::new(p.PB14, Level::High, Speed::Low);

    loop {
        info!("high");
        led.set_high();
        Timer::after_millis(500).await;

        info!("low");
        led.set_low();
        Timer::after_millis(500).await;
    }
}
