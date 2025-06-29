// #![no_std]
// #![no_main]

// mod madgwick_service;
// mod sbg_manager;

// use core::cell::RefCell;
// use core::mem::MaybeUninit;

// use chrono::{NaiveDate, NaiveDateTime, Timelike};
// use defmt::*;
// use embassy_executor::Spawner;
// use embassy_stm32::gpio::{Level, Output, Speed};
// use embassy_stm32::rcc::LsConfig;
// use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex};
// use embassy_sync::channel::Channel;
// use embassy_sync::blocking_mutex::Mutex;
// use embassy_time::Timer;
// use embassy_time::{Duration, Ticker};
// use messages_prost::sensor::sbg::Air;
// use embassy_stm32::usart::{Config, Uart};
// use static_cell::StaticCell;
// use embassy_stm32::{bind_interrupts, peripherals, usart};
// use embassy_executor::Executor;

// use embassy_stm32::rtc::{Rtc, RtcConfig};

// use {defmt_rtt as _, panic_probe as _};

// static SENSOR_CHANNEL: Channel<CriticalSectionRawMutex, Air, 4> = Channel::new();

// pub static RTC: Mutex<CriticalSectionRawMutex, RefCell<MaybeUninit<Rtc>>> =
//     Mutex::new(RefCell::new(MaybeUninit::uninit()));

// bind_interrupts!(struct Irqs {
//     UART7 => usart::InterruptHandler<peripherals::UART7>;
// });

// #[inline(never)]
// #[defmt::panic_handler]
// fn panic() -> ! {
//     // Resets the system if a panic occurs.
//     stm32h7xx_hal::pac::SCB::sys_reset()
// }

// #[embassy_executor::task]
// async fn sensor_task() {
//     info!("Sensor task started");
//     // A Ticker provides a tick at a fixed interval.
//     let mut ticker = Ticker::every(Duration::from_secs(1));
//     let mut temp_reading = 20;

//     loop {
//         // Simulate reading temperature
//         let reading = Air::default();

//         // 3. Send the data.
//         //    .send() is async. If the channel is full, this task will yield
//         //    and wait until there is space available.
//         SENSOR_CHANNEL.send(reading).await;

//         temp_reading += 1; // Increment for the next reading

//         // Occasionally, let's send a different kind of reading.
//         if temp_reading % 5 == 0 {
//             let humidity_reading = Air::default();
//             SENSOR_CHANNEL.send(humidity_reading).await;
//         }

//         // Wait for the next tick.
//         ticker.next().await;
//     }
// }

// #[embassy_executor::main]
// async fn main(spawner: Spawner) {
//     let mut config = embassy_stm32::Config::default();
//     config.rcc.ls = LsConfig::default_lse();

//     let p = embassy_stm32::init(config);

//     let config_uart_sbg = Config::default();
//     let mut usart = Uart::new(p.UART7, p.PF6, p.PF7, Irqs, p.DMA1_CH0, p.DMA1_CH1, config_uart_sbg).unwrap();

//     let now = NaiveDate::from_ymd_opt(2020, 5, 15)
//         .unwrap()
//         .and_hms_opt(10, 30, 15)
//         .unwrap();

//     let mut rtc = Rtc::new(p.RTC, RtcConfig::default());
//     info!("Got RTC! {:?}", now.and_utc().timestamp());

//     rtc.set_datetime(now.into()).expect("datetime not set");

//     // In reality the delay would be much longer
//     Timer::after_millis(20000).await;

//     let then: NaiveDateTime = rtc.now().unwrap().into();

//     info!("Boot: {}", then.time().num_seconds_from_midnight());

//     RTC.lock(|cell| {
//         // The closure gives us direct, mutable access to the MaybeUninit inside.
//         // We can then write the initialized `rtc` instance into it.
//         cell.borrow_mut().write(rtc);
//     });

//     info!("Hello World!");

//     let mut led = Output::new(p.PB14, Level::High, Speed::Low);

//     spawner.must_spawn(sensor_task());

//     loop {
//         info!("high");
//         led.set_high();
//         Timer::after_millis(500).await;

//         info!("low");
//         led.set_low();
//         Timer::after_millis(500).await;
//     }
// }

#![no_std]
#![no_main]

mod madgwick_service;
mod sbg_manager;

use core::cell::RefCell;
use core::mem::MaybeUninit;

use chrono::NaiveDate;
use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::rcc::LsConfig;
use embassy_stm32::rtc::{Rtc, RtcConfig};
// Import the correct, modern type names `UartRx` and `UartTx`.
use embassy_stm32::usart::{Config, RingBufferedUartRx, Uart, UartTx};
use embassy_stm32::{bind_interrupts, mode, peripherals, usart};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::channel::Channel;
use embassy_time::Timer;
use heapless::Vec;
use messages_prost::sensor::sbg::SbgData;
use sbg_rs::sbg::SBG_BUFFER_SIZE;
use static_cell::StaticCell;
// Ensure panic handler is included.
use embedded_alloc::Heap;
use {defmt_rtt as _, panic_probe as _};

// =================================================================================
// Shared Resources & Types
// =================================================================================

type DmaBuffer = [u8; SBG_BUFFER_SIZE];

// The type of message passed from the UART reader to the sbg read.
type UartMessage = Vec<u8, SBG_BUFFER_SIZE>;

static HEAP: Heap = Heap::empty();

// A static Channel for UART data communication.
static UART_CHANNEL: Channel<CriticalSectionRawMutex, UartMessage, 4> = Channel::new();

static SBG_CHANNEL: Channel<CriticalSectionRawMutex, SbgData, 10> = Channel::new();

static BUFFER_CHANNEL: Channel<CriticalSectionRawMutex, DmaBuffer, 2> = Channel::new();

// A static, shareable RTC instance using the Mutex + RefCell pattern.
pub static RTC: Mutex<CriticalSectionRawMutex, RefCell<MaybeUninit<Rtc>>> =
    Mutex::new(RefCell::new(MaybeUninit::uninit()));

pub static SBG_MANAGER: Mutex<
    CriticalSectionRawMutex,
    RefCell<MaybeUninit<sbg_manager::SBGManager>>,
> = Mutex::new(RefCell::new(MaybeUninit::uninit()));

// Bind interrupts for the UART peripheral.
bind_interrupts!(struct Irqs {
    UART7 => usart::InterruptHandler<peripherals::UART7>;
});

// =================================================================================
// Application Tasks
// =================================================================================

/// Task dedicated to blinking an LED to show the system is alive.
#[embassy_executor::task]
async fn led_blinker_task(pin: peripherals::PB14) {
    let mut led = Output::new(pin, Level::High, Speed::Low);
    info!("LED blinker task started.");
    loop {
        led.set_high();
        Timer::after_millis(500).await;
        led.set_low();
        Timer::after_millis(500).await;
    }
}

#[embassy_executor::task]
async fn uart_dma_reader_task(mut rx: RingBufferedUartRx<'static>) {
    loop {
        let mut buf: [u8; SBG_BUFFER_SIZE] = [0; SBG_BUFFER_SIZE];

        match rx.read(&mut buf).await {
            Ok(len) => {
                if len > 0 {
                    BUFFER_CHANNEL.send(buf);
                }
            }
            Err(e) => error!("{:?}", e),
        }
    }
}

#[embassy_executor::task]
async fn sbg_parser_task(tx: UartTx<'static, mode::Async>) {
    let mut sbg = sbg_manager::SBGManager::new(tx);

    loop {
        // Wait for a full buffer reference from the reader task.
        let full_buffer = BUFFER_CHANNEL.receive().await;

        info!(
            "PARSER: Received buffer with {} bytes. Processing...",
            full_buffer.len()
        );

        sbg.sbg_device
            .read_data(full_buffer.as_slice().try_into().unwrap());

        // The buffer is dropped here, its memory is now free to be reused by the DMA reader.
        info!("PARSER: Finished processing buffer.");
    }
}

// /// Task dedicated to writing (echoing) data back to the UART via DMA.
// /// The function signature now uses the correct `UartTx` type.
// #[embassy_executor::task]
// async fn writer_task(mut tx: UartTx<'static, mode::Async>) {
//     info!("UART writer task started.");
//     loop {
//         // Wait for a message from the reader task.
//         let data = UART_CHANNEL.receive().await;

//         info!("UART TX: Echoing {} bytes: {:?}", data.len(), data.as_slice());

//         if let Err(e) = tx.write(&data).await {
//             error!("UART TX error: {:?}", e);
//         }
//     }
// }

#[embassy_executor::task]
async fn sbg_channel_task() {
    loop {
        // Wait for a message from the reader task.
        let data = SBG_CHANNEL.receive().await;

        info!("Data on SBG Channel.")
    }
}

// =================================================================================
// Main Entry Point
// =================================================================================

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("System starting...");
    /* Initialize the Heap */
    {
        use core::mem::MaybeUninit;
        const HEAP_SIZE: usize = 1024;
        // TODO: Could add a link section here to memory.
        static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
        unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE) }
    }

    // 1. Configure system clocks.
    let mut config = embassy_stm32::Config::default();
    config.rcc.ls = LsConfig::default_lse(); // Use LSE for better RTC accuracy
    let p = embassy_stm32::init(config);

    // unsafe {
    //     // Convert an uninitialised array into an array of uninitialised
    //     let buf: &mut [core::mem::MaybeUninit<u8>; SBG_BUFFER_SIZE] =
    //         &mut *(core::ptr::addr_of_mut!(SBG_BUFFER) as *mut _);
    //     buf.iter_mut().for_each(|x| x.as_mut_ptr().write(0));
    // }

    // 2. Initialize and configure the RTC.
    let mut rtc = Rtc::new(p.RTC, RtcConfig::default());
    if rtc.now().is_err() {
        info!("RTC not set. Configuring with default time...");
        let now = NaiveDate::from_ymd_opt(2025, 1, 1)
            .unwrap()
            .and_hms_opt(10, 30, 0)
            .unwrap();
        rtc.set_datetime(now.into())
            .expect("Failed to set RTC datetime");
    }
    // Place the initialized RTC into the static Mutex for sharing.
    RTC.lock(|cell| {
        cell.borrow_mut().write(rtc);
    });
    info!("RTC initialized.");

    // 3. Initialize and configure the UART.
    let uart_config = Config::default();
    // The `new` function takes the DMA channels directly.
    let usart = Uart::new(
        p.UART7,
        p.PF6, // RX pin
        p.PF7, // TX pin
        Irqs,
        p.DMA1_CH1, // TX DMA
        p.DMA1_CH0, // RX DMA
        uart_config,
    )
    .unwrap();

    // Use the modern `.split()` method.
    let (tx, rx) = usart.split();
    info!("UART initialized and split.");

    #[link_section = ".axisram.buffers"]
    static BUF_A: StaticCell<DmaBuffer> = StaticCell::new();

    #[link_section = ".axisram.buffers"]
    static BUF_B: StaticCell<DmaBuffer> = StaticCell::new();

    let buf_a = BUF_A.init([0; SBG_BUFFER_SIZE]);
    let buf_b = BUF_B.init([0; SBG_BUFFER_SIZE]);

    let ring_rx = rx.into_ring_buffered(buf_a);

    // 4. Spawn all the concurrent application tasks.
    spawner.must_spawn(led_blinker_task(p.PB14));
    spawner.must_spawn(uart_dma_reader_task(ring_rx));
    spawner.must_spawn(sbg_parser_task(tx));

    info!("All tasks spawned. Main function is complete.");
    loop {
        // state machine here
    }
}
