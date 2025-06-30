#![feature(impl_trait_in_assoc_type)]
#![no_std]
#![no_main]

mod madgwick_service;
mod sbg_manager;
mod traits; 

use core::cell::RefCell;
use chrono::NaiveDate;
use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::rtc::{Rtc, RtcConfig};
use embassy_stm32::spi::{Config as SpiConfig, Spi};
use embassy_stm32::time::mhz;
use embassy_stm32::usart::{Config as UartConfig, RingBufferedUartRx, Uart, UartTx};
use embassy_stm32::{bind_interrupts, mode, peripherals, rcc, usart};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Timer, Delay};
use embedded_alloc::Heap;
use heapless::Vec;
use messages_prost::sensor::sbg::SbgData;
use sbg_rs::sbg::SBG_BUFFER_SIZE;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

// Use a modern ms5611 driver that supports embedded-hal v1.0
use common_arm::drivers::ms5611::{Ms5611, Oversampling};

// Use the asynchronous SpiDevice from embassy-embedded-hal
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;

use smlang::statemachine;

// =================================================================================
// Shared Resources & Types
// =================================================================================

type DmaBuffer = [u8; SBG_BUFFER_SIZE];
type UartMessage = Vec<u8, SBG_BUFFER_SIZE>;

#[global_allocator]
static HEAP: Heap = Heap::empty();

static UART_CHANNEL: Channel<CriticalSectionRawMutex, UartMessage, 4> = Channel::new();
static SBG_CHANNEL: Channel<CriticalSectionRawMutex, SbgData, 10> = Channel::new();
static BUFFER_CHANNEL: Channel<CriticalSectionRawMutex, DmaBuffer, 2> = Channel::new();

// The SPI bus is protected by a Mutex, so the RefCell is not needed.
static SPI_BUS: StaticCell<embassy_sync::mutex::Mutex<CriticalSectionRawMutex, Spi<mode::Async>>> = StaticCell::new();

// Static variable for the RTC
pub static RTC: Mutex<CriticalSectionRawMutex, RefCell<Option<Rtc>>> =
    Mutex::new(RefCell::new(None));

bind_interrupts!(struct Irqs {
    UART7 => usart::InterruptHandler<peripherals::UART7>;
});

statemachine! {
    transitions: {
        *Init + Start = Idle,
        WaitForLaunch + Launch = Ascent,
        Fault + FaultCleared = Init,
        _ + FaultDetected = Fault,
    }
}

// =================================================================================
// Application Tasks
// =================================================================================

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
        if let Ok(len) = rx.read(&mut buf).await {
            if len > 0 {
                let _ = BUFFER_CHANNEL.try_send(buf);
            }
        }
    }
}

#[embassy_executor::task]
async fn sbg_parser_task(tx: UartTx<'static, mode::Async>) {
    let mut sbg = sbg_manager::SBGManager::new(tx);
    loop {
        let full_buffer = BUFFER_CHANNEL.receive().await;
        sbg.sbg_device.read_data(&full_buffer.try_into().unwrap());
    }
}

type BaroSpiDevice<'a> = SpiDevice<'a,
    CriticalSectionRawMutex,
    Spi<'a, mode::Async>,
    Output<'a>>;

#[embassy_executor::task]
async fn baro_reader_task(mut baro: Ms5611<BaroSpiDevice<'static>, Delay>) {
    info!("Barometer reader task started.");
    loop {
        match baro.get_pressure_and_temperature(Oversampling::Osr512).await {
            Ok(reading) => {
                info!(
                    "Baro: Temp: {} C, Pressure: {} mbar",
                    reading.0, reading.1
                );
            }
            Err(e) => {
                // error!("Baro: Driver reading failed: {:?}", e);
            }
        }
        Timer::after(Duration::from_millis(1000)).await;
    }
}

// =================================================================================
// Main Entry Point
// =================================================================================

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("System starting...");
    {
        use core::mem::MaybeUninit;
        const HEAP_SIZE: usize = 1024;
        static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
        unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE) }
    }

    let mut config = embassy_stm32::Config::default();
    config.rcc.ls = rcc::LsConfig::default_lse();
    let p = embassy_stm32::init(config);

    // --- RTC Setup ---
    let now = NaiveDate::from_ymd_opt(2025, 6, 29)
        .unwrap()
        .and_hms_opt(15, 30, 0)
        .unwrap();
    let mut rtc = Rtc::new(p.RTC, RtcConfig::default());
    rtc.set_datetime(now.into()).expect("Failed to set RTC time");

    RTC.lock(|cell| {
        *cell.borrow_mut() = Some(rtc);
    });

    // --- UART Setup ---
    let uart_config = UartConfig::default();
    let usart = Uart::new(
        p.UART7, p.PF6, p.PF7, Irqs, p.DMA1_CH1, p.DMA1_CH0, uart_config,
    ).unwrap();
    let (tx, rx) = usart.split();
    static mut RX_BUF: [u8; SBG_BUFFER_SIZE] = [0; SBG_BUFFER_SIZE];
    let ring_rx = rx.into_ring_buffered(unsafe { &mut RX_BUF });

    // --- SPI Setup ---
    let mut spi_config = SpiConfig::default();
    spi_config.frequency = mhz(1);
    spi_config.mode = embassy_stm32::spi::Mode {
        polarity: embassy_stm32::spi::Polarity::IdleLow,
        phase: embassy_stm32::spi::Phase::CaptureOnFirstTransition,
    };

    let spi_bus = Spi::new(
        p.SPI4, p.PE2, p.PE6, p.PE5, p.DMA2_CH4, p.DMA2_CH1, spi_config,
    );
    info!("SPI4 bus configured.");

    // Initialize the Mutex without the RefCell.
    let spi_bus_mutex = SPI_BUS.init(embassy_sync::mutex::Mutex::new(spi_bus));

    let baro_cs = Output::new(p.PE4, Level::High, Speed::VeryHigh);
    info!("Barometer CS pin configured.");

    // SpiDevice::new takes an immutable reference, which spi_bus_mutex can be coerced into.
    let baro_spi_device = SpiDevice::new(spi_bus_mutex, baro_cs);
    let baro = Ms5611::new(baro_spi_device, Delay).await.unwrap();

    let state_machine = StateMachine::new(traits::Context {});

    // --- Spawning Tasks ---
    spawner.must_spawn(led_blinker_task(p.PB14));
    spawner.must_spawn(uart_dma_reader_task(ring_rx));
    spawner.must_spawn(sbg_parser_task(tx));
    spawner.must_spawn(baro_reader_task(baro));

    loop {
        // state machine loop
        match state_machine.state {
            
        } 
    }
}