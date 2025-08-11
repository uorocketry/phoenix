#![allow(dead_code)]

use core::cell::RefCell;
use defmt::*;
use embassy_stm32::{bind_interrupts, peripherals, usart};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::channel::Channel;
use embassy_sync::signal::Signal;
use embassy_time::Instant;
use static_cell::StaticCell;

use sbg_rs::sbg::SBG_BUFFER_SIZE;

// Global allocator
use embedded_alloc::LlffHeap as Heap;

#[global_allocator]
pub static HEAP: Heap = Heap::empty();

// Shared channels and resources
pub type DmaBuffer = [u8; SBG_BUFFER_SIZE];

pub const GPS_BUFFER_SIZE: usize = 26;
pub const RADIO_BUFFER_SIZE: usize = 255;

pub static PRESSURE_SIGNAL: Signal<CriticalSectionRawMutex, (f32, u8, Instant)> = Signal::new();

pub static SBG_CHANNEL: Channel<CriticalSectionRawMutex, messages_prost::sensor::sbg::SbgData, 10> =
    Channel::new();
pub static BUFFER_CHANNEL: Channel<CriticalSectionRawMutex, DmaBuffer, 10> = Channel::new();
// Note: EVENT_CHANNEL remains in main until the state machine is modularized
pub static COMMAND_CHANNEL: Channel<
    CriticalSectionRawMutex,
    messages_prost::command::command::Data,
    2,
> = Channel::new();
pub static RADIO_CHANNEL: Channel<CriticalSectionRawMutex, [u8; RADIO_BUFFER_SIZE], 10> =
    Channel::new();

// DMA receive buffers in AXI SRAM
#[link_section = ".axisram.buffers"]
pub static mut RX_SBG_BUF: [u8; SBG_BUFFER_SIZE] = [0; SBG_BUFFER_SIZE];
#[link_section = ".axisram.buffers"]
pub static mut RX_RADIO_BUF: [u8; SBG_BUFFER_SIZE] = [0; SBG_BUFFER_SIZE];
#[link_section = ".axisram.buffers"]
pub static mut RX_GPS_BUF: [u8; GPS_BUFFER_SIZE] = [0; GPS_BUFFER_SIZE];

// Optional shared SPI bus placeholder for future use (currently unused)
// pub static SPI_BUS: StaticCell<embassy_sync::mutex::Mutex<CriticalSectionRawMutex, embassy_stm32::spi::Spi<embassy_stm32::mode::Async>>>> =
//     StaticCell::new();

// RTC and Recovery manager shared handles
pub static RTC: Mutex<CriticalSectionRawMutex, RefCell<Option<embassy_stm32::rtc::Rtc>>> =
    Mutex::new(RefCell::new(None));

pub static RECOVERY_MANAGER: Mutex<
    CriticalSectionRawMutex,
    RefCell<Option<crate::drivers::recovery::RecoveryManager>>,
> = Mutex::new(RefCell::new(None));

// USART IRQ bindings used across modules
bind_interrupts!(pub struct Irqs {
    UART4 => usart::InterruptHandler<peripherals::UART4>;
    UART8 => usart::InterruptHandler<peripherals::UART8>;
    UART7 => usart::InterruptHandler<peripherals::UART7>;
});
