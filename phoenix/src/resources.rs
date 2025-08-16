// =================================================================================
// Shared Resources & Types
// =================================================================================

use crate::recovery;
use crate::state_machine::Events;
use burn::backend::NdArray;
use core::cell::RefCell;
use embassy_stm32::rtc::Rtc;
use embassy_stm32::spi::Spi;
use embassy_stm32::{bind_interrupts, mode, peripherals, usart};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::channel::Channel;
use embassy_sync::signal::Signal;
use embassy_time::Instant;
use embedded_alloc::LlffHeap as Heap;
use messages_prost::sbg::SbgData;
use sbg_rs::sbg::SBG_BUFFER_SIZE;
use static_cell::StaticCell;

pub type Backend = NdArray<f32>;
pub type BackendDevice = <Backend as burn::tensor::backend::Backend>::Device;

type DmaBuffer = [u8; SBG_BUFFER_SIZE];

pub const GPS_BUFFER_SIZE: usize = 128;

pub const RADIO_BUFFER_SIZE: usize = 255;

pub const SD_BUFFER_SIZE: usize = 255;

#[global_allocator]
pub static HEAP: Heap = Heap::empty();

pub static PRESSURE_CHANNEL: Channel<CriticalSectionRawMutex, (f32, f32, u8, Instant), 10> =
    Channel::new();
pub static SD_CHANNEL: Channel<CriticalSectionRawMutex, (&str, [u8; SD_BUFFER_SIZE]), 5> = Channel::new(); // file name, data
pub static SBG_CHANNEL: Channel<CriticalSectionRawMutex, SbgData, 10> = Channel::new();
pub static BUFFER_CHANNEL: Channel<CriticalSectionRawMutex, DmaBuffer, 10> = Channel::new();
pub static EVENT_CHANNEL: Channel<CriticalSectionRawMutex, Events, 2> = Channel::new();

pub static COMMAND_CHANNEL: Channel<
    CriticalSectionRawMutex,
    messages_prost::command::command::Data,
    2,
> = Channel::new();
pub static RADIO_CHANNEL: Channel<CriticalSectionRawMutex, [u8; RADIO_BUFFER_SIZE], 10> =
    Channel::new();
#[link_section = ".axisram.buffers"]
pub static mut RX_SBG_BUF: [u8; SBG_BUFFER_SIZE] = [0; SBG_BUFFER_SIZE];

#[link_section = ".axisram.buffers"]
pub static mut RX_RADIO_BUF: [u8; SBG_BUFFER_SIZE] = [0; SBG_BUFFER_SIZE];

#[link_section = ".axisram.buffers"]
pub static mut RX_GPS_BUF: [u8; GPS_BUFFER_SIZE] = [0; GPS_BUFFER_SIZE];
// The SPI bus is protected by a Mutex, so the RefCell is not needed.
pub static SPI_BUS: StaticCell<
    embassy_sync::mutex::Mutex<CriticalSectionRawMutex, Spi<mode::Async>>,
> = StaticCell::new();
pub static SPI_BUS_CELL: StaticCell<RefCell<Spi<mode::Blocking>>> = StaticCell::new();

// Static variable for the RTC
pub static RTC: Mutex<CriticalSectionRawMutex, RefCell<Option<Rtc>>> =
    Mutex::new(RefCell::new(None));

pub static RECOVERY_MANAGER: Mutex<
    CriticalSectionRawMutex,
    RefCell<Option<recovery::RecoveryManager>>,
> = Mutex::new(RefCell::new(None));

bind_interrupts!(pub struct Irqs {
    UART4 => usart::InterruptHandler<peripherals::UART4>;
    UART8 => usart::InterruptHandler<peripherals::UART8>;
    UART7 => usart::InterruptHandler<peripherals::UART7>;
});
