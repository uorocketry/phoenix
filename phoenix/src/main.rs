#![feature(impl_trait_in_assoc_type)]
#![no_std]
#![no_main]

mod madgwick_service;
mod sbg_manager;
mod traits; 

use core::cell::RefCell;
use core::marker::PhantomData;
use chrono::NaiveDate;
use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::mode::{Async, Blocking};
use embassy_stm32::rtc::{Rtc, RtcConfig};
use embassy_stm32::spi::{BitOrder, Config as SpiConfig, Spi};
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
use ublox::{CfgPrtUartBuilder, DataBits, InProtoMask, OutProtoMask, Parity, StopBits, UartMode, UartPortId, UbxPacketRequest};
use {panic_probe as _};
use embedded_sdmmc::{Mode, SdCard, SdCardError, TimeSource, VolumeIdx, VolumeManager};
use embedded_hal_bus::spi::RefCellDevice;
// Use a modern ms5611 driver that supports embedded-hal v1.0
use common_arm::drivers::ms5611::{Ms5611, OversamplingRatio};

// Use the asynchronous SpiDevice from embassy-embedded-hal
use embassy_embedded_hal::shared_bus::blocking::spi::SpiDevice;

use smlang::statemachine;

// =================================================================================
// Shared Resources & Types
// =================================================================================

type DmaBuffer = [u8; SBG_BUFFER_SIZE];

const GPS_BUFFER_SIZE: usize = 256;

#[global_allocator]
static HEAP: Heap = Heap::empty();

static SBG_CHANNEL: Channel<CriticalSectionRawMutex, SbgData, 10> = Channel::new();
static BUFFER_CHANNEL: Channel<CriticalSectionRawMutex, DmaBuffer, 2> = Channel::new();

// The SPI bus is protected by a Mutex, so the RefCell is not needed.
static SPI_BUS: StaticCell<embassy_sync::mutex::Mutex<CriticalSectionRawMutex, Spi<mode::Async>>> = StaticCell::new();

// Static variable for the RTC
pub static RTC: Mutex<CriticalSectionRawMutex, RefCell<Option<Rtc>>> =
    Mutex::new(RefCell::new(None));

bind_interrupts!(struct Irqs {
    UART7 => usart::InterruptHandler<peripherals::UART7>;
    UART8 => usart::InterruptHandler<peripherals::UART8>;
});

statemachine! {
    transitions: {
        *Init + Start = Idle,
        WaitForLaunch + Launch = Ascent,
        Fault + FaultCleared = Init,
        _ + FaultDetected = Fault,
    }
}

pub struct TimeSink {
    _marker: PhantomData<*const ()>,
}

impl TimeSink {
    fn new() -> Self {
        TimeSink {
            _marker: PhantomData,
        }
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
    info!("DMA reader task spawned.");
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
async fn uart_gps_dma_reader_task(mut gps_rx: usart::UartRx<'static,mode::Blocking> , mut gps_tx: UartTx<'static, mode::Blocking>) {
    info!("DMA reader task spawned.");
    loop {
        let mut buf_data: [u8; GPS_BUFFER_SIZE] = [0; GPS_BUFFER_SIZE];
        if let Ok(len) = gps_rx.blocking_read(&mut buf_data) {
                info!("read");
            // if len > 0 {
                // let _ = BUFFER_CHANNEL.try_send(buf);

                let request =
                    UbxPacketRequest::request_for::<ublox::NavPosLlh>().into_packet_bytes();
                gps_tx.blocking_write(&request).unwrap();
                cortex_m::asm::delay(10_000);
                let mut buf: [u8; 256] = [0; 256];
                let mut bytes: [u8; 256] = [0; 256];
                let buf = ublox::FixedLinearBuffer::new(&mut buf[..]);
                let mut parser = ublox::Parser::new(buf);
                let mut msgs = parser.consume(&buf_data);
                while let Some(msg) = msgs.next() {
                    match msg {
                        Ok(msg) => match msg {
                            ublox::PacketRef::NavPosLlh(x) => {
                                info!(
                                    "GPS latitude: {:?}, longitude {:?}",
                                    x.lat_degrees(),
                                    x.lon_degrees()
                                );
                                // let message = Message::new(
                                //     cortex_m::interrupt::free(|cs| {
                                //         let mut rc = RTC.borrow(cs).borrow_mut();
                                //         let rtc = rc.as_mut().unwrap();
                                //         rtc.count32()
                                //     }),
                                //     COM_ID,
                                //     messages::sensor::Sensor::new(message_data),
                                // );
                                // spawn!(send_internal, message).ok();
                            }
                            ublox::PacketRef::NavStatus(x) => {
                                info!("GPS fix stat: {:?}", x.fix_stat_raw());
                            }
                            ublox::PacketRef::NavDop(x) => {
                                info!("GPS geometric drop: {:?}", x.geometric_dop());
                            }
                            ublox::PacketRef::NavSat(x) => {
                                info!("GPS num sats used: {:?}", x.num_svs());
                            }
                            ublox::PacketRef::NavVelNed(x) => {
                                info!("GPS velocity north: {:?}", x.vel_north());
                            }
                            ublox::PacketRef::NavPvt(x) => {
                                info!("GPS nun sats PVT: {:?}", x.num_satellites());
                            }
                            _ => {
                                info!("GPS Message not handled.");
                            }
                        },
                        Err(e) => {
                            info!("GPS parse Error");
                        }
                    }
                }
            // }
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

#[embassy_executor::task] 
async fn baro_reader_task(mut baro: Ms5611<Spi<'static, Blocking>, Output<'static>, Delay>) {
    info!("Barometer reader task started.");
    loop {
        match baro.read_pressure_temperature(OversamplingRatio::Osr512) {
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
        const HEAP_SIZE: usize = 40000;
        static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
        unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE) }
    }

    let mut config = embassy_stm32::Config::default();

    {
        use embassy_stm32::rcc::*;
        config.rcc.hsi = Some(HSIPrescaler::DIV1);
        config.rcc.csi = true;
        config.rcc.pll1 = Some(Pll {
            source: PllSource::HSI,
            prediv: PllPreDiv::DIV4,
            mul: PllMul::MUL50,
            divp: Some(PllDiv::DIV2),
            divq: Some(PllDiv::DIV8), // used by SPI3. 100Mhz.
            divr: None,
        });
        config.rcc.sys = Sysclk::PLL1_P; // 400 Mhz
        config.rcc.ahb_pre = AHBPrescaler::DIV2; // 200 Mhz
        config.rcc.apb1_pre = APBPrescaler::DIV2; // 100 Mhz
        config.rcc.apb2_pre = APBPrescaler::DIV2; // 100 Mhz
        config.rcc.apb3_pre = APBPrescaler::DIV2; // 100 Mhz
        config.rcc.apb4_pre = APBPrescaler::DIV2; // 100 Mhz
        config.rcc.voltage_scale = VoltageScale::Scale1;
    }
    // config.rcc.ls = rcc::LsConfig::default_lse();
    let p = embassy_stm32::init(config);
    
    // --- SD Card Setup ---

    // --- GPS Setup --- 
    // let gps_uart_config = UartConfig::default();
    // let gps_uart = Uart::new(
    //     p.UART7, p.PF6, p.PF7, Irqs, p.DMA1_CH1, p.DMA1_CH0, gps_uart_config,
    // ).unwrap();
    // let (tx, rx) = gps_uart.split();
    // static mut RX_BUF: [u8; SBG_BUFFER_SIZE] = [0; SBG_BUFFER_SIZE];
    // let ring_rx = rx.into_ring_buffered(unsafe { &mut RX_BUF });

    // // --- RTC Setup ---
    // let now = NaiveDate::from_ymd_opt(2025, 6, 29)
    //     .unwrap()
    //     .and_hms_opt(15, 30, 0)
    //     .unwrap();
    // let mut rtc = Rtc::new(p.RTC, RtcConfig::default());
    // rtc.set_datetime(now.into()).expect("Failed to set RTC time");

    // RTC.lock(|cell| {
    //     *cell.borrow_mut() = Some(rtc);
    // });

    // --- SBG Setup ---
    let mut uart_config = UartConfig::default();
    uart_config.baudrate = 115200; 
    
    let usart = Uart::new(
        p.UART7, p.PF6, p.PF7, Irqs, p.DMA1_CH1, p.DMA1_CH0, uart_config,
    ).unwrap();
    let (tx, rx) = usart.split();
    static mut RX_SBG_BUF: [u8; SBG_BUFFER_SIZE] = [0; SBG_BUFFER_SIZE];
    let ring_rx = rx.into_ring_buffered(unsafe { &mut RX_SBG_BUF });
    let mut sbg_pwr = Output::new(p.PD8, Level::High, Speed::Low);

    // --- Baro SPI Setup ---
    let mut spi_config = SpiConfig::default();
    spi_config.frequency = mhz(16);
    spi_config.mode = embassy_stm32::spi::Mode {
        polarity: embassy_stm32::spi::Polarity::IdleLow,
        phase: embassy_stm32::spi::Phase::CaptureOnFirstTransition,
    };

    let spi_bus = Spi::new_blocking(
        p.SPI4, p.PE2, p.PE6, p.PE5, spi_config,
    );
    info!("SPI4 bus configured.");

    // Initialize the Mutex without the RefCell.
    // let spi_bus_mutex = SPI_BUS.init(embassy_sync::mutex::Mutex::new(spi_bus));

    let baro_cs = Output::new(p.PB8, Level::High, Speed::VeryHigh);
    info!("Barometer CS pin configured.");

    // SpiDevice::new takes an immutable reference, which spi_bus_mutex can be coerced into.
    // let baro_spi_device = SpiDevice::new(spi_bus_mutex, baro_cs);
    let baro = Ms5611::new(spi_bus, baro_cs, Delay).unwrap();


    // --- SD Card ---
    let mut sd_spi_config = SpiConfig::default();

    sd_spi_config.frequency = mhz(16);
    
    sd_spi_config.mode = embassy_stm32::spi::Mode {
        polarity: embassy_stm32::spi::Polarity::IdleLow,
        phase: embassy_stm32::spi::Phase::CaptureOnFirstTransition,
    };

    sd_spi_config.bit_order = BitOrder::MsbFirst;

    let sd_spi_bus = Spi::new(
        p.SPI1, p.PA5, p.PA7, p.PA6, p.DMA1_CH4, p.DMA1_CH5, sd_spi_config,
    );

    let sd_cs = Output::new(p.PB9, Level::High, Speed::VeryHigh);

    let sd_spi_bus_ref_cell = RefCell::new(sd_spi_bus);
    let sd_spi_device = RefCellDevice::new(&sd_spi_bus_ref_cell, sd_cs, Delay);

    let sdcard = SdCard::new(sd_spi_device.unwrap(), Delay);
    println!("Card size is {} bytes", sdcard.num_bytes().unwrap());
    let volume_mgr = VolumeManager::new(sdcard, TimeSink::new());
    let volume0 = volume_mgr.open_volume(VolumeIdx(0)).unwrap();
    let root_dir = volume0.open_root_dir().unwrap();
    let mut my_file = root_dir.open_file_in_dir("MY_FILE.TXT", Mode::ReadOnly).unwrap();
    while !my_file.is_eof() {
        let mut buffer = [0u8; 32];
        let num_read = my_file.read(&mut buffer).unwrap();
        for b in &buffer[0..num_read] {
            info!("{}", *b as char);
        }
    }
    info!("Sd write and setup complete");

    // --- GPS Setup ---
    let mut gps_enable = Output::new(p.PA4, Level::Low, Speed::Low); 
    let mut gps_reset = Output::new(p.PB2, Level::Low, Speed::Low); 
    let mut uart_gps_config = UartConfig::default();
    uart_gps_config.baudrate = 9600; 
    let uart_gps = Uart::new_blocking(
        p.UART8,  p.PE0, p.PE1, uart_gps_config
    ).unwrap();

    let (mut gps_tx, mut gps_rx) = uart_gps.split();
    // static mut RX_GPS_BUF: [u8; GPS_BUFFER_SIZE] = [0; GPS_BUFFER_SIZE];
    // let ring_gps_rx = gps_rx.into_ring_buffered(unsafe { &mut RX_GPS_BUF });

    gps_reset.set_low();
    cortex_m::asm::delay(300_000);
    gps_reset.set_high();
    gps_enable.set_low();

    gps_enable.set_low();
    let packet: [u8; 28] = CfgPrtUartBuilder {
        portid: UartPortId::Uart1,
        reserved0: 0,
        tx_ready: 0,
        mode: UartMode::new(DataBits::Eight, Parity::None, StopBits::One),
        baud_rate: 9600,
        in_proto_mask: InProtoMask::all(),
        out_proto_mask: OutProtoMask::UBLOX,
        flags: 0,
        reserved5: 0,
    }
    .into_packet_bytes();

    gps_tx.blocking_write(&packet).unwrap();


    // --- State Machine ---
    let state_machine = StateMachine::new(traits::Context {});



    // --- Spawning Tasks ---
    spawner.must_spawn(led_blinker_task(p.PB14));
    spawner.must_spawn(uart_dma_reader_task(ring_rx));
    spawner.must_spawn(uart_gps_dma_reader_task(gps_rx, gps_tx));
    spawner.must_spawn(sbg_parser_task(tx));
    // spawner.must_spawn(baro_reader_task(baro));

    // loop {
    //     // state machine loop
    //     match state_machine.state {
    //         States::Ascent => {

    //         },
    //         States::Fault => {

    //         },
    //         States::Idle => {

    //         },
    //         States::Init => {
                
    //         },
    //         States::WaitForLaunch => {
                
    //         },
    //     } 
    // }
}