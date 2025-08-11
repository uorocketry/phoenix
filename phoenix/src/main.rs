#![feature(impl_trait_in_assoc_type)]
#![feature(ascii_char)]
#![no_std]
#![no_main]

mod board;
mod comm;
mod drivers;
mod features;
mod imu;
mod inference;
mod madgwick_service;
mod model;
mod music;
mod resources;
mod sbg_manager;
mod setup;
mod tasks;
mod traits;

use burn::{
    backend::NdArray,
    module::Module, // <-- FIX: Trait needed for .load_record()
    prelude::*,
    record::{BinBytesRecorder, FullPrecisionSettings, Recorder}, // <-- FIX: Use BinBytesRecorder
};
use core::cell::RefCell;
use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Input, Level, Output, OutputType, Pull, Speed};
use embassy_stm32::mode::{Async, Blocking};
use embassy_stm32::usart::{RingBufferedUartRx, UartTx};
use embassy_stm32::{bind_interrupts, mode};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::channel::Channel;
use embassy_sync::signal::Signal;
use embassy_time::{Delay, Duration, Instant, Timer};
use embedded_alloc::LlffHeap as Heap;
use embedded_hal_1::delay::DelayNs;
use embedded_hal_1::digital::{OutputPin, PinState};
use libm::powf;
use messages_prost::mavlink;
use messages_prost::mavlink::peek_reader::PeekReader;
use messages_prost::mavlink::uorocketry::MavMessage;
use messages_prost::prost::Message;
use messages_prost::radio::radio_frame::Payload;
use messages_prost::sensor::{gps, sbg};
use ublox::{cfg_val::CfgVal::*, CfgLayerSet};
// use embedded_alloc::Heap;
use crate::traits::Context;
use defmt_rtt as _;
use embedded_hal_bus::spi::RefCellDevice;
use embedded_io_async::Read;
use heapless::{HistoryBuffer, Vec};
use messages_prost::sensor::sbg::{SbgData, SbgMessage};
use nmea::SentenceType;
use panic_probe as _;
use sbg_rs::sbg::SBG_BUFFER_SIZE;
use static_cell::StaticCell;
use ublox::{
    CfgPrtUartBuilder, CfgRstBuilder, CfgValSetBuilder, DataBits, InProtoMask, NavBbrMask,
    OutProtoMask, Parity, ResetMode, StopBits, UartMode, UartPortId, UbxPacketRequest,
};
// Use a modern ms5611 driver that supports embedded-hal v1.0
use common_arm::drivers::ms5611::{Ms5611, OversamplingRatio};

// Use the asynchronous SpiDevice from embassy-embedded-hal

use smlang::statemachine;
use ublox::cfg_val::CfgVal;

use crate::drivers::recovery::{Arming, Fire, RecoveryManager};
use crate::resources::HEAP;

// =================================================================================
// Main Entry Point
// =================================================================================

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("System starting...");
    {
        use core::mem::MaybeUninit;
        const HEAP_SIZE: usize = 40_000;
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
            divq: Some(PllDiv::DIV8),
            divr: None,
        });
        config.rcc.pll2 = Some(Pll {
            source: PllSource::HSI,
            prediv: PllPreDiv::DIV4,
            mul: PllMul::MUL50,
            divp: Some(PllDiv::DIV2),
            divq: Some(PllDiv::DIV8),
            divr: None,
        });
        config.rcc.pll3 = Some(Pll {
            source: PllSource::HSI,
            prediv: PllPreDiv::DIV4,
            mul: PllMul::MUL50,
            divp: Some(PllDiv::DIV2),
            divq: Some(PllDiv::DIV8),
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
    let mut board = board::Board::new(config);

    info!("Heap usage: {} bytes", HEAP.used());

    // --- IMU Setup ---
    board.setup_imu();
    // let mut imu = imu::Iim20670::new(imu_spi, imu_cs, Some(imu_nreset), Delay).unwrap();

    // loop {
    //     Timer::after(Duration::from_millis(100)).await;
    //     let data = imu.read_all_converted();
    //     match data {
    //         Ok((accel, gyro)) => {
    //             info!("Accel: x: {}, y: {}, z: {}", accel.x, accel.y, accel.z);
    //             info!("Gyro: x: {}, y: {}, z: {}", gyro.x, gyro.y, gyro.z);
    //         }
    //         Err(e) => {
    //         }
    //     }
    // }

    // --- SBG Setup ---
    let (tx, ring_rx) = board.setup_sbg_uart();

    // --- Baro SPI Setup ---
    let baro = Some(board.setup_baro());

    // --- SD Card ---
    if features::ENABLE_SD {
        let (sd_spi_bus, sd_cs) = board.setup_sd_spi();
        let _ = drivers::sd::init_and_demo(sd_spi_bus, sd_cs);
    }

    // --- GPS Setup ---
    let (mut gps_tx, mut ring_gps_rx) = {
        let (tx, rx) = board.setup_gps().await;
        (Some(tx), Some(rx))
    };

    Delay.delay_ms(2000);

    let config_packet = CfgValSetBuilder {
        version: 1,
        // Save to RAM (to apply now) and BBR (to make it persistent)
        layers: CfgLayerSet::BBR | CfgLayerSet::RAM,
        reserved1: 0,
        cfg_data: &[
            // --- Port Settings ---
            CfgVal::Uart1Baudrate(38400),
            CfgVal::Uart1InProtUbx(true),
            CfgVal::Uart1InProtNmea(false), // Explicitly disable NMEA
            CfgVal::Uart1InProtRtcm3x(false),
            CfgVal::Uart1OutProtUbx(true),
            CfgVal::Uart1OutProtNmea(false), // Explicitly disable NMEA
            // --- Message Settings ---
            CfgVal::MsgOutUbxNavPvtUart1(1), // Enable NAV-PVT on UART1
            // --- Rate Settings ---
            CfgVal::RateMeas(200), // 200ms = 5Hz
            CfgVal::RateNav(1),    // Navigation rate = Measurement rate
        ],
    }
    .into_packet_vec();

    if let Some(ref mut tx) = gps_tx {
        comm::gps::configure_ublox(tx).await;
    }

    // // --- Boom Boom Setup ---
    // /*
    //     MAIN_ARM/TEST = PD6
    //     MAIN_FIRE = PD5
    //     MAIN_ARM/TEST_B = PD14
    //     MAIN_FIRE_B = PD13
    //     DROGUE_ARM/TEST = PC11
    //     DROGUE_FIRE = PC12
    //     DROGUE_ARM/TEST_B = PD2
    //     DROGUE_FIRE_B = PD1
    //     MAIN_MCU_EMATCH_SENSE = PA2
    //     MAIN_MCU_EMATCH_SENSE_B = PB0
    //     DROUGE_MCU_EMATCH_SENSE = PA3
    //     DROGUE_MCU_EMATCH_SENSE_B = PC5
    //  */
    let recovery_manager = board.setup_recovery_manager();

    crate::resources::RECOVERY_MANAGER.lock(|cell| {
        *cell.borrow_mut() = Some(recovery_manager);
    });

    // --- Camera Triggers ---
    let (cam_trigger, cam_trigger_b) = board.setup_camera_triggers();
    // moved sequence to drivers::camera to keep main lean (behavior unchanged)
    drivers::camera::run_boot_and_record(cam_trigger, cam_trigger_b).await;

    // --- Buzzer 🐝 ---
    let mut pwm = board.setup_buzzer();
    let mut ch1 = pwm.ch1();
    info!("Duty Cycle: {}", ch1.max_duty_cycle());
    ch1.set_duty_cycle(ch1.max_duty_cycle() / 4);
    ch1.enable();
    music::play_song(&mut pwm, music::TWINKLE_MELODY, 130).await;

    // --- State Machine ---
    if features::ENABLE_STATE_MACHINE {
        let state_machine = tasks::state_machine::StateMachine::new(traits::Context {});
        spawner.must_spawn(tasks::state_machine::sm_task(spawner, state_machine));
    }

    // --- Radio ---
    let (mut radio_tx, mut radio_ring_rx) = board.setup_radio();

    // --- Inference ---
    if features::ENABLE_INFERENCE {
        spawner.must_spawn(inference::inference_task());
    }

    // --- Spawning Tasks ---
    if features::ENABLE_LED {
        spawner.must_spawn(tasks::led::led_blinker_task(board.take_led_pin()));
    }
    if let (true, Some(rx), Some(tx)) = (features::ENABLE_GPS, ring_gps_rx.take(), gps_tx.take()) {
        spawner.must_spawn(tasks::gps::uart_gps_dma_reader_task(rx, tx));
    }
    if features::ENABLE_SBG {
        spawner.must_spawn(comm::sbg::sbg_uart_reader_task(ring_rx));
        spawner.must_spawn(comm::sbg::sbg_parser_task(tx));
        spawner.must_spawn(comm::sbg::sbg_receiver_task());
    }
    if let (true, Some(baro_dev)) = (features::ENABLE_BARO, baro) {
        spawner.must_spawn(tasks::baro::baro_reader_task(baro_dev));
    }
    if features::ENABLE_RADIO {
        spawner.must_spawn(comm::radio::radio_reader_task(radio_ring_rx));
        spawner.must_spawn(comm::radio::radio_writer_task(radio_tx));
    }
}