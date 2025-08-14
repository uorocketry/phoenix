#![feature(impl_trait_in_assoc_type)]
#![feature(ascii_char)]
#![no_std]
#![no_main]

mod ai;
mod camera;
mod communication;
mod madgwick_service;
mod model;
mod music;
mod recovery;
mod resources;
mod sd;
mod sensors;
mod state_machine;

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Input, Level, Output, OutputType, Pull, Speed};
use embassy_stm32::peripherals;
use embassy_stm32::spi::{Config as SpiConfig, Spi};
use embassy_stm32::time::{khz, mhz};
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};
use embassy_stm32::usart::{Config as UartConfig, Uart};
use embassy_time::{Delay, Duration, Timer};
use embedded_hal_1::delay::DelayNs;
use embedded_hal_1::digital::OutputPin;
// use embedded_alloc::Heap;
use crate::state_machine::StateMachine;
use defmt_rtt as _;
use panic_probe as _;
// Use a modern ms5611 driver that supports embedded-hal v1.0
use common_arm::drivers::ms5611::Ms5611;

// Use the asynchronous SpiDevice from embassy-embedded-hal

use crate::camera::Cameras;
use crate::communication::{radio_reader_task, radio_writer_task};
use crate::recovery::RecoveryManager;
use crate::resources::{Irqs, HEAP, RECOVERY_MANAGER, RX_SBG_BUF};

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

    // --- Clock configuration ---
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
    let p = embassy_stm32::init(config);

    info!("Heap usage: {} bytes", HEAP.used());

    // --- IMU Setup ---
    // let imu = sensors::imu::init_imu(p.SPI3, p.PC10, p.PB5, p.PB4, p.PC0, p.PB6, p.PD4);

    // --- SBG Setup ---
    let mut uart_config = UartConfig::default();
    uart_config.baudrate = 115200;
    let usart = Uart::new(
        p.UART4,
        p.PA1,
        p.PA0,
        Irqs,
        p.DMA1_CH1,
        p.DMA1_CH0,
        uart_config,
    )
    .unwrap();
    let (tx, rx) = usart.split();
    let ring_rx = rx.into_ring_buffered(unsafe { &mut RX_SBG_BUF });
    let sbg_pwr = Output::new(p.PD8, Level::High, Speed::Low);

    // // --- Baro SPI Setup ---
    let mut spi_config = SpiConfig::default();
    spi_config.frequency = mhz(16);
    let spi_bus = Spi::new_blocking(p.SPI4, p.PE2, p.PE6, p.PE5, spi_config);
    info!("SPI4 bus configured.");
    let baro_cs = Output::new(p.PB8, Level::High, Speed::Low);
    info!("Barometer CS pin configured.");
    let baro = Ms5611::new(spi_bus, baro_cs, Delay).unwrap();

    // --- SD Card ---
    let sd_card = sd::setup_sdmmc_interface(p.SPI1, p.PA5, p.PA7, p.PA6, p.PE9);

    // --- GPS Setup ---
    let (ring_gps_rx, gps_tx) = sensors::gps::setup_gps(
        p.PA4, p.PB2, p.UART8, p.PE0, p.PE1, p.DMA1_CH5, p.DMA1_CH6, Irqs,
    )
    .await;

    // --- Recovery manager ---
    let recovery_manager = RecoveryManager::new(
        p.PD6, p.PD14, p.PC11, p.PD2, p.PD5, p.PD13, p.PC12, p.PD1, p.PA2, p.PB0, p.PA3, p.PC5,
        p.ADC1, p.PC1,
    );

    RECOVERY_MANAGER.lock(|cell| {
        *cell.borrow_mut() = Some(recovery_manager);
    });

    // --- Camera Triggers ---
    let mut cameras = Cameras::new(p.PE14, p.PE12);

    // cameras.start_recording();
    // Delay.delay_ms(10_000);
    // cameras.stop_recording();
    // info!("Camera recording started and stopped.");

    // --- Buzzer 🐝 ---
    let buzz_out_pin = PwmPin::new_ch1(p.PC6, OutputType::PushPull);
    let mut pwm = SimplePwm::new(
        p.TIM3,
        Some(buzz_out_pin),
        None,
        None,
        None,
        khz(4),
        Default::default(),
    );
    let mut ch1 = pwm.ch1();
    info!("Duty Cycle: {}", ch1.max_duty_cycle());
    ch1.set_duty_cycle(ch1.max_duty_cycle() / 4);
    ch1.enable();
    music::play_song(&mut pwm, music::TWINKLE_MELODY, 130).await;

    // --- State Machine ---
    let state_machine = StateMachine::new(state_machine::Context {});

    // --- Radio ---
    let (radio_tx, radio_ring_rx) =
        communication::init_radio(p.UART7, p.PE7, p.PE8, p.DMA2_CH3, p.DMA2_CH5, Irqs);

    // --- Spawning Tasks ---
    spawner.must_spawn(sensors::sbg_manager::uart_dma_reader_task(ring_rx));
    // spawner.must_spawn(uart_gps_dma_reader_task(ring_gps_rx, gps_tx));
    spawner.must_spawn(sensors::sbg_manager::sbg_parser_task(tx));
    spawner.must_spawn(sensors::sbg_manager::sbg_receiver_task());
    spawner.must_spawn(sensors::baro::baro_reader_task(baro));
    // spawner.must_spawn(ai_task());
    // pass control of the spawner to the state machine
    // spawner.must_spawn(sm_task(spawner, state_machine));
    // spawner.must_spawn(radio_reader_task(radio_ring_rx));
    spawner.must_spawn(radio_writer_task(radio_tx));
    spawner.must_spawn(recovery::recovery_algorithm_task());
}
