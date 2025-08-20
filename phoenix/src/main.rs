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

use core::cell::RefCell;

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Input, Level, Output, OutputType, Pull, Speed};
use embassy_stm32::spi::{Config as SpiConfig, Spi};
use embassy_stm32::time::{khz, mhz};
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};
use embassy_stm32::usart::{Config as UartConfig, Uart};
use embassy_stm32::wdg::IndependentWatchdog;
use embassy_stm32::{mode, peripherals};
use embassy_time::{Delay, Duration, Instant, Timer};
use embedded_hal_1::delay::DelayNs;
use embedded_hal_bus::spi::RefCellDevice;
use messages_prost::phoenix_state::Event;
use messages_prost::prost::Message;
use static_cell::StaticCell;
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
use crate::resources::{
    Irqs, EVENT_CHANNEL, HEAP, RADIO_CHANNEL, RECOVERY_MANAGER, RX_SBG_BUF, SD_CHANNEL,
};

pub static IMU_BUS_CELL: StaticCell<RefCell<Spi<mode::Blocking>>> = StaticCell::new();

#[embassy_executor::task]
async fn imu_task(
    mut imu: sensors::iim20670::Iim20670<
        RefCellDevice<'static, Spi<'static, mode::Blocking>, Output<'static>, Delay>,
        Delay,
    >,
) {
    if imu.init().is_ok() {
        loop {
            if let Ok(accel) = imu.read_accel() {
                info!("Accel: x={}, y={}, z={}", accel[0], accel[1], accel[2]);
            }
            if let Ok(gyro) = imu.read_gyro() {
                info!("Gyro: x={}, y={}, z={}", gyro[0], gyro[1], gyro[2]);
            }
            Timer::after(Duration::from_millis(100)).await;
        }
    } else {
        warn!("IMU initialization failed.");
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
    // embassy_stm32::rcc::enable_and_reset::<peripherals::SPI3>();
    // let mut spi_config = SpiConfig::default();
    // spi_config.frequency = mhz(1); // Max 10 MHz for IIM-20670
    // let mut imu_spi = Spi::new_blocking(
    //     p.SPI3, p.PB3, // SCK
    //     p.PB5, // MOSI
    //     p.PB4, // MISO
    //     spi_config,
    // );
    // let imu_odr = Input::new(p.PC0, Pull::None);
    // let imu_cs = Output::new(p.PA15, Level::Low, Speed::Low);
    // let imu_nreset = Output::new(p.PD4, Level::High, Speed::Low);
    // let imu_bus_ref = IMU_BUS_CELL.init(RefCell::new(imu_spi));
    // let imu_spi_device = RefCellDevice::new(imu_bus_ref, imu_cs, Delay).unwrap();
    // let mut imu = sensors::iim20670::Iim20670::new(imu_spi_device, Delay);

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
    let sd_card: embedded_sdmmc::SdCard<
        embedded_hal_bus::spi::RefCellDevice<
            'static,
            Spi<'static, embassy_stm32::mode::Blocking>,
            Output<'static>,
            Delay,
        >,
        Delay,
    > = sd::setup_sdmmc_interface(p.SPI1, p.PA5, p.PA7, p.PA6, p.PE9);

    // --- GPS Setup ---
    let (ring_gps_rx, gps_tx) = sensors::gps::setup_gps(
        p.PA4, p.PB2, p.UART8, p.PE0, p.PE1, p.DMA1_CH5, p.DMA1_CH6, Irqs,
    )
    .await;

    // --- Recovery manager ---
    let mut recovery_manager = RecoveryManager::new(
        p.PD6, p.PD14, p.PC11, p.PD2, p.PD5, p.PD13, p.PC12, p.PD1, p.PA2, p.PB0, p.PA3, p.PC5,
        p.ADC1, p.PC1,
    );

    RECOVERY_MANAGER.lock(|cell| {
        *cell.borrow_mut() = Some(recovery_manager);
    });

    // --- Camera Triggers ---
    // let mut cameras = Cameras::new(p.PE14, p.PE12);

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

    // --- State Machine ---
    let state_machine = StateMachine::new(state_machine::Context {});

    // --- Radio ---
    let (radio_tx, radio_ring_rx) =
        communication::init_radio(p.UART7, p.PE7, p.PE8, p.DMA2_CH3, p.DMA2_CH5, Irqs);

    // --- Spawning Tasks ---
    spawner.must_spawn(sensors::sbg_manager::uart_dma_reader_task(ring_rx));
    // spawner.must_spawn(sensors::gps::uart_gps_dma_reader_task(ring_gps_rx, gps_tx));
    spawner.must_spawn(sensors::sbg_manager::sbg_parser_task(tx));
    spawner.must_spawn(sensors::sbg_manager::sbg_receiver_task());
    spawner.must_spawn(sensors::baro::baro_reader_task(baro));
    // // spawner.must_spawn(ai::ai_task());
    // spawner.must_spawn(radio_reader_task(radio_ring_rx));
    spawner.must_spawn(radio_writer_task(radio_tx));
    spawner.must_spawn(sd::sdmmc_task(sd_card));
    // spawner.must_spawn(imu_task(imu));

    // pass control of the spawner to the state machine
    spawner.must_spawn(state_machine::sm_task(spawner, state_machine));

    // // watch dog with 10 second timeout
    // let mut watch_dog = IndependentWatchdog::new(p.IWDG1, 10_000);
    // watch_dog.unleash();

    info!("Device {} has started.", embassy_stm32::uid::uid_hex());
    music::play_song(&mut pwm, music::TWINKLE_MELODY, 1300).await;
}
