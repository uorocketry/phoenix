#![feature(impl_trait_in_assoc_type)]
#![feature(ascii_char)]
#![no_std]
#![no_main]

mod madgwick_service;
mod sbg_manager;
mod traits;
mod music;
mod model;
mod imu;

use messages_prost::radio::radio_frame::Payload;
use ublox::{cfg_val::CfgVal::*, CfgLayer};
use messages_prost::prost::Message;
use embedded_hal_1::delay::DelayNs;
use embedded_hal_1::digital::{OutputPin, PinState};
use libm::powf;
use messages_prost::mavlink;
use messages_prost::mavlink::peek_reader::PeekReader;
use messages_prost::mavlink::uorocketry::MavMessage;
use messages_prost::sensor::{gps, sbg};
use core::cell::RefCell;
use core::marker::PhantomData;
use defmt::*;
use embedded_alloc::LlffHeap as Heap;
use burn::{
    backend::NdArray,
    module::Module, // <-- FIX: Trait needed for .load_record()
    prelude::*,
    record::{BinBytesRecorder, Recorder, FullPrecisionSettings}, // <-- FIX: Use BinBytesRecorder
};
use embassy_executor::Spawner;
use embassy_stm32::adc::Adc;
use embassy_stm32::gpio::{Input, Level, Output, OutputType, Pull, Speed};
use embassy_stm32::mode::{Async, Blocking};
use embassy_stm32::rtc::Rtc;
use embassy_stm32::spi::{BitOrder, Config as SpiConfig, Spi};
use embassy_stm32::time::{khz, mhz};
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};
use embassy_stm32::usart::{Config as UartConfig, RingBufferedUartRx, Uart, UartRx, UartTx};
use embassy_stm32::{bind_interrupts, mode, peripherals, rcc, usart};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::channel::Channel;
use embassy_sync::signal::Signal;
use embassy_time::{Delay, Duration, Instant, Timer};
// use embedded_alloc::Heap;
use heapless::{HistoryBuffer, Vec};
use messages_prost::sensor::sbg::{SbgData, SbgMessage};
use sbg_rs::sbg::SBG_BUFFER_SIZE;
use static_cell::StaticCell;
use ublox::{CfgPrtUartBuilder, CfgValSetBuilder, DataBits, InProtoMask, OutProtoMask, Parity, StopBits, UartMode, UartPortId, UbxPacketRequest};
use crate::traits::Context;
use {defmt_rtt as _};
use {panic_probe as _};
use embedded_sdmmc::{Mode, SdCard, VolumeIdx, VolumeManager};
use embedded_hal_bus::spi::RefCellDevice;
use embedded_io_async::Read;
use nmea::SentenceType;
// Use a modern ms5611 driver that supports embedded-hal v1.0
use common_arm::drivers::ms5611::{Ms5611, OversamplingRatio};

// Use the asynchronous SpiDevice from embassy-embedded-hal

use smlang::statemachine;
use ublox::cfg_val::CfgVal;

struct Arming {
    main: Output<'static>,
    drogue: Output<'static>,
    main_b: Output<'static>,
    drogue_b: Output<'static>,
}

struct Fire {
    main: Output<'static>,
    drogue: Output<'static>,
    main_b: Output<'static>,
    drogue_b: Output<'static>,
}

struct RecoveryManager {
    arming: Arming,
    fire: Fire, 
}


// use crate::model::sine::Model;
// =================================================================================
// Shared Resources & Types
// =================================================================================

type Backend = NdArray<f32>;
type BackendDevice = <Backend as burn::tensor::backend::Backend>::Device;

type DmaBuffer = [u8; SBG_BUFFER_SIZE];

const GPS_BUFFER_SIZE: usize = 26;

const RADIO_BUFFER_SIZE: usize = 255;


#[global_allocator]
static HEAP: Heap = Heap::empty();

static PRESSURE_SIGNAL: Signal<CriticalSectionRawMutex, (f32, u8, Instant)> = Signal::new();

static SBG_CHANNEL: Channel<CriticalSectionRawMutex, SbgData, 10> = Channel::new();
static BUFFER_CHANNEL: Channel<CriticalSectionRawMutex, DmaBuffer, 10> = Channel::new();
static EVENT_CHANNEL: Channel<CriticalSectionRawMutex, Events, 2> = Channel::new();

static COMMAND_CHANNEL: Channel<CriticalSectionRawMutex, messages_prost::command::command::Data, 2> = Channel::new();
// static FAULT_CHANNEL: Channel<CriticalSectionRawMutex, , 2> = Channel::new();
static RADIO_CHANNEL: Channel<CriticalSectionRawMutex, [u8; 255], 10> = Channel::new();
#[link_section = ".axisram.buffers"]
static mut RX_SBG_BUF: [u8; SBG_BUFFER_SIZE] = [0; SBG_BUFFER_SIZE];

#[link_section = ".axisram.buffers"]
static mut RX_RADIO_BUF: [u8; SBG_BUFFER_SIZE] = [0; SBG_BUFFER_SIZE];

#[link_section = ".axisram.buffers"]
static mut RX_GPS_BUF: [u8; GPS_BUFFER_SIZE] = [0; GPS_BUFFER_SIZE];
// The SPI bus is protected by a Mutex, so the RefCell is not needed.
static SPI_BUS: StaticCell<embassy_sync::mutex::Mutex<CriticalSectionRawMutex, Spi<mode::Async>>> = StaticCell::new();

// Static variable for the RTC
pub static RTC: Mutex<CriticalSectionRawMutex, RefCell<Option<Rtc>>> =
    Mutex::new(RefCell::new(None));

pub static RECOVERY_MANAGER: Mutex<CriticalSectionRawMutex, RefCell<Option<RecoveryManager>>> =
    Mutex::new(RefCell::new(None));

bind_interrupts!(struct Irqs {
    UART4 => usart::InterruptHandler<peripherals::UART4>;
    UART8 => usart::InterruptHandler<peripherals::UART8>;
    UART7 => usart::InterruptHandler<peripherals::UART7>;
});

statemachine! {
    transitions: {
        *Init + Start = WaitForLaunch,
        WaitForLaunch + Launch = Ascent,
        Ascent + Apogee = Descent,
        Descent + MainDeployment = Fuck,
        Descent + DrogueDeployment = DrogueDescent,
        DrogueDescent + MainDeployment = MainDescent,
        MainDescent + NoMovement = Landed,
        Fault + FaultCleared = _,
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
type AiBackend = NdArray<f32>;
type AiDevice = <AiBackend as burn::tensor::backend::Backend>::Device;

const SEQ_LENGTH: usize = 50;
const NUM_FEATURES: usize = 12;

// --- PASTE CONSTANTS FROM PYTHON SCRIPT HERE ---
// Replace these dummy values with the actual output from clean_rocket_data.py
const SCALE_MIN: [f32; NUM_FEATURES] = [0.0f32; 12];
const SCALE_MAX: [f32; NUM_FEATURES] = [1.0f32; 12];
// ------------------------------------------------

/// Normalizes a single feature value using the pre-calculated min/max.
fn normalize_value(value: f32, min: f32, max: f32) -> f32 {
    if (max - min) == 0.0 {
        return 0.0; // Avoid division by zero
    }
    (value - min) / (max - min)
}

#[embassy_executor::task]
async fn ai_task() {
    info!("AI Inference Task starting...");

    let device = AiDevice::default();

    // 1. Create the model structure
    info!("Initializing model structure...");
    let model: model::LstmNetwork<AiBackend> = model::LstmNetwork::new(&device);

    // 2. Load the trained weights from the embedded file
    info!("Loading trained weights...");
    let recorder = burn::record::NoStdInferenceRecorder::new();
    let record_bytes = include_bytes!("models/model.bin");
    info!("Loaded {} bytes of model weights.", record_bytes.len());
    let record = recorder
        .load(record_bytes.to_vec(), &device)
        .expect("Failed to load model weights");
    let model = model.load_record(record);
    info!("Model loaded successfully.");

    let mut sensor_history: HistoryBuffer<[f32; NUM_FEATURES], SEQ_LENGTH> = HistoryBuffer::new();

    loop {
        let latest_sensor_data: [f32; NUM_FEATURES] = [0.0; 12]; // Dummy data

        let mut normalized_data = [0.0f32; NUM_FEATURES];
        for i in 0..NUM_FEATURES {
            normalized_data[i] = normalize_value(latest_sensor_data[i], SCALE_MIN[i], SCALE_MAX[i]);
        }
        sensor_history.write(normalized_data);

        if sensor_history.len() == sensor_history.capacity() {
            let mut flat_history: Vec<f32, { SEQ_LENGTH * NUM_FEATURES }> = Vec::new();
            for frame in sensor_history.iter() {
                for value in frame.iter() {
                    flat_history.push(*value).ok();
                }
            }

            let input = Tensor::<AiBackend, 3>::from_floats(flat_history.as_slice(), &device)
                .reshape([1, SEQ_LENGTH, NUM_FEATURES]);

            let output_log = model.forward(input);
            let output_sec = (output_log.exp() - 1.0).into_data();
            let predictions = output_sec.as_slice::<f32>().unwrap();

            info!("PREDICTIONS -> Burnout: {=f32}s, Apogee: {=f32}s, Impact: {=f32}s",
                predictions[0], predictions[1], predictions[2]
            );
        }

        Timer::after(Duration::from_millis(100)).await; // Run at 10Hz
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
                let _ = BUFFER_CHANNEL.send(buf).await;
            }
        }
        Delay.delay_ms(100);
    }
}

#[embassy_executor::task]
async fn uart_gps_dma_reader_task(mut gps_rx: RingBufferedUartRx<'static> , mut gps_tx: UartTx<'static, mode::Async>) {
    info!("DMA reader task spawned.");
    let request =
        UbxPacketRequest::request_for::<ublox::NavPosLlh>().into_packet_bytes();
    // loop {
    //     gps_tx.write(&request).await;
    //     Delay.delay_ms(1000); // Wait for GPS data to be ready    
    // }
    // gps_tx.blocking_write(&request);

    // loop {
    //     let mut buf_data: [u8; GPS_BUFFER_SIZE] = [0; GPS_BUFFER_SIZE];
    //     if let Ok(len) = gps_rx.read(&mut buf_data).await {
    //             info!("read");
    //             let request =
    //                 UbxPacketRequest::request_for::<ublox::NavPosLlh>().into_packet_bytes();
    //             gps_tx.blocking_write(&request);
    //             cortex_m::asm::delay(10_000);
    //             let mut buf: [u8; 256] = [0; 256];
    //             let bytes: [u8; 256] = [0; 256];
    //             let buf = ublox::FixedLinearBuffer::new(&mut buf[..]);
    //             let mut parser = ublox::Parser::new(buf);
    //             let mut msgs = parser.consume(&buf_data[..len]);
    //             while let Some(msg) = msgs.next() {
    //                 match msg {
    //                     Ok(msg) => match msg {
    //                         ublox::PacketRef::NavPosLlh(x) => {
    //                             info!(
    //                                 "GPS latitude: {:?}, longitude {:?}",
    //                                 x.lat_degrees(),
    //                                 x.lon_degrees()
    //                             );
    //                         }
    //                         ublox::PacketRef::NavStatus(x) => {
    //                             info!("GPS fix stat: {:?}", x.fix_stat_raw());
    //                         }
    //                         ublox::PacketRef::NavDop(x) => {
    //                             info!("GPS geometric drop: {:?}", x.geometric_dop());
    //                         }
    //                         ublox::PacketRef::NavSat(x) => {
    //                             info!("GPS num sats used: {:?}", x.num_svs());
    //                         }
    //                         ublox::PacketRef::NavVelNed(x) => {
    //                             info!("GPS velocity north: {:?}", x.vel_north());
    //                         }
    //                         ublox::PacketRef::NavPvt(x) => {
    //                             info!("GPS nun sats PVT: {:?}", x.num_satellites());
    //                         }
    //                         _ => {
    //                             info!("GPS Message not handled.");
    //                         }
    //                     },
    //                     Err(e) => {
    //                         info!("GPS parse Error");
    //                     }
    //                 }
    //             }
    //         // }
    //     }
    // }
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
async fn sbg_receiver_task() {
    loop {
        let data = SBG_CHANNEL.receive().await;
        match data.data {
            Some(x) => {
                let mut buf: [u8; 255] = [0; 255];
                let msg = messages_prost::radio::RadioFrame {
                    node: messages_prost::common::Node::Phoenix.into(), 
                    payload: Some(messages_prost::radio::radio_frame::Payload::Sbg(data))
                };
                msg.encode_length_delimited(&mut buf.as_mut())
                    .expect("Failed to encode SBG GPS Position");
                RADIO_CHANNEL.send(buf).await;

                // match x {
                    // messages_prost::sensor::sbg::sbg_data::Data:: => {

                    // }
                    // messages_prost::sensor::sbg::sbg_data::Data::GpsPos(gps_pos) => {
                    //     let msg: SbgMessage = SbgMessage {
                    //         node: 0,
                    //         data: Some(data),
                    //     };
                    //     msg.encode_length_delimited(&mut buf.as_mut())
                    //         .expect("Failed to encode SBG GPS Position");
                    //     RADIO_CHANNEL.send(buf).await;
                    //     // info!("Received SBG GPS Position: {:?}", gps_pos);
                    // },
                    // messages_prost::sensor::sbg::sbg_data::Data::UtcTime(utc_time) => {
                    //     // info!("Received SBG UTC Time: {:?}", utc_time.time_stamp);
                    // },
                    // messages_prost::sensor::sbg::sbg_data::Data::Imu(imu) => {
                    //     // info!("Received SBG IMU data: {:?}", imu.time_stamp);
                    // },
                    // messages_prost::sensor::sbg::sbg_data::Data::EkfQuat(ekf_quat) => {
                    //     // info!("Received SBG EKF Quaternion: {:?}", ekf_quat.time_stamp);
                    // },
                    // messages_prost::sensor::sbg::sbg_data::Data::EkfNav(ekf_nav) => {
                    //     // info!("Received SBG EKF Navigation: {:?}", ekf_nav.time_stamp);
                    // },
                    // messages_prost::sensor::sbg::sbg_data::Data::GpsVel(gps_vel) => {
                    //     // info!("Received SBG GPS Velocity: {:?}", gps_vel.time_stamp);
                    // },
                    // messages_prost::sensor::sbg::sbg_data::Data::Air(air) => {
                    //     let msg: SbgMessage = SbgMessage {
                    //         node: 0,
                    //         data: Some(data),
                    //     };
                    //     msg.encode_length_delimited(&mut buf.as_mut())
                    //         .expect("Failed to encode SBG GPS Position");
                    //     RADIO_CHANNEL.send(buf).await;
                    //     info!("air data send");
                    // },
                // }
            },
            None => {
                info!("No SBG data received");
            },
        }
    }
}

#[embassy_executor::task] 
async fn baro_reader_task(mut baro: Ms5611<Spi<'static, Blocking>, Output<'static>, Delay>) {
    info!("Barometer reader task started.");
    const MAIN_HEIGHT: f32 = GROUND_HEIGHT + 500.0; // meters ASL
    const HEIGHT_MIN: f32 = GROUND_HEIGHT + 300.0; // meters ASL
    const GROUND_HEIGHT: f32 = 300.0; // meters ASL
    const ASCENT_LOCKOUT: f32 = 0.1; 
    const DATA_POINTS: usize = 20;
    const VALID_DESCENT_RATE: f32 = -0.005; // meters per millise

    let mut historical_barometer_altitude: HistoryBuffer<(f32, Instant), 20> = HistoryBuffer::new();

    loop {
        match baro.read_pressure_temperature(OversamplingRatio::Osr4096) {
            Ok(reading) => {
                // info!(
                //     "Baro: Temp: {} C, Pressure: {} mbar",
                //     reading.0, reading.1
                // );

                // Hypsometric Formula
                let altitude = ((powf(101.325 / reading.1, 1.0/5.257) - 1.0) * (25.0 + 273.15)) / 0.0065;
                historical_barometer_altitude.write((altitude, Instant::now()));

                // Apogee detection logic
                if historical_barometer_altitude.len() < 8 {
                    info!("not enough data points to detect apogee");
                    continue;
                }

                let mut buf = historical_barometer_altitude.oldest_ordered();
                if let Some(mut prev_reading) = buf.next() { // `prev_reading` is now a tuple: (f32, Instant)
                    let mut avg_sum: f32 = 0.0;
                    let mut datapoints_used = 0;

                    for current_reading in buf { // `current_reading` is also a tuple
                        // Calculate time diff between the actual measurement times.
                        // Convert from micros to seconds for a more standard rate unit (meters/sec).
                        let time_diff = current_reading.1.duration_since(prev_reading.1).as_millis();

                        info!(
                            "prev alt: {}, new alt: {}, time diff: {} ms",
                            prev_reading.0, current_reading.0, time_diff
                        );

                        if time_diff == 0 {
                            continue; // Avoid division by zero
                        }

                        let slope = (current_reading.0 - prev_reading.0) / time_diff as f32;
                        // info!("Slope: {} m/ms", slope);
                        // Your existing logic for ascent lockout
                        if slope > ASCENT_LOCKOUT {
                            continue;
                        }

                        avg_sum += slope;
                        datapoints_used += 1;
                        prev_reading = current_reading; // Update to the current reading for the next iteration
                    }

                    // Check if the average descent rate is valid
                    if datapoints_used > 0 {
                        let avg_slope = avg_sum / (datapoints_used as f32);
                        // info!("Average slope: {} m/ms", avg_slope);
                        if avg_slope <= VALID_DESCENT_RATE {
                            info!("Apogee detected! Average vertical speed: {} m/s", avg_slope * 1000.0);
                            // todo!("Send Apogee event over events channel to state machine to process.");
                        }
                    }
                }
            }
            Err(e) => {
                // error!("Baro: Driver reading failed: {:?}", e);
            }
        }
        Timer::after(Duration::from_millis(100)).await;
    }
}

#[embassy_executor::task]
async fn radio_reader_task(mut rx: RingBufferedUartRx<'static>) {
    loop {
        let mut buf: [u8; RADIO_BUFFER_SIZE] = [0; RADIO_BUFFER_SIZE];
        if let Ok(len) = rx.read(&mut buf).await {
            if len > 0 {
                // Process the received data
                info!("Received {} bytes from radio: {:?}", len, &buf[..len]);
                let (_header, msg): (_, MavMessage) =
                    mavlink::read_versioned_msg(&mut PeekReader::new(&buf[..len]), mavlink::MavlinkVersion::V2).unwrap();

                match msg {
                    mavlink::uorocketry::MavMessage::POSTCARD_MESSAGE(msg) => {
                        info!("Received postcard message");
                        // decode the msg
                        if let Ok(recv) = messages_prost::radio::RadioFrame::decode_length_delimited(
                            &mut &msg.message[..],
                        ) {
                            // info!("Received radio frame: {:?}", recv.node);
                            if let Some(payload) = recv.payload {
                                match payload {
                                    Payload::Sbg(sbg_data) => {
                                        info!("Received SBG data: {:?}", sbg_data.data.is_some());
                                    }
                                    Payload::Gps(gps_data) => {
                                        info!("Received GPS data: {:?}", gps_data.data.len());
                                        // Handle GPS data
                                    }
                                    Payload::Madgwick(madgwick_data) => {
                                        info!("Received Madgwick data: {:?}", madgwick_data.data.is_some());
                                        // Handle Madgwick data
                                    }
                                    Payload::Iim20670(imu_data) => {
                                        info!("Received IMU data: {:?}", imu_data.data.is_some());
                                        // Handle IMU data
                                    }
                                    Payload::Log(log_data) => {
                                        info!("Received Log data: {:?}", log_data.level );
                                        // Handle Log data
                                    }
                                    Payload::State(state_message) => {
                                        info!("Received State message: {:?}", state_message.state);
                                        // Handle State message
                                    }
                                    Payload::Command(command) => {
                                        info!("Received Command: {:?}", command.data.is_some());
                                        if let Some(command_data) = command.data {
                                            match command_data {
                                                messages_prost::command::command::Data::Ping(ping) => {
                                                    // info!("Received Ping command: {:?}", ping);
                                                    info!("Ping");
                                                    let mut buf: [u8; 255] = [0; 255];
                                                    let msg = messages_prost::radio::RadioFrame {
                                                        node: messages_prost::common::Node::Phoenix.into(), 
                                                        payload: Some(messages_prost::radio::radio_frame::Payload::Command(
                                                            messages_prost::command::Command {
                                                                node: 0,
                                                                data: Some(messages_prost::command::command::Data::Pong(
                                                                    messages_prost::command::Pong {
                                                                        id: ping.id,
                                                                    }
                                                                )),
                                                            }
                                                        ))
                                                    };
                                                    msg.encode_length_delimited(&mut buf.as_mut())
                                                        .expect("Failed to encode SBG GPS Position");
                                                    RADIO_CHANNEL.send(buf).await; 
                                                }
                                                messages_prost::command::command::Data::Pong(pong) => {
                                                    // info!("Received Pong command: {:?}", pong);
                                                    info!("Pong");
                                                }
                                                messages_prost::command::command::Data::Online(online) => {
                                                    // info!("Received Online command: {:?}", online);
                                                }
                                                messages_prost::command::command::Data::DeployDrogue(deploy_drogue) => {
                                                    RECOVERY_MANAGER.lock(|cell| {
                                                        // *cell.borrow_mut() = Some(recovery_manager);
                                                        if let Some(recovery_manager) = cell.borrow_mut().as_mut() {
                                                            recovery_manager.arming.drogue.set_high();
                                                            recovery_manager.arming.drogue_b.set_high();
                                                            recovery_manager.fire.drogue.set_high();
                                                            recovery_manager.fire.drogue_b.set_high();
                                                            Delay.delay_ms(500);
                                                            
                                                            recovery_manager.fire.drogue.set_low();
                                                            recovery_manager.fire.drogue_b.set_low();
                                                            recovery_manager.arming.drogue.set_low();
                                                            recovery_manager.arming.drogue_b.set_low();
                                                        } else {
                                                            info!("Recovery manager not initialized.");
                                                        }
                                                    });
                                                    
                                                    // COMMAND_CHANNEL.send(command_data).await; 
                                                    // info!("Received Deploy Drogue command: {:?}", deploy_drogue);
                                                }
                                                messages_prost::command::command::Data::DeployMain(deploy_main) => {
                                                    RECOVERY_MANAGER.lock(|cell| {
                                                        info!("Boom boom");
                                                        // *cell.borrow_mut() = Some(recovery_manager);
                                                        if let Some(recovery_manager) = cell.borrow_mut().as_mut() {
                                                            recovery_manager.arming.main.set_high();
                                                            recovery_manager.arming.main_b.set_high();
                                                            recovery_manager.fire.main.set_high();
                                                            recovery_manager.fire.main_b.set_high();
                                                            Delay.delay_ms(500);

                                                            recovery_manager.fire.main.set_low();
                                                            recovery_manager.fire.main_b.set_low();
                                                            recovery_manager.arming.main.set_low();
                                                            recovery_manager.arming.main_b.set_low();
                                                        } else {
                                                            info!("Recovery manager not initialized.");
                                                        }
                                                    });
                                                    // info!("Received Deploy Main command: {:?}", deploy_main);
                                                    // COMMAND_CHANNEL.send(command_data).await; 

                                                }
                                                messages_prost::command::command::Data::PowerDown(power_down) => {
                                                    // info!("Received Power Down command: {:?}", power_down);
                                                }
                                                messages_prost::command::command::Data::RadioRateChange(rate_change) => {
                                                    // info!("Received Radio Rate Change command: {:?}", rate_change);
                                                }
                                            }
                                        }
                                        // Handle Command
                                    }
                                }
                            }

                        } else {
                            info!("Failed to decode radio frame.");
                        }

                    }
                    mavlink::uorocketry::MavMessage::COMMAND_MESSAGE(command) => {
                        info!("Received command");
                    }
                    mavlink::uorocketry::MavMessage::HEARTBEAT(_) => {
                        info!("Received heartbeat message.");
                    }
                    _ => {
                        info!("Unknown mavlink message.");
                        // info!("Received unknown MAVLink message: {:?}", msg);
                    }
                }
            }
        }
        Timer::after(Duration::from_millis(100)).await;
    }
}

#[embassy_executor::task]
async fn radio_writer_task(mut tx: UartTx<'static, mode::Async>) {
    loop {
        let data = RADIO_CHANNEL.receive().await;

        let mav_header = mavlink::MavHeader {
            system_id: 1,
            component_id: 1,
            sequence: 1,
        };

        let mav_message = mavlink::uorocketry::MavMessage::POSTCARD_MESSAGE(
            mavlink::uorocketry::POSTCARD_MESSAGE_DATA {
                message: data,
            },
        );
        // info!("Writing radio message");
        mavlink::write_versioned_msg_async(&mut tx, mavlink::MavlinkVersion::V2, mav_header, &mav_message).await;
    }
}

#[embassy_executor::task] 
async fn sm_task(spawner: Spawner, state_machine: StateMachine<Context>) {
    info!("State Machine task started.");

    loop {  
        match state_machine.state {
            States::Ascent => {

            },
            States::Fault => {

            },
            States::Init => {
                
            },
            States::WaitForLaunch => {
                
            },
            States::Descent => {

            },
            States::DrogueDescent => {

            },
            States::Fuck => {

            },
            States::Landed => {

            },
            States::MainDescent => {

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
    let p = embassy_stm32::init(config);
    
    info!("Heap usage: {} bytes", HEAP.used());

    // --- IMU Setup --- 
    let mut imu_spi_config = SpiConfig::default();
    imu_spi_config.frequency = mhz(9);
    imu_spi_config.mode = embassy_stm32::spi::Mode {
        polarity: embassy_stm32::spi::Polarity::IdleLow,
        phase: embassy_stm32::spi::Phase::CaptureOnFirstTransition,
    };
    // let imu_spi = Spi::new(
        // p.SPI3, p.PC10, p.PB5, p.PB4, p.DMA2_CH5, p.DMA2_CH6, imu_spi_config,
    // );
    let imu_spi = Spi::new_blocking(
        p.SPI3, p.PC10, p.PB5, p.PB4, imu_spi_config,
    );
    let imu_odr = Input::new(p.PC0, Pull::None);
    let imu_cs = Output::new(p.PB6, Level::High, Speed::Low);
    let imu_nreset = Output::new(p.PD4, Level::High, Speed::Low);
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
    let mut uart_config = UartConfig::default();
    uart_config.baudrate = 115200; 
    let usart = Uart::new(
        p.UART4, p.PA1, p.PA0, Irqs, p.DMA1_CH1, p.DMA1_CH0, uart_config,
    ).unwrap();
    let (tx, rx) = usart.split();
    let ring_rx = rx.into_ring_buffered(unsafe { &mut RX_SBG_BUF });
    let mut sbg_pwr = Output::new(p.PD8, Level::High, Speed::Low);
  
    // // --- Baro SPI Setup ---
    let mut spi_config = SpiConfig::default();
    spi_config.frequency = mhz(16);
    // let spi_bus = Spi::new(
    //     p.SPI4, p.PE2, p.PE6, p.PE5, p.DMA1_CH6, p.DMA1_CH7, spi_config,
    // );
    let spi_bus = Spi::new_blocking(
        p.SPI4, p.PE2, p.PE6, p.PE5, spi_config,
    );
    info!("SPI4 bus configured.");

    // Initialize the Mutex without the RefCell.
    // let spi_bus_mutex = SPI_BUS.init(embassy_sync::mutex::Mutex::new(spi_bus));

    let baro_cs = Output::new(p.PB8, Level::High, Speed::Low);
    info!("Barometer CS pin configured.");

    // SpiDevice::new takes an immutable reference, which spi_bus_mutex can be coerced into.
    // let baro_spi_device = SpiDevice::new(spi_bus_mutex, baro_cs);
    let baro = Ms5611::new(spi_bus, baro_cs, Delay).unwrap();


    // // --- SD Card ---
    // let mut sd_spi_config = SpiConfig::default();

    // sd_spi_config.frequency = mhz(16);
    // sd_spi_config.bit_order = BitOrder::MsbFirst;

    // let mut sd_spi_bus = Spi::new_blocking(
    //     p.SPI1, p.PA5, p.PA7, p.PA6, sd_spi_config,
    // );

    // let sd_cs = Output::new(p.PE9, Level::High, Speed::Low);
    // let data: [u8; 10] = [0xFF; 10];
    // sd_spi_bus.blocking_write(&data).unwrap();

    // let sd_spi_bus_ref_cell = RefCell::new(sd_spi_bus);
    // let sd_spi_device = RefCellDevice::new(&sd_spi_bus_ref_cell, sd_cs, Delay);
    // let sdcard = SdCard::new(sd_spi_device.unwrap(), Delay);
    // println!("Card size is {} bytes", sdcard.num_bytes().unwrap());
    // let volume_mgr = VolumeManager::new(sdcard, TimeSink::new());
    // let volume0 = volume_mgr.open_volume(VolumeIdx(0)).unwrap();
    // let root_dir = volume0.open_root_dir().unwrap();
    // // let my_file = root_dir.open_file_in_dir("MY_FILE.TXT", Mode::ReadOnly).unwrap();
    // // while !my_file.is_eof() {
    // //     let mut buffer = [0u8; 32];
    // //     let num_read = my_file.read(&mut buffer).unwrap();
    // //     for b in &buffer[0..num_read] {
    // //         info!("{}", *b as char);
    // //     }
    // // }
    // info!("Sd write and setup complete");

    // // --- GPS Setup ---
    let mut gps_enable = Output::new(p.PA4, Level::Low, Speed::Low); 
    let mut gps_reset = Output::new(p.PB2, Level::Low, Speed::Low); 
    let mut uart_gps_config = UartConfig::default();
    uart_gps_config.baudrate = 9600; 
    uart_gps_config.data_bits = embassy_stm32::usart::DataBits::DataBits8;
    uart_gps_config.parity = embassy_stm32::usart::Parity::ParityNone;
    uart_gps_config.stop_bits = embassy_stm32::usart::StopBits::STOP1;
    uart_gps_config.detect_previous_overrun = false; 
    // let uart_gps = Uart::new_blocking(
    //     p.UART8,  p.PE0, p.PE1, uart_gps_config
    // ).unwrap();
    let mut uart_gps = Uart::new(
        p.UART8, p.PE0, p.PE1, Irqs, p.DMA1_CH5, p.DMA1_CH6, uart_gps_config
    ).unwrap();

    let (mut gps_tx, mut gps_rx) = uart_gps.split();
    let mut ring_gps_rx = gps_rx.into_ring_buffered(unsafe { &mut RX_GPS_BUF });
    gps_reset.set_low();
    Delay.delay_ms(3000);
    gps_reset.set_high();
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

    info!("Sending GPS packet: {:?}", &packet);
    gps_tx.blocking_write(&packet).expect("TODO: panic message");

    Delay.delay_ms(100);

    // let val_packet = CfgValSetBuilder {
    //     version: 1,
    //     layers: CfgLayer::RAM,
    //     reserved1: 0,
    //     cfg_data: &[Uart1OutProtUbx(true), Uart1InProtUbx(true)],
    // }.into_packet_vec();

    // info!("Packet val {}", val_packet.clone().as_slice());

    // gps_tx.blocking_write(val_packet.as_slice());

    Delay.delay_ms(1000);

    // let request =
    //     UbxPacketRequest::request_for::<ublox::NavPosLlh>().into_packet_bytes();
    // gps_tx.blocking_write(&request);

    // loop {
    //     let request =
    //         UbxPacketRequest::request_for::<ublox::NavPosLlh>().into_packet_bytes();
    //     gps_tx.blocking_write(&request);
    //     // Delay.delay_ms(1000);
    //     let mut buf_data: [u8; GPS_BUFFER_SIZE] = [0; GPS_BUFFER_SIZE];
    //     ring_gps_rx.read(&mut buf_data).await.unwrap();
    //     info!("GPS data read: {:?}", &buf_data[..]);
    //     // if let Ok(len) = gps_rx.read(&mut buf_data).await {
    //             // info!("read");

    //             Delay.delay_ms(1000);
    //             // cortex_m::asm::delay(10_000);
    //     let mut buf: [u8; GPS_BUFFER_SIZE] = [0; GPS_BUFFER_SIZE];
    //     let bytes: [u8; GPS_BUFFER_SIZE] = [0; GPS_BUFFER_SIZE];

    //     // let mut nmea = nmea::Nmea::default();
    //     // let ascii_buf = unsafe {buf_data.as_ascii_unchecked()};
    //     // info!("BUFFER: {}", ascii_buf.as_str());
    //     // // if let Some(ascii_buf) = ascii_buf {
    //     //     let res = nmea.parse(ascii_buf.as_str());

    //     //     match res {
    //     //         Ok(strings) => {
    //     //             info!("Result: {}", strings.as_str());
    //     //         }
    //     //         _ => {
    //     //             info!("nmea parser none found");
    //     //         }
    //     //     }


    //     // } else {
    //     //     info!("No valid sentence");
    //     // }

    //     let buf: ublox::FixedLinearBuffer<'_> = ublox::FixedLinearBuffer::new(&mut buf[..]);
    //     let mut parser = ublox::Parser::new(buf);
    //     // info!("GPS Parser initialized.");
    //     let mut msgs = parser.consume(&buf_data);
    //     // info!("GPS Messages consumed. {}", msgs.next().is_some());
    //     while let Some(msg) = msgs.next() {
    //         match msg {
    //             Ok(msg) => match msg {
    //                 ublox::PacketRef::NavPosLlh(x) => {
    //                     info!(
    //                         "GPS latitude: {:?}, longitude {:?}",
    //                         x.lat_degrees(),
    //                         x.lon_degrees()
    //                     );
    //                 }
    //                 ublox::PacketRef::NavStatus(x) => {
    //                     info!("GPS fix stat: {:?}", x.fix_stat_raw());
    //                 }
    //                 ublox::PacketRef::NavDop(x) => {
    //                     info!("GPS geometric drop: {:?}", x.geometric_dop());
    //                 }
    //                 ublox::PacketRef::NavSat(x) => {
    //                     info!("GPS num sats used: {:?}", x.num_svs());
    //                 }
    //                 ublox::PacketRef::NavVelNed(x) => {
    //                     info!("GPS velocity north: {:?}", x.vel_north());
    //                 }
    //                 ublox::PacketRef::NavPvt(x) => {
    //                     info!("GPS nun sats PVT: {:?}", x.num_satellites());
    //                 }
    //                 ublox::PacketRef::Unknown(msg) => {
    //                     info!("Unknown GPS type.");
    //                 }
    //                 _ => {
    //                     info!("GPS Message not handled.");
    //                 }
    //             },
    //             Err(e) => {
    //                 info!("GPS parse Error");
    //             }
    //         }
    //     }
    //     // }
    // }


    // --- Radio Setup --- 
    // let mut radio_uart_config = UartConfig::default();
    // radio_uart_config.baudrate = 57600; 
    // let radio_uart = Uart::new(
    //     p.UART7, p.PE7, p.PE8, Irqs, p.DMA2_CH2, p.DMA2_CH7, radio_uart_config,
    // ).unwrap();
    // let (radio_tx, radio_rx) = radio_uart.split();
    // let radio_ring_rx = radio_rx.into_ring_buffered(unsafe { &mut RX_RADIO_BUF });
  

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

    // // let main_arm_test = Input::new(p.PD6, Pull::Down);
    let mut main_arm_test = Output::new(p.PD6, Level::Low, Speed::Low);
    let main_arm_test_b = Output::new(p.PD14, Level::Low, Speed::Low);
    let drogue_arm_test = Output::new(p.PC11, Level::Low, Speed::Low);
    let drogue_arm_test_b = Output::new(p.PD2, Level::Low, Speed::Low);

    let mut main_fire = Output::new(p.PD5, Level::Low, Speed::Low);
    let mut main_fire_b = Output::new(p.PD13, Level::Low, Speed::Low);
    let drogue_fire = Output::new(p.PC12, Level::Low, Speed::Low);
    let drogue_fire_b = Output::new(p.PD1, Level::Low, Speed::Low);

    let mut main_mcu_ematch_sense = p.PA2; 
    let mut main_mcu_ematch_sense_b = p.PB0; 

    let mut drogue_mcu_ematch_sense = p.PA3; 
    let mut drogue_mcu_ematch_sense_b = p.PC5; 

    let mut adc = Adc::new(p.ADC1);
    
    info!("ADC measurement main ematch {}", adc.blocking_read(&mut main_mcu_ematch_sense));
    info!("ADC measurement main B ematch {}", adc.blocking_read(&mut main_mcu_ematch_sense_b));
    info!("ADC measurement drogue ematch {}", adc.blocking_read(&mut drogue_mcu_ematch_sense));
    info!("ADC measurement drogue B ematch {}", adc.blocking_read(&mut drogue_mcu_ematch_sense_b));
    info!("ADC measurement main ematch {}", adc.blocking_read(&mut main_mcu_ematch_sense));

    let recovery_manager = RecoveryManager {
        arming: Arming {
            main: main_arm_test,
            drogue: drogue_arm_test,
            main_b: main_arm_test_b,
            drogue_b: drogue_arm_test_b,
        },
        fire: Fire {
            main: main_fire,
            drogue: drogue_fire,
            main_b: main_fire_b,
            drogue_b: drogue_fire_b,
        }
    };
    
    RECOVERY_MANAGER.lock(|cell| {
        *cell.borrow_mut() = Some(recovery_manager);
    });

    // // --- Camera Triggers ---
    // let mut cam_trigger = Output::new(p.PE14, Level::Low, Speed::Low);
    // let mut cam_trigger_b = Output::new(p.PE12, Level::Low, Speed::Low);

    // // // --- Buzzer 🐝 ---
    // let buzz_out_pin = PwmPin::new_ch1(p.PC6, OutputType::PushPull);
    // let mut pwm = SimplePwm::new(p.TIM3, Some(buzz_out_pin), None, None, None, khz(4), Default::default());
    // let mut ch1 = pwm.ch1();
    // info!("Duty Cycle: {}", ch1.max_duty_cycle());
    // ch1.set_duty_cycle(ch1.max_duty_cycle() / 4);
    // ch1.enable();   
    // // music::play_song(&mut pwm, music::MARIO_MELODY, 100).await;

    // // --- State Machine ---
    // let state_machine = StateMachine::new(traits::Context {});

    // // --- Radio --- 
    let mut uart_radio_config = UartConfig::default();
    uart_radio_config.baudrate = 57600; 
    uart_radio_config.data_bits = embassy_stm32::usart::DataBits::DataBits8;
    uart_radio_config.parity = embassy_stm32::usart::Parity::ParityNone;
    uart_radio_config.stop_bits = embassy_stm32::usart::StopBits::STOP1;

    let mut uart_radio = Uart::new(
        p.UART7, p.PE7, p.PE8, Irqs, p.DMA2_CH3, p.DMA2_CH5, uart_radio_config
    ).unwrap();

    let (mut radio_tx, mut radio_rx) = uart_radio.split();
    let mut radio_ring_rx = radio_rx.into_ring_buffered(unsafe { &mut RX_RADIO_BUF });
   

    // // --- AI ---
    // // // Get a default device for the backend
    // let device = BackendDevice::default();
    //
    // // // Create a new model and load the state
    // // let model: Model<Backend> = Model::default();
    //
    // // let output = run_model(&model, &device, 1.0);
    // let recorder = CompactRecorder::new();
    //
    // let record_bytes = include_bytes!("model/tte.mpk");
    //
    // let record = recorder
    //     .load(record_bytes.as_ref(), &device)
    //     .expect("Failed to load recorder");
    //
    // let model: model::LstmNetwork<NdArray> = config.model.init(&device).load_record(record);
    // NOTE 
    // Creating multiple executor instances is supported, to run tasks with multiple priority levels. This allows higher-priority tasks to preempt lower-priority tasks.

    // --- Spawning Tasks ---
    // spawner.must_spawn(led_blinker_task(p.PB14));

    spawner.must_spawn(uart_dma_reader_task(ring_rx));
    // spawner.must_spawn(uart_gps_dma_reader_task(ring_gps_rx, gps_tx));
    spawner.must_spawn(sbg_parser_task(tx));
    spawner.must_spawn(sbg_receiver_task());
    // spawner.must_spawn(baro_reader_task(baro));
    // spawner.must_spawn(ai_task());
    // pass control of the spawner to the state machine
    // spawner.must_spawn(sm_task(spawner, state_machine));
    spawner.must_spawn(radio_reader_task(radio_ring_rx));
    spawner.must_spawn(radio_writer_task(radio_tx));
}

// fn run_model<'a>(model: &Model<NdArray>, device: &BackendDevice, input: f32) -> Tensor<Backend, 2> {
//     // Define the tensor
//     let input = Tensor::<Backend, 2>::from_floats([[input]], &device);
//
//     // Run the model on the input
//     let output = model.forward(input);
//
//     output
// }