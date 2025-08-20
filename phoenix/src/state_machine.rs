use defmt::info;
use embassy_executor::Spawner;
use embassy_time::{Instant, Timer};
use heapless::{HistoryBuffer, Vec};
use messages_prost::phoenix_state::{Event, State};
use messages_prost::prost::Message;
use smlang::statemachine;

use crate::recovery::{recovery_algorithm_task, DATA_POINTS, SLOPES};
use crate::resources::{
    EVENT_CHANNEL, PRESSURE_CHANNEL, RADIO_CHANNEL, RECOVERY_MANAGER, SD_CHANNEL,
};

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

pub struct Context {}

impl StateMachineContext for Context {}

impl From<States> for State {
    fn from(value: States) -> Self {
        match value {
            States::Fuck => State::Fuck,
            States::Init => State::Init,
            States::Fault => State::Fault,
            States::WaitForLaunch => State::WaitForLaunch,
            States::Ascent => State::Ascent,
            States::Descent => State::Descent,
            States::DrogueDescent => State::DrogueDescent,
            States::MainDescent => State::MainDescent,
            States::Landed => State::Landed,
        }
    }
}

#[embassy_executor::task]
pub async fn sm_task(spawner: Spawner, mut state_machine: StateMachine<Context>) {
    info!("State Machine task started.");
    let mut recovery_spawned = false; 
    let mut historical_barometer_altitude_sbg: HistoryBuffer<(f32, Instant), DATA_POINTS> =
        HistoryBuffer::new();
    const CONSECUTIVE_NEGATIVE_THRESHOLD: usize = 10;
    const NO_MOVEMENT_RATE: f32 = 0.0001;

    loop {
        if let Ok(event) = EVENT_CHANNEL.try_receive() {
            state_machine.process_event(event);
        }

        match state_machine.state {
            States::Ascent => {
                let mut buf: [u8; 255] = [0; 255];

                let msg = messages_prost::radio::RadioFrame {
                    node: messages_prost::common::Node::Phoenix.into(),
                    payload: Some(messages_prost::radio::radio_frame::Payload::PhoenixState(
                        State::Ascent.into(),
                    )),
                    millis_since_start: Instant::now().as_millis(),
                };
                msg.encode_length_delimited(&mut buf.as_mut()).unwrap();
                RADIO_CHANNEL.send(buf.clone()).await;
                SD_CHANNEL.send(("state.txt", buf)).await;
            }
            States::Fault => {
                let mut buf: [u8; 255] = [0; 255];

                let msg = messages_prost::radio::RadioFrame {
                    node: messages_prost::common::Node::Phoenix.into(),
                    payload: Some(messages_prost::radio::radio_frame::Payload::PhoenixState(
                        State::Fault.into(),
                    )),
                    millis_since_start: Instant::now().as_millis(),
                };
                msg.encode_length_delimited(&mut buf.as_mut()).unwrap();
                RADIO_CHANNEL.send(buf.clone()).await;
                SD_CHANNEL.send(("state.txt", buf)).await;
            }
            States::Init => {
                let mut buf: [u8; 255] = [0; 255];

                let msg = messages_prost::radio::RadioFrame {
                    node: messages_prost::common::Node::Phoenix.into(),
                    payload: Some(messages_prost::radio::radio_frame::Payload::PhoenixState(
                        State::Init.into(),
                    )),
                    millis_since_start: Instant::now().as_millis(),
                };
                msg.encode_length_delimited(&mut buf.as_mut()).unwrap();
                RADIO_CHANNEL.send(buf.clone()).await;
                SD_CHANNEL.send(("state.txt", buf)).await;

                // let mut should_start = false;
                // // await both channels to be armed.
                // RECOVERY_MANAGER.lock(|cell| {
                //     if let Some(recovery_manager) = cell.borrow_mut().as_mut() {
                //         if recovery_manager.is_armed() {
                //             // this could be it's own task this has the posibility to be bad since
                //             // it's just evaluated for the first arming, if disarmed it will still be potentionally live.
                //             should_start = true;
                //         }
                //     }
                // });

                // if should_start {
                    EVENT_CHANNEL.send(Events::Start).await;

                    let msg = messages_prost::radio::RadioFrame {
                        node: messages_prost::common::Node::Phoenix.into(),
                        payload: Some(messages_prost::radio::radio_frame::Payload::PhoenixEvent(
                            Event::Start.into(),
                        )),
                        millis_since_start: Instant::now().as_millis(),
                    };

                    msg.encode_length_delimited(&mut buf.as_mut()).unwrap();
                    RADIO_CHANNEL.send(buf.clone()).await;
                    SD_CHANNEL.send(("event.txt", buf)).await;
                // }
            }
            States::WaitForLaunch => {
                let mut buf: [u8; 255] = [0; 255];

                let msg = messages_prost::radio::RadioFrame {
                    node: messages_prost::common::Node::Phoenix.into(),
                    payload: Some(messages_prost::radio::radio_frame::Payload::PhoenixState(
                        State::WaitForLaunch.into(),
                    )),
                    millis_since_start: Instant::now().as_millis(),
                };
                let (altitude, temperature, sender, timestamp) = PRESSURE_CHANNEL.receive().await;

                // sbg data
                if sender == 0 && !recovery_spawned {
                    if altitude >= crate::recovery::HEIGHT_MIN {
                        EVENT_CHANNEL.send(Events::Launch).await;
                        spawner.must_spawn(recovery_algorithm_task());
                        recovery_spawned = true; 

                        let msg = messages_prost::radio::RadioFrame {
                            node: messages_prost::common::Node::Phoenix.into(),
                            payload: Some(messages_prost::radio::radio_frame::Payload::PhoenixEvent(
                                Event::Launch.into(),
                            )),
                            millis_since_start: Instant::now().as_millis(),
                        };

                        msg.encode_length_delimited(&mut buf.as_mut()).unwrap();
                        RADIO_CHANNEL.send(buf.clone()).await;
                        SD_CHANNEL.send(("event.txt", buf)).await;
                    }
                }

                msg.encode_length_delimited(&mut buf.as_mut()).unwrap();
                RADIO_CHANNEL.send(buf.clone()).await;
                SD_CHANNEL.send(("state.txt", buf)).await;
            }
            States::Descent => {
                RECOVERY_MANAGER.lock(|cell| {
                    if let Some(recovery_manager) = cell.borrow_mut().as_mut() {
                        recovery_manager.fire_drogue();
                    }
                });

                // Fire the main
                EVENT_CHANNEL.send(Events::DrogueDeployment).await;

                let mut buf: [u8; 255] = [0; 255];

                let msg = messages_prost::radio::RadioFrame {
                    node: messages_prost::common::Node::Phoenix.into(),
                    payload: Some(messages_prost::radio::radio_frame::Payload::PhoenixState(
                        State::Descent.into(),
                    )),
                    millis_since_start: Instant::now().as_millis(),
                };

                msg.encode_length_delimited(&mut buf.as_mut()).unwrap();
                RADIO_CHANNEL.send(buf.clone()).await;
                SD_CHANNEL.send(("state.txt", buf)).await;

                let msg = messages_prost::radio::RadioFrame {
                    node: messages_prost::common::Node::Phoenix.into(),
                    payload: Some(messages_prost::radio::radio_frame::Payload::PhoenixEvent(
                        Event::DrogueDeployment.into(),
                    )),
                    millis_since_start: Instant::now().as_millis(),
                };
                msg.encode_length_delimited(&mut buf.as_mut()).unwrap();
                RADIO_CHANNEL.send(buf.clone()).await;
                SD_CHANNEL.send(("event.txt", buf)).await;
            }
            States::DrogueDescent => {
                let mut buf: [u8; 255] = [0; 255];

                let msg = messages_prost::radio::RadioFrame {
                    node: messages_prost::common::Node::Phoenix.into(),
                    payload: Some(messages_prost::radio::radio_frame::Payload::PhoenixState(
                        State::DrogueDescent.into(),
                    )),
                    millis_since_start: Instant::now().as_millis(),
                };
                msg.encode_length_delimited(&mut buf.as_mut()).unwrap();
                RADIO_CHANNEL.send(buf.clone()).await;
                SD_CHANNEL.send(("state.txt", buf)).await;

                let (altitude, temperature, sender, timestamp) = PRESSURE_CHANNEL.receive().await;

                // sbg data
                if sender == 0 {
                    if altitude <= crate::recovery::MAIN_HEIGHT {
                        RECOVERY_MANAGER.lock(|cell| {
                            if let Some(recovery_manager) = cell.borrow_mut().as_mut() {
                                recovery_manager.fire_main();
                            }
                        });

                        EVENT_CHANNEL.send(Events::MainDeployment).await;

                        let msg = messages_prost::radio::RadioFrame {
                            node: messages_prost::common::Node::Phoenix.into(),
                            payload: Some(
                                messages_prost::radio::radio_frame::Payload::PhoenixEvent(
                                    Event::MainDeployment.into(),
                                ),
                            ),
                            millis_since_start: Instant::now().as_millis(),
                        };
                        msg.encode_length_delimited(&mut buf.as_mut()).unwrap();
                        RADIO_CHANNEL.send(buf.clone()).await;
                        SD_CHANNEL.send(("event.txt", buf)).await;
                    }
                }
            }
            States::Fuck => {
                let mut buf: [u8; 255] = [0; 255];

                let msg = messages_prost::radio::RadioFrame {
                    node: messages_prost::common::Node::Phoenix.into(),
                    payload: Some(messages_prost::radio::radio_frame::Payload::PhoenixState(
                        State::Fuck.into(),
                    )),
                    millis_since_start: Instant::now().as_millis(),
                };
                msg.encode_length_delimited(&mut buf.as_mut()).unwrap();
                RADIO_CHANNEL.send(buf).await;
                SD_CHANNEL.send(("state.txt", buf)).await;
            }
            States::Landed => {
                let mut buf: [u8; 255] = [0; 255];

                let msg = messages_prost::radio::RadioFrame {
                    node: messages_prost::common::Node::Phoenix.into(),
                    payload: Some(messages_prost::radio::radio_frame::Payload::PhoenixState(
                        State::Landed.into(),
                    )),
                    millis_since_start: Instant::now().as_millis(),
                };
                msg.encode_length_delimited(&mut buf.as_mut()).unwrap();
                RADIO_CHANNEL.send(buf).await;
                SD_CHANNEL.send(("state.txt", buf)).await;
            }
            States::MainDescent => {
                let mut buf: [u8; 255] = [0; 255];

                let msg = messages_prost::radio::RadioFrame {
                    node: messages_prost::common::Node::Phoenix.into(),
                    payload: Some(messages_prost::radio::radio_frame::Payload::PhoenixState(
                        State::MainDescent.into(),
                    )),
                    millis_since_start: Instant::now().as_millis(),
                };
                msg.encode_length_delimited(&mut buf.as_mut()).unwrap();
                RADIO_CHANNEL.send(buf).await;
                SD_CHANNEL.send(("state.txt", buf)).await;

                let (altitude, temperature, sender, timestamp) = PRESSURE_CHANNEL.receive().await;

                if sender == 0 {
                    // Source is the SBG
                    info!("Altitude: {}", altitude);
                    historical_barometer_altitude_sbg.write((altitude, timestamp));
                    
                    // --- Apogee Detection Logic ---
                    // Ensure there are enough data points in the buffer to perform a reliable calculation.
                    if historical_barometer_altitude_sbg.len() < DATA_POINTS {
                        continue;
                    }

                    // --- DEBUG PRINT: Show the current data buffer ---
                    let altitude_history_for_print: Vec<_, DATA_POINTS> = historical_barometer_altitude_sbg
                        .oldest_ordered()
                        .map(|(alt, _)| alt)
                        .collect();
                    info!("[SBG] Altitude History: {:?}", altitude_history_for_print.as_slice());

                    let mut buf_sbg = historical_barometer_altitude_sbg.oldest_ordered();

                    // --- SBG Apogee Check ---
                    if let Some(mut prev_reading) = buf_sbg.next() {
                        let mut slopes: HistoryBuffer<f32, SLOPES> = HistoryBuffer::new();

                        for current_reading in buf_sbg {
                            let time_diff_ms = current_reading.1.duration_since(prev_reading.1).as_millis();
                            let alt_diff = current_reading.0 - prev_reading.0;

                            // --- DEBUG PRINT: Show inputs for slope calculation ---
                            info!("[SBG] Slope calc: alt_diff={}, time_diff={} ms", alt_diff, time_diff_ms);

                            if time_diff_ms > 0 {
                                let slope_mpms = alt_diff / time_diff_ms as f32;
                                
                                // --- DEBUG PRINT: Show the calculated slope ---
                                info!("[SBG]  -> New Slope: {} m/ms ({} m/s)", slope_mpms, slope_mpms * 1000.0);
                                slopes.write(slope_mpms);
                            } else {
                                // --- DEBUG PRINT: Indicate when a calculation is skipped ---
                                info!("[SBG]  -> Skipping slope calc: time_diff_ms is 0");
                            }
                            prev_reading = current_reading;
                        }

                        if slopes.len() >= CONSECUTIVE_NEGATIVE_THRESHOLD {
                            let slopes_vec: Vec<_, DATA_POINTS> = slopes.oldest_ordered().collect();

                            // --- DEBUG PRINT: Show the buffer of slopes being evaluated ---
                            info!("[SBG] Slopes buffer for check: {:?}", slopes_vec.as_slice());
                            
                            let mut consecutive_negative_count = 0;
                            let mut sum_of_recent_slopes = 0.0;

                            for slope in slopes_vec.iter().rev().take(CONSECUTIVE_NEGATIVE_THRESHOLD) {
                                if **slope <= 0.0 {
                                    consecutive_negative_count += 1;
                                    sum_of_recent_slopes += *slope;
                                } else {
                                    break;
                                }
                            }

                            // --- DEBUG PRINT: Show the result of the consecutive check ---
                            info!("[SBG] Consecutive negative count: {}", consecutive_negative_count);


                            if consecutive_negative_count == CONSECUTIVE_NEGATIVE_THRESHOLD {
                                let avg_slope_mpms =
                                    sum_of_recent_slopes / CONSECUTIVE_NEGATIVE_THRESHOLD as f32;

                                if avg_slope_mpms <= NO_MOVEMENT_RATE {
                                    let msg = messages_prost::radio::RadioFrame {
                                        node: messages_prost::common::Node::Phoenix.into(),
                                        payload: Some(
                                            messages_prost::radio::radio_frame::Payload::PhoenixEvent(
                                                Event::NoMovement.into(),
                                            ),
                                        ),
                                        millis_since_start: Instant::now().as_millis(),
                                    };
                                    msg.encode_length_delimited(&mut buf.as_mut()).unwrap();
                                    RADIO_CHANNEL.send(buf.clone()).await;
                                    SD_CHANNEL.send(("event.txt", buf)).await;
                                    if EVENT_CHANNEL
                                        .try_send(crate::state_machine::Events::NoMovement)
                                        .is_err()
                                    {
                                        // todo!("Log failure to radio");
                                    }
                                    break; // Exit loop once apogee is detected.
                                }
                            }
                        }
                    }
                }
            }
        }

        Timer::after_millis(100).await;
    }
}
