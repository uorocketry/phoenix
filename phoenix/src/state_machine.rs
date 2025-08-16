use defmt::info;
use embassy_executor::Spawner;
use embassy_time::Instant;
use messages_prost::phoenix_state::{Event, State};
use smlang::statemachine;
use messages_prost::prost::Message;

use crate::resources::{EVENT_CHANNEL, PRESSURE_CHANNEL, RADIO_CHANNEL, RECOVERY_MANAGER, SD_CHANNEL};

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
                    millis_since_start: Instant::now().as_millis()
                };
                msg.encode_length_delimited(&mut buf.as_mut()).unwrap();
                RADIO_CHANNEL.send(buf.clone()).await;
                SD_CHANNEL.send(("state.txt", buf)).await;
                info!("Ascent");
            }
            States::Fault => {
                let mut buf: [u8; 255] = [0; 255];

                let msg = messages_prost::radio::RadioFrame {
                    node: messages_prost::common::Node::Phoenix.into(),
                    payload: Some(messages_prost::radio::radio_frame::Payload::PhoenixState(
                        State::Fault.into(),
                    )),
                    millis_since_start: Instant::now().as_millis()
                };
                msg.encode_length_delimited(&mut buf.as_mut()).unwrap();
                RADIO_CHANNEL.send(buf.clone()).await;
                SD_CHANNEL.send(("state.txt", buf)).await;
                info!("Fault");
            }
            States::Init => {
                let mut buf: [u8; 255] = [0; 255];

                let msg = messages_prost::radio::RadioFrame {
                    node: messages_prost::common::Node::Phoenix.into(),
                    payload: Some(messages_prost::radio::radio_frame::Payload::PhoenixState(
                        State::Init.into(),
                    )),
                    millis_since_start: Instant::now().as_millis()
                };
                msg.encode_length_delimited(&mut buf.as_mut()).unwrap();
                RADIO_CHANNEL.send(buf.clone()).await;
                SD_CHANNEL.send(("state.txt", buf)).await; 

                let mut should_start = false; 
                // await both channels to be armed. 
                RECOVERY_MANAGER.lock(|cell| {
                    if let Some(recovery_manager) = cell.borrow_mut().as_mut() {
                        if recovery_manager.is_armed() {
                            // this could be it's own task this has the posibility to be bad since 
                            // it's just evaluated for the first arming, if disarmed it will still be potentionally live. 
                            should_start = true; 
                        }

                    }
                });

                if should_start {
                    EVENT_CHANNEL.send(Events::Start).await;
                    
                    let msg = messages_prost::radio::RadioFrame {
                        node: messages_prost::common::Node::Phoenix.into(),
                        payload: Some(messages_prost::radio::radio_frame::Payload::PhoenixEvent(
                            Event::Start.into(),
                        )),
                        millis_since_start: Instant::now().as_millis()
                    };

                    msg.encode_length_delimited(&mut buf.as_mut()).unwrap();
                    RADIO_CHANNEL.send(buf.clone()).await;
                    SD_CHANNEL.send(("event.txt", buf)).await; 
                }
                info!("Init");
            }
            States::WaitForLaunch => {
                let mut buf: [u8; 255] = [0; 255];

                let msg = messages_prost::radio::RadioFrame {
                    node: messages_prost::common::Node::Phoenix.into(),
                    payload: Some(messages_prost::radio::radio_frame::Payload::PhoenixState(
                        State::WaitForLaunch.into(),
                    )),
                    millis_since_start: Instant::now().as_millis()
                };
                msg.encode_length_delimited(&mut buf.as_mut()).unwrap();
                RADIO_CHANNEL.send(buf.clone()).await;
                SD_CHANNEL.send(("state.txt", buf)).await; 
                info!("Wait For Launch");
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
                    millis_since_start: Instant::now().as_millis()
                };

                msg.encode_length_delimited(&mut buf.as_mut()).unwrap();
                RADIO_CHANNEL.send(buf.clone()).await;
                SD_CHANNEL.send(("state.txt", buf)).await; 

                let msg = messages_prost::radio::RadioFrame {
                    node: messages_prost::common::Node::Phoenix.into(),
                    payload: Some(messages_prost::radio::radio_frame::Payload::PhoenixEvent(
                        Event::DrogueDeployment.into(),
                    )),
                    millis_since_start: Instant::now().as_millis()
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
                    millis_since_start: Instant::now().as_millis()
                };
                msg.encode_length_delimited(&mut buf.as_mut()).unwrap();
                RADIO_CHANNEL.send(buf.clone()).await;
                SD_CHANNEL.send(("state.txt", buf)).await; 

                let (altitude, temperature, sender, timestamp)  = PRESSURE_CHANNEL.receive().await; 
            
                // sbg data
                if sender == 0 {
                    if altitude >= crate::recovery::MAIN_HEIGHT {
                        RECOVERY_MANAGER.lock(|cell| {
                            if let Some(recovery_manager) = cell.borrow_mut().as_mut() {
                                recovery_manager.fire_main();
                            }
                        });

                        EVENT_CHANNEL.send(Events::MainDeployment).await; 
                    

                        let msg = messages_prost::radio::RadioFrame {
                            node: messages_prost::common::Node::Phoenix.into(),
                            payload: Some(messages_prost::radio::radio_frame::Payload::PhoenixEvent(
                                Event::MainDeployment.into(),
                            )),
                            millis_since_start: Instant::now().as_millis()
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
                    millis_since_start: Instant::now().as_millis()
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
            }
        }
    }
}
