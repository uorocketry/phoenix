use defmt::*;
use embassy_executor::task;
use embassy_stm32::{mode, usart::RingBufferedUartRx, usart::UartTx};
use embassy_time::Delay;
use embedded_hal_1::delay::DelayNs;
use messages_prost::prost::Message;

use messages_prost::mavlink;
use messages_prost::mavlink::peek_reader::PeekReader;
use messages_prost::mavlink::uorocketry::MavMessage;
use messages_prost::radio::radio_frame::Payload;

use crate::resources::{RADIO_CHANNEL, RECOVERY_MANAGER};

#[task]
pub async fn radio_reader_task(mut rx: RingBufferedUartRx<'static>) {
    loop {
        let mut buf: [u8; crate::resources::RADIO_BUFFER_SIZE] =
            [0; crate::resources::RADIO_BUFFER_SIZE];
        if let Ok(len) = rx.read(&mut buf).await {
            if len > 0 {
                info!("Received {} bytes from radio: {:?}", len, &buf[..len]);
                let (_header, msg): (_, MavMessage) = mavlink::read_versioned_msg(
                    &mut PeekReader::new(&buf[..len]),
                    mavlink::MavlinkVersion::V2,
                )
                .unwrap();

                match msg {
                    mavlink::uorocketry::MavMessage::POSTCARD_MESSAGE(msg) => {
                        info!("Received postcard message");
                        if let Ok(recv) = messages_prost::radio::RadioFrame::decode_length_delimited(
                            &mut &msg.message[..],
                        ) {
                            if let Some(payload) = recv.payload {
                                match payload {
                                    Payload::Sbg(sbg_data) => {
                                        info!("Received SBG data: {:?}", sbg_data.data.is_some());
                                    }
                                    Payload::Gps(gps_data) => {
                                        info!("Received GPS data: {:?}", gps_data.data.len());
                                    }
                                    Payload::Madgwick(madgwick_data) => {
                                        info!(
                                            "Received Madgwick data: {:?}",
                                            madgwick_data.data.is_some()
                                        );
                                    }
                                    Payload::Iim20670(imu_data) => {
                                        info!("Received IMU data: {:?}", imu_data.data.is_some());
                                    }
                                    Payload::Log(log_data) => {
                                        info!("Received Log data: {:?}", log_data.level);
                                    }
                                    Payload::State(state_message) => {
                                        info!("Received State message: {:?}", state_message.state);
                                    }
                                    Payload::Command(command) => {
                                        info!("Received Command: {:?}", command.data.is_some());
                                        if let Some(command_data) = command.data {
                                            match command_data {
                                                messages_prost::command::command::Data::Ping(ping) => {
                                                    info!("Ping");
                                                    let mut msg_buf: [u8; 255] = [0; 255];
                                                    let msg = messages_prost::radio::RadioFrame {
                                                        node: messages_prost::common::Node::Phoenix.into(),
                                                        payload: Some(
                                                            messages_prost::radio::radio_frame::Payload::Command(
                                                                messages_prost::command::Command {
                                                                    node: 0,
                                                                    data: Some(
                                                                        messages_prost::command::command::Data::Pong(
                                                                            messages_prost::command::Pong { id: ping.id },
                                                                        ),
                                                                    ),
                                                                },
                                                            ),
                                                        ),
                                                    };
                                                    msg.encode_length_delimited(&mut msg_buf.as_mut())
                                                        .expect("Failed to encode SBG GPS Position");
                                                    RADIO_CHANNEL.send(msg_buf).await;
                                                }
                                                messages_prost::command::command::Data::Pong(_pong) => {
                                                    info!("Pong");
                                                }
                                                messages_prost::command::command::Data::Online(_online) => {}
                                                messages_prost::command::command::Data::DeployDrogue(_deploy_drogue) => {
                                                    RECOVERY_MANAGER.lock(|cell| {
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
                                                }
                                                messages_prost::command::command::Data::DeployMain(_deploy_main) => {
                                                    RECOVERY_MANAGER.lock(|cell| {
                                                        info!("Boom boom");
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
                                                }
                                                messages_prost::command::command::Data::PowerDown(_power_down) => {}
                                                messages_prost::command::command::Data::RadioRateChange(_rate_change) => {}
                                            }
                                        }
                                    }
                                }
                            }
                        } else {
                            info!("Failed to decode radio frame.");
                        }
                    }
                    mavlink::uorocketry::MavMessage::COMMAND_MESSAGE(_command) => {
                        info!("Received command");
                    }
                    mavlink::uorocketry::MavMessage::HEARTBEAT(_) => {
                        info!("Received heartbeat message.");
                    }
                    _ => {
                        info!("Unknown mavlink message.");
                    }
                }
            }
        }
    }
}

#[task]
pub async fn radio_writer_task(mut tx: UartTx<'static, mode::Async>) {
    loop {
        let data = RADIO_CHANNEL.receive().await;

        let mav_header = mavlink::MavHeader {
            system_id: 1,
            component_id: 1,
            sequence: 1,
        };
        let mav_message = mavlink::uorocketry::MavMessage::POSTCARD_MESSAGE(
            mavlink::uorocketry::POSTCARD_MESSAGE_DATA { message: data },
        );
        let _ = mavlink::write_versioned_msg_async(
            &mut tx,
            mavlink::MavlinkVersion::V2,
            mav_header,
            &mav_message,
        )
        .await;
    }
}
