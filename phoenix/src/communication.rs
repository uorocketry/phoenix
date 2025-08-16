use crate::resources::{RADIO_BUFFER_SIZE, RADIO_CHANNEL, RX_RADIO_BUF};
use crate::RECOVERY_MANAGER;
use defmt::info;
use embassy_stm32::mode;
use embassy_stm32::mode::Async;
use embassy_stm32::peripherals::{DMA2_CH3, DMA2_CH5, PE7, PE8, UART7};
use embassy_stm32::usart::{RingBufferedUartRx, Uart, UartTx};
use embassy_time::Instant;
use messages_prost::mavlink;
use messages_prost::mavlink::peek_reader::PeekReader;
use messages_prost::prost::Message;
use messages_prost::radio::radio_frame::Payload;

pub fn init_radio(
    uart: UART7,
    rx: PE7,
    tx: PE8,
    tx_dma: DMA2_CH3,
    rx_dma: DMA2_CH5,
    irqs: crate::Irqs,
) -> (UartTx<'static, Async>, RingBufferedUartRx<'static>) {
    let mut uart_radio_config = embassy_stm32::usart::Config::default();
    uart_radio_config.baudrate = 57600;
    uart_radio_config.data_bits = embassy_stm32::usart::DataBits::DataBits8;
    uart_radio_config.parity = embassy_stm32::usart::Parity::ParityNone;
    uart_radio_config.stop_bits = embassy_stm32::usart::StopBits::STOP1;

    let uart_radio = Uart::new(uart, rx, tx, irqs, tx_dma, rx_dma, uart_radio_config).unwrap();

    let (radio_tx, radio_rx) = uart_radio.split();
    let radio_ring_rx = radio_rx.into_ring_buffered(unsafe { &mut RX_RADIO_BUF });

    (radio_tx, radio_ring_rx)
}

#[embassy_executor::task]
pub async fn radio_reader_task(mut rx: RingBufferedUartRx<'static>) {
    loop {
        let mut buf: [u8; RADIO_BUFFER_SIZE] = [0; RADIO_BUFFER_SIZE];
        if let Ok(len) = rx.read(&mut buf).await {
            if len > 0 {
                // Process the received data
                info!("Received {} bytes from radio: {:?}", len, &buf[..len]);
                if let Ok((_header, msg)) = mavlink::read_versioned_msg(
                    &mut PeekReader::new(&buf[..len]),
                    mavlink::MavlinkVersion::V2,
                ) {
                    match msg {
                        mavlink::uorocketry::MavMessage::POSTCARD_MESSAGE(msg) => {
                            info!("Received postcard message");
                            // decode the msg
                            if let Ok(recv) =
                                messages_prost::radio::RadioFrame::decode_length_delimited(
                                    &mut &msg.message[..],
                                )
                            {
                                // info!("Received radio frame: {:?}", recv.node);
                                if let Some(payload) = recv.payload {
                                    match payload {
                                        Payload::ArgusTemperature(_) => {
                                            
                                        }
                                        Payload::ArgusStrain(_) => {

                                        }
                                        Payload::ArgusPressure(_) => {

                                        }
                                        Payload::ArgusEvent(_) => {

                                        }
                                        Payload::ArgusState(_) => {

                                        }
                                        Payload::Sbg(sbg_data) => {
                                            info!(
                                                "Received SBG data: {:?}",
                                                sbg_data.data.is_some()
                                            );
                                        }
                                        Payload::Gps(gps_data) => {
                                            info!("Received GPS data: {:?}", gps_data.data.len());
                                            // Handle GPS data
                                        }
                                        Payload::Madgwick(madgwick_data) => {
                                            info!(
                                                "Received Madgwick data: {:?}",
                                                madgwick_data.data.is_some()
                                            );
                                            // Handle Madgwick data
                                        }
                                        Payload::Iim20670(imu_data) => {
                                            info!(
                                                "Received IMU data: {:?}",
                                                imu_data.data.is_some()
                                            );
                                            // Handle IMU data
                                        }
                                        Payload::Log(log_data) => {
                                            info!("Received Log data: {:?}", log_data.level);
                                            // Handle Log data
                                        }
                                        Payload::PhoenixState(state) => {
                                            info!("Received State message: {:?}", state);
                                            // Handle State message
                                        }
                                        Payload::PhoenixEvent(event) => {
                                            
                                        }
                                        Payload::Barometer(barometer_data) => {
                                            // Handle Barometer data
                                        }
                                        Payload::Command(command) => {
                                            info!("Received Command: {:?}", command.data.is_some());
                                            if let Some(command_data) = command.data {
                                                match command_data {
                                                    messages_prost::command::command::Data::PowerUpCamera(power_up_camera) => {
                                                        info!("Powering up camera");
                                                        todo!("Powering up camera not implemented yet");
                                                    }
                                                    messages_prost::command::command::Data::PowerDownCamera(power_down_camera) => {
                                                        info!("Powering down camera");
                                                        todo!("Powering down camera not implemented yet");
                                                    }
                                                    messages_prost::command::command::Data::Ping(ping) => {
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
                                                            )),
                                                            millis_since_start: Instant::now().as_millis()
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
                                                                recovery_manager.arm();
                                                                recovery_manager.fire_drogue();
                                                                recovery_manager.disarm();
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
                                                                recovery_manager.arm();
                                                                recovery_manager.fire_main();
                                                                recovery_manager.disarm();
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
        }
        // Timer::after(Duration::from_millis(100)).await;
    }
}

#[embassy_executor::task]
pub async fn radio_writer_task(mut tx: UartTx<'static, mode::Async>) {
    let mut sequence = 0; 

    loop {
        let data = RADIO_CHANNEL.receive().await;

        let mav_header = mavlink::MavHeader {
            system_id: 1,
            component_id: 1,
            sequence,
        };

        let mav_message = mavlink::uorocketry::MavMessage::POSTCARD_MESSAGE(
            mavlink::uorocketry::POSTCARD_MESSAGE_DATA { message: data },
        );
        match mavlink::write_versioned_msg_async(
            &mut tx,
            mavlink::MavlinkVersion::V2,
            mav_header,
            &mav_message,
        )
        .await {
            Ok(bytes) => {
                sequence = sequence.wrapping_add(1);
            }
            _ => {
                info!("Failed to write");
            }
        }


    }
}
