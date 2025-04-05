use crate::data_manager::DataManager;
use crate::types::COM_ID;
use common_arm::HydraError;
use defmt::{error, info};
use fdcan::{
    frame::{FrameFormat, TxFrameHeader},
    id::StandardId,
};
use mavlink::peek_reader::PeekReader;
use messages::{mavlink::uorocketry::MavMessage, RadioMessage};
use messages::mavlink::{self};
use postcard::from_bytes;

pub struct RadioDevice {
    transmitter: stm32h7xx_hal::serial::Tx<stm32h7xx_hal::pac::UART4>,
    pub receiver: PeekReader<stm32h7xx_hal::serial::Rx<stm32h7xx_hal::pac::UART4>>,
}

impl RadioDevice {
    pub fn new(uart: stm32h7xx_hal::serial::Serial<stm32h7xx_hal::pac::UART4>) -> Self {
        let (tx, mut rx) = uart.split();

        rx.listen();
        // setup interrupts

        RadioDevice {
            transmitter: tx,
            receiver: PeekReader::new(rx),
        }
    }
}

pub struct RadioManager {
    pub radio: RadioDevice,
    mav_sequence: u8,
}

impl RadioManager {
    pub fn new(radio: RadioDevice) -> Self {
        RadioManager {
            radio,
            mav_sequence: 0,
        }
    }
    pub fn send_message(&mut self, payload: &[u8]) -> Result<(), HydraError> {
        let mav_header = mavlink::MavHeader {
            system_id: 1,
            component_id: 1,
            sequence: self.increment_mav_sequence(),
        };
        // Create a fixed-size array and copy the payload into it
        let mut fixed_payload = [0u8; 255];
        let len = payload.len().min(255);
        fixed_payload[..len].copy_from_slice(&payload[..len]);

        let mav_message = mavlink::uorocketry::MavMessage::POSTCARD_MESSAGE(
            mavlink::uorocketry::POSTCARD_MESSAGE_DATA {
                message: fixed_payload,
            },
        );
        mavlink::write_versioned_msg(
            &mut self.radio.transmitter,
            mavlink::MavlinkVersion::V2,
            mav_header,
            &mav_message,
        )?;
        Ok(())
    }
    pub fn increment_mav_sequence(&mut self) -> u8 {
        self.mav_sequence = self.mav_sequence.wrapping_add(1);
        self.mav_sequence
    }
    pub fn receive_message<'a>(&mut self) -> Result<RadioMessage<'a>, HydraError> {
        let (_header, msg): (_, MavMessage) =
            mavlink::read_versioned_msg(&mut self.radio.receiver, mavlink::MavlinkVersion::V2)?;

        // info!("{:?}", );
        // Do we need the header?
        match msg {
            mavlink::uorocketry::MavMessage::POSTCARD_MESSAGE(msg) => {
                // let message_bytes = msg.message.clone();
                // let decoded_msg = postcard::from_bytes::<RadioMessage<'a>>(&message_bytes)?;
                let msg = postcard::from_bytes::<RadioMessage<'a>>(&(msg.message.clone()))?;

                Ok(msg)
                // weird Ok syntax to coerce to hydra error type.
            }
            mavlink::uorocketry::MavMessage::COMMAND_MESSAGE(command) => {
                info!("{}", command.command);
                Ok(postcard::from_bytes::<RadioMessage>(&command.command)?)
            }
            mavlink::uorocketry::MavMessage::HEARTBEAT(_) => {
                info!("Heartbeat");
                Err(mavlink::error::MessageReadError::Io.into())
            }
            _ => {
                error!("Error, ErrorContext::UnkownPostcardMessage");
                Err(mavlink::error::MessageReadError::Io.into())
            }
        }
    }
}
