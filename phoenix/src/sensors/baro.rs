use common_arm::drivers::ms5611::{Ms5611, OversamplingRatio};
use defmt::{error, info};
use embassy_stm32::gpio::Output;
use embassy_stm32::mode::Blocking;
use embassy_stm32::spi::Spi;
use embassy_time::Instant;
use embassy_time::{Delay, Duration, Timer};
use messages_prost::prost::Message;

use crate::resources::RADIO_CHANNEL;
use crate::resources::PRESSURE_CHANNEL;

#[embassy_executor::task]
pub async fn baro_reader_task(mut baro: Ms5611<Spi<'static, Blocking>, Output<'static>, Delay>) {
    info!("Barometer reader task started.");

    loop {
        match baro.read_pressure_temperature(OversamplingRatio::Osr4096) {
            Ok(reading) => {
                // pressure, temperature 

                PRESSURE_CHANNEL.try_send((reading.1, reading.0, 1, embassy_time::Instant::now()));
                let mut buf: [u8; 255] = [0; 255];
                let msg = messages_prost::radio::RadioFrame {
                    node: messages_prost::common::Node::Phoenix.into(),
                    payload: Some(messages_prost::radio::radio_frame::Payload::Barometer(
                        messages_prost::sensor::ms5611::Barometer {
                            pressure_kpa: reading.1,
                            temperature_celsius: reading.0
                        }
                    )),
                    millis_since_start: Instant::now().as_millis()
                };
                msg.encode_length_delimited(&mut buf.as_mut())
                    .expect("Failed to encode SBG GPS Position");
                RADIO_CHANNEL.send(buf).await;
            }
            Err(e) => {
                error!("Baro: Driver reading failed");
            }
        }
        Timer::after(Duration::from_millis(10)).await;
    }
}
