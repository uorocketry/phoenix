use defmt::*;
use embassy_executor::task;
use embassy_stm32::{mode, usart::RingBufferedUartRx, usart::UartTx};
use embassy_time::Delay;
use embedded_hal_1::delay::DelayNs;
use messages_prost::prost::Message;

use crate::resources::{BUFFER_CHANNEL, RADIO_CHANNEL, SBG_CHANNEL};

#[task]
pub async fn sbg_parser_task(tx: UartTx<'static, mode::Async>) {
    let mut sbg = crate::sbg_manager::SBGManager::new(tx);
    loop {
        let full_buffer = BUFFER_CHANNEL.receive().await;
        sbg.sbg_device.read_data(&full_buffer.try_into().unwrap());
    }
}

#[task]
pub async fn sbg_uart_reader_task(mut rx: RingBufferedUartRx<'static>) {
    info!("SBG DMA reader task spawned.");
    loop {
        let mut buf: [u8; sbg_rs::sbg::SBG_BUFFER_SIZE] = [0; sbg_rs::sbg::SBG_BUFFER_SIZE];
        if let Ok(len) = rx.read(&mut buf).await {
            if len > 0 {
                let _ = BUFFER_CHANNEL.send(buf).await;
            }
        }
        Delay.delay_ms(100);
    }
}

#[task]
pub async fn sbg_receiver_task() {
    loop {
        let data = SBG_CHANNEL.receive().await;
        match data.data {
            Some(_x) => {
                let mut buf: [u8; 255] = [0; 255];
                let msg = messages_prost::radio::RadioFrame {
                    node: messages_prost::common::Node::Phoenix.into(),
                    payload: Some(messages_prost::radio::radio_frame::Payload::Sbg(data)),
                };
                msg.encode_length_delimited(&mut buf.as_mut())
                    .expect("Failed to encode SBG GPS Position");
                RADIO_CHANNEL.send(buf).await;
            }
            None => {
                info!("No SBG data received");
            }
        }
    }
}
