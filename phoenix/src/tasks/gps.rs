use defmt::*;
use embassy_executor::task;
use embassy_stm32::{mode, usart::RingBufferedUartRx, usart::UartTx};
use embedded_hal_1::delay::DelayNs;
use ublox::UbxPacketRequest;

#[task]
pub async fn uart_gps_dma_reader_task(
    mut gps_rx: RingBufferedUartRx<'static>,
    mut gps_tx: UartTx<'static, mode::Async>,
) {
    info!("GPS DMA reader task spawned.");
    let request = UbxPacketRequest::request_for::<ublox::NavPosLlh>().into_packet_bytes();
    loop {
        if crate::features::ENABLE_GPS_POLL {
            let _ = gps_tx.write(&request).await;
        }

        let mut buf_data: [u8; crate::resources::GPS_BUFFER_SIZE] =
            [0; crate::resources::GPS_BUFFER_SIZE];
        let _ = gps_rx.read(&mut buf_data).await;

        if crate::features::ENABLE_GPS_PARSE {
            // Minimal parser skeleton; extend as needed
            let mut ublox_buf: [u8; 256] = [0; 256];
            let linear = ublox::FixedLinearBuffer::new(&mut ublox_buf[..]);
            let mut parser = ublox::Parser::new(linear);
            let mut msgs = parser.consume_ubx(&buf_data);
            while let Some(msg) = msgs.next() {
                match msg {
                    Ok(ublox::PacketRef::NavPosLlh(x)) => {
                        info!("GPS lat: {}, lon: {}", x.lat_degrees(), x.lon_degrees());
                    }
                    Ok(_) => {}
                    Err(_) => info!("GPS parse error"),
                }
            }
        }

        embassy_time::Delay.delay_ms(1000);
    }
}

// Reference: Verbose GPS polling + parsing example that used to live in main.rs.
// Kept here to avoid cluttering main while preserving the original logic.
//
// let request =
//     UbxPacketRequest::request_for::<ublox::NavPosLlh>().into_packet_bytes();
// gps_tx.blocking_write(&request);
//
// loop {
//     let request =
//         UbxPacketRequest::request_for::<ublox::NavPosLlh>().into_packet_bytes();
//     gps_tx.blocking_write(&request);
//     // Delay.delay_ms(1000);
//     let mut buf_data: [u8; GPS_BUFFER_SIZE] = [0; GPS_BUFFER_SIZE];
//     ring_gps_rx.read(&mut buf_data).await;
//     info!("GPS data read: {:?}", &buf_data[..]);
//     // if let Ok(len) = gps_rx.read(&mut buf_data).await {
//             // info!("read");
//
//             Delay.delay_ms(1000);
//             // cortex_m::asm::delay(10_000);
//     let mut buf: [u8; GPS_BUFFER_SIZE] = [0; GPS_BUFFER_SIZE];
//     let bytes: [u8; GPS_BUFFER_SIZE] = [0; GPS_BUFFER_SIZE];
//
//     let mut nmea = nmea::Nmea::default();
//     let ascii_buf = unsafe {buf_data.as_ascii_unchecked()};
//     info!("BUFFER: {}", ascii_buf.as_str());
//     // if let Some(ascii_buf) = ascii_buf {
//         let res = nmea.parse(ascii_buf.as_str());
//
//         match res {
//             Ok(strings) => {
//                 info!("Result: {}", strings.as_str());
//             }
//             _ => {
//                 info!("nmea parser none found");
//             }
//         }
//
//     // } else {
//     //     info!("No valid sentence");
//     // }
//
//     let buf: ublox::FixedLinearBuffer<'_> = ublox::FixedLinearBuffer::new(&mut buf[..]);
//     let mut parser = ublox::Parser::new(buf);
//     // info!("GPS Parser initialized.");
//     let mut msgs = parser.consume_ubx(&buf_data);
//     info!("GPS Messages consumed. {}", msgs.next().is_some());
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
//                 ublox::PacketRef::Unknown(_msg) => {
//                     info!("Unknown GPS type.");
//                 }
//                 _ => {
//                     info!("GPS Message not handled.");
//                 }
//             },
//             Err(_e) => {
//                 info!("GPS parse Error");
//             }
//         }
//     }
//     // }
// }
