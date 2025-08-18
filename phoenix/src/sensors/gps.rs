// use crate::resources::{Irqs, GPS_BUFFER_SIZE, RX_GPS_BUF, SD_CHANNEL};
// use defmt::info;
// use embassy_stm32::gpio::{Level, Output, Speed};
// use embassy_stm32::mode;
// use embassy_stm32::peripherals::{DMA1_CH5, DMA1_CH6, PA4, PB2, PE0, PE1, UART8};
// use embassy_stm32::usart::{Config, RingBufferedUartRx, Uart, UartTx};
// use embassy_time::{Delay, Instant};
// use embedded_hal_1::delay::DelayNs;
// use messages_prost::gps::Gps;
// use ublox::cfg_val::CfgVal;
// use ublox::{
//     CfgLayerSet, CfgPrtUartBuilder, CfgRstBuilder, CfgValSetBuilder, DataBits, InProtoMask,
//     NavBbrMask, OutProtoMask, Parity, ResetMode, StopBits, UartMode, UartPortId, UbxPacketRequest,
// };
// use messages_prost::prost::Message;
// use crate::resources::RADIO_CHANNEL;

// pub async fn setup_gps(
//     gps_enable: PA4,
//     gps_reset: PB2,
//     uart: UART8,
//     rx: PE0,
//     tx: PE1,
//     tx_dma: DMA1_CH5,
//     rx_dma: DMA1_CH6,
//     irqs: Irqs,
// ) -> (RingBufferedUartRx<'static>, UartTx<'static, mode::Async>) {
//     let mut gps_enable = Output::new(gps_enable, Level::High, Speed::Low);
//     let mut gps_reset = Output::new(gps_reset, Level::High, Speed::Low);
//     let mut uart_gps_config = Config::default();
//     uart_gps_config.baudrate = 9600;
//     uart_gps_config.data_bits = embassy_stm32::usart::DataBits::DataBits8;
//     uart_gps_config.parity = embassy_stm32::usart::Parity::ParityNone;
//     uart_gps_config.stop_bits = embassy_stm32::usart::StopBits::STOP1;
//     uart_gps_config.detect_previous_overrun = false;

//     let uart_gps = Uart::new(uart, rx, tx, irqs, tx_dma, rx_dma, uart_gps_config).unwrap();

//     let (mut gps_tx, gps_rx) = uart_gps.split();
//     let ring_gps_rx = gps_rx.into_ring_buffered(unsafe { &mut RX_GPS_BUF });
//     gps_reset.set_low();
//     Delay.delay_ms(3000);
//     gps_reset.set_high();
//     gps_enable.set_low();
//     Delay.delay_ms(2000);
//     let packet: [u8; 28] = CfgPrtUartBuilder {
//         portid: UartPortId::Uart1,
//         reserved0: 0,
//         tx_ready: 0,
//         mode: UartMode::new(DataBits::Eight, Parity::None, StopBits::One),
//         baud_rate: 9600,
//         in_proto_mask: InProtoMask::all(),
//         out_proto_mask: OutProtoMask::UBLOX,
//         flags: 0,
//         reserved5: 0,
//     }
//     .into_packet_bytes();

//     info!("Sending GPS packet: {:?}", &packet);
//     gps_tx.write(&packet).await.expect("TODO: panic message");

//     // Delay.delay_ms(2000);

//     // let config_packet = CfgValSetBuilder {
//     //     version: 1,
//     //     // Save to RAM (to apply now) and BBR (to make it persistent)
//     //     layers: CfgLayerSet::BBR | CfgLayerSet::RAM,
//     //     reserved1: 0,
//     //     cfg_data: &[
//     //         // --- Port Settings ---
//     //         CfgVal::Uart1Baudrate(38400),
//     //         CfgVal::Uart1InProtUbx(true),
//     //         CfgVal::Uart1InProtNmea(false), // Explicitly disable NMEA
//     //         CfgVal::Uart1InProtRtcm3x(false),
//     //         CfgVal::Uart1OutProtUbx(true),
//     //         CfgVal::Uart1OutProtNmea(false), // Explicitly disable NMEA
//     //         // --- Message Settings ---
//     //         CfgVal::MsgOutUbxNavPvtUart1(1), // Enable NAV-PVT on UART1
//     //         // --- Rate Settings ---
//     //         CfgVal::RateMeas(200), // 200ms = 5Hz
//     //         CfgVal::RateNav(1),    // Navigation rate = Measurement rate
//     //     ],
//     // }
//     // .into_packet_vec();

//     // gps_tx
//     //     .write(config_packet.as_slice())
//     //     .await
//     //     .expect("TODO: panic message");

//     // Delay.delay_ms(1000);

//     // let reset_packet = CfgRstBuilder {
//     //     nav_bbr_mask: NavBbrMask::empty(), // .empty() preserves all BBR data
//     //     reset_mode: ResetMode::ControlledSoftwareReset,
//     //     reserved1: 0,
//     // }
//     // .into_packet_bytes();

//     // info!("Sending software reset to apply configuration");
//     // gps_tx.write(&reset_packet).await.unwrap();
//     // Delay.delay_ms(500); // Give module time to reset
//     (ring_gps_rx, gps_tx)
// }

// #[embassy_executor::task]
// pub async fn uart_gps_dma_reader_task(
//     mut gps_rx: RingBufferedUartRx<'static>,
//     mut gps_tx: UartTx<'static, mode::Async>,
// ) {
//     info!("GPS reader task spawned.");
//     loop {
//         let request =
//             ublox::UbxPacketRequest::request_for::<ublox::NavPosLlh>().into_packet_bytes();
//         info!("gps write");
//         gps_tx.write(&request).await;
//         let mut buf_data: [u8; GPS_BUFFER_SIZE] = [0; GPS_BUFFER_SIZE];
//         info!("gps read");
//         gps_rx.read(&mut buf_data).await;
//         info!("GPS data read: {:?}", &buf_data[..]);
//         let buf: [u8; GPS_BUFFER_SIZE] = [0; GPS_BUFFER_SIZE];
//         let bytes: [u8; GPS_BUFFER_SIZE] = [0; GPS_BUFFER_SIZE];
//         let mut buf: [u8; 255] = [0; 255];
//         info!("gps sent");
//         let msg = messages_prost::radio::RadioFrame {
//             node: messages_prost::common::Node::Phoenix.into(),
//             payload: Some(messages_prost::radio::radio_frame::Payload::Gps(
//                 Gps {
//                     data: bytes.to_vec()
//                 },
//             )),
//             millis_since_start: Instant::now().as_millis()
//         };
//         msg.encode_length_delimited(&mut buf.as_mut()).unwrap();
//         info!("Radio channel");
//         RADIO_CHANNEL.send(buf.clone()).await;
//         info!("sd c");
//         SD_CHANNEL.send(("gps.txt", buf)).await;
//     }
// }


use crate::resources::{Irqs, GPS_BUFFER_SIZE, RX_GPS_BUF, SD_CHANNEL, RADIO_CHANNEL};
use defmt::info;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::peripherals::{DMA1_CH5, DMA1_CH6, PA4, PB2, PE0, PE1, UART8};
use embassy_stm32::usart::{Config, RingBufferedUartRx, Uart, UartTx};
use embassy_stm32::mode;
use embassy_time::{Delay, Instant};
use embedded_hal_1::delay::DelayNs;
use messages_prost::gps::Gps;
use messages_prost::prost::Message;
use ublox::cfg_val::{CfgKey, CfgVal};
use ublox::nav_pvt_proto14::NavPvt;
use ublox::PacketRef;
use ublox::{CfgLayerSet, CfgRstBuilder, CfgValSetBuilder, ResetMode, UbxPacketRequest, NavBbrMask};

/// #
/// # Setup and configure the U-BLOX GPS Module
/// #
/// This function performs a hardware reset, then configures the module to:
/// 1. Use only the UBX protocol on its UART1 interface.
/// 2. Set the measurement rate to 1 Hz.
/// 3. Automatically stream NAV-PVT messages at 1 Hz.
/// 4. Save this configuration to both RAM and Battery-Backed RAM (BBR) for persistence.
/// 5. Perform a controlled software reset to apply the new configuration.
///
pub async fn setup_gps(
    gps_enable: PA4,
    gps_reset: PB2,
    uart: UART8,
    rx: PE0,
    tx: PE1,
    tx_dma: DMA1_CH5,
    rx_dma: DMA1_CH6,
    irqs: crate::resources::Irqs,
) -> (RingBufferedUartRx<'static>, UartTx<'static, mode::Async>) {
    // --- Initialize GPIOs and UART ---
    let mut gps_enable = Output::new(gps_enable, Level::High, Speed::Low);
    let mut gps_reset = Output::new(gps_reset, Level::High, Speed::Low);
    
    let mut uart_gps_config = Config::default();
    uart_gps_config.baudrate = 9600; // Default baud rate for the module
    // Other UART settings are correct by default (8N1)

    let uart_gps = Uart::new(uart, rx, tx, irqs, tx_dma, rx_dma, uart_gps_config).unwrap();
    let (mut gps_tx, gps_rx) = uart_gps.split();
    let ring_gps_rx = gps_rx.into_ring_buffered(unsafe { &mut RX_GPS_BUF });

    // --- Hardware Reset Sequence ---
    // This ensures the module is in a known state.
    // Note: A hard reset like this will clear BBR if V_BCKP is not supplied.
    info!("Performing hardware reset on GPS module...");
    gps_reset.set_low();
    Delay.delay_ms(500); // Keep reset low for a moment
    gps_reset.set_high();
    gps_enable.set_low(); // Enable the module
    Delay.delay_ms(1000); // Give the module time to boot up
    info!("GPS module reset complete.");

    // --- Build Configuration Packet using CFG-VALSET ---
    // This is the modern and recommended way to configure M10 series modules.
    // We will configure the module to stream NAV-PVT messages at 1Hz.
    let config_packet = CfgValSetBuilder {
        version: 1,
        // Save to RAM (to apply now) and BBR (to make it persistent across power cycles)
        layers: CfgLayerSet::RAM | CfgLayerSet::BBR,
        reserved1: 0,
        cfg_data: &[
            // --- Port Settings: Disable NMEA, Enable UBX ---
            CfgVal::Uart1InProtUbx(true),
            CfgVal::Uart1InProtNmea(false),
            CfgVal::Uart1OutProtUbx(true),
            CfgVal::Uart1OutProtNmea(false),
            
            // --- Message Settings: Enable NAV-PVT on UART1 ---
            // The '1' means the message will be sent once per navigation solution.
            CfgVal::MsgOutUbxNavPvtUart1(1),

            // --- Rate Settings: Set navigation rate to 1 Hz ---
            CfgVal::RateMeas(1000), // 1000ms = 1Hz
            CfgVal::RateNav(1),     // Navigation rate matches measurement rate
        ],
    }
    .into_packet_vec();

    info!("Sending GPS configuration packet...");
    gps_tx.write(&config_packet).await.expect("Failed to send GPS config");
    Delay.delay_ms(250); // Give module time to process the config

    // --- Software Reset to Apply Configuration ---
    // A controlled software reset is needed to apply the settings we just sent.
    // This reset mode does not clear the BBR, so our saved config is safe.
    let reset_packet = CfgRstBuilder {
        nav_bbr_mask: NavBbrMask::empty(), // .empty() preserves all BBR data
        reset_mode: ResetMode::ControlledSoftwareReset,
        reserved1: 0,
    }
    .into_packet_bytes();

    info!("Sending software reset to apply configuration...");
    gps_tx.write(&reset_packet).await.unwrap();
    Delay.delay_ms(500); // Give module time to reset and apply settings
    
    info!("GPS setup complete. Module is now streaming NAV-PVT packets.");

    (ring_gps_rx, gps_tx)
}

/// #
/// # GPS Reader and Parser Task
/// #
/// This task continuously reads from the UART and feeds the bytes into a UBX parser.
/// It no longer polls the device. Instead, it waits for the automatically streamed
/// NAV-PVT packets, parses them, and then acts on the valid data.
///
#[embassy_executor::task]
pub async fn uart_gps_dma_reader_task(
    mut gps_rx: RingBufferedUartRx<'static>,
    mut _gps_tx: UartTx<'static, mode::Async>, // Renamed to indicate it's not used here
) {
    info!("GPS reader task spawned.");
    let mut parser_buf: [u8; GPS_BUFFER_SIZE] = [0; GPS_BUFFER_SIZE];
    let mut read_buf: [u8; GPS_BUFFER_SIZE] = [0; GPS_BUFFER_SIZE];
    
    // Create a parser to handle the incoming UBX byte stream
    let buf = ublox::FixedLinearBuffer::new(&mut parser_buf[..]);
    let mut parser = ublox::Parser::new(buf);
    loop {
        // Wait for and read incoming data from the GPS module
        let result = gps_rx.read(&mut read_buf).await;
        info!("read");
        if let Ok(bytes_read) = result {
            if bytes_read > 0 {
                // Feed the received bytes into the parser
                let mut it = parser.consume_ubx(&read_buf[..bytes_read]);
                
                // Iterate through any fully parsed packets
                while let Some(packet) = it.next() {
                    match packet {
                        Ok(PacketRef::NavPvt(pvt)) => {
                            // We successfully parsed a NAV-PVT packet!
                            // Now you can use the structured data.
                            info!("Received and Parsed NAV-PVT:");
                            info!("  Timestamp: {:?}-{:?}-{:?} {:?}:{:?}:{:?} UTC", pvt.year(), pvt.month(), pvt.day(), pvt.hour(), pvt.min(), pvt.sec());
                            info!("  Fix Sats: {:?}, Fix Type: {:?}", pvt.num_satellites(), pvt.fix_type() as u8);
                            info!("  Coords (deg): lon={}, lat={}", pvt.longitude() as f32 * 1e-7, pvt.latitude() as f32 * 1e-7);

                            // TODO: Adapt this section to your specific needs.
                            // Instead of sending the raw buffer, you should now create your
                            // Gps protobuf message using the parsed data from `pvt`.
                            // For example:
                            // let gps_data = Gps {
                            //     latitude: pvt.lat(),
                            //     longitude: pvt.lon(),
                            //     num_satellites: pvt.num_sv() as u32,
                            //     // ... other fields
                            // };
                            
                            // For demonstration, we'll just log. You can serialize `gps_data`
                            // and send it over your channels here.
                        }
                        Ok(packet) => {
                            // Handle other packet types if you need them
                            info!("Received other UBX packet: {:?}", packet.class_and_msg_id());
                        }
                        Err(e) => {
                            // This can happen if the parser encounters invalid data
                            defmt::warn!("GPS parser error");
                        }
                    }
                }
            }
        } else {
            defmt::error!("GPS read error");
        }
    }
}
