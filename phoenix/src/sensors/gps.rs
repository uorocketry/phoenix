use crate::resources::{Irqs, GPS_BUFFER_SIZE, RX_GPS_BUF};
use defmt::info;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::mode;
use embassy_stm32::peripherals::{DMA1_CH5, DMA1_CH6, PA4, PB2, PE0, PE1, UART8};
use embassy_stm32::usart::{Config, RingBufferedUartRx, Uart, UartTx};
use embassy_time::{Delay, Instant};
use embedded_hal_1::delay::DelayNs;
use messages_prost::gps::Gps;
use ublox::cfg_val::CfgVal;
use ublox::{
    CfgLayerSet, CfgPrtUartBuilder, CfgRstBuilder, CfgValSetBuilder, DataBits, InProtoMask,
    NavBbrMask, OutProtoMask, Parity, ResetMode, StopBits, UartMode, UartPortId,
};
use messages_prost::prost::Message;
use crate::resources::RADIO_CHANNEL;

pub async fn setup_gps(
    gps_enable: PA4,
    gps_reset: PB2,
    uart: UART8,
    rx: PE0,
    tx: PE1,
    tx_dma: DMA1_CH5,
    rx_dma: DMA1_CH6,
    irqs: Irqs,
) -> (RingBufferedUartRx<'static>, UartTx<'static, mode::Async>) {
    let mut gps_enable = Output::new(gps_enable, Level::High, Speed::Low);
    let mut gps_reset = Output::new(gps_reset, Level::High, Speed::Low);
    let mut uart_gps_config = Config::default();
    uart_gps_config.baudrate = 9600;
    uart_gps_config.data_bits = embassy_stm32::usart::DataBits::DataBits8;
    uart_gps_config.parity = embassy_stm32::usart::Parity::ParityNone;
    uart_gps_config.stop_bits = embassy_stm32::usart::StopBits::STOP1;
    uart_gps_config.detect_previous_overrun = false;

    let uart_gps = Uart::new(uart, rx, tx, irqs, tx_dma, rx_dma, uart_gps_config).unwrap();

    let (mut gps_tx, gps_rx) = uart_gps.split();
    let ring_gps_rx = gps_rx.into_ring_buffered(unsafe { &mut RX_GPS_BUF });
    gps_reset.set_low();
    Delay.delay_ms(3000);
    gps_reset.set_high();
    gps_enable.set_low();
    Delay.delay_ms(2000);
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
    gps_tx.write(&packet).await.expect("TODO: panic message");

    Delay.delay_ms(2000);

    let config_packet = CfgValSetBuilder {
        version: 1,
        // Save to RAM (to apply now) and BBR (to make it persistent)
        layers: CfgLayerSet::BBR | CfgLayerSet::RAM,
        reserved1: 0,
        cfg_data: &[
            // --- Port Settings ---
            CfgVal::Uart1Baudrate(38400),
            CfgVal::Uart1InProtUbx(true),
            CfgVal::Uart1InProtNmea(false), // Explicitly disable NMEA
            CfgVal::Uart1InProtRtcm3x(false),
            CfgVal::Uart1OutProtUbx(true),
            CfgVal::Uart1OutProtNmea(false), // Explicitly disable NMEA
            // --- Message Settings ---
            CfgVal::MsgOutUbxNavPvtUart1(1), // Enable NAV-PVT on UART1
            // --- Rate Settings ---
            CfgVal::RateMeas(200), // 200ms = 5Hz
            CfgVal::RateNav(1),    // Navigation rate = Measurement rate
        ],
    }
    .into_packet_vec();

    gps_tx
        .write(config_packet.as_slice())
        .await
        .expect("TODO: panic message");

    Delay.delay_ms(1000);

    let reset_packet = CfgRstBuilder {
        nav_bbr_mask: NavBbrMask::empty(), // .empty() preserves all BBR data
        reset_mode: ResetMode::ControlledSoftwareReset,
        reserved1: 0,
    }
    .into_packet_bytes();

    info!("Sending software reset to apply configuration");
    gps_tx.write(&reset_packet).await.unwrap();
    Delay.delay_ms(500); // Give module time to reset
    (ring_gps_rx, gps_tx)
}

#[embassy_executor::task]
pub async fn uart_gps_dma_reader_task(
    mut gps_rx: RingBufferedUartRx<'static>,
    gps_tx: UartTx<'static, mode::Async>,
) {
    info!("GPS reader task spawned.");
    loop {
        let mut buf_data: [u8; GPS_BUFFER_SIZE] = [0; GPS_BUFFER_SIZE];
        gps_rx.read(&mut buf_data).await;
        info!("GPS data read: {:?}", &buf_data[..]);
        let buf: [u8; GPS_BUFFER_SIZE] = [0; GPS_BUFFER_SIZE];
        let bytes: [u8; GPS_BUFFER_SIZE] = [0; GPS_BUFFER_SIZE];
        let mut buf: [u8; 255] = [0; 255];

        let msg = messages_prost::radio::RadioFrame {
            node: messages_prost::common::Node::Phoenix.into(),
            payload: Some(messages_prost::radio::radio_frame::Payload::Gps(
                Gps {
                    data: bytes.to_vec()
                },
            )),
            millis_since_start: Instant::now().as_millis()
        };
        msg.encode_length_delimited(&mut buf.as_mut()).unwrap();
        RADIO_CHANNEL.send(buf).await;
    }
}
