use defmt::*;
use embassy_stm32::{mode, usart::UartTx};
use embassy_time::Delay;
use embedded_hal_1::delay::DelayNs;
use ublox::cfg_val::CfgVal;
use ublox::CfgLayerSet;
use ublox::{CfgRstBuilder, CfgValSetBuilder, NavBbrMask, ResetMode};

pub async fn configure_ublox(gps_tx: &mut UartTx<'static, mode::Async>) {
    if crate::features::ENABLE_GPS_CFG_BBR_RAM {
        let config_packet = CfgValSetBuilder {
            version: 1,
            layers: CfgLayerSet::BBR | CfgLayerSet::RAM,
            reserved1: 0,
            cfg_data: &[
                CfgVal::Uart1Baudrate(38400),
                CfgVal::Uart1InProtUbx(true),
                CfgVal::Uart1InProtNmea(false),
                CfgVal::Uart1InProtRtcm3x(false),
                CfgVal::Uart1OutProtUbx(true),
                CfgVal::Uart1OutProtNmea(false),
                CfgVal::MsgOutUbxNavPvtUart1(1),
                CfgVal::RateMeas(200),
                CfgVal::RateNav(1),
            ],
        }
        .into_packet_vec();

        gps_tx
            .write(config_packet.as_slice())
            .await
            .expect("ublox cfg write failed");
    }

    Delay.delay_ms(1000);

    if crate::features::ENABLE_GPS_SOFT_RESET {
        let reset_packet = CfgRstBuilder {
            nav_bbr_mask: NavBbrMask::empty(),
            reset_mode: ResetMode::ControlledSoftwareReset,
            reserved1: 0,
        }
        .into_packet_bytes();

        info!("Sending software reset to apply configuration");
        gps_tx.write(&reset_packet).await.unwrap();
        Delay.delay_ms(500);
    }
}
