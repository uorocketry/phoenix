use defmt::*;
use embassy_stm32::adc::Adc;
use embassy_stm32::gpio::{Input, Level, Output, OutputType, Pull, Speed};
use embassy_stm32::mode;
use embassy_stm32::peripherals;
use embassy_stm32::spi::{Config as SpiConfig, Spi};
use embassy_stm32::time::{khz, mhz};
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};
use embassy_stm32::usart::{Config as UartConfig, RingBufferedUartRx, Uart, UartTx};
use embassy_time::Delay;
use embedded_hal_1::delay::DelayNs;

use common_arm::drivers::ms5611::Ms5611;

use crate::drivers::recovery::{Arming, Fire, RecoveryManager};
use crate::resources::{Irqs, RX_GPS_BUF, RX_RADIO_BUF, RX_SBG_BUF};

pub struct Board {
    // SBG UART4 + pins + DMA
    uart4: Option<peripherals::UART4>,
    pa1: Option<peripherals::PA1>,
    pa0: Option<peripherals::PA0>,
    dma1_ch1: Option<peripherals::DMA1_CH1>,
    dma1_ch0: Option<peripherals::DMA1_CH0>,

    // SBG power
    pd8: Option<peripherals::PD8>,

    // Baro SPI4 + pins + CS
    spi4: Option<peripherals::SPI4>,
    pe2: Option<peripherals::PE2>,
    pe6: Option<peripherals::PE6>,
    pe5: Option<peripherals::PE5>,
    pb8: Option<peripherals::PB8>,

    // GPS UART8 + pins + DMA + enable/reset
    uart8: Option<peripherals::UART8>,
    pe0: Option<peripherals::PE0>,
    pe1: Option<peripherals::PE1>,
    dma1_ch5: Option<peripherals::DMA1_CH5>,
    dma1_ch6: Option<peripherals::DMA1_CH6>,
    pa4: Option<peripherals::PA4>,
    pb2: Option<peripherals::PB2>,

    // Radio UART7 + pins + DMA
    uart7: Option<peripherals::UART7>,
    pe7: Option<peripherals::PE7>,
    pe8: Option<peripherals::PE8>,
    dma2_ch3: Option<peripherals::DMA2_CH3>,
    dma2_ch5: Option<peripherals::DMA2_CH5>,

    // Camera triggers
    pe14: Option<peripherals::PE14>,
    pe12: Option<peripherals::PE12>,

    // Buzzer TIM3 + PC6
    tim3: Option<peripherals::TIM3>,
    pc6: Option<peripherals::PC6>,

    // IMU SPI3 + pins
    spi3: Option<peripherals::SPI3>,
    pc10: Option<peripherals::PC10>,
    pb5: Option<peripherals::PB5>,
    pb4: Option<peripherals::PB4>,
    pc0: Option<peripherals::PC0>,
    pb6: Option<peripherals::PB6>,
    pd4: Option<peripherals::PD4>,

    // LED
    pb14: Option<peripherals::PB14>,

    // SD: SPI1 + pins + CS
    spi1: Option<peripherals::SPI1>,
    pa5: Option<peripherals::PA5>,
    pa7: Option<peripherals::PA7>,
    pa6: Option<peripherals::PA6>,
    pe9: Option<peripherals::PE9>,

    // Recovery GPIO + ADC
    pc1: Option<peripherals::PC1>,
    pd6: Option<peripherals::PD6>,
    pd14: Option<peripherals::PD14>,
    pc11: Option<peripherals::PC11>,
    pd2: Option<peripherals::PD2>,
    pd5: Option<peripherals::PD5>,
    pd13: Option<peripherals::PD13>,
    pc12: Option<peripherals::PC12>,
    pd1: Option<peripherals::PD1>,
    pa2: Option<peripherals::PA2>,
    pb0: Option<peripherals::PB0>,
    pa3: Option<peripherals::PA3>,
    pc5: Option<peripherals::PC5>,
    adc1: Option<peripherals::ADC1>,
}

impl Board {
    pub fn new(config: embassy_stm32::Config) -> Self {
        let p = embassy_stm32::init(config);
        Self {
            uart4: Some(p.UART4),
            pa1: Some(p.PA1),
            pa0: Some(p.PA0),
            dma1_ch1: Some(p.DMA1_CH1),
            dma1_ch0: Some(p.DMA1_CH0),

            pd8: Some(p.PD8),

            spi4: Some(p.SPI4),
            pe2: Some(p.PE2),
            pe6: Some(p.PE6),
            pe5: Some(p.PE5),
            pb8: Some(p.PB8),

            uart8: Some(p.UART8),
            pe0: Some(p.PE0),
            pe1: Some(p.PE1),
            dma1_ch5: Some(p.DMA1_CH5),
            dma1_ch6: Some(p.DMA1_CH6),
            pa4: Some(p.PA4),
            pb2: Some(p.PB2),

            uart7: Some(p.UART7),
            pe7: Some(p.PE7),
            pe8: Some(p.PE8),
            dma2_ch3: Some(p.DMA2_CH3),
            dma2_ch5: Some(p.DMA2_CH5),

            pe14: Some(p.PE14),
            pe12: Some(p.PE12),

            tim3: Some(p.TIM3),
            pc6: Some(p.PC6),

            spi3: Some(p.SPI3),
            pc10: Some(p.PC10),
            pb5: Some(p.PB5),
            pb4: Some(p.PB4),
            pc0: Some(p.PC0),
            pb6: Some(p.PB6),
            pd4: Some(p.PD4),

            pb14: Some(p.PB14),

            spi1: Some(p.SPI1),
            pa5: Some(p.PA5),
            pa7: Some(p.PA7),
            pa6: Some(p.PA6),
            pe9: Some(p.PE9),

            pc1: Some(p.PC1),
            pd6: Some(p.PD6),
            pd14: Some(p.PD14),
            pc11: Some(p.PC11),
            pd2: Some(p.PD2),
            pd5: Some(p.PD5),
            pd13: Some(p.PD13),
            pc12: Some(p.PC12),
            pd1: Some(p.PD1),
            pa2: Some(p.PA2),
            pb0: Some(p.PB0),
            pa3: Some(p.PA3),
            pc5: Some(p.PC5),
            adc1: Some(p.ADC1),
        }
    }

    pub fn take_led_pin(&mut self) -> peripherals::PB14 {
        self.pb14.take().unwrap()
    }

    pub fn setup_sbg_uart(
        &mut self,
    ) -> (UartTx<'static, mode::Async>, RingBufferedUartRx<'static>) {
        let mut uart_config = UartConfig::default();
        uart_config.baudrate = 115200;
        let usart = Uart::new(
            self.uart4.take().unwrap(),
            self.pa1.take().unwrap(),
            self.pa0.take().unwrap(),
            Irqs,
            self.dma1_ch1.take().unwrap(),
            self.dma1_ch0.take().unwrap(),
            uart_config,
        )
        .unwrap();
        let (tx, rx) = usart.split();
        let ring_rx = rx.into_ring_buffered(unsafe { &mut RX_SBG_BUF });
        // Power pin if needed
        if let Some(pd8) = self.pd8.take() {
            let _sbg_pwr = Output::new(pd8, Level::High, Speed::Low);
        }
        (tx, ring_rx)
    }

    pub fn setup_baro(
        &mut self,
    ) -> Ms5611<Spi<'static, mode::Blocking>, Output<'static>, embassy_time::Delay> {
        let mut spi_config = SpiConfig::default();
        spi_config.frequency = mhz(16);
        let spi_bus = Spi::new_blocking(
            self.spi4.take().unwrap(),
            self.pe2.take().unwrap(),
            self.pe6.take().unwrap(),
            self.pe5.take().unwrap(),
            spi_config,
        );
        let baro_cs = Output::new(self.pb8.take().unwrap(), Level::High, Speed::Low);
        Ms5611::new(spi_bus, baro_cs, Delay).unwrap()
    }

    pub async fn setup_gps(
        &mut self,
    ) -> (UartTx<'static, mode::Async>, RingBufferedUartRx<'static>) {
        use embassy_stm32::usart::{DataBits, Parity as UParity, StopBits};
        let mut gps_enable = Output::new(self.pa4.take().unwrap(), Level::High, Speed::Low);
        let mut gps_reset = Output::new(self.pb2.take().unwrap(), Level::High, Speed::Low);

        let mut uart_gps_config = UartConfig::default();
        uart_gps_config.baudrate = 9600;
        uart_gps_config.data_bits = DataBits::DataBits8;
        uart_gps_config.parity = UParity::ParityNone;
        uart_gps_config.stop_bits = StopBits::STOP1;
        uart_gps_config.detect_previous_overrun = false;

        let usart = Uart::new(
            self.uart8.take().unwrap(),
            self.pe0.take().unwrap(),
            self.pe1.take().unwrap(),
            Irqs,
            self.dma1_ch5.take().unwrap(),
            self.dma1_ch6.take().unwrap(),
            uart_gps_config,
        )
        .unwrap();
        let (gps_tx, gps_rx) = usart.split();
        let ring_gps_rx = gps_rx.into_ring_buffered(unsafe { &mut RX_GPS_BUF });

        gps_reset.set_low();
        Delay.delay_ms(3000);
        gps_reset.set_high();
        gps_enable.set_low();
        Delay.delay_ms(2000);

        (gps_tx, ring_gps_rx)
    }

    pub fn setup_radio(&mut self) -> (UartTx<'static, mode::Async>, RingBufferedUartRx<'static>) {
        let mut uart_radio_config = UartConfig::default();
        uart_radio_config.baudrate = 57600;
        uart_radio_config.data_bits = embassy_stm32::usart::DataBits::DataBits8;
        uart_radio_config.parity = embassy_stm32::usart::Parity::ParityNone;
        uart_radio_config.stop_bits = embassy_stm32::usart::StopBits::STOP1;

        let uart_radio = Uart::new(
            self.uart7.take().unwrap(),
            self.pe7.take().unwrap(),
            self.pe8.take().unwrap(),
            Irqs,
            self.dma2_ch3.take().unwrap(),
            self.dma2_ch5.take().unwrap(),
            uart_radio_config,
        )
        .unwrap();
        let (radio_tx, radio_rx) = uart_radio.split();
        let ring_rx = radio_rx.into_ring_buffered(unsafe { &mut RX_RADIO_BUF });
        (radio_tx, ring_rx)
    }

    pub fn setup_camera_triggers(&mut self) -> (Output<'static>, Output<'static>) {
        let cam_trigger = Output::new(self.pe14.take().unwrap(), Level::Low, Speed::Low);
        let cam_trigger_b = Output::new(self.pe12.take().unwrap(), Level::Low, Speed::Low);
        (cam_trigger, cam_trigger_b)
    }

    pub fn setup_buzzer(&mut self) -> SimplePwm<'static, peripherals::TIM3> {
        let buzz_out_pin = PwmPin::new_ch1(self.pc6.take().unwrap(), OutputType::PushPull);
        SimplePwm::new(
            self.tim3.take().unwrap(),
            Some(buzz_out_pin),
            None,
            None,
            None,
            khz(4),
            Default::default(),
        )
    }

    pub fn setup_imu(&mut self) {
        // Keep behavior identical to previous main: build peripherals but don't use yet.
        let mut imu_spi_config = embassy_stm32::spi::Config::default();
        imu_spi_config.frequency = mhz(9);
        imu_spi_config.mode = embassy_stm32::spi::Mode {
            polarity: embassy_stm32::spi::Polarity::IdleLow,
            phase: embassy_stm32::spi::Phase::CaptureOnFirstTransition,
        };
        let _imu_spi = Spi::new_blocking(
            self.spi3.take().unwrap(),
            self.pc10.take().unwrap(),
            self.pb5.take().unwrap(),
            self.pb4.take().unwrap(),
            imu_spi_config,
        );
        let _imu_odr = Input::new(self.pc0.take().unwrap(), Pull::None);
        let _imu_cs = Output::new(self.pb6.take().unwrap(), Level::High, Speed::Low);
        let _imu_nreset = Output::new(self.pd4.take().unwrap(), Level::High, Speed::Low);
    }

    pub fn setup_sd_spi(&mut self) -> (Spi<'static, mode::Blocking>, Output<'static>) {
        let mut sd_spi_config = SpiConfig::default();
        sd_spi_config.frequency = mhz(16);
        sd_spi_config.bit_order = embassy_stm32::spi::BitOrder::MsbFirst;

        let sd_spi_bus = Spi::new_blocking(
            self.spi1.take().unwrap(),
            self.pa5.take().unwrap(),
            self.pa7.take().unwrap(),
            self.pa6.take().unwrap(),
            sd_spi_config,
        );
        let sd_cs = Output::new(self.pe9.take().unwrap(), Level::High, Speed::Low);
        (sd_spi_bus, sd_cs)
    }

    pub fn setup_recovery_manager(&mut self) -> RecoveryManager {
        let mut ejection_enable = Input::new(self.pc1.take().unwrap(), Pull::Down);

        let main_arm_test = Output::new(self.pd6.take().unwrap(), Level::Low, Speed::Low);
        let main_arm_test_b = Output::new(self.pd14.take().unwrap(), Level::Low, Speed::Low);
        let drogue_arm_test = Output::new(self.pc11.take().unwrap(), Level::Low, Speed::Low);
        let drogue_arm_test_b = Output::new(self.pd2.take().unwrap(), Level::Low, Speed::Low);

        let main_fire = Output::new(self.pd5.take().unwrap(), Level::Low, Speed::Low);
        let main_fire_b = Output::new(self.pd13.take().unwrap(), Level::Low, Speed::Low);
        let drogue_fire = Output::new(self.pc12.take().unwrap(), Level::Low, Speed::Low);
        let drogue_fire_b = Output::new(self.pd1.take().unwrap(), Level::Low, Speed::Low);

        let mut main_mcu_ematch_sense = self.pa2.take().unwrap();
        let mut main_mcu_ematch_sense_b = self.pb0.take().unwrap();
        let mut drogue_mcu_ematch_sense = self.pa3.take().unwrap();
        let mut drogue_mcu_ematch_sense_b = self.pc5.take().unwrap();

        let mut adc = Adc::new(self.adc1.take().unwrap());

        info!(
            "Ejection enable measurement: {}",
            ejection_enable.get_level()
        );
        info!(
            "ADC measurement main ematch {}",
            adc.blocking_read(&mut main_mcu_ematch_sense)
        );
        info!(
            "ADC measurement main B ematch {}",
            adc.blocking_read(&mut main_mcu_ematch_sense_b)
        );
        info!(
            "ADC measurement drogue ematch {}",
            adc.blocking_read(&mut drogue_mcu_ematch_sense)
        );
        info!(
            "ADC measurement drogue B ematch {}",
            adc.blocking_read(&mut drogue_mcu_ematch_sense_b)
        );
        info!(
            "ADC measurement main ematch {}",
            adc.blocking_read(&mut main_mcu_ematch_sense)
        );

        RecoveryManager {
            arming: Arming {
                main: main_arm_test,
                drogue: drogue_arm_test,
                main_b: main_arm_test_b,
                drogue_b: drogue_arm_test_b,
            },
            fire: Fire {
                main: main_fire,
                drogue: drogue_fire,
                main_b: main_fire_b,
                drogue_b: drogue_fire_b,
            },
        }
    }
}
