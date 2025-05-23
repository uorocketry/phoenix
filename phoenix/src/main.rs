#![no_std]
#![no_main]

mod communication;
mod data_manager;
mod madgwick_service;
mod types;

use chrono::NaiveDate;
use common_arm::*;
use communication::{CanCommandManager, CanDataManager};
use communication::{RadioDevice, RadioManager};
use core::num::{NonZeroU16, NonZeroU8};
use data_manager::DataManager;
use defmt::info;
use fdcan::{
    config::NominalBitTiming,
    filter::{StandardFilter, StandardFilterSlot},
};
use messages::command::RadioRate;
use messages::{sensor, Data};
use panic_probe as _;
use rtic_monotonics::systick::prelude::*;
use rtic_sync::{channel::*, make_channel};
use stm32h7xx_hal::gpio::gpioa::{PA2, PA3};
use stm32h7xx_hal::gpio::gpiob::PB4;
use stm32h7xx_hal::gpio::Speed;
use stm32h7xx_hal::gpio::{Output, PushPull};
use stm32h7xx_hal::prelude::*;
use stm32h7xx_hal::rtc;
use stm32h7xx_hal::{rcc, rcc::rec};
use types::COM_ID; // global logger

const DATA_CHANNEL_CAPACITY: usize = 10;
systick_monotonic!(Mono, 500);

#[inline(never)]
#[defmt::panic_handler]
fn panic() -> ! {
    // stm32h7xx_hal::pac::SCB::sys_reset()
    cortex_m::asm::udf()
}

#[rtic::app(device = stm32h7xx_hal::stm32, peripherals = true, dispatchers = [EXTI0, EXTI1, EXTI2, SPI3, SPI2])]
mod app {

    use common_arm::drivers::ms5611::OversamplingRatio;
    use messages::Message;
    use stm32h7xx_hal::gpio::{Alternate, Pin};

    use super::*;

    #[shared]
    struct SharedResources {
        data_manager: DataManager,
        madgwick_service: madgwick_service::MadgwickService,
        em: ErrorManager,
        // sd_manager: SdManager<
        //     stm32h7xx_hal::spi::Spi<stm32h7xx_hal::pac::SPI1, stm32h7xx_hal::spi::Enabled>,
        //     PA4<Output<PushPull>>,
        // >,
        radio_manager: RadioManager,
        can_command_manager: CanCommandManager,
        can_data_manager: CanDataManager,
        sbg_power: PB4<Output<PushPull>>,
        rtc: rtc::Rtc,
    }
    #[local]
    struct LocalResources {
        led_red: PA2<Output<PushPull>>,
        led_green: PA3<Output<PushPull>>,
        buzzer: stm32h7xx_hal::pwm::Pwm<
            stm32h7xx_hal::pac::TIM12,
            0,
            stm32h7xx_hal::pwm::ComplementaryImpossible,
        >,
        // Baro uses:
        // PB_08 for CS
        // PE_02 for SCK
        // PE_05 for MISO
        // PE_06 for MOSI
        baro: common_arm::drivers::ms5611::Ms5611<
            stm32h7xx_hal::spi::Spi<stm32h7xx_hal::pac::SPI4, stm32h7xx_hal::spi::Enabled>,
            stm32h7xx_hal::gpio::Pin<
                'B',
                8,
                stm32h7xx_hal::gpio::Output<stm32h7xx_hal::gpio::PushPull>,
            >,
            stm32h7xx_hal::delay::DelayFromCountDownTimer<
                stm32h7xx_hal::timer::Timer<stm32h7xx_hal::pac::TIM2>,
            >,
        >,
    }

    #[init]
    fn init(ctx: init::Context) -> (SharedResources, LocalResources) {
        // channel setup
        let (_s, r) = make_channel!(Message, DATA_CHANNEL_CAPACITY);

        let core = ctx.core;

        /* Logging Setup */
        HydraLogging::set_ground_station_callback(queue_gs_message);

        let pwr = ctx.device.PWR.constrain();
        // We could use smps, but the board is not designed for it
        // let pwrcfg = example_power!(pwr).freeze();
        let mut pwrcfg = pwr.freeze();

        info!("Power enabled");
        let backup = pwrcfg.backup().unwrap();
        info!("Backup domain enabled");
        // RCC
        let mut rcc = ctx.device.RCC.constrain();
        let reset = rcc.get_reset_reason();
        let fdcan_prec_unsafe = unsafe { rcc.steal_peripheral_rec() }
            .FDCAN
            .kernel_clk_mux(rec::FdcanClkSel::Pll1Q);

        let ccdr = rcc
            .use_hse(48.MHz()) // check the clock hardware
            .sys_ck(200.MHz())
            .pll1_strategy(rcc::PllConfigStrategy::Iterative)
            .pll1_q_ck(32.MHz())
            .freeze(pwrcfg, &ctx.device.SYSCFG);
        info!("RCC configured");
        let fdcan_prec = ccdr
            .peripheral
            .FDCAN
            .kernel_clk_mux(rec::FdcanClkSel::Pll1Q);

        let btr = NominalBitTiming {
            prescaler: NonZeroU16::new(10).unwrap(),
            seg1: NonZeroU8::new(13).unwrap(),
            seg2: NonZeroU8::new(2).unwrap(),
            sync_jump_width: NonZeroU8::new(1).unwrap(),
        };

        // let data_bit_timing = DataBitTiming {
        //     prescaler: NonZeroU8::new(10).unwrap(),
        //     seg1: NonZeroU8::new(13).unwrap(),
        //     seg2: NonZeroU8::new(2).unwrap(),
        //     sync_jump_width: NonZeroU8::new(4).unwrap(),
        //     transceiver_delay_compensation: true,
        // };

        info!("CAN enabled");
        // GPIO
        let gpioa = ctx.device.GPIOA.split(ccdr.peripheral.GPIOA);
        let gpiod = ctx.device.GPIOD.split(ccdr.peripheral.GPIOD);
        let gpiob = ctx.device.GPIOB.split(ccdr.peripheral.GPIOB);

        let pins = gpiob.pb14.into_alternate();
        let mut c0 = ctx
            .device
            .TIM12
            .pwm(pins, 4.kHz(), ccdr.peripheral.TIM12, &ccdr.clocks);

        c0.set_duty(c0.get_max_duty() / 4);
        // PWM outputs are disabled by default
        // c0.enable();

        info!("PWM enabled");
        // assert_eq!(ccdr.clocks.pll1_q_ck().unwrap().raw(), 32_000_000);
        info!("PLL1Q:");
        // https://github.com/stm32-rs/stm32h7xx-hal/issues/369 This needs to be stolen. Grrr I hate the imaturity of the stm32-hal
        let can2: fdcan::FdCan<
            stm32h7xx_hal::can::Can<stm32h7xx_hal::pac::FDCAN2>,
            fdcan::ConfigMode,
        > = {
            let rx = gpiob.pb12.into_alternate().speed(Speed::VeryHigh);
            let tx = gpiob.pb13.into_alternate().speed(Speed::VeryHigh);
            ctx.device.FDCAN2.fdcan(tx, rx, fdcan_prec)
        };

        let mut can_data = can2;
        can_data.set_protocol_exception_handling(false);

        can_data.set_nominal_bit_timing(btr);

        // can_data.set_automatic_retransmit(false); // data can be dropped due to its volume.

        // can_command.set_data_bit_timing(data_bit_timing);

        can_data.set_standard_filter(
            StandardFilterSlot::_0,
            StandardFilter::accept_all_into_fifo0(),
        );

        can_data.set_standard_filter(
            StandardFilterSlot::_1,
            StandardFilter::accept_all_into_fifo0(),
        );

        can_data.set_standard_filter(
            StandardFilterSlot::_2,
            StandardFilter::accept_all_into_fifo0(),
        );

        can_data.enable_interrupt(fdcan::interrupt::Interrupt::RxFifo0NewMsg);

        can_data.enable_interrupt_line(fdcan::interrupt::InterruptLine::_0, true);

        let config = can_data
            .get_config()
            .set_frame_transmit(fdcan::config::FrameTransmissionConfig::AllowFdCanAndBRS);
        can_data.apply_config(config);

        let can_data_manager = CanDataManager::new(can_data.into_normal());

        let can1: fdcan::FdCan<
            stm32h7xx_hal::can::Can<stm32h7xx_hal::pac::FDCAN1>,
            fdcan::ConfigMode,
        > = {
            let rx = gpioa.pa11.into_alternate().speed(Speed::VeryHigh);
            let tx = gpioa.pa12.into_alternate().speed(Speed::VeryHigh);
            ctx.device.FDCAN1.fdcan(tx, rx, fdcan_prec_unsafe)
        };

        let mut can_command = can1;
        can_command.set_protocol_exception_handling(false);

        can_command.set_nominal_bit_timing(btr);
        can_command.set_standard_filter(
            StandardFilterSlot::_0,
            StandardFilter::accept_all_into_fifo0(),
        );

        can_command.set_standard_filter(
            StandardFilterSlot::_1,
            StandardFilter::accept_all_into_fifo0(),
        );

        can_command.set_standard_filter(
            StandardFilterSlot::_2,
            StandardFilter::accept_all_into_fifo0(),
        );

        // can_data.set_data_bit_timing(data_bit_timing);
        can_command.enable_interrupt(fdcan::interrupt::Interrupt::RxFifo0NewMsg);

        can_command.enable_interrupt_line(fdcan::interrupt::InterruptLine::_0, true);

        let config = can_command
            .get_config()
            .set_frame_transmit(fdcan::config::FrameTransmissionConfig::AllowFdCanAndBRS); // check this maybe don't bit switch allow.
        can_command.apply_config(config);

        let can_command_manager = CanCommandManager::new(can_command.into_normal());

        // let spi_sd: stm32h7xx_hal::spi::Spi<
        //     stm32h7xx_hal::stm32::SPI1,
        //     stm32h7xx_hal::spi::Enabled,
        //     u8,
        // > = ctx.device.SPI1.spi(
        //     (
        //         gpioa.pa5.into_alternate::<5>(),
        //         gpioa.pa6.into_alternate(),
        //         gpioa.pa7.into_alternate(),
        //     ),
        //     spi::Config::new(spi::MODE_0),
        //     16.MHz(),
        //     ccdr.peripheral.SPI1,
        //     &ccdr.clocks,
        // );

        // let cs_sd = gpioa.pa4.into_push_pull_output();

        // let sd_manager = SdManager::new(spi_sd, cs_sd);

        // leds
        let led_red = gpioa.pa2.into_push_pull_output();
        let led_green = gpioa.pa3.into_push_pull_output();

        // sbg power pin
        let mut sbg_power = gpiob.pb4.into_push_pull_output();
        sbg_power.set_high();

        // Configure SPI4 for barometer
        let gpioe = ctx.device.GPIOE.split(ccdr.peripheral.GPIOE);
        let spi4 = ctx.device.SPI4.spi(
            (
                gpioe.pe2.into_alternate(), // SCK
                gpioe.pe5.into_alternate(), // MISO
                gpioe.pe6.into_alternate(), // MOSI
            ),
            stm32h7xx_hal::spi::Config::new(stm32h7xx_hal::spi::MODE_0),
            16.MHz(),
            ccdr.peripheral.SPI4,
            &ccdr.clocks,
        );
        let baro_cs = gpiob.pb8.into_push_pull_output();
        let timer2 = ctx
            .device
            .TIM2
            .timer(1.MHz(), ccdr.peripheral.TIM2, &ccdr.clocks);
        let delay_tim = stm32h7xx_hal::delay::DelayFromCountDownTimer::new(timer2);
        /* Monotonic clock */
        Mono::start(core.SYST, 200_000_000);

        let baro = common_arm::drivers::ms5611::Ms5611::new(spi4, baro_cs, delay_tim).unwrap();

        // UART for sbg
        let tx: Pin<'D', 1, Alternate<8>> = gpiod.pd1.into_alternate();
        let rx: Pin<'D', 0, Alternate<8>> = gpiod.pd0.into_alternate();

        // let stream_tuple = StreamsTuple::new(ctx.device.DMA1, ccdr.peripheral.DMA1);
        let uart_radio = ctx
            .device
            .UART4
            .serial((tx, rx), 57600.bps(), ccdr.peripheral.UART4, &ccdr.clocks)
            .unwrap();
        // let mut sbg_manager = sbg_manager::SBGManager::new(uart_sbg, stream_tuple);

        let radio = RadioDevice::new(uart_radio);

        let radio_manager = RadioManager::new(radio);

        let mut rtc = stm32h7xx_hal::rtc::Rtc::open_or_init(
            ctx.device.RTC,
            backup.RTC,
            stm32h7xx_hal::rtc::RtcClock::Lsi,
            &ccdr.clocks,
        );

        // TODO: Get current time from some source
        let now = NaiveDate::from_ymd_opt(2001, 1, 1)
            .unwrap()
            .and_hms_opt(0, 0, 0)
            .unwrap();

        rtc.set_date_time(now);

        let madgwick_service = madgwick_service::MadgwickService::new();

        let mut data_manager = DataManager::new();
        data_manager.set_reset_reason(reset);
        let em = ErrorManager::new();
        blink::spawn().ok();
        send_data_internal::spawn(r).ok();
        reset_reason_send::spawn().ok();
        state_send::spawn().ok();
        baro_read::spawn().ok();
        // generate_random_messages::spawn().ok();
        // sensor_send::spawn().ok();
        info!("Online");

        (
            SharedResources {
                data_manager,
                madgwick_service,
                em,
                // sd_manager,
                radio_manager,
                can_command_manager,
                can_data_manager,
                sbg_power,
                rtc,
            },
            LocalResources {
                led_red,
                led_green,
                buzzer: c0,
                baro,
            },
        )
    }

    // it would be nice to have RTIC be able to return objects, but the current procedural macro
    // does not allow for this.
    #[task(priority = 3, local = [baro], shared = [&em, data_manager])]
    async fn baro_read(mut cx: baro_read::Context) {
        let baro = cx.local.baro; // Get mutable access to the driver
        loop {
            cx.shared.em.run(|| {
                // Choose the desired Oversampling Ratio for this reading
                let osr = OversamplingRatio::Osr512; // Example: Highest precision

                match baro.read_pressure_temperature(osr) {
                    Ok((temp_c, press_kpa)) => {
                        cx.shared.data_manager.lock(|dm| {
                            dm.baro_temperature = Some(temp_c);
                            dm.baro_pressure = Some(press_kpa);
                        });
                        Ok(())
                    }
                    Err(e) => {
                        info!("Baro: Driver reading failed!");
                        cx.shared.data_manager.lock(|dm| {
                            dm.baro_temperature = None;
                            dm.baro_pressure = None;
                        });
                        Err(HydraError::from(e))
                    }
                }
            });
            Mono::delay(1000.millis()).await;
        }
    }

    #[task(priority = 3, shared = [&em, rtc])]
    async fn generate_random_messages(mut cx: generate_random_messages::Context) {
        loop {
            cx.shared.em.run(|| {
                let message = Message::new(
                    cx.shared
                        .rtc
                        .lock(|rtc| messages::FormattedNaiveDateTime(rtc.date_time().unwrap())),
                    COM_ID,
                    messages::state::State::new(messages::state::StateData::Initializing),
                );
                spawn!(send_gs, message.clone())?;
                // spawn!(send_data_internal, message)?;
                Ok(())
            });
            Mono::delay(1.secs()).await;
        }
    }

    #[task(priority = 3, shared = [data_manager, &em, rtc])]
    async fn reset_reason_send(mut cx: reset_reason_send::Context) {
        let reason = cx
            .shared
            .data_manager
            .lock(|data_manager| data_manager.clone_reset_reason());
        match reason {
            Some(reason) => {
                let x = match reason {
                    stm32h7xx_hal::rcc::ResetReason::BrownoutReset => sensor::ResetReason::BrownoutReset,
                    stm32h7xx_hal::rcc::ResetReason::CpuReset => sensor::ResetReason::CpuReset,
                    stm32h7xx_hal::rcc::ResetReason::D1EntersDStandbyErroneouslyOrCpuEntersCStopErroneously => sensor::ResetReason::D1EntersDStandbyErroneouslyOrCpuEntersCStopErroneously,
                    stm32h7xx_hal::rcc::ResetReason::D1ExitsDStandbyMode => sensor::ResetReason::D1ExitsDStandbyMode,
                    stm32h7xx_hal::rcc::ResetReason::D2ExitsDStandbyMode => sensor::ResetReason::D2ExitsDStandbyMode,
                    stm32h7xx_hal::rcc::ResetReason::GenericWatchdogReset => sensor::ResetReason::GenericWatchdogReset,
                    stm32h7xx_hal::rcc::ResetReason::IndependentWatchdogReset => sensor::ResetReason::IndependentWatchdogReset,
                    stm32h7xx_hal::rcc::ResetReason::PinReset => sensor::ResetReason::PinReset,
                    stm32h7xx_hal::rcc::ResetReason::PowerOnReset => sensor::ResetReason::PowerOnReset,
                    stm32h7xx_hal::rcc::ResetReason::SystemReset => sensor::ResetReason::SystemReset,
                    stm32h7xx_hal::rcc::ResetReason::Unknown { rcc_rsr } => sensor::ResetReason::Unknown { rcc_rsr },
                    stm32h7xx_hal::rcc::ResetReason::WindowWatchdogReset => sensor::ResetReason::WindowWatchdogReset,
                };
                let message = messages::Message::new(
                    cx.shared
                        .rtc
                        .lock(|rtc| messages::FormattedNaiveDateTime(rtc.date_time().unwrap())),
                    COM_ID,
                    sensor::Sensor::new(x),
                );

                cx.shared.em.run(|| {
                    spawn!(send_gs, message)?;
                    Ok(())
                })
            }
            None => return,
        }
    }

    #[task(shared = [data_manager, &em, rtc])]
    async fn state_send(mut cx: state_send::Context) {
        let state_data = cx
            .shared
            .data_manager
            .lock(|data_manager| data_manager.state.clone());
        cx.shared.em.run(|| {
            if let Some(x) = state_data {
                let message = Message::new(
                    cx.shared
                        .rtc
                        .lock(|rtc| messages::FormattedNaiveDateTime(rtc.date_time().unwrap())),
                    COM_ID,
                    messages::state::State::new(x),
                );
                spawn!(send_gs, message)?;
            } // if there is none we still return since we simply don't have data yet.
            Ok(())
        });
        Mono::delay(5.secs()).await;
        // spawn_after!(state_send, ExtU64::secs(5)).ok();
    }

    /**
     * Sends information about the sensors.
     */
    #[task(priority = 3, shared = [data_manager, &em])]
    async fn sensor_send(mut cx: sensor_send::Context) {
        loop {
            let (sensors, logging_rate) = cx.shared.data_manager.lock(|data_manager| {
                (data_manager.take_sensors(), data_manager.get_logging_rate())
            });

            cx.shared.em.run(|| {
                for msg in sensors {
                    match msg {
                        Some(x) => {
                            // info!("Sending sensor data {}", x.clone());
                            spawn!(send_gs, x)?;
                            //                     spawn!(sd_dump, x)?;
                        }
                        None => {
                            info!("No sensor data to send");
                            continue;
                        }
                    }
                }

                Ok(())
            });
            match logging_rate {
                RadioRate::Fast => {
                    Mono::delay(100.millis()).await;
                }
                RadioRate::Slow => {
                    Mono::delay(250.millis()).await;
                }
            }
        }
    }

    /// Receives a log message from the custom logger so that it can be sent over the radio.
    pub fn queue_gs_message(d: impl Into<Data>) {
        info!("Queueing message");
        send_gs_intermediate::spawn(d.into()).ok();
    }

    #[task(priority = 3, shared = [rtc, &em])]
    async fn send_gs_intermediate(mut cx: send_gs_intermediate::Context, m: Data) {
        cx.shared.em.run(|| {
            cx.shared.rtc.lock(|rtc| {
                let message = messages::Message::new(
                    messages::FormattedNaiveDateTime(rtc.date_time().unwrap()),
                    COM_ID,
                    m,
                );
                spawn!(send_gs, message)?;
                Ok(())
            })
        });
    }

    #[task(priority = 2, binds = FDCAN1_IT0, shared = [can_command_manager, data_manager, &em])]
    fn can_command(mut cx: can_command::Context) {
        // info!("CAN Command");
        cx.shared.can_command_manager.lock(|can| {
            cx.shared
                .data_manager
                .lock(|data_manager| cx.shared.em.run(|| can.process_data(data_manager)));
        })
    }

    #[task(priority = 3, shared = [sbg_power])]
    async fn sbg_power_on(mut cx: sbg_power_on::Context) {
        loop {
            cx.shared.sbg_power.lock(|sbg| {
                sbg.set_high();
            });
            Mono::delay(10000.millis()).await;
        }
    }

    /**
     * Sends a message to the radio over UART.
     */
    #[task(priority = 3, shared = [&em, radio_manager])]
    async fn send_gs(mut cx: send_gs::Context, m: Message) {
        // info!("{}", m.clone());

        cx.shared.radio_manager.lock(|radio_manager| {
            cx.shared.em.run(|| {
                // info!("Sending message {}", m);
                let mut buf = [0; 255];
                let data = postcard::to_slice(&m, &mut buf)?;
                radio_manager.send_message(data)?;
                Ok(())
            })
        });
    }

    #[task(priority = 3, binds = FDCAN2_IT0, shared = [&em, can_data_manager, data_manager, madgwick_service])]
    fn can_data(mut cx: can_data::Context) {
        cx.shared.can_data_manager.lock(|can| {
            while let Ok(Some(message)) = can.receive_message() {
                // process IMU data through madgwick service
                cx.shared.madgwick_service.lock(|madgwick| {
                    if let Some(result) = madgwick.process_imu_data(&message) {
                        cx.shared.data_manager.lock(|dm| {
                            dm.store_madgwick_result(result);
                        });
                    }
                });
            }
            cx.shared.em.run(|| Ok(()))
        });
    }

    #[task(priority = 2, shared = [&em, can_data_manager, data_manager])]
    async fn send_data_internal(
        mut cx: send_data_internal::Context,
        mut receiver: Receiver<'static, Message, DATA_CHANNEL_CAPACITY>,
    ) {
        loop {
            if let Ok(m) = receiver.recv().await {
                cx.shared.can_data_manager.lock(|can| {
                    cx.shared.em.run(|| {
                        can.send_message(m)?;
                        Ok(())
                    })
                });
            }
        }
    }

    #[task(priority = 2, shared = [&em, can_command_manager, data_manager])]
    async fn send_command_internal(mut cx: send_command_internal::Context, m: Message) {
        // while let Ok(m) = receiver.recv().await {
        cx.shared.can_command_manager.lock(|can| {
            cx.shared.em.run(|| {
                can.send_message(m)?;
                Ok(())
            })
        });
        // }
    }

    #[task(priority = 1, local = [led_red, led_green, buzzer, buzzed: bool = false], shared = [&em])]
    async fn blink(cx: blink::Context) {
        loop {
            if cx.shared.em.has_error() {
                cx.local.led_red.toggle();
                if *cx.local.buzzed {
                    cx.local.buzzer.set_duty(0);
                    *cx.local.buzzed = false;
                } else {
                    let duty = cx.local.buzzer.get_max_duty() / 4;
                    cx.local.buzzer.set_duty(duty);
                    *cx.local.buzzed = true;
                }
                Mono::delay(500.millis()).await;
            } else {
                cx.local.led_green.toggle();
                if *cx.local.buzzed {
                    cx.local.buzzer.set_duty(0);
                    *cx.local.buzzed = false;
                } else {
                    let duty = cx.local.buzzer.get_max_duty() / 4;
                    cx.local.buzzer.set_duty(duty);
                    *cx.local.buzzed = true;
                }
                Mono::delay(2000.millis()).await;
            }
        }
    }

    #[task(priority = 3, shared = [&em, sbg_power])]
    async fn sleep_system(mut cx: sleep_system::Context) {
        // Turn off the SBG and CAN, also start a timer to wake up the system. Put the chip in sleep mode.
        cx.shared.sbg_power.lock(|sbg| {
            sbg.set_low();
        });
    }
}
