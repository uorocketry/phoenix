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
        
        // WARNING turns out it's actually unsafe, not unsafe on HYDRA thought? 🤔
        // let fdcan_prec_unsafe = unsafe { rcc.steal_peripheral_rec() }
        //     .FDCAN
        //     .kernel_clk_mux(rec::FdcanClkSel::Pll1Q);

        let ccdr = rcc
            .use_hse(48.MHz()) // check the clock hardware
            .sys_ck(200.MHz())
            .freeze(pwrcfg, &ctx.device.SYSCFG);
        info!("RCC configured");
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

        // UART for radio
        let tx = gpioe.pe8.into_alternate();
        let rx = gpioe.pe7.into_alternate();

        // let stream_tuple = StreamsTuple::new(ctx.device.DMA1, ccdr.peripheral.DMA1);
        let uart_radio = ctx
            .device
            .UART7
            .serial((tx, rx), 57600.bps(), ccdr.peripheral.UART7, &ccdr.clocks)
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
        reset_reason_send::spawn().ok();
        state_send::spawn().ok();
        baro_read::spawn().ok();
        generate_random_messages::spawn().ok();
        // sensor_send::spawn().ok();
        
        info!("Online");

        (
            SharedResources {
                data_manager,
                madgwick_service,
                em,
                // sd_manager,
                radio_manager,
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
                        // .lock(|rtc| messages::FormattedNaiveDateTime(rtc.date_time().unwrap())),
                        .lock(|rtc| messages::FormattedNaiveDateTime(
                            NaiveDate::from_ymd_opt(2001, 1, 1)
                                .unwrap()
                                .and_hms_opt(0, 0, 0)
                                .unwrap(),
                        )),
                    COM_ID,
                    messages::state::State::new(messages::state::StateData::Initializing),
                );
                spawn!(send_gs, message.clone())?;
                // spawn!(send_data_internal, message)?;
                Ok(())
            });
            Mono::delay(50.millis()).await;
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
                        // .lock(|rtc| messages::FormattedNaiveDateTime(rtc.date_time().unwrap())),
                        .lock(|rtc| messages::FormattedNaiveDateTime(
                            NaiveDate::from_ymd_opt(2001, 1, 1)
                                .unwrap()
                                .and_hms_opt(0, 0, 0)
                                .unwrap(),
                        )),
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
                info!("Sending message {}", m);
                let mut buf = [0; 255];
                let data = postcard::to_slice(&m, &mut buf)?;
                radio_manager.send_message(data)?;
                info!("Send message called");
                Ok(())
            })
        });
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
