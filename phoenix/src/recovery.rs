use common_arm::drivers::ms5611::{Ms5611, OversamplingRatio};
use defmt::info;
use embassy_stm32::gpio::{Input, Level, Output, Pull, Speed};
use embassy_stm32::mode::Blocking;
use embassy_stm32::peripherals::{
    ADC1, PA2, PA3, PB0, PC1, PC11, PC12, PC5, PD1, PD13, PD14, PD2, PD5, PD6,
};
use embassy_stm32::spi::Spi;
use embassy_time::{Delay, Duration, Instant, Timer};
use embedded_hal_1::delay::DelayNs;
use embedded_hal_1::digital::OutputPin;
use heapless::HistoryBuffer;
use libm::powf;
// --- Boom Boom Setup ---
/*
   MAIN_ARM/TEST = PD6
   MAIN_FIRE = PD5
   MAIN_ARM/TEST_B = PD14
   MAIN_FIRE_B = PD13
   DROGUE_ARM/TEST = PC11
   DROGUE_FIRE = PC12
   DROGUE_ARM/TEST_B = PD2
   DROGUE_FIRE_B = PD1
   MAIN_MCU_EMATCH_SENSE = PA2
   MAIN_MCU_EMATCH_SENSE_B = PB0
   DROUGE_MCU_EMATCH_SENSE = PA3
   DROGUE_MCU_EMATCH_SENSE_B = PC5
*/

struct Arming {
    main: Output<'static>,
    drogue: Output<'static>,
    main_b: Output<'static>,
    drogue_b: Output<'static>,
}

struct Fire {
    main: Output<'static>,
    drogue: Output<'static>,
    main_b: Output<'static>,
    drogue_b: Output<'static>,
}

struct Sensing {
    adc: embassy_stm32::peripherals::ADC1,
    main: PA2,
    main_b: PB0,
    drogue: PA3,
    drogue_b: PC5,
}

pub struct RecoveryManager {
    ejection_enable: Input<'static>,
    sensing: Sensing,
    arming: Arming,
    fire: Fire,
}

impl RecoveryManager {
    pub fn new(
        main_arm: PD6,
        main_arm_b: PD14,
        drogue_arm: PC11,
        drogue_arm_b: PD2,
        main_fire: PD5,
        main_fire_b: PD13,
        drogue_fire: PC12,
        drogue_fire_b: PD1,
        main_sense: PA2,
        main_b_sense: PB0,
        drogue_sense: PA3,
        drogue_sense_b: PC5,
        adc: ADC1,
        ejection_enable: PC1,
    ) -> Self {
        RecoveryManager {
            ejection_enable: Input::new(ejection_enable, Pull::Down),
            sensing: Sensing {
                adc,
                main: main_sense,
                main_b: main_b_sense,
                drogue: drogue_sense,
                drogue_b: drogue_sense_b,
            },
            arming: Arming {
                main: Output::new(main_arm, Level::Low, Speed::Low),
                main_b: Output::new(main_arm_b, Level::Low, Speed::Low),
                drogue: Output::new(drogue_arm, Level::Low, Speed::Low),
                drogue_b: Output::new(drogue_arm_b, Level::Low, Speed::Low),
            },
            fire: Fire {
                main: Output::new(main_fire, Level::Low, Speed::Low),
                main_b: Output::new(main_fire_b, Level::Low, Speed::Low),
                drogue: Output::new(drogue_fire, Level::Low, Speed::Low),
                drogue_b: Output::new(drogue_fire_b, Level::Low, Speed::Low),
            },
        }
    }

    pub fn arm(&mut self) {
        if self.ejection_enable.is_high() {
            info!("arm ejection enabled");
            self.arming.main.set_high();
            self.arming.main_b.set_high();

            self.arming.drogue.set_high();
            self.arming.main_b.set_high();
        }
    }

    pub fn disarm(&mut self) {
        info!("arm ejection disabled");
        self.arming.main.set_low();
        self.arming.main_b.set_low();

        self.arming.drogue.set_low();
        self.arming.main_b.set_low();
    }

    pub fn fire_main(&mut self) {
        self.fire.main.set_high();
        self.fire.main_b.set_high();
        embassy_time::Delay.delay_ms(500);
        self.fire.main.set_low();
        self.fire.main_b.set_low();
    }

    pub fn fire_drogue(&mut self) {
        self.fire.drogue.set_high();
        self.fire.drogue_b.set_high();
        embassy_time::Delay.delay_ms(500);
        self.fire.drogue.set_low();
        self.fire.drogue_b.set_low();
    }
}

#[embassy_executor::task]
async fn recovery_algorithm_task(mut baro: Ms5611<Spi<'static, Blocking>, Output<'static>, Delay>) {
    info!("Barometer reader task started.");
    const MAIN_HEIGHT: f32 = GROUND_HEIGHT + 500.0; // meters ASL
    const HEIGHT_MIN: f32 = GROUND_HEIGHT + 300.0; // meters ASL
    const GROUND_HEIGHT: f32 = 300.0; // meters ASL
    const ASCENT_LOCKOUT: f32 = 0.1;
    const DATA_POINTS: usize = 20;
    const VALID_DESCENT_RATE: f32 = -0.005; // meters per millise

    let mut historical_barometer_altitude: HistoryBuffer<(f32, Instant), 20> = HistoryBuffer::new();

    loop {
        match baro.read_pressure_temperature(OversamplingRatio::Osr4096) {
            Ok(reading) => {
                // info!(
                //     "Baro: Temp: {} C, Pressure: {} mbar",
                //     reading.0, reading.1
                // );

                // Hypsometric Formula
                let altitude =
                    ((powf(101.325 / reading.1, 1.0 / 5.257) - 1.0) * (25.0 + 273.15)) / 0.0065;
                historical_barometer_altitude.write((altitude, Instant::now()));

                // Apogee detection logic
                if historical_barometer_altitude.len() < 8 {
                    info!("not enough data points to detect apogee");
                    continue;
                }

                let mut buf = historical_barometer_altitude.oldest_ordered();
                if let Some(mut prev_reading) = buf.next() {
                    // `prev_reading` is now a tuple: (f32, Instant)
                    let mut avg_sum: f32 = 0.0;
                    let mut datapoints_used = 0;

                    for current_reading in buf {
                        // `current_reading` is also a tuple
                        // Calculate time diff between the actual measurement times.
                        // Convert from micros to seconds for a more standard rate unit (meters/sec).
                        let time_diff =
                            current_reading.1.duration_since(prev_reading.1).as_millis();

                        info!(
                            "prev alt: {}, new alt: {}, time diff: {} ms",
                            prev_reading.0, current_reading.0, time_diff
                        );

                        if time_diff == 0 {
                            continue; // Avoid division by zero
                        }

                        let slope = (current_reading.0 - prev_reading.0) / time_diff as f32;
                        // info!("Slope: {} m/ms", slope);
                        // Your existing logic for ascent lockout
                        if slope > ASCENT_LOCKOUT {
                            continue;
                        }

                        avg_sum += slope;
                        datapoints_used += 1;
                        prev_reading = current_reading; // Update to the current reading for the next iteration
                    }

                    // Check if the average descent rate is valid
                    if datapoints_used > 0 {
                        let avg_slope = avg_sum / (datapoints_used as f32);
                        // info!("Average slope: {} m/ms", avg_slope);
                        if avg_slope <= VALID_DESCENT_RATE {
                            info!(
                                "Apogee detected! Average vertical speed: {} m/s",
                                avg_slope * 1000.0
                            );
                            // todo!("Send Apogee event over events channel to state machine to process.");
                        }
                    }
                }
            }
            Err(e) => {
                // error!("Baro: Driver reading failed: {:?}", e);
            }
        }
        Timer::after(Duration::from_millis(100)).await;
    }
}
