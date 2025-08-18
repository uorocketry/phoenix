use defmt::info;
use embassy_stm32::gpio::{Input, Level, Output, Pull, Speed};
use embassy_stm32::peripherals::{
    ADC1, PA2, PA3, PB0, PC1, PC11, PC12, PC5, PD1, PD13, PD14, PD2, PD5, PD6,
};
use embassy_time::{Duration, Instant, Timer};
use embedded_hal_1::delay::DelayNs;
use embedded_hal_1::digital::OutputPin;
use heapless::{HistoryBuffer, Vec};
use libm::powf;

use crate::resources::{EVENT_CHANNEL, PRESSURE_CHANNEL};

const SENSOR_TIMEOUT: Duration = Duration::from_millis(10_000);
pub const MAIN_HEIGHT: f32 = GROUND_HEIGHT + 500.0; // meters ASL
const HEIGHT_MIN: f32 = GROUND_HEIGHT + 300.0; // meters ASL
const GROUND_HEIGHT: f32 = 300.0; // meters ASL
const ASCENT_LOCKOUT: f32 = 0.01;
const DATA_POINTS: usize = 10;
const VALID_DESCENT_RATE: f32 = -0.005; // meters per millise

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
            self.arming.drogue_b.set_high();
        }
    }

    pub fn is_armed(&mut self) -> bool {
        self.ejection_enable.is_high()
    }

    pub fn disarm(&mut self) {
        info!("arm ejection disabled");
        self.arming.main.set_low();
        self.arming.main_b.set_low();

        self.arming.drogue.set_low();
        self.arming.drogue_b.set_low();
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

// #[embassy_executor::task]
// pub async fn recovery_algorithm_task() {
//     info!("Barometer reader task started.");

//     // History buffers to store recent altitude readings from two different sources.
//     // Each entry is a tuple of (altitude, timestamp).
//     let mut historical_barometer_altitude_sbg: HistoryBuffer<(f32, Instant), 20> =
//         HistoryBuffer::new();
//     let mut historical_barometer_altitude_baro: HistoryBuffer<(f32, Instant), 20> =
//         HistoryBuffer::new();

//     // These flags are declared but not used in the provided snippet.
//     // They might be intended for logic to ignore faulty sensors.
//     let ignore_baro = false;
//     let ignore_sbg = false;

//     loop {
//         // Flags to track apogee detection, not used in the provided logic.
//         let baro_apogee_detected = false;
//         let sbg_apogee_detected = false;

//         // Wait for a new pressure reading from the channel.
//         // The reading includes pressure, temperature, a source identifier, and a timestamp.
//         let reading: (f32, f32, u8, Instant) = PRESSURE_CHANNEL.receive().await;
        
//         // This variable will hold the calculated altitude.
//         let mut altitude = 0.0;

//         // Process the reading based on its source identifier.
//         if reading.2 == 1 { // Source is the barometer (baro)
//             // Hypsometric Formula to convert pressure and temperature to altitude.
//             altitude =
//                 ((powf(101.325 / reading.0, 1.0 / 5.257) - 1.0) * (reading.1 + 273.15)) / 0.0065;
//             historical_barometer_altitude_baro.write((altitude, reading.3));
//         } else if reading.2 == 0 { // Source is the SBG
//             info!("SBG Altitude data {}, {}", reading.0, reading.3);
//             altitude = reading.0; // SBG provides altitude directly.
//             historical_barometer_altitude_sbg.write((altitude, reading.3));
//         }

//         // --- Apogee Detection Logic ---
//         // Ensure there are enough data points in the buffer to perform a reliable calculation.
//         if historical_barometer_altitude_sbg.len() < 8 {
//             info!("Not enough SBG data points to detect apogee.");
//             continue;
//         }

//         // Get an ordered iterator over the historical data.
//         let mut buf_sbg = historical_barometer_altitude_sbg.oldest_ordered();
//         let mut buf_baro = historical_barometer_altitude_baro.oldest_ordered();

//         // --- SBG Apogee Check ---
//         if let Some(mut prev_reading) = buf_sbg.next() {
//             let mut avg_sum: f32 = 0.0;
//             let mut datapoints_used = 0;

//             for current_reading in buf_sbg {
//                 // Calculate the time difference between measurements in milliseconds.
//                 let time_diff_ms = (current_reading.1.as_millis() - prev_reading.1.as_millis());
//                 info!("time diff: {}", time_diff_ms);
//                 if time_diff_ms == 0 {
//                     continue; // Avoid division by zero.
//                 }

//                 // Calculate slope (vertical speed) and immediately convert it to m/s.
//                 // Formula: (delta_altitude_meters / delta_time_ms) * 1000 ms/s = speed_m/s
//                 let slope_mps = ((current_reading.0 - prev_reading.0) / time_diff_ms as f32);
//                 info!("SBG Slope: {} m/s", slope_mps);
                
//                 // Lockout check: if the rocket is ascending too fast, ignore this data point.
//                 // This assumes ASCENT_LOCKOUT is defined in m/s.
//                 if slope_mps > ASCENT_LOCKOUT {
//                     continue;
//                 }

//                 avg_sum += slope_mps;
//                 datapoints_used += 1;
//                 prev_reading = current_reading; // Update for the next iteration.
//             }

//             // Check if we have enough valid data points for an average.
//             if datapoints_used >= DATA_POINTS / 2 {
//                 let avg_slope_mps = avg_sum / (datapoints_used as f32);
                
//                 // Apogee condition: if the average vertical speed indicates a sufficient descent.
//                 // This assumes VALID_DESCENT_RATE is a negative value in m/s (e.g., -5.0).
//                 if avg_slope_mps <= VALID_DESCENT_RATE {
//                     info!(
//                         "SBG Apogee detected! Average vertical speed: {} m/s",
//                         avg_slope_mps
//                     );
//                     // Send an Apogee event to the state machine.
//                     if EVENT_CHANNEL.try_send(crate::state_machine::Events::Apogee).is_err() {
//                         // If sending fails, log it. This is a critical failure.
//                         todo!("Log failure to radio");
//                     }
//                     break; // Exit the loop once apogee is detected.
//                 }
//             }
//         }

//         // --- Barometer Apogee Check ---
//         // This logic is duplicated for the second sensor.
//         if let Some(mut prev_reading) = buf_baro.next() {
//             let mut avg_sum: f32 = 0.0;
//             let mut datapoints_used = 0;

//             for current_reading in buf_baro {
//                 let time_diff_ms = current_reading.1.as_millis() - prev_reading.1.as_millis();

//                 if time_diff_ms == 0 {
//                     continue;
//                 }

//                 // Calculate and convert slope to m/s, same as for the SBG.
//                 let slope_mps = ((current_reading.0 - prev_reading.0) / time_diff_ms as f32) * 1000.0;
//                 // info!("Baro Slope: {} m/s", slope_mps);

//                 if slope_mps > ASCENT_LOCKOUT {
//                     continue;
//                 }

//                 avg_sum += slope_mps;
//                 datapoints_used += 1;
//                 prev_reading = current_reading;
//             }

//             if datapoints_used >= DATA_POINTS / 2 {
//                 let avg_slope_mps = avg_sum / (datapoints_used as f32);
                
//                 if avg_slope_mps <= VALID_DESCENT_RATE {
//                     // info!(
//                     //     "Baro Apogee detected! Average vertical speed: {} m/s",
//                     //     avg_slope_mps
//                     // );
//                     // Consider sending the Apogee event here as well, or having a voting system.
//                 }
//             }
//         }
//     }
// }
#[embassy_executor::task]
pub async fn recovery_algorithm_task() {
    info!("Barometer reader task started.");

    // --- CONFIGURATION CONSTANTS ---
    // The number of consecutive negative slope readings required to confirm descent.
    const CONSECUTIVE_NEGATIVE_THRESHOLD: usize = 3;

    // History buffers to store recent altitude readings from two different sources.
    // Each entry is a tuple of (altitude, timestamp).
    let mut historical_barometer_altitude_sbg: HistoryBuffer<(f32, Instant), 8> =
        HistoryBuffer::new();
    let mut historical_barometer_altitude_baro: HistoryBuffer<(f32, Instant), 8> =
        HistoryBuffer::new();

    loop {
        // Wait for a new pressure reading from the channel.
        let reading: (f32, f32, u8, Instant) = PRESSURE_CHANNEL.receive().await;
        
        // This variable will hold the calculated altitude.
        let mut altitude = 0.0;

        // Process the reading based on its source identifier.
        if reading.2 == 1 { // Source is the barometer (baro)
            // Hypsometric Formula to convert pressure and temperature to altitude.
            altitude =
                ((powf(101.325 / reading.0, 1.0 / 5.257) - 1.0) * (reading.1 + 273.15)) / 0.0065;
            historical_barometer_altitude_baro.write((altitude, reading.3));
        } else if reading.2 == 0 { // Source is the SBG
            altitude = reading.0; // SBG provides altitude directly.
            info!("Altitude: {}", altitude);
            historical_barometer_altitude_sbg.write((altitude, reading.3));
        }

        // --- Apogee Detection Logic ---
        // Ensure there are enough data points in the buffer to perform a reliable calculation.
        if historical_barometer_altitude_sbg.len() < 8 {
            continue;
        }

        // Get an ordered iterator over the historical data.
        let mut buf_sbg = historical_barometer_altitude_sbg.oldest_ordered();
        let mut buf_baro = historical_barometer_altitude_baro.oldest_ordered();

        // --- SBG Apogee Check ---
        if let Some(mut prev_reading) = buf_sbg.next() {
            // This buffer will store the calculated slopes (max 7 slopes from 8 points).
            let mut slopes: HistoryBuffer<f32, 7> = HistoryBuffer::new();

            // First, iterate through the altitude history and calculate the slope between each point.
            for current_reading in buf_sbg {
                let time_diff_ms = current_reading.1.duration_since(prev_reading.1).as_millis();
                if time_diff_ms > 0 {
                    let slope_mpms = (current_reading.0 - prev_reading.0) / time_diff_ms as f32;
                    // Optional: Add ascent lockout here if needed
                    // if slope_mpms > ASCENT_LOCKOUT { continue; }
                    slopes.write(slope_mpms);
                }
                prev_reading = current_reading;
            }

            // Ensure we have enough slopes to check for a consecutive sequence.
            if slopes.len() >= CONSECUTIVE_NEGATIVE_THRESHOLD {
                // Collect slopes into a vector to easily access the most recent items from the end.
                // In a `no_std` environment, this would typically be a `heapless::Vec`.
                let slopes_vec: Vec<_, DATA_POINTS> = slopes.oldest_ordered().collect();
                let mut consecutive_negative_count = 0;
                let mut sum_of_recent_slopes = 0.0;

                // Check if the last N slopes are all negative by iterating backwards from the end.
                for slope in slopes_vec.iter().rev().take(CONSECUTIVE_NEGATIVE_THRESHOLD) {
                    if **slope <= 0.0 {
                        consecutive_negative_count += 1;
                        sum_of_recent_slopes += *slope;
                    } else {
                        break; // Sequence is broken by a positive slope.
                    }
                }

                // If we found a consecutive sequence of negative slopes, check for apogee.
                if consecutive_negative_count == CONSECUTIVE_NEGATIVE_THRESHOLD {
                    let avg_slope_mpms = sum_of_recent_slopes / CONSECUTIVE_NEGATIVE_THRESHOLD as f32;
                    
                    if avg_slope_mpms <= VALID_DESCENT_RATE {
                        info!(
                            "SBG Apogee detected! Avg speed of last {} readings: {} m/s",
                            CONSECUTIVE_NEGATIVE_THRESHOLD,
                            avg_slope_mpms * 1000.0 // Convert to m/s for logging
                        );
                        
                        if EVENT_CHANNEL.try_send(crate::state_machine::Events::Apogee).is_err() {
                            todo!("Log failure to radio");
                        }
                        break; // Exit loop once apogee is detected.
                    }
                }
            }
        }

        // --- Barometer Apogee Check (Identical logic) ---
        if let Some(mut prev_reading) = buf_baro.next() {
            let mut slopes: HistoryBuffer<f32, 7> = HistoryBuffer::new();

            for current_reading in buf_baro {
                let time_diff_ms = current_reading.1.duration_since(prev_reading.1).as_millis();
                if time_diff_ms > 0 {
                    let slope_mpms = (current_reading.0 - prev_reading.0) / time_diff_ms as f32;
                    slopes.write(slope_mpms);
                }
                prev_reading = current_reading;
            }

            if slopes.len() >= CONSECUTIVE_NEGATIVE_THRESHOLD {
                let slopes_vec: Vec<_, DATA_POINTS> = slopes.oldest_ordered().collect();
                let mut consecutive_negative_count = 0;
                let mut sum_of_recent_slopes = 0.0;

                for slope in slopes_vec.iter().rev().take(CONSECUTIVE_NEGATIVE_THRESHOLD) {
                    if **slope <= 0.0 {
                        consecutive_negative_count += 1;
                        sum_of_recent_slopes += *slope;
                    } else {
                        break;
                    }
                }

                if consecutive_negative_count == CONSECUTIVE_NEGATIVE_THRESHOLD {
                    let avg_slope_mpms = sum_of_recent_slopes / CONSECUTIVE_NEGATIVE_THRESHOLD as f32;
                    
                    if avg_slope_mpms <= VALID_DESCENT_RATE {
                        info!(
                            "Baro Apogee detected! Avg speed of last {} readings: {} m/s",
                            CONSECUTIVE_NEGATIVE_THRESHOLD,
                            avg_slope_mpms * 1000.0
                        );
                        // Consider sending event or using a voting system with the SBG.
                    }
                }
            }
        }
        Timer::after_millis(25).await;
    }
}
