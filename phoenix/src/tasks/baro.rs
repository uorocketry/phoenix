use defmt::*;
use embassy_executor::task;
use embassy_stm32::{gpio::Output, spi::Spi};
use embassy_time::Instant;
use embassy_time::{Duration, Timer};
use heapless::HistoryBuffer;
use libm::powf;

use common_arm::drivers::ms5611::{Ms5611, OversamplingRatio};

#[task]
pub async fn baro_reader_task(
    mut baro: Ms5611<
        Spi<'static, embassy_stm32::mode::Blocking>,
        Output<'static>,
        embassy_time::Delay,
    >,
) {
    info!("Barometer reader task started.");
    const GROUND_HEIGHT: f32 = 300.0; // meters ASL
    const ASCENT_LOCKOUT: f32 = 0.1;
    const VALID_DESCENT_RATE: f32 = -0.005; // meters per millisecond

    let mut historical_barometer_altitude: HistoryBuffer<(f32, Instant), 20> = HistoryBuffer::new();

    loop {
        match baro.read_pressure_temperature(OversamplingRatio::Osr4096) {
            Ok(reading) => {
                let altitude =
                    ((powf(101.325 / reading.1, 1.0 / 5.257) - 1.0) * (25.0 + 273.15)) / 0.0065;
                historical_barometer_altitude.write((altitude, Instant::now()));

                if historical_barometer_altitude.len() < 8 {
                    continue;
                }

                let mut buf = historical_barometer_altitude.oldest_ordered();
                if let Some(mut prev_reading) = buf.next() {
                    let mut avg_sum: f32 = 0.0;
                    let mut datapoints_used = 0;
                    for current_reading in buf {
                        let time_diff =
                            current_reading.1.duration_since(prev_reading.1).as_millis();
                        if time_diff == 0 {
                            continue;
                        }
                        let slope = (current_reading.0 - prev_reading.0) / time_diff as f32;
                        if slope > ASCENT_LOCKOUT {
                            continue;
                        }
                        avg_sum += slope;
                        datapoints_used += 1;
                        prev_reading = current_reading;
                    }
                    if datapoints_used > 0 {
                        let avg_slope = avg_sum / (datapoints_used as f32);
                        if avg_slope <= VALID_DESCENT_RATE {
                            info!(
                                "Apogee detected! Average vertical speed: {} m/s",
                                avg_slope * 1000.0
                            );
                        }
                    }
                }
            }
            Err(_e) => {}
        }
        Timer::after(Duration::from_millis(100)).await;
    }
}
