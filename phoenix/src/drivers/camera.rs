use embassy_stm32::gpio::{Output, Level, Speed};
use embassy_time::Delay;
use embedded_hal_1::delay::DelayNs;

// Runs the same boot/record/stop sequence that was in main.rs, unchanged.
pub async fn run_boot_and_record(mut cam_trigger: Output<'static>, mut cam_trigger_b: Output<'static>) {
    // power on
    cam_trigger.set_high();
    Delay.delay_ms(2_000);
    cam_trigger.set_low();
    Delay.delay_ms(100);
    // trigger the camera
    cam_trigger.set_high();
    Delay.delay_ms(500);
    cam_trigger.set_low();
    Delay.delay_ms(10_000);
    // stop recording
    cam_trigger.set_high();
    Delay.delay_ms(500);
    cam_trigger.set_low();

    // currently unused, but kept to preserve signature
    let _ = cam_trigger_b.set_low();
}


