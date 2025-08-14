use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::peripherals::{PE12, PE14};
use embassy_time::Delay;
use embedded_hal_1::delay::DelayNs;
use embedded_hal_1::digital::OutputPin;

pub struct Cameras {
    trigger_a: Output<'static>,
    trigger_b: Output<'static>,
}

impl Cameras {
    pub fn new(trigger_a: PE14, trigger_b: PE12) -> Self {
        Cameras {
            trigger_b: Output::new(trigger_b, Level::Low, Speed::Low),
            trigger_a: Output::new(trigger_a, Level::Low, Speed::Low),
        }
    }

    pub fn start_recording(&mut self) {
        // power on
        // self.trigger_b.set_high();
        // self.trigger_a.set_high();
        // Delay.delay_ms(2_000);
        // self.trigger_a.set_low();
        // self.trigger_b.set_low();
        // Delay.delay_ms(1000);
        // trigger the camera
        self.trigger_b.set_high();
        self.trigger_a.set_high();
        Delay.delay_ms(10);
        self.trigger_a.set_low();
        self.trigger_b.set_low();
    }

    pub fn stop_recording(&mut self) {
        self.trigger_b.set_high();
        self.trigger_a.set_high();
        Delay.delay_ms(10);
        self.trigger_a.set_low();
        self.trigger_b.set_low();
        Delay.delay_ms(1000);
        // power off
        self.trigger_b.set_high();
        self.trigger_a.set_high();
        Delay.delay_ms(2_000);
        self.trigger_a.set_low();
        self.trigger_b.set_low();
    }
}
