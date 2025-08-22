use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::peripherals::{PE12, PE14};
use embedded_hal_1::digital::OutputPin;

pub struct Cameras {
    power: Output<'static>,
    osd: Output<'static>,
}

impl Cameras {
    pub fn new(osd: PE14, power: PE12) -> Self {
        Cameras {
            osd: Output::new(osd, Level::Low, Speed::Low),
            power: Output::new(power, Level::Low, Speed::Low),
        }
    }

    pub fn start_recording(&mut self) {
        // for i in 0..3 {
            self.osd.set_high();
        //     Delay.delay_ms(250);

        //     self.osd.set_low();
        //     Delay.delay_ms(250);
        // }

        // Delay.delay_ms(1000);

        // OSD is reset, now power on
        self.power.set_high();
        // Delay.delay_ms(2000);
        // info!("Camera on");
        // self.power.set_low();

        // Delay.delay_ms(500);

        // self.power.set_low();
        // Delay.delay_ms(2000);
        // self.power.set_high();

        // Delay.delay_ms(1000);
        // self.power.set_high();
        // Delay.delay_ms(250);
        // self.power.set_low();
    }

    pub fn stop_recording(&mut self) {
        // self.power.set_high();
        // Delay.delay_ms(100);
        // self.power.set_low();
        // Delay.delay_ms(200);
        // self.power.set_high();
        // Delay.delay_ms(500);
        // self.power.set_low();
    }
}
