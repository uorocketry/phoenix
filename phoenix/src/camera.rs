use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::peripherals::{PE12, PE14};
use embassy_time::Delay;
use embedded_hal_1::delay::DelayNs;
use embedded_hal_1::digital::OutputPin;

pub struct Cameras {
    power: Output<'static>,
    osd: Output<'static>,
}

impl Cameras {
    pub fn new(osd: PE14, power: PE12) -> Self {
        Cameras {
            osd: Output::new(osd, Level::High, Speed::Low),
            power: Output::new(power, Level::High, Speed::Low),
        }
    }

    pub fn start_recording(&mut self) {
        for i in 0..2 {
            self.osd.set_high();
            Delay.delay_ms(100);

            self.osd.set_low(); 
            Delay.delay_ms(100); 
        }

        // OSD is reset, now power on 
        self.power.set_low(); 
        Delay.delay_ms(1000);
        self.power.set_high(); 

        Delay.delay_ms(100); 
        self.power.set_low(); 
        Delay.delay_ms(100); 
        self.power.set_high(); 
    }

    pub fn stop_recording(&mut self) {
        self.power.set_low();
        Delay.delay_ms(100); 
        self.power.set_high(); 
        Delay.delay_ms(200);
        self.power.set_low(); 
        Delay.delay_ms(1000);
        self.power.set_high(); 

    }
}
