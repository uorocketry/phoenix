//! Recovery pin mapping (moved from main.rs for clarity)
//!
//! MAIN_ARM/TEST = PD6
//! MAIN_FIRE = PD5
//! MAIN_ARM/TEST_B = PD14
//! MAIN_FIRE_B = PD13
//! DROGUE_ARM/TEST = PC11
//! DROGUE_FIRE = PC12
//! DROGUE_ARM/TEST_B = PD2
//! DROGUE_FIRE_B = PD1
//! MAIN_MCU_EMATCH_SENSE = PA2
//! MAIN_MCU_EMATCH_SENSE_B = PB0
//! DROGUE_MCU_EMATCH_SENSE = PA3
//! DROGUE_MCU_EMATCH_SENSE_B = PC5

use embassy_stm32::gpio::Output;

pub struct Arming {
    pub main: Output<'static>,
    pub drogue: Output<'static>,
    pub main_b: Output<'static>,
    pub drogue_b: Output<'static>,
}

pub struct Fire {
    pub main: Output<'static>,
    pub drogue: Output<'static>,
    pub main_b: Output<'static>,
    pub drogue_b: Output<'static>,
}

pub struct RecoveryManager {
    pub arming: Arming,
    pub fire: Fire,
}
