use defmt::*;
use embassy_executor::task;
use embassy_time::{Duration, Timer};
use smlang::statemachine;

use crate::traits::Context;

// Single source of truth for the state machine
statemachine! {
    transitions: {
        *Init + Start = WaitForLaunch,
        WaitForLaunch + Launch = Ascent,
        Ascent + Apogee = Descent,
        Descent + MainDeployment = Fuck,
        Descent + DrogueDeployment = DrogueDescent,
        DrogueDescent + MainDeployment = MainDescent,
        MainDescent + NoMovement = Landed,
        Fault + FaultCleared = _,
        _ + FaultDetected = Fault,
    }
}

#[task]
pub async fn sm_task(_spawner: embassy_executor::Spawner, mut state_machine: StateMachine<Context>) {
    info!("State Machine task started.");
    loop {
        match state_machine.state {
            States::Ascent => {}
            States::Fault => {}
            States::Init => {}
            States::WaitForLaunch => {}
            States::Descent => {}
            States::DrogueDescent => {}
            States::Fuck => {}
            States::Landed => {}
            States::MainDescent => {}
        }
        Timer::after(Duration::from_millis(1000)).await;
    }
}
