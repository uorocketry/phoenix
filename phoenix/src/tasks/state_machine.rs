use defmt::*;
use embassy_executor::task;
use embassy_time::{Duration, Timer};
use smlang::statemachine;

use crate::traits::Context;

// Moved from main.rs for cleanliness. Behavior unchanged.
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

// Original signature (kept for reference; requires Context to implement StateMachineContext):
// #[task]
// pub async fn sm_task(_spawner: embassy_executor::Spawner, state_machine: StateMachine<Context>) {
//     info!("State Machine task started.");
//     loop {
//         match state_machine.state {
//             States::Ascent => {}
//             States::Fault => {}
//             States::Init => {}
//             States::WaitForLaunch => {}
//             States::Descent => {}
//             States::DrogueDescent => {}
//             States::Fuck => {}
//             States::Landed => {}
//             States::MainDescent => {}
//         }
//         Timer::after(Duration::from_millis(1000)).await;
//     }
// }

// Compilable stub until Context implements the required trait. Behavior preserved in comments above.
#[task]
pub async fn sm_task_stub() {
    info!("State Machine task stub started.");
    loop {
        Timer::after(Duration::from_millis(1000)).await;
    }
}
