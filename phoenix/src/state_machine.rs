use defmt::info;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use messages_prost::state::State;
use smlang::statemachine;

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

pub struct Context {}

impl StateMachineContext for Context {}

impl From<States> for State {
    fn from(value: States) -> Self {
        match value {
            States::Fuck => State::Abort,
            States::Init => State::Initializing,
            States::Fault => State::Abort,
            States::WaitForLaunch => State::WaitForTakeoff,
            States::Ascent => State::Ascent,
            States::Descent => State::Descent,
            States::DrogueDescent => State::Descent,
            States::MainDescent => State::TerminalDescent,
            States::Landed => State::WaitForRecovery,
        }
    }
}

#[embassy_executor::task]
pub async fn sm_task(spawner: Spawner, state_machine: StateMachine<Context>) {
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
