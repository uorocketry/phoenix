use crate::{StateMachineContext, States};
use messages_prost::state::State;

pub struct Context {}

impl StateMachineContext for Context {}

impl From<States> for State {
    fn from(value: States) -> Self {
        match value {
            States::Init => State::Initializing,
            States::Fault => State::Abort, 
            States::WaitForLaunch => State::WaitForTakeoff,
            States::Ascent => State::Ascent,
            States::Idle => State::Unspecified
        }
    }
}