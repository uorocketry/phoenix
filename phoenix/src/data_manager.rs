use common_arm::HydraError;
use messages::command::RadioRate;
use messages::sensor::{Gps, SbgData};
use messages::state::State;
use messages::{CanData, CanMessage, Common, RadioData, RadioMessage};
use stm32h7xx_hal::rcc::ResetReason;

#[derive(Clone)]
pub struct DataManager<'a> {
    pub air: Option<RadioMessage<'a>>,
    pub ekf_nav: Option<RadioMessage<'a>>,
    pub ekf_quat: Option<RadioMessage<'a>>,
    pub madgwick_quat: Option<RadioMessage<'a>>,
    pub imu: Option<RadioMessage<'a>>,
    pub utc_time: Option<RadioMessage<'a>>,
    pub gps_vel: Option<RadioMessage<'a>>,
    pub gps_pos: Option<RadioMessage<'a>>,
    pub recovery_sensing: Option<RadioMessage<'a>>,
    pub nav_pos_l1h: Option<RadioMessage<'a>>,
    pub state: Option<State>,
    pub reset_reason: Option<ResetReason>,
    pub logging_rate: Option<RadioRate>,
}

impl <'a> DataManager<'a> {
    pub fn new() -> Self {
        Self {
            air: None,
            ekf_nav: None,
            ekf_quat: None,
            madgwick_quat: None,
            imu: None,
            utc_time: None,
            gps_vel: None,
            gps_pos: None,
            state: None,
            reset_reason: None,
            logging_rate: Some(RadioRate::Slow), // start slow.
            recovery_sensing: None,
            nav_pos_l1h: None,
        }
    }

    pub fn get_logging_rate(&mut self) -> RadioRate {
        if let Some(rate) = self.logging_rate.take() {
            let rate_cln = rate.clone();
            self.logging_rate = Some(rate);
            return rate_cln;
        }
        self.logging_rate = Some(RadioRate::Slow);
        RadioRate::Slow
    }

    /// Do not clone instead take to reduce CPU load.
    pub fn take_sensors(&mut self) -> [Option<RadioMessage>; 10] {
        [
            self.air.take(),
            self.ekf_nav.take(),
            self.ekf_quat.take(),
            self.madgwick_quat.take(),
            self.imu.take(),
            self.utc_time.take(),
            self.gps_vel.take(),
            self.gps_pos.take(),
            self.nav_pos_l1h.take(),
            self.recovery_sensing.take(),
        ]
    }

    pub fn clone_states(&self) -> [Option<State>; 1] {
        [self.state.clone()]
    }

    pub fn clone_reset_reason(&self) -> Option<ResetReason> {
        self.reset_reason
    }

    pub fn set_reset_reason(&mut self, reset: ResetReason) {
        self.reset_reason = Some(reset);
    }

    pub fn handle_command(&mut self, data: CanMessage) -> Result<(), HydraError> {
        if let CanData::Common(Common::Command(command)) = data.data {
            match command {
                messages::command::Command::PowerDown(_) => {
                    crate::app::sleep_system::spawn().ok();
                }
                messages::command::Command::RadioRateChange(command_data) => {
                    self.logging_rate = Some(command_data.rate);
                }
                _ => {} // We don't care atm about these other commands.
            }
        }
        
        Ok(())
    }

    pub fn handle_data(&mut self, data: RadioMessage) {
        match data.data {
            RadioData::Sbg(ref sbg_data) => match sbg_data {
                SbgData::UtcTime(utc_time) => {
                    self.utc_time = Some(data);
                },
                SbgData::Air(_) => {
                    self.air = Some(data);
                },
                SbgData::EkfQuat(ekf_quat) => {
                    self.ekf_quat = Some(data);
                },
                SbgData::EkfNav(ekf_nav) => {
                    self.ekf_nav = Some(data);
                },
                SbgData::Imu(imu) => {
                    self.imu = Some(data);
                },
                SbgData::GpsVel(gps_vel) => {
                    self.gps_vel = Some(data);
                },
                SbgData::GpsPos(gps_pos) => {
                    self.gps_pos = Some(data);
                },
            },
            RadioData::Gps(Gps::NavPosLlh(_)) => {
                self.gps_pos = Some(data);
            },
            messages::Data::Sensor(ref sensor) => match sensor.data {
                messages::sensor::SensorData::RecoverySensing(_) => {
                    self.recovery_sensing = Some(data);
                }
                messages::sensor::SensorData::ResetReason(_) => {}
            },
            RadioData::Common(Common::State(state)) => {
                self.state = Some(state);
            },
            _ => {}
        }
    }

    pub fn store_madgwick_result(&mut self, result: RadioMessage<'a>) {
        self.madgwick_quat = Some(result);
    }
}

impl Default for DataManager<'_> {
    fn default() -> Self {
        Self::new()
    }
}
