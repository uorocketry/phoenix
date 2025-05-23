use common_arm::HydraError;
use messages::command::RadioRate;
use messages::state::StateData;
use messages::Message;
use stm32h7xx_hal::rcc::ResetReason;
#[derive(Clone)]
pub struct DataManager {
    pub air: Option<Message>,
    pub ekf_nav_1: Option<Message>,
    pub ekf_nav_2: Option<Message>,
    pub ekf_nav_acc: Option<Message>,
    pub ekf_quat: Option<Message>,
    pub madgwick_quat: Option<Message>,
    pub imu_1: Option<Message>,
    pub imu_2: Option<Message>,
    pub utc_time: Option<Message>,
    pub gps_vel: Option<Message>,
    pub gps_vel_acc: Option<Message>,
    pub gps_pos_1: Option<Message>,
    pub gps_pos_2: Option<Message>,
    pub gps_pos_acc: Option<Message>,
    pub state: Option<StateData>,
    pub reset_reason: Option<ResetReason>,
    pub logging_rate: Option<RadioRate>,
    pub recovery_sensing: Option<Message>,
    pub nav_pos_l1h: Option<Message>,
    // Barometer
    pub baro_temperature: Option<f32>,
    pub baro_pressure: Option<f32>,
}

impl DataManager {
    pub fn new() -> Self {
        Self {
            air: None,
            ekf_nav_1: None,
            ekf_nav_2: None,
            ekf_nav_acc: None,
            ekf_quat: None,
            madgwick_quat: None,
            imu_1: None,
            imu_2: None,
            utc_time: None,
            gps_vel: None,
            gps_vel_acc: None,
            gps_pos_1: None,
            gps_pos_2: None,
            gps_pos_acc: None,
            state: None,
            reset_reason: None,
            logging_rate: Some(RadioRate::Slow), // start slow.
            recovery_sensing: None,
            nav_pos_l1h: None,
            baro_temperature: None,
            baro_pressure: None,
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
    pub fn take_sensors(&mut self) -> [Option<Message>; 16] {
        [
            self.air.take(),
            self.ekf_nav_1.take(),
            self.ekf_nav_2.take(),
            self.ekf_nav_acc.take(),
            self.ekf_quat.take(),
            self.madgwick_quat.take(),
            self.imu_1.take(),
            self.imu_2.take(),
            self.utc_time.take(),
            self.gps_vel.take(),
            self.gps_vel_acc.take(),
            self.gps_pos_1.take(),
            self.gps_pos_2.take(),
            self.gps_pos_acc.take(),
            self.nav_pos_l1h.take(),
            self.recovery_sensing.take(),
        ]
    }

    pub fn clone_states(&self) -> [Option<StateData>; 1] {
        [self.state.clone()]
    }

    pub fn clone_reset_reason(&self) -> Option<ResetReason> {
        self.reset_reason
    }

    pub fn set_reset_reason(&mut self, reset: ResetReason) {
        self.reset_reason = Some(reset);
    }

    pub fn handle_command(&mut self, data: Message) -> Result<(), HydraError> {
        match data.data {
            messages::Data::Command(command) => match command.data {
                messages::command::CommandData::PowerDown(_) => {
                    crate::app::sleep_system::spawn().ok();
                }
                messages::command::CommandData::RadioRateChange(command_data) => {
                    self.logging_rate = Some(command_data.rate);
                }
                _ => {
                    // We don't care atm about these other commands.
                }
            },
            _ => {
                // we can disregard all other messages for now.
            }
        }
        Ok(())
    }
    pub fn handle_data(&mut self, data: Message) {
        match data.data {
            messages::Data::Sensor(ref sensor) => match sensor.data {
                messages::sensor::SensorData::SbgData(ref sbg_data) => match sbg_data {
                    messages::sensor::SbgData::EkfNavAcc(_) => {
                        self.ekf_nav_acc = Some(data);
                    }
                    messages::sensor::SbgData::GpsPosAcc(_) => {
                        self.gps_pos_acc = Some(data);
                    }
                    messages::sensor::SbgData::Air(_) => {
                        self.air = Some(data);
                    }
                    messages::sensor::SbgData::EkfNav1(_) => {
                        self.ekf_nav_1 = Some(data);
                    }
                    messages::sensor::SbgData::EkfNav2(_) => {
                        self.ekf_nav_2 = Some(data);
                    }
                    messages::sensor::SbgData::EkfQuat(_) => {
                        self.ekf_quat = Some(data);
                    }
                    messages::sensor::SbgData::GpsVel(_) => {
                        self.gps_vel = Some(data);
                    }
                    messages::sensor::SbgData::GpsVelAcc(_) => {
                        self.gps_vel_acc = Some(data);
                    }
                    messages::sensor::SbgData::Imu1(_) => {
                        self.imu_1 = Some(data);
                    }
                    messages::sensor::SbgData::Imu2(_) => {
                        self.imu_2 = Some(data);
                    }
                    messages::sensor::SbgData::UtcTime(_) => {
                        self.utc_time = Some(data);
                    }
                    messages::sensor::SbgData::GpsPos1(_) => {
                        self.gps_pos_1 = Some(data);
                    }
                    messages::sensor::SbgData::GpsPos2(_) => {
                        self.gps_pos_2 = Some(data);
                    }
                },
                messages::sensor::SensorData::RecoverySensing(_) => {
                    self.recovery_sensing = Some(data);
                }
                messages::sensor::SensorData::NavPosLlh(_) => {
                    self.nav_pos_l1h = Some(data);
                }
                messages::sensor::SensorData::ResetReason(_) => {}
            },
            messages::Data::State(state) => {
                self.state = Some(state.data);
            }
            // messages::Data::Command(command) => match command.data {
            //     messages::command::CommandData::RadioRateChange(command_data) => {
            //         self.logging_rate = Some(command_data.rate);
            //     }
            //     messages::command::CommandData::DeployDrogue(_) => {}
            //     messages::command::CommandData::DeployMain(_) => {}
            //     messages::command::CommandData::PowerDown(_) => {}
            // },
            _ => {}
        }
    }
    pub fn store_madgwick_result(&mut self, result: Message) {
        self.madgwick_quat = Some(result);
    }
}

impl Default for DataManager {
    fn default() -> Self {
        Self::new()
    }
}
