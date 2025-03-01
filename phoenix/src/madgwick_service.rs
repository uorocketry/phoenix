use madgwick::Marg;
use messages::{Message, sensor::{self, SbgData, EkfQuat}};
use messages::sensor::Sensor;
use messages::sensor_status::EkfStatus;

pub struct MadgwickService {
    madgwick: Marg,
    // Store the latest quaternion
    latest_quat: (f32, f32, f32, f32),
    // Store configuration parameters
    beta: f32,
    sample_period: f32,
}

impl MadgwickService {
    pub fn new() -> Self {
        let beta = 0.1;
        let sample_period = 0.01; // 100Hz
        
        let mut instance = Self {
            madgwick: Marg::new(beta, sample_period),
            latest_quat: (1.0, 0.0, 0.0, 0.0), // Identity quaternion
            beta,
            sample_period,
        };
        
        // Initialize the filter with standard gravity to ensure stability
        instance.initialize();
        
        instance
    }
    
    // Initialize the filter with standard gravity readings
    fn initialize(&mut self) {
        let accel = madgwick::F32x3 { x: 0.0, y: 0.0, z: 1.0 };
        let gyro = madgwick::F32x3 { x: 0.0, y: 0.0, z: 0.0 };
        let mag = madgwick::F32x3 { x: 1.0, y: 0.0, z: 0.0 };
        
        for _ in 0..5 {
            let quat = self.madgwick.update(mag, gyro, accel);
            self.latest_quat = (quat.0, quat.1, quat.2, quat.3);
        }
    }

    pub fn process_imu_data(&mut self, data: &Message) -> Option<Message> {
        match &data.data {
            messages::Data::Sensor(sensor) => match &sensor.data {
                messages::sensor::SensorData::SbgData(ref sbg_data) => match sbg_data {
                    SbgData::Imu1(imu_data) => {
                        if let (Some(accel), Some(gyro)) = (imu_data.accelerometers, imu_data.gyroscopes) {
                            let mag = madgwick::F32x3 { x: 0.0, y: 0.0, z: 0.0 };

                            let gyro = madgwick::F32x3 {
                                x: gyro[0],
                                y: gyro[1],
                                z: gyro[2],
                            };
                            
                            let accel = madgwick::F32x3 {
                                x: accel[0],
                                y: accel[1],
                                z: accel[2],
                            };

                            let quat = self.madgwick.update(mag, gyro, accel);
                            
                            // Store the latest quaternion
                            self.latest_quat = (quat.0, quat.1, quat.2, quat.3);
                            
                            Some(Message::new(
                                data.timestamp.clone(),
                                data.node.clone(),
                                Sensor::new(
                                    sensor::SensorData::SbgData(
                                        SbgData::EkfQuat(
                                            EkfQuat {
                                                time_stamp: imu_data.time_stamp,
                                                quaternion: Some([quat.0, quat.1, quat.2, quat.3]),
                                                euler_std_dev: None,
                                                status: EkfStatus::new(0),
                                            }
                                        )
                                    )
                                )
                            ))
                        } else {
                            None
                        }
                    },
                    _ => None,
                },
                _ => None,
            },
            _ => None,
        }
    }

    pub fn get_quaternion(&self) -> (f32, f32, f32, f32) {
        self.latest_quat
    }

    pub fn set_beta(&mut self, beta: f32) {
        self.beta = beta;
        
        self.madgwick = Marg::new(self.beta, self.sample_period);
        
        self.initialize();
    }
}