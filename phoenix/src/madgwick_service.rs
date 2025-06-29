use heapless::HistoryBuffer;
use madgwick::Marg;
use messages_prost::sensor::madgwick::Madgwick;
use messages_prost::sensor::madgwick::Quaternion;
use messages_prost::sensor::sbg::EkfQuat;
use messages_prost::sensor::sbg::EkfStatus;
use messages_prost::sensor::sbg::Imu;
use messages_prost::sensor::sbg::SbgData;
/// Service that implements the Madgwick sensor fusion algorithim for orientation
/// This service processes IMU data (accelerometer and gyroscope)
pub struct MadgwickService {
    madgwick: Marg,
    // Store the latest quaternions
    quat_history: HistoryBuffer<Quaternion, 20>,
    // Store configuration parameters
    beta: f32, // 'beta' is the filter gain parameter that determines how much the accelerometer influences the orientation estimation; the higher the value, the more weight the accelerometer data has
    sample_period: f32, // 'sample_period' is the time in seconds between sensor readings; it is reciprocal of the sensor sampling frequency
}

impl MadgwickService {
    // Default values as constants will be used if parameters cannot be used
    const DEFAULT_BETA: f32 = 0.1;
    const DEFAULT_SAMPLE_PERIOD: f32 = 0.01; // 100Hz

    pub fn default() -> Self {
        Self::new(Self::DEFAULT_BETA, Self::DEFAULT_SAMPLE_PERIOD)
    }

    /// New constructor that accepts parameters
    pub fn new(beta: f32, sample_period: f32) -> Self {
        // Create the filter with specified parameters
        let mut madgwick = Marg::new(beta, sample_period);
        let mut quat_history = HistoryBuffer::new(); 

        // Initialize with standard measurements
        let accel = madgwick::F32x3 {
            x: 0.0,
            y: 0.0,
            z: 1.0,
        }; // "z: 1.0" represents the accelerometer pointing in the positive z-direction (upwards)
        let gyro = madgwick::F32x3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        };
        let mag = madgwick::F32x3 {
            x: 1.0,
            y: 0.0,
            z: 0.0,
        }; // "x: 1.0" represents the magnetometer pointing in the positive x-direction

        // Get initial quaternion from filter
        let mut quat = (1.0, 0.0, 0.0, 0.0); // Default identity quaternion with no rotation

        // Apply multiple updates to ensure convergence, and stores the resulting quaternion after each update
        for _ in 0..5 {
            let updated_quat = madgwick.update(mag, gyro, accel);
            
            quat_history.wirte(Quaternion {
                w: updated_quat.0,
                x: updated_quat.1, 
                y: updated_quat.2,
                z: updated_quat.3,
            });
        }

        Self {
            madgwick,
            quat_history, // Use the quaternion from the filter
            beta,
            sample_period,
        }
    }

    /// Method for re-initialization the filter with standard gravity readings
    /// This is mainly used when parameters are changed
    fn initialize(&mut self) {
        // "z: 1.0" represents the accelerometer pointing in the positive z-direction (upwards)
        // If our data looks really off, we can try changing the z value to -1.0
        let accel = madgwick::F32x3 {
            x: 0.0,
            y: 0.0,
            z: 1.0,
        };
        let gyro = madgwick::F32x3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        };
        let mag = madgwick::F32x3 {
            x: 1.0,
            y: 0.0,
            z: 0.0,
        };

        // Apply multiple updates to ensure convergence
        for _ in 0..5 {
            let quat = self.madgwick.update(mag, gyro, accel);
            self.latest_quat = (quat.0, quat.1, quat.2, quat.3);
        }
    }

    /// Method for processing incoming IMU data; returns a new Message with an updated quaternion from the filter
    pub fn process_imu_data(&mut self, data: &Imu) -> Option<Madgwick> {
        let accel = data.accelerometers;
        let gyro = data.gyroscopes;

        let mag = madgwick::F32x3 {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        };
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

        self.quat_history.write(Madgwick { node: 3, data: quat });

        // Store the latest quaternion
        self.latest_quat = (quat.0, quat.1, quat.2, quat.3);

        // match &data.data {
        //     messages::Data::Sensor(sensor) => match &sensor.data {
        //         messages::sensor::SensorData::SbgData(ref sbg_data) => match sbg_data {
        //             SbgData::Imu1(imu_data) => {
        //                 if let (Some(accel), Some(gyro)) = (imu_data.accelerometers, imu_data.gyroscopes) {
        // let mag = madgwick::F32x3 { x: 0.0, y: 0.0, z: 0.0 };
        // let gyro = madgwick::F32x3 {
        //     x: gyro[0],
        //     y: gyro[1],
        //     z: gyro[2],
        // };

        // let accel = madgwick::F32x3 {
        //     x: accel[0],
        //     y: accel[1],
        //     z: accel[2],
        // };

        // let quat = self.madgwick.update(mag, gyro, accel);

        // // Store the latest quaternion
        // self.latest_quat = (quat.0, quat.1, quat.2, quat.3);

        //                     Some(Message::new(
        //                         data.timestamp.clone(),
        //                         data.node.clone(),
        //                         Sensor::new(
        //                             sensor::SensorData::SbgData(
        //                                 SbgData::EkfQuat(
        //                                     EkfQuat {
        //                                         time_stamp: imu_data.time_stamp,
        //                                         quaternion: Some([quat.0, quat.1, quat.2, quat.3]),
        //                                         euler_std_dev: None,
        //                                         status: EkfStatus::new(0),
        //                                     }
        //                                 )
        //                             )
        //                         )
        //                     ))
        //                 } else {
        //                     None
        //                 }
        //             },
        //             _ => None,
        //         },
        //         _ => None,
        //     },
        //     _ => None,
        // }
    }

    /// Method for getting the latest quaternion method
    pub fn get_quaternion(&self) -> Option<&Madgwick> {
        self.quat_history.last()
    }

    /// Method to set new beta value
    pub fn set_beta(&mut self, beta: f32) {
        self.beta = beta;

        self.madgwick = Marg::new(self.beta, self.sample_period);

        self.initialize();
    }

    /// Method to set sample period
    pub fn set_sample_period(&mut self, sample_period: f32) {
        self.sample_period = sample_period;

        self.madgwick = Marg::new(self.beta, self.sample_period);

        self.initialize();
    }

    /// Method to get current beta value
    pub fn get_beta(&self) -> f32 {
        self.beta
    }

    /// Method to get current sample period
    pub fn get_sample_period(&self) -> f32 {
        self.sample_period
    }
}
