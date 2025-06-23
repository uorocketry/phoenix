use madgwick::Marg;
use messages::{Message, sensor::{self, SbgData, EkfQuat}};
use messages::sensor::Sensor;
use messages::sensor_status::EkfStatus;

/// Service that implements the Madgwick sensor fusion algorithm for orientation
/// This service processes IMU data (accelerometer and gyroscope)
pub struct MadgwickService {
    madgwick: Marg,
    // Store the latest quaternion
    latest_quat: (f32, f32, f32, f32),
    // Store configuration parameters
    beta: f32, // 'beta' is the filter gain parameter that determines how much the accelerometer influences the orientation estimation
    sample_period: f32, // 'sample_period' is the time in seconds between sensor readings
}

impl MadgwickService {
    // Default values as constants will be used if parameters can't be used
    const DEFAULT_BETA: f32 = 0.1;
    const DEFAULT_SAMPLE_PERIOD: f32 = 0.01; // 100Hz

    /// Method for creating a new instance of 'MadgwickService' with default parameters 
    pub fn new() -> Self {
        // Use the version with parameters but provide defaults incase we can't get parameters for some reason
        Self::new_with_params(Self::DEFAULT_BETA, Self::DEFAULT_SAMPLE_PERIOD)
    }
    
    /// New constructor that accepts parameters
    pub fn new_with_params(beta: f32, sample_period: f32) -> Self {
        // Create the filter with specified parameters
        let mut madgwick = Marg::new(beta, sample_period);
        
        // Initialize with standard measurements
        let accel = madgwick::F32x3 { x: 0.0, y: 0.0, z: 1.0 }; // gravity pointing down (positive z)
        let gyro = madgwick::F32x3 { x: 0.0, y: 0.0, z: 0.0 };  // no rotation
        let mag = madgwick::F32x3 { x: 1.0, y: 0.0, z: 0.0 };   // magnetic field pointing north (positive x)
        
        // Get initial quaternion from filter
        let mut quat = (1.0, 0.0, 0.0, 0.0); // Default identity quaternion (no rotation)
        
        // Apply multiple updates to ensure convergence
        for _ in 0..5 {
            let updated_quat = madgwick.update(mag, gyro, accel);
            quat = (updated_quat.0, updated_quat.1, updated_quat.2, updated_quat.3);
        }
        
        Self {
            madgwick,
            latest_quat: quat,
            beta,
            sample_period,
        }
    }
    
    /// Method for re-initialization the filter with standard gravity readings
    /// This is mainly used when parameters are changed
    fn initialize(&mut self) {
        // "z: 1.0" represents the accelerometer pointing in the positive z-direction (upwards)
        // If our data looks really off, we can try changing the z value to -1.0
        let accel = madgwick::F32x3 { x: 0.0, y: 0.0, z: 1.0 }; 
        let gyro = madgwick::F32x3 { x: 0.0, y: 0.0, z: 0.0 };
        let mag = madgwick::F32x3 { x: 1.0, y: 0.0, z: 0.0 };
        
        // Apply multiple updates to ensure convergence
        for _ in 0..5 {
            let quat = self.madgwick.update(mag, gyro, accel);
            self.latest_quat = (quat.0, quat.1, quat.2, quat.3);
        }
    }
    
    /// Method for processing incoming IMU data from Messages (legacy support for CAN messages)
    /// Returns a new Message with updated quaternion from the filter
    pub fn process_imu_data(&mut self, data: &Message) -> Option<Message> {
        match &data.data {
            messages::Data::Sensor(sensor) => match &sensor.data {
                messages::sensor::SensorData::SbgData(ref sbg_data) => match sbg_data {
                    SbgData::Imu1(imu_data) => {
                        if let (Some(accel), Some(gyro)) = (imu_data.accelerometers, imu_data.gyroscopes) {
                            let mag = madgwick::F32x3 { x: 0.0, y: 0.0, z: 0.0 }; // No magnetometer available
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
                            
                            // Create and return a new Message with the computed quaternion
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

    /// Process raw IMU data directly
    /// Returns the computed quaternion as [w, x, y, z]
    pub fn process_raw_imu_data(&mut self, gyro: [f32; 3], accel: [f32; 3]) -> [f32; 4] {
        let mag = madgwick::F32x3 { x: 0.0, y: 0.0, z: 0.0 }; // No magnetometer available
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
        
        // Return as array format [w, x, y, z]
        [quat.0, quat.1, quat.2, quat.3]
    }

    /// Update filter with raw data and store result directly in DataManager
    pub fn update_and_store(&mut self, gyro: [f32; 3], accel: [f32; 3], data_manager: &mut crate::data_manager::DataManager) {
        let quaternion = self.process_raw_imu_data(gyro, accel);
        data_manager.imu_quaternion = Some(quaternion);
    }

    /// Get the latest quaternion as tuple (w, x, y, z)
    pub fn get_quaternion(&self) -> (f32, f32, f32, f32) {
        self.latest_quat
    }

    /// Get the latest quaternion as array [w, x, y, z] - matches DataManager format
    pub fn get_quaternion_array(&self) -> [f32; 4] {
        [self.latest_quat.0, self.latest_quat.1, self.latest_quat.2, self.latest_quat.3]
    }

    /// Set new beta value (filter gain parameter)
    /// Higher values give more weight to accelerometer data
    pub fn set_beta(&mut self, beta: f32) {
        self.beta = beta;
        self.madgwick = Marg::new(self.beta, self.sample_period);
        self.initialize();
    }
    
    /// Set new sample period (time between sensor readings in seconds) 
    /// Should match the actual sensor sampling rate
    pub fn set_sample_period(&mut self, sample_period: f32) {
        self.sample_period = sample_period;
        self.madgwick = Marg::new(self.beta, self.sample_period);
        self.initialize();
    }
    
    /// Get current beta value
    pub fn get_beta(&self) -> f32 {
        self.beta
    }
    
    /// Get current sample period
    pub fn get_sample_period(&self) -> f32 {
        self.sample_period
    }

    /// Reset the filter to initial state
    /// Useful for error recovery or when sensor calibration changes
    pub fn reset(&mut self) {
        self.madgwick = Marg::new(self.beta, self.sample_period);
        self.initialize();
    }

    /// Update filter parameters without resetting state
    /// Use this when we want to change parameters but keep the current orientation estimate
    pub fn update_parameters(&mut self, beta: f32, sample_period: f32) {
        self.beta = beta;
        self.sample_period = sample_period;
        self.madgwick = Marg::new(self.beta, self.sample_period);
    }

    /// Check if the filter has been properly initialized
    /// Returns true if the quaternion is not the default identity quaternion
    pub fn is_initialized(&self) -> bool {
        // Check if quaternion has changed from identity (allowing for small floating point errors)
        let (w, x, y, z) = self.latest_quat;
        !(w > 0.99 && x.abs() < 0.01 && y.abs() < 0.01 && z.abs() < 0.01)
    }

    /// Get quaternion magnitude (should be close to 1.0 for a valid quaternion)
    pub fn get_quaternion_magnitude(&self) -> f32 {
        let (w, x, y, z) = self.latest_quat;
        (w * w + x * x + y * y + z * z).sqrt()
    }
}