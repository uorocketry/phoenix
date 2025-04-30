use madgwick::Marg;
use messages::{RadioData, RadioMessage};
use messages::sensor::{EkfQuat, SbgData};
use messages::sensor_status::EkfStatus;

/// Service that implements the Madgwick sensor fusion algorithim for orientation
/// This service processes IMU data (accelerometer and gyroscope)
pub struct MadgwickService {
    madgwick: Marg,
    // Store the latest quaternion
    latest_quat: (f32, f32, f32, f32),
    // Store configuration parameters
    beta: f32, // 'beta' is the filter gain parameter that determines how much the accelerometer influences the orientation estimation; the higher the value, the more weight the accelerometer data has
    sample_period: f32, // 'sample_period' is the time in seconds between sensor readings; it is reciprocal of the sensor sampling frequency
}

impl MadgwickService {
    // Default values as constants will be used if parameters cannot be used
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
        let accel = madgwick::F32x3 { x: 0.0, y: 0.0, z: 1.0 }; // "z: 1.0" represents the accelerometer pointing in the positive z-direction (upwards)
        let gyro = madgwick::F32x3 { x: 0.0, y: 0.0, z: 0.0 };
        let mag = madgwick::F32x3 { x: 1.0, y: 0.0, z: 0.0 }; // "x: 1.0" represents the magnetometer pointing in the positive x-direction 
        
        // Get initial quaternion from filter
        let mut quat = (1.0, 0.0, 0.0, 0.0); // Default identity quaternion with no rotation
        
        // Apply multiple updates to ensure convergence, and stores the resulting quaternion after each update
        for _ in 0..5 {
            let updated_quat = madgwick.update(mag, gyro, accel);
            quat = (updated_quat.0, updated_quat.1, updated_quat.2, updated_quat.3);
        }
        
        Self {
            madgwick,
            latest_quat: quat, // Use the quaternion from the filter
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
    
    /// Method for processing incoming IMU data. Returns a new RadioMessage with an updated quaternion from the filter.
    pub fn process_imu_data(&mut self, data: &RadioMessage) -> Option<RadioMessage> {
        match &data.data {
            RadioData::Sbg(SbgData::Imu(imu_data))
                if imu_data.accelerometers.is_some() && imu_data.gyroscopes.is_some() =>
            {
                // Unwrap cannot panic here because we checked is_some() above
                let accel = imu_data.accelerometers.unwrap();
                let gyro = imu_data.gyroscopes.unwrap();

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

                Some(RadioMessage::new(
                    data.timestamp.clone(),
                    data.node.clone(),
                    RadioData::Sbg(SbgData::EkfQuat(EkfQuat {
                        time_stamp: imu_data.time_stamp,
                        quaternion: Some([quat.0, quat.1, quat.2, quat.3]),
                        euler_std_dev: None,
                        status: EkfStatus::new(0),
                    })),
                ))
            }
            _ => None,
        }
    }

    /// Method for getting the latest quaternion method
    pub fn get_quaternion(&self) -> (f32, f32, f32, f32) {
        self.latest_quat
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
