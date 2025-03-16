#![no_std]

use madgwick::Marg;

pub struct MadgwickTest {
    madgwick: Marg,
    // Store a known good quaternion for initial testing
    initial_quat: (f32, f32, f32, f32),
}

impl MadgwickTest {
    // Default values as constants will be used if parameters cannot be used
    const DEFAULT_BETA: f32 = 0.1;
    const DEFAULT_SAMPLE_PERIOD: f32 = 0.01; // 100Hz
    
    pub fn new() -> Self {
        // Beta and sample period values will be passed through the filter
        Self::new_with_params(Self::DEFAULT_BETA, Self::DEFAULT_SAMPLE_PERIOD)
    }
    
    // New constructor that accepts parameters
    pub fn new_with_params(beta: f32, sample_period: f32) -> Self {
        // Create the filter with specified parameters
        let mut madgwick = Marg::new(beta, sample_period);
        
        // Initialize with standard gravity measurements
        let accel = madgwick::F32x3 { x: 0.0, y: 0.0, z: 1.0 }; // "z: 1.0" represents the accelerometer pointing in the positive z-direction (upwards)
        let gyro = madgwick::F32x3 { x: 0.0, y: 0.0, z: 0.0 };
        let mag = madgwick::F32x3 { x: 1.0, y: 0.0, z: 0.0 }; // "x: 1.0" represents the magnetometer pointing in the positive x-direction 
        
        // Get initial quaternion from filter
        let mut quat = (1.0, 0.0, 0.0, 0.0); // Default identity quaternion
        
        // Apply multiple updates to ensure convergence
        for _ in 0..5 {
            let updated_quat = madgwick.update(mag, gyro, accel);
            quat = (updated_quat.0, updated_quat.1, updated_quat.2, updated_quat.3);
        }
        
        Self {
            madgwick,
            initial_quat: quat, // Use the quaternion from the filter
        }
    }

    pub fn update(&mut self, accel: [f32; 3], gyro: [f32; 3]) -> (f32, f32, f32, f32) {
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
        (quat.0, quat.1, quat.2, quat.3)
    }

    pub fn get_quaternion(&self) -> (f32, f32, f32, f32) {
        // Return our stored quaternion
        self.initial_quat
    }
    
    // Methods to get and set parameters
    pub fn get_beta(&self) -> f32 {
        Self::DEFAULT_BETA // For now we return the default, would need access to inner filter
    }
    
    pub fn get_sample_period(&self) -> f32 {
        Self::DEFAULT_SAMPLE_PERIOD // For now we return the default, would need access to inner filter
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // Intiaiization Test
    #[test]
    fn test_madgwick_initialization() {
        let service = MadgwickTest::new();
        
        // Get the quaternion (already initialized during construction)
        let (w, x, y, z) = service.get_quaternion();
        
        // Check quaternion is roughly identity (or close to it)
        assert!(w > 0.9, "Expected w to be close to 1.0, got {}", w);
        assert!(x.abs() < 0.1, "Expected x to be close to 0.0, got {}", x);
        assert!(y.abs() < 0.1, "Expected y to be close to 0.0, got {}", y);
        assert!(z.abs() < 0.1, "Expected z to be close to 0.0, got {}", z);
    }
    
    // Custom Parameters Test (making sure madgwick works with values being passed through rather than using default constants) 
    #[test]
    fn test_custom_parameters() {
        // Test with custom beta and sample period
        let service = MadgwickTest::new_with_params(0.05, 0.02);
        
        // Get the quaternion (already initialized during construction)
        let (w, x, y, z) = service.get_quaternion();
        
        // Check quaternion is roughly identity (or close to it)
        assert!(w > 0.9, "Expected w to be close to 1.0, got {}", w);
        assert!(x.abs() < 0.1, "Expected x to be close to 0.0, got {}", x);
        assert!(y.abs() < 0.1, "Expected y to be close to 0.0, got {}", y);
        assert!(z.abs() < 0.1, "Expected z to be close to 0.0, got {}", z);
    }
    
    // Continuous Updates Test (making sure that the service is constantly being updated while running so accurate values are being used)
    #[test]
    fn test_continuous_updates() {
        let mut service = MadgwickTest::new();
        
        // Store initial quaternion
        let (w1, x1, y1, z1) = service.get_quaternion();
        
        // Process 10 updates with Y-axis gravity and Z-axis rotation
        for _ in 0..10 {
            service.update([0.0, 1.0, 0.0], [0.0, 0.0, 0.1]);
        }
        
        // Get the updated quaternion
        let latest_update = service.update([0.0, 1.0, 0.0], [0.0, 0.0, 0.0]);
        
        // Verify quaternion changed - comparing initial with latest update result
        assert!(
            (w1 != latest_update.0) || (x1 != latest_update.1) || 
            (y1 != latest_update.2) || (z1 != latest_update.3),
            "Quaternion should change after processing gyroscope data"
        );
    }
}