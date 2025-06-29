use crate::bindings::{
    SbgLogAirData, SbgLogEkfNavData, SbgLogEkfQuatData, SbgLogGpsPos, SbgLogGpsVel, SbgLogImuData,
    SbgLogUtcData,
};
use messages_prost::sensor::sbg::{
    Air, AirData, AirStatus, AirStatusFlag, EkfNav, EkfPositionData, EkfQuat, EkfStatus,
    EkfStatusFlag, EkfVelocityData, GpsPos, GpsPosData, GpsPositionStatus, GpsPositionStatusE,
    GpsPositionType, GpsVel, GpsVelData, GpsVelStatus, GpsVelStatusE, GpsVelType, Imu,
    ImuAccelData, ImuGyroData, ImuStatus, ImuStatusFlag, Quaternion, QuaternionData, UtcStatus,
    UtcTime, UtcTimeData, UtcTimeStatus, Vector3,
};

/// Simple helper function to work with boolean flags and set the fields as needed.
#[inline]
fn check<T>(flag: bool, value: T) -> Option<T> {
    if flag {
        Some(value)
    } else {
        None
    }
}

/// Convert array to Vector3
fn array_to_vector3(arr: [f32; 3]) -> Vector3 {
    Vector3 {
        x: arr[0] as f64,
        y: arr[1] as f64,
        z: arr[2] as f64,
    }
}

/// Convert array to Quaternion
fn array_to_quaternion(arr: [f32; 4]) -> Quaternion {
    Quaternion {
        w: arr[0] as f64,
        x: arr[1] as f64,
        y: arr[2] as f64,
        z: arr[3] as f64,
    }
}

/// Convert array to Vector3 for f64 arrays (position data)
fn array_to_vector3_f64(arr: [f64; 3]) -> Vector3 {
    Vector3 {
        x: arr[0],
        y: arr[1],
        z: arr[2],
    }
}

impl From<SbgLogGpsPos> for GpsPos {
    fn from(value: SbgLogGpsPos) -> Self {
        let status = GpsPositionStatus {
            status: value.status as i32,
            r#type: GpsPositionType::Unspecified as i32, // Adjust as needed
        };
        let valid = status.status == GpsPositionStatusE::SolComputed as i32;
        let data = if valid {
            Some(GpsPosData {
                latitude: value.latitude,
                longitude: value.longitude,
                time_of_week: value.timeOfWeek,
                undulation: value.undulation,
                altitude: value.altitude,
                latitude_accuracy: value.latitudeAccuracy,
                longitude_accuracy: value.longitudeAccuracy,
                altitude_accuracy: value.altitudeAccuracy,
                num_sv_used: value.numSvUsed as u32,
                base_station_id: value.baseStationId as u32,
                differential_age: value.differentialAge as u32,
            })
        } else {
            None
        };
        GpsPos {
            time_stamp: value.timeStamp,
            status: Some(status),
            data,
        }
    }
}

impl From<SbgLogUtcData> for UtcTime {
    fn from(value: SbgLogUtcData) -> Self {
        let status = UtcTimeStatus {
            clock_status: 0, // Default value
            utc_status: value.status as i32,
        };
        let valid = status.utc_status == UtcStatus::NoLeapSec as i32
            || status.utc_status == UtcStatus::UtcValid as i32;
        let data = if valid {
            Some(UtcTimeData {
                year: value.year as u32,
                month: value.month as i32,
                day: value.day as i32,
                hour: value.hour as i32,
                minute: value.minute as i32,
                second: value.second as i32,
                nano_second: value.nanoSecond,
                gps_time_of_week: value.gpsTimeOfWeek,
            })
        } else {
            None
        };
        UtcTime {
            time_stamp: value.timeStamp,
            status: Some(status),
            data,
        }
    }
}

impl From<SbgLogAirData> for Air {
    fn from(value: SbgLogAirData) -> Self {
        let status = AirStatus {
            flags: value.status as u32,
        };
        let data = Some(AirData {
            pressure_abs: if (status.flags & AirStatusFlag::AirStatusPressureAbsValid as u32) != 0 {
                value.pressureAbs
            } else {
                0.0
            },
            altitude: if (status.flags & AirStatusFlag::AirStatusAltitudeValid as u32) != 0 {
                value.altitude
            } else {
                0.0
            },
            pressure_diff: if (status.flags & AirStatusFlag::AirStatusPressureDiffValid as u32) != 0
            {
                value.pressureDiff
            } else {
                0.0
            },
            true_airspeed: if (status.flags & AirStatusFlag::AirStatusAirspeedValid as u32) != 0 {
                value.trueAirspeed
            } else {
                0.0
            },
            air_temperature: if (status.flags & AirStatusFlag::AirStatusTemperatureValid as u32)
                != 0
            {
                value.airTemperature
            } else {
                0.0
            },
        });
        Air {
            time_stamp: value.timeStamp,
            status: Some(status),
            data,
        }
    }
}

impl From<SbgLogEkfQuatData> for EkfQuat {
    fn from(value: SbgLogEkfQuatData) -> Self {
        let heading_valid = (value.status & EkfStatusFlag::EkfStatusHeadingValid as u32) != 0;
        let status = EkfStatus {
            flags: value.status,
        };
        let data = if heading_valid {
            Some(QuaternionData {
                quaternion: Some(array_to_quaternion(value.quaternion)),
                euler_std_dev: Some(array_to_vector3(value.eulerStdDev)),
            })
        } else {
            None
        };
        EkfQuat {
            time_stamp: value.timeStamp,
            status: Some(status),
            data,
        }
    }
}

impl From<SbgLogEkfNavData> for EkfNav {
    fn from(value: SbgLogEkfNavData) -> Self {
        let status = EkfStatus {
            flags: value.status as u32,
        };
        let velocity_valid = (status.flags & EkfStatusFlag::EkfStatusVelocityValid as u32) != 0;
        let position_valid = (status.flags & EkfStatusFlag::EkfStatusPositionValid as u32) != 0;
        let attitude_valid = (status.flags & EkfStatusFlag::EkfStatusAttitudeValid as u32) != 0;
        let velocity = if velocity_valid {
            Some(EkfVelocityData {
                velocity: Some(array_to_vector3(value.velocity)),
                velocity_std_dev: Some(array_to_vector3(value.velocityStdDev)),
            })
        } else {
            None
        };
        let position = if position_valid {
            Some(EkfPositionData {
                position: Some(array_to_vector3_f64(value.position)),
                position_std_dev: Some(array_to_vector3(value.positionStdDev)),
            })
        } else {
            None
        };
        EkfNav {
            time_stamp: value.timeStamp,
            status: Some(status),
            velocity,
            position,
            undulation: if attitude_valid {
                Some(value.undulation)
            } else {
                None
            },
        }
    }
}

impl From<SbgLogImuData> for Imu {
    fn from(value: SbgLogImuData) -> Self {
        let status = ImuStatus {
            flags: value.status as u32,
        };
        let gyros_in_range = (status.flags & ImuStatusFlag::ImuStatusGyrosInRange as u32) != 0;
        let accels_in_range = (status.flags & ImuStatusFlag::ImuStatusAccelsInRange as u32) != 0;
        let gyroscopes = if gyros_in_range {
            Some(ImuGyroData {
                gyroscopes: Some(array_to_vector3(value.gyroscopes)),
                delta_angle: Some(array_to_vector3(value.deltaAngle)),
            })
        } else {
            None
        };
        let accelerometers = if accels_in_range {
            Some(ImuAccelData {
                accelerometers: Some(array_to_vector3(value.accelerometers)),
                delta_velocity: Some(array_to_vector3(value.deltaVelocity)),
            })
        } else {
            None
        };
        Imu {
            time_stamp: value.timeStamp,
            status: Some(status),
            gyroscopes,
            accelerometers,
            temperature: Some(value.temperature),
        }
    }
}

impl From<SbgLogGpsVel> for GpsVel {
    fn from(value: SbgLogGpsVel) -> Self {
        let status = GpsVelStatus {
            status: value.status as i32,
            r#type: GpsVelType::Unspecified as i32, // Adjust as needed
        };
        let valid = status.status == GpsVelStatusE::VelSolComputed as i32;
        let data = if valid {
            Some(GpsVelData {
                velocity: Some(array_to_vector3(value.velocity)),
                velocity_acc: Some(array_to_vector3(value.velocityAcc)),
                course: value.course,
                course_acc: value.courseAcc,
                time_of_week: value.timeOfWeek,
            })
        } else {
            None
        };
        GpsVel {
            time_stamp: value.timeStamp,
            status: Some(status),
            data,
        }
    }
}
