use crate::bindings::{
    SbgLogAirData, SbgLogEkfNavData, SbgLogEkfQuatData, SbgLogGpsPos, SbgLogGpsVel, SbgLogImuData,
    SbgLogUtcData, SBG_ECOM_AIR_DATA_AIRPSEED_VALID, SBG_ECOM_AIR_DATA_ALTITUDE_VALID,
    SBG_ECOM_AIR_DATA_PRESSURE_ABS_VALID, SBG_ECOM_AIR_DATA_PRESSURE_DIFF_VALID,
    SBG_ECOM_AIR_DATA_TEMPERATURE_VALID, SBG_ECOM_IMU_ACCELS_IN_RANGE, SBG_ECOM_IMU_GYROS_IN_RANGE,
    SBG_ECOM_SOL_ATTITUDE_VALID, SBG_ECOM_SOL_HEADING_VALID, SBG_ECOM_SOL_POSITION_VALID,
    SBG_ECOM_SOL_VELOCITY_VALID,
};
use messages_prost::sensor::sbg::{
    Air, AirStatus, EkfNav, EkfQuat, EkfStatus, GpsPos, GpsPositionStatus, GpsPositionStatusE,
    GpsVel, GpsVelStatus, GpsVelStatusE, Imu, ImuStatus, Quaternion, UtcStatus, UtcTime,
    UtcTimeStatus, Vector3,
};

/// Simple helper function to work with the flags structure and set the fields as needed.
#[inline]
fn check<F, T>(flags: &Option<F>, test: F, value: T) -> Option<T>
where
    F: Flags,
{
    match flags {
        Some(x) if x.contains(test) => Some(value),
        _ => None,
    }
}

impl From<SbgLogGpsPos> for GpsPos {
    fn from(value: SbgLogGpsPos) -> Self {
        let status = GpsPositionStatus {
            status: value.status as i32,
            r#type: 0, // You may need to extract type from value.status if encoded
        };
        let valid = (value.status & 0x3F) == 1; // 1 = SolComputed
        GpsPos {
            latitude: if valid { Some(value.latitude) } else { None },
            longitude: if valid { Some(value.longitude) } else { None },
            altitude: if valid { Some(value.altitude) } else { None },
            undulation: if valid { Some(value.undulation) } else { None },
            time_of_week: if valid { Some(value.timeOfWeek) } else { None },
            status: Some(status),
            time_stamp: value.timeStamp,
            latitude_accuracy: if valid {
                Some(value.latitudeAccuracy)
            } else {
                None
            },
            longitude_accuracy: if valid {
                Some(value.longitudeAccuracy)
            } else {
                None
            },
            altitude_accuracy: if valid {
                Some(value.altitudeAccuracy)
            } else {
                None
            },
            num_sv_used: if valid {
                Some(value.numSvUsed as u32)
            } else {
                None
            },
            base_station_id: if valid {
                Some(value.baseStationId as u32)
            } else {
                None
            },
            differential_age: if valid {
                Some(value.differentialAge as u32)
            } else {
                None
            },
        }
    }
}

impl From<SbgLogUtcData> for UtcTime {
    fn from(value: SbgLogUtcData) -> Self {
        let status = UtcTimeStatus {
            clock_status: ((value.status >> 1) & 0xF) as i32,
            utc_status: ((value.status >> 6) & 0xF) as i32,
        };
        let valid = status.utc_status == 3 || status.utc_status == 2; // UtcValid or NoLeapSec
        UtcTime {
            time_stamp: value.timeStamp,
            status: Some(status),
            year: if valid { Some(value.year as u32) } else { None },
            month: if valid {
                Some(value.month as i32)
            } else {
                None
            },
            day: if valid { Some(value.day as i32) } else { None },
            hour: if valid { Some(value.hour as i32) } else { None },
            minute: if valid {
                Some(value.minute as i32)
            } else {
                None
            },
            second: if valid {
                Some(value.second as i32)
            } else {
                None
            },
            nano_second: if valid {
                Some(value.nanoSecond as i32)
            } else {
                None
            },
            gps_time_of_week: if valid {
                Some(value.gpsTimeOfWeek)
            } else {
                None
            },
        }
    }
}

impl From<SbgLogAirData> for Air {
    fn from(value: SbgLogAirData) -> Self {
        let flags = value.status as u32;
        Air {
            time_stamp: value.timeStamp,
            status: None, // prost AirStatus is not a bitmask, skip or fill as needed
            pressure_abs: check_flag(
                flags,
                SBG_ECOM_AIR_DATA_PRESSURE_ABS_VALID,
                value.pressureAbs,
            ),
            altitude: check_flag(flags, SBG_ECOM_AIR_DATA_ALTITUDE_VALID, value.altitude),
            pressure_diff: check_flag(
                flags,
                SBG_ECOM_AIR_DATA_PRESSURE_DIFF_VALID,
                value.pressureDiff,
            ),
            true_airspeed: check_flag(flags, SBG_ECOM_AIR_DATA_AIRPSEED_VALID, value.trueAirspeed),
            air_temperature: check_flag(
                flags,
                SBG_ECOM_AIR_DATA_TEMPERATURE_VALID,
                value.airTemperature,
            ),
        }
    }
}

impl From<SbgLogEkfQuatData> for EkfQuat {
    fn from(value: SbgLogEkfQuatData) -> Self {
        EkfQuat {
            time_stamp: value.timeStamp,
            status: value.status,
            quaternion: Some(Quaternion {
                w: value.quaternion[0],
                x: value.quaternion[1],
                y: value.quaternion[2],
                z: value.quaternion[3],
            }),
            euler_std_dev: Some(Vector3 {
                x: value.eulerStdDev[0],
                y: value.eulerStdDev[1],
                z: value.eulerStdDev[2],
            }),
        }
    }
}

impl From<SbgLogEkfNavData> for EkfNav {
    fn from(value: SbgLogEkfNavData) -> Self {
        let flags = value.status;
        EkfNav {
            time_stamp: value.timeStamp,
            velocity: Some(Vector3 {
                x: value.velocity[0],
                y: value.velocity[1],
                z: value.velocity[2],
            }),
            velocity_std_dev: Some(Vector3 {
                x: value.velocityStdDev[0],
                y: value.velocityStdDev[1],
                z: value.velocityStdDev[2],
            }),
            position: Some(Vector3 {
                x: value.position[0] as f32, // prost expects f32, SbgLogEkfNavData has f64
                y: value.position[1] as f32,
                z: value.position[2] as f32,
            }),
            undulation: Some(value.undulation),
            position_std_dev: Some(Vector3 {
                x: value.positionStdDev[0],
                y: value.positionStdDev[1],
                z: value.positionStdDev[2],
            }),
            status: None, // prost EkfStatus is not a bitmask, skip or fill as needed
        }
    }
}

impl From<SbgLogImuData> for Imu {
    fn from(value: SbgLogImuData) -> Self {
        let flags = value.status as u32;
        Imu {
            time_stamp: value.timeStamp,
            status: None, // prost ImuStatus is not a bitmask, skip or fill as needed
            accelerometers: check_flag(
                flags,
                SBG_ECOM_IMU_ACCELS_IN_RANGE,
                Vector3 {
                    x: value.accelerometers[0],
                    y: value.accelerometers[1],
                    z: value.accelerometers[2],
                },
            ),
            gyroscopes: check_flag(
                flags,
                SBG_ECOM_IMU_GYROS_IN_RANGE,
                Vector3 {
                    x: value.gyroscopes[0],
                    y: value.gyroscopes[1],
                    z: value.gyroscopes[2],
                },
            ),
            temperature: Some(value.temperature),
            delta_velocity: check_flag(
                flags,
                SBG_ECOM_IMU_ACCELS_IN_RANGE,
                Vector3 {
                    x: value.deltaVelocity[0],
                    y: value.deltaVelocity[1],
                    z: value.deltaVelocity[2],
                },
            ),
            delta_angle: check_flag(
                flags,
                SBG_ECOM_IMU_GYROS_IN_RANGE,
                Vector3 {
                    x: value.deltaAngle[0],
                    y: value.deltaAngle[1],
                    z: value.deltaAngle[2],
                },
            ),
        }
    }
}

impl From<SbgLogGpsVel> for GpsVel {
    fn from(value: SbgLogGpsVel) -> Self {
        let status = GpsVelStatus {
            status: value.status as i32,
            r#type: 0, // You may need to extract type from value.status if encoded
        };
        let valid = (value.status & 0x3F) == 1; // 1 = VelSolComputed
        GpsVel {
            time_stamp: value.timeStamp,
            status: Some(status),
            time_of_week: if valid { Some(value.timeOfWeek) } else { None },
            velocity: if valid {
                Some(Vector3 {
                    x: value.velocity[0],
                    y: value.velocity[1],
                    z: value.velocity[2],
                })
            } else {
                None
            },
            velocity_acc: if valid {
                Some(Vector3 {
                    x: value.velocityAcc[0],
                    y: value.velocityAcc[1],
                    z: value.velocityAcc[2],
                })
            } else {
                None
            },
            course: if valid { Some(value.course) } else { None },
            course_acc: if valid { Some(value.courseAcc) } else { None },
        }
    }
}

// Helper for bitmask checks
fn check_flag<T>(flags: u32, mask: u32, value: T) -> Option<T> {
    if flags & mask != 0 {
        Some(value)
    } else {
        None
    }
}
