use crate::bindings::{
    SbgLogAirData, SbgLogEkfNavData, SbgLogEkfQuatData, SbgLogGpsPos,
    SbgLogGpsVel, SbgLogImuData, SbgLogUtcData,
};
use bitflags::Flags;
use messages::sensor::{Air, EkfNav, EkfQuat, GpsPos, GpsVel, Imu, UtcTime};
use messages::sensor_status::{
    AirFlags, AirStatus, EkfFlags, EkfStatus, GpsPositionStatus, GpsPositionStatusE, GpsVelStatus,
    GpsVelStatusE, ImuFlags, ImuStatus, UtcStatus, UtcTimeStatus,
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
        let status = GpsPositionStatus::new(value.status);
        let valid = matches!(status.get_status(), Some(GpsPositionStatusE::SolComputed));
        
        Self {
            latitude: valid.then_some(value.latitude),
            longitude: valid.then_some(value.longitude),
            time_of_week: valid.then_some(value.timeOfWeek),
            undulation: valid.then_some(value.undulation),
            altitude: valid.then_some(value.altitude),
            time_stamp: value.timeStamp,
            status,
            latitude_accuracy: valid.then_some(value.latitudeAccuracy),
            longitude_accuracy: valid.then_some(value.longitudeAccuracy),
            altitude_accuracy: valid.then_some(value.altitudeAccuracy),
            num_sv_used: valid.then_some(value.numSvUsed),
            base_station_id: valid.then_some(value.baseStationId),
            differential_age: valid.then_some(value.differentialAge),
        }
    }
}

impl From<SbgLogUtcData> for UtcTime {
    fn from(value: SbgLogUtcData) -> Self {
        let status = UtcTimeStatus::new(value.status);
        let valid = matches!(status.get_utc_status(), Some(UtcStatus::Valid | UtcStatus::NoLeapSec));

        Self {
            time_stamp: value.timeStamp, // not convinced this is matched valid to the Utc Status bitmask.
            status,
            year: valid.then_some(value.year),
            month: valid.then_some(value.month),
            day: valid.then_some(value.day),
            hour: valid.then_some(value.hour),
            minute: valid.then_some(value.minute),
            second: valid.then_some(value.second),
            nano_second: valid.then_some(value.nanoSecond),
            gps_time_of_week: valid.then_some(value.gpsTimeOfWeek),
        }
    }
}

impl From<SbgLogAirData> for Air {
    fn from(value: SbgLogAirData) -> Self {
        let status = AirStatus::new(value.status);
        let flags = status.get_flags();

        Self {
            time_stamp: value.timeStamp, // TODO: check if valid.
            status,
            pressure_abs: check(&flags, AirFlags::PressureAbsValid, value.pressureAbs),
            altitude: check(&flags, AirFlags::AltitudeValid, value.altitude),
            pressure_diff: check(&flags, AirFlags::PressureDiffValid, value.pressureDiff),
            true_airspeed: check(&flags, AirFlags::AirpseedValid, value.trueAirspeed),
            air_temperature: check(&flags, AirFlags::TemperatureValid, value.airTemperature),
        }
    }
}

impl From<SbgLogEkfQuatData> for EkfQuat {
    fn from(value: SbgLogEkfQuatData) -> Self {
        let status = EkfStatus::new(value.status);
        let flags = status.get_flags();

        Self {
            time_stamp: value.timeStamp,
            quaternion: check(&flags, EkfFlags::HeadingValid, value.quaternion),
            euler_std_dev: check(&flags, EkfFlags::HeadingValid, value.eulerStdDev),
            status,
        }
    }
}

impl From<SbgLogEkfNavData> for EkfNav {
    fn from(value: SbgLogEkfNavData) -> Self {
        let status = EkfStatus::new(value.status);
        let flags = status.get_flags();

        Self {
            status,
            velocity_std_dev: check(&flags, EkfFlags::VelocityValid, value.velocityStdDev),
            position_std_dev: check(&flags, EkfFlags::PositionValid, value.positionStdDev),
            time_stamp: value.timeStamp,
            velocity: check(&flags, EkfFlags::VelocityValid, value.velocity),
            position: check(&flags, EkfFlags::PositionValid, value.position),
            undulation: check(&flags, EkfFlags::AttitudeValid, value.undulation),
        }
    }
}

impl From<SbgLogImuData> for Imu {
    fn from(value: SbgLogImuData) -> Self {
        let status = ImuStatus::new(value.status);
        let flags = status.get_flags();

        Self {
            time_stamp: value.timeStamp,
            status,
            accelerometers: check(&flags, ImuFlags::AccelsInRange, value.accelerometers),
            gyroscopes: check(&flags, ImuFlags::GyrosInRange, value.gyroscopes),
            temperature: Some(value.temperature), // we cannot check since no flag exists. Keep in option for uniformity.
            delta_velocity: check(&flags, ImuFlags::AccelsInRange, value.deltaVelocity),
            delta_angle: check(&flags, ImuFlags::GyrosInRange, value.deltaAngle),
        }
    }
}

impl From<SbgLogGpsVel> for GpsVel {
    fn from(value: SbgLogGpsVel) -> Self {
        let status = GpsVelStatus::new(value.status);
        let valid = matches!(status.get_status(), Some(GpsVelStatusE::SolComputed));

        Self {
            time_of_week: valid.then_some(value.timeOfWeek),
            time_stamp: value.timeStamp,
            status,
            velocity: valid.then_some(value.velocity),
            course: valid.then_some(value.course),
            course_acc: valid.then_some(value.courseAcc),
            velocity_acc: valid.then_some(value.velocityAcc),
        }
    }
}
