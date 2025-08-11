//! Central feature toggles for phoenix. Flip booleans to enable/disable subsystems.
//! This is runtime gating (compile-time Cargo features can be added later if needed).

pub const ENABLE_LED: bool = false;
pub const ENABLE_SBG: bool = false;
// TODO: remove/condense these GPS features
pub const ENABLE_GPS: bool = false;
pub const ENABLE_GPS_POLL: bool = false;
pub const ENABLE_GPS_PARSE: bool = false;
pub const ENABLE_GPS_CFG_FLASH: bool = false;
pub const ENABLE_GPS_CFG_BBR_RAM: bool = false;
pub const ENABLE_GPS_SOFT_RESET: bool = false;
pub const ENABLE_BARO: bool = false;
pub const ENABLE_RADIO: bool = true;
pub const ENABLE_INFERENCE: bool = false;
pub const ENABLE_SD: bool = false;
pub const ENABLE_STATE_MACHINE: bool = true;
