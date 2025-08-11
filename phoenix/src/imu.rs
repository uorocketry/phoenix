//! Blocking SPI driver for the TDK IIM-20670 IMU with unit conversions and self-test.

use defmt::{error, info, warn};
use embedded_hal_1::delay::DelayNs;
use embedded_hal_1::digital::OutputPin;
use embedded_hal_1::spi::SpiBus;

/// Acceleration data in g's (1g = 9.8 m/s^2)
#[derive(Debug, Clone, Copy)]
pub struct Acceleration {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

/// Angular rate data in degrees per second (dps)
#[derive(Debug, Clone, Copy)]
pub struct AngularRate {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

/// Holds the raw difference values from the self-test routine.
#[derive(Debug, Clone, Copy)]
pub struct SelfTestValues {
    pub accel_x_diff: i16,
    pub accel_y_diff: i16,
    pub accel_z_diff: i16,
    pub gyro_x_diff: i16,
    pub gyro_y_diff: i16,
    pub gyro_z_diff: i16,
}

/// Represents the IIM-20670 device.
pub struct Iim20670<SPI, CS, NRESET, DELAY> {
    spi: SPI,
    cs: CS,
    nreset: Option<NRESET>,
    delay: DELAY,
    accel_fsr: AccelFsr,
    gyro_fsr: GyroFsr,
}

/// Represents errors that can occur while interacting with the IIM-20670.
#[derive(Debug)]
pub enum Error<SPIE, CSE, RESETE> {
    Spi(SPIE),
    Cs(CSE),
    Reset(RESETE),
    InvalidDeviceId,
    /// The IMU returned an error status in its SPI response
    ImuError(u8),
}

#[allow(dead_code)]
mod registers {
    // Bank 0
    pub const GYRO_X_DATA: u8 = 0x00;
    pub const GYRO_Y_DATA: u8 = 0x01;
    pub const GYRO_Z_DATA: u8 = 0x02;
    pub const ACCEL_X_DATA: u8 = 0x04;
    pub const ACCEL_Y_DATA: u8 = 0x05;
    pub const ACCEL_Z_DATA: u8 = 0x06;
    pub const SELF_TEST_CONFIG: u8 = 0x11;
    pub const SELF_TEST_TRIGGER: u8 = 0x16;
    pub const RESET_CTRL: u8 = 0x18;
    pub const MODE_CTRL: u8 = 0x19;
    pub const BANK_SELECT: u8 = 0x1F;

    // Bank 6 & 7 (for sensitivity config)
    pub const SENSITIVITY_CONFIG: u8 = 0x14;
}

#[repr(u16)]
#[derive(Clone, Copy, Debug)]
enum Bank {
    Bank0 = 0x0000,
    Bank1 = 0x0001,
    Bank6 = 0x0006,
    Bank7 = 0x0007,
}

#[repr(u16)]
#[derive(Clone, Copy, Debug)]
pub enum GyroFsr {
    Dps1966 = 0b0011,
}

impl GyroFsr {
    fn as_f32(&self) -> f32 {
        1966.0
    }
}

#[repr(u16)]
#[derive(Clone, Copy, Debug)]
pub enum AccelFsr {
    G16 = 0b001,
}

impl AccelFsr {
    fn as_f32(&self) -> f32 {
        16.384
    }
}

struct CsGuard<'a, CS: OutputPin> {
    cs: &'a mut CS,
}

impl<'a, CS: OutputPin> CsGuard<'a, CS> {
    fn new(cs: &'a mut CS) -> Result<Self, CS::Error> {
        cs.set_low()?;
        Ok(Self { cs })
    }
}

impl<'a, CS: OutputPin> Drop for CsGuard<'a, CS> {
    fn drop(&mut self) {
        let _ = self.cs.set_high();
    }
}

impl<SPI, CS, NRESET, DELAY, SPIE, CSE, RESETE> Iim20670<SPI, CS, NRESET, DELAY>
where
    SPI: SpiBus<u8, Error = SPIE>,
    CS: OutputPin<Error = CSE>,
    NRESET: OutputPin<Error = RESETE>,
    DELAY: DelayNs,
{
    pub fn new(
        spi: SPI,
        mut cs: CS,
        mut nreset: Option<NRESET>,
        mut delay: DELAY,
    ) -> Result<Self, Error<SPIE, CSE, RESETE>> {
        info!("Starting IIM-20670 initialization...");
        cs.set_high().map_err(Error::Cs)?;

        let mut driver = Self {
            spi,
            cs,
            nreset,
            delay,
            accel_fsr: AccelFsr::G16,
            gyro_fsr: GyroFsr::Dps1966,
        };

        if let Some(ref mut reset_pin) = driver.nreset {
            info!("Performing hardware reset...");
            reset_pin.set_high().map_err(Error::Reset)?;
            driver.delay.delay_ms(1);
            reset_pin.set_low().map_err(Error::Reset)?;
            driver.delay.delay_ms(40);
            reset_pin.set_high().map_err(Error::Reset)?;
            driver.delay.delay_ms(250);
        }

        // *** FIX: Perform a single read to check for life ***
        info!("Attempting to read a register to verify communication...");
        let _ = driver.read_reg_16(registers::ACCEL_X_DATA)?;

        info!("IMU initialization check passed. Device is responsive.");
        // We will skip other configuration for now to isolate the issue.

        Ok(driver)
    }

    // ... (rest of the functions are unchanged) ...

    fn spi_transaction(
        &mut self,
        reg: u8,
        data: u16,
        is_write: bool,
    ) -> Result<u16, Error<SPIE, CSE, RESETE>> {
        let _guard = CsGuard::new(&mut self.cs).map_err(Error::Cs)?;

        let rw_bit = if is_write { 1u32 } else { 0u32 };
        let command_part = (rw_bit << 31) | ((reg as u32) << 26) | ((data as u32) << 8);
        let crc = Self::calculate_crc(command_part >> 8);
        let tx_word = command_part | crc as u32;
        let mut buffer = tx_word.to_be_bytes();

        info!("  SPI TX -> {=[u8]:#X}", buffer);
        self.spi
            .transfer_in_place(&mut buffer)
            .map_err(Error::Spi)?;
        info!("  SPI RX <- {=[u8]:#X}", buffer);

        let response_word = u32::from_be_bytes(buffer);
        let status = ((response_word >> 24) & 0b11) as u8;

        if status != 1 {
            if status == 2 && !is_write {
                warn!("IMU returned self-test status (2), which is normal during test reads.");
            } else {
                error!("IMU returned error status: {}", status);
                return Err(Error::ImuError(status));
            }
        }

        Ok(((response_word >> 8) & 0xFFFF) as u16)
    }

    fn send_raw_command(&mut self, command: u32) -> Result<(), Error<SPIE, CSE, RESETE>> {
        let _guard = CsGuard::new(&mut self.cs).map_err(Error::Cs)?;
        let mut buffer = command.to_be_bytes();
        info!("  SPI TX -> {=[u8]:#X}", buffer);
        self.spi
            .transfer_in_place(&mut buffer)
            .map_err(Error::Spi)?;
        info!("  SPI RX <- {=[u8]:#X}", buffer);

        let response_word = u32::from_be_bytes(buffer);
        let status = ((response_word >> 24) & 0b11) as u8;
        if status != 1 {
            error!("IMU returned error status: {} on raw command", status);
            return Err(Error::ImuError(status));
        }
        Ok(())
    }

    fn unlock_banks(&mut self) -> Result<(), Error<SPIE, CSE, RESETE>> {
        info!("  unlock_banks: step 1");
        self.write_reg_16(registers::MODE_CTRL, 0b010)?;
        info!("  unlock_banks: step 2");
        self.write_reg_16(registers::MODE_CTRL, 0b001)?;
        info!("  unlock_banks: step 3");
        self.write_reg_16(registers::MODE_CTRL, 0b100)?;
        Ok(())
    }

    fn unlock_fsr(&mut self) -> Result<(), Error<SPIE, CSE, RESETE>> {
        info!("  unlock_fsr: step 1");
        self.send_raw_command(0xE4000288)?;
        info!("  unlock_fsr: step 2");
        self.send_raw_command(0xE400018B)?;
        info!("  unlock_fsr: step 3");
        self.send_raw_command(0xE400048E)?;
        info!("  unlock_fsr: step 4");
        self.send_raw_command(0xE40300AD)?;
        info!("  unlock_fsr: step 5");
        self.send_raw_command(0xE4018017)?;
        info!("  unlock_fsr: step 6");
        self.send_raw_command(0xE4028030)?;
        Ok(())
    }

    fn write_reg_16(&mut self, reg: u8, data: u16) -> Result<(), Error<SPIE, CSE, RESETE>> {
        self.spi_transaction(reg, data, true)?;
        Ok(())
    }

    fn read_reg_16(&mut self, reg: u8) -> Result<u16, Error<SPIE, CSE, RESETE>> {
        self.spi_transaction(reg, 0, false)?;
        self.spi_transaction(reg, 0, false)
    }

    fn calculate_crc(data_to_encode: u32) -> u8 {
        let mut crc: u8 = 0xFF;
        for i in (0..24).rev() {
            let bit = (data_to_encode >> i) & 1;
            let crc7 = (crc >> 7) & 1;
            let mut crc_new = 0u8;
            crc_new |= ((crc >> 6) & 1) << 7;
            crc_new |= ((crc >> 5) & 1) << 6;
            crc_new |= ((crc >> 4) & 1) << 5;
            crc_new |= (((crc >> 3) & 1) ^ crc7) << 4;
            crc_new |= (((crc >> 2) & 1) ^ crc7) << 3;
            crc_new |= (((crc >> 1) & 1) ^ crc7) << 2;
            crc_new |= (crc & 1) << 1;
            crc_new |= (bit as u8) ^ crc7;
            crc = crc_new;
        }
        !crc
    }

    fn select_bank(&mut self, bank: Bank) -> Result<(), Error<SPIE, CSE, RESETE>> {
        info!("  Selecting bank: {:?}", bank as u16);
        self.write_reg_16(registers::BANK_SELECT, bank as u16)
    }

    pub fn set_gyro_fsr(&mut self, fsr: GyroFsr) -> Result<(), Error<SPIE, CSE, RESETE>> {
        info!("Setting Gyro FSR...");
        self.select_bank(Bank::Bank7)?;
        self.write_reg_16(registers::SENSITIVITY_CONFIG, fsr as u16)?;
        self.gyro_fsr = fsr;
        self.select_bank(Bank::Bank0)
    }

    pub fn set_accel_fsr(&mut self, fsr: AccelFsr) -> Result<(), Error<SPIE, CSE, RESETE>> {
        info!("Setting Accel FSR...");
        self.select_bank(Bank::Bank6)?;
        self.write_reg_16(registers::SENSITIVITY_CONFIG, fsr as u16)?;
        self.accel_fsr = fsr;
        self.select_bank(Bank::Bank0)
    }

    pub fn read_gyro_raw(&mut self) -> Result<[i16; 3], Error<SPIE, CSE, RESETE>> {
        let x = self.read_reg_16(registers::GYRO_X_DATA)? as i16;
        let y = self.read_reg_16(registers::GYRO_Y_DATA)? as i16;
        let z = self.read_reg_16(registers::GYRO_Z_DATA)? as i16;
        Ok([x, y, z])
    }

    pub fn read_accel_raw(&mut self) -> Result<[i16; 3], Error<SPIE, CSE, RESETE>> {
        let x = self.read_reg_16(registers::ACCEL_X_DATA)? as i16;
        let y = self.read_reg_16(registers::ACCEL_Y_DATA)? as i16;
        let z = self.read_reg_16(registers::ACCEL_Z_DATA)? as i16;
        Ok([x, y, z])
    }

    pub fn read_accel_g(&mut self) -> Result<Acceleration, Error<SPIE, CSE, RESETE>> {
        let raw = self.read_accel_raw()?;
        let fsr = self.accel_fsr.as_f32();
        Ok(Acceleration {
            x: (raw[0] as f32 / 32767.0) * fsr,
            y: (raw[1] as f32 / 32767.0) * fsr,
            z: (raw[2] as f32 / 32767.0) * fsr,
        })
    }

    pub fn read_gyro_dps(&mut self) -> Result<AngularRate, Error<SPIE, CSE, RESETE>> {
        let raw = self.read_gyro_raw()?;
        let fsr = self.gyro_fsr.as_f32();
        Ok(AngularRate {
            x: (raw[0] as f32 / 32767.0) * fsr,
            y: (raw[1] as f32 / 32767.0) * fsr,
            z: (raw[2] as f32 / 32767.0) * fsr,
        })
    }

    pub fn read_all_converted(
        &mut self,
    ) -> Result<(Acceleration, AngularRate), Error<SPIE, CSE, RESETE>> {
        let accel = self.read_accel_g()?;
        let gyro = self.read_gyro_dps()?;
        Ok((accel, gyro))
    }
}
