use defmt::{info, warn, Format};
use embedded_hal_1::delay::DelayNs;
use embedded_hal_1::spi::{Operation, SpiDevice};

// Register addresses from the datasheet
const GYRO_X_DATA: u8 = 0x00;
const ACCEL_X_DATA: u8 = 0x04;

// Banked register addresses
const WHO_AM_I: u8 = 0x0E; // In Bank 1
const ACCEL_FS_SEL: u8 = 0x14; // In Bank 6
const GYRO_FS_SEL: u8 = 0x14; // In Bank 7

// Registers available in all banks
const BANK_SELECT: u8 = 0x1F;

// Registers in Bank 0
const TCODE_STATUS: u8 = 0x19;
const RESET_CONTROL: u8 = 0x18;

#[derive(Debug, Format)]
pub enum Error<E> {
    /// SPI bus error
    Spi(E),
    /// The WHO_AM_I check failed
    WhoAmI,
    /// The device returned a bad response (e.g., error status bits)
    BadResponse { response: [u8; 4], status: u8 },
}

pub struct Iim20670<SPI, D> {
    spi: SPI,
    delay: D,
    accel_fs: f32,
    gyro_fs: f32,
}

impl<SPI, D> Iim20670<SPI, D>
where
    SPI: SpiDevice,
    SPI::Error: core::fmt::Debug,
    D: DelayNs,
{
    /// Creates a new driver instance.
    pub fn new(spi: SPI, delay: D) -> Self {
        Self {
            spi,
            delay,
            accel_fs: 0.0,
            gyro_fs: 0.0,
        }
    }

    /// Initializes the sensor, checks the WHO_AM_I register, and reads
    /// the default full-scale settings.
    pub fn init(&mut self) -> Result<(), Error<SPI::Error>> {
        self.delay.delay_ms(10);

        info!("Performing hardware reset...");
        self.write_reg(RESET_CONTROL, 1 << 2)?;
        self.delay.delay_ms(250);
        info!("IMU startup complete after reset.");

        self.unlock_bank_select()?;
        info!("Bank selection unlocked.");

        self.set_bank(1)?;
        let whoami_val = self.read_reg(WHO_AM_I)?;
        if (whoami_val as u8) != 0xF3 {
            warn!(
                "WHO_AM_I check failed. Expected 0xF3, got {:#02x}",
                whoami_val
            );
            return Err(Error::WhoAmI);
        }
        info!("WHO_AM_I check passed.");

        self.set_bank(6)?;
        let accel_fs_sel = self.read_reg(ACCEL_FS_SEL)?;
        self.update_accel_fs(accel_fs_sel as u8);

        self.set_bank(7)?;
        let gyro_fs_sel = self.read_reg(GYRO_FS_SEL)?;
        self.update_gyro_fs(gyro_fs_sel as u8);

        self.set_bank(0)?;
        info!("Switched to Bank 0. IMU initialization successful.");
        Ok(())
    }

    /// Reads accelerometer data [x, y, z] in g.
    pub fn read_accel(&mut self) -> Result<[f32; 3], Error<SPI::Error>> {
        let mut raw = [0i16; 3];
        for i in 0..3 {
            raw[i] = self.read_reg(ACCEL_X_DATA + i as u8)? as i16;
        }
        let accel_data = [
            (raw[0] as f32 / 32768.0) * self.accel_fs,
            (raw[1] as f32 / 32768.0) * self.accel_fs,
            (raw[2] as f32 / 32768.0) * self.accel_fs,
        ];
        Ok(accel_data)
    }

    /// Reads gyroscope data [x, y, z] in degrees per second.
    pub fn read_gyro(&mut self) -> Result<[f32; 3], Error<SPI::Error>> {
        let mut raw = [0i16; 3];
        for i in 0..3 {
            raw[i] = self.read_reg(GYRO_X_DATA + i as u8)? as i16;
        }
        let gyro_data = [
            (raw[0] as f32 / 32768.0) * self.gyro_fs,
            (raw[1] as f32 / 32768.0) * self.gyro_fs,
            (raw[2] as f32 / 32768.0) * self.gyro_fs,
        ];
        Ok(gyro_data)
    }

    /// Reads a 16-bit value from a register using the byte-oriented protocol.
    fn read_reg(&mut self, reg: u8) -> Result<u16, Error<SPI::Error>> {
        let mut tx_buf = [0u8; 4];
        tx_buf[0] = ((reg & 0x1F) << 2); // RW=0, Addr in bits 6..2
        tx_buf[1] = 0;
        tx_buf[2] = 0;
        tx_buf[3] = self.calculate_crc(&[tx_buf[0], tx_buf[1], tx_buf[2]]);

        // First transfer (request)
        let mut rx_buf = [0u8; 4];
        self.spi
            .transaction(&mut [Operation::Transfer(&mut rx_buf, &tx_buf)])
            .map_err(Error::Spi)?;

        self.delay.delay_ms(1);

        // Second transfer (fetch data)
        self.spi
            .transaction(&mut [Operation::Transfer(&mut rx_buf, &tx_buf)])
            .map_err(Error::Spi)?;

        // Status is in the LSBs of the first received byte
        let status = rx_buf[0] & 0b11;
        if status != 0b01 {
            // 0b01 is success
            warn!(
                "Read from reg {:#02x} failed. Status: {:#04b}, Response: {=[u8]:#02x}",
                reg, status, rx_buf
            );
            return Err(Error::BadResponse {
                response: rx_buf,
                status,
            });
        }

        // Data is in the second and third bytes
        Ok(u16::from_be_bytes([rx_buf[1], rx_buf[2]]))
    }

    /// Writes a 16-bit value to a register using the byte-oriented protocol.
    fn write_reg(&mut self, reg: u8, data: u16) -> Result<(), Error<SPI::Error>> {
        let mut tx_buf = [0u8; 4];
        tx_buf[0] = (1 << 7) | ((reg & 0x1F) << 2); // RW=1, Addr in bits 6..2
        let data_bytes = data.to_be_bytes();
        tx_buf[1] = data_bytes[0]; // Data MSB
        tx_buf[2] = data_bytes[1]; // Data LSB
        tx_buf[3] = self.calculate_crc(&[tx_buf[0], tx_buf[1], tx_buf[2]]);

        let mut rx_buf = [0u8; 4];
        self.spi
            .transaction(&mut [Operation::Transfer(&mut rx_buf, &tx_buf)])
            .map_err(Error::Spi)?;

        // Status is in the LSBs of the first received byte
        let status = rx_buf[0] & 0b11;
        if status != 0b01 && reg != RESET_CONTROL {
            // 0b01 is success
            warn!(
                "Write to reg {:#02x} failed. Status: {:#04b}, Response: {=[u8]:#02x}",
                reg, status, rx_buf
            );
            return Err(Error::BadResponse {
                response: rx_buf,
                status,
            });
        }
        Ok(())
    }

    /// Switches the active register bank.
    fn set_bank(&mut self, bank: u16) -> Result<(), Error<SPI::Error>> {
        self.write_reg(BANK_SELECT, bank)
    }

    /// Unlocks the bank selection register as per datasheet section 6.13.
    fn unlock_bank_select(&mut self) -> Result<(), Error<SPI::Error>> {
        info!("Unlocking bank select...");
        self.write_reg(TCODE_STATUS, 0b010)?;
        self.write_reg(TCODE_STATUS, 0b001)?;
        self.write_reg(TCODE_STATUS, 0b100)?;
        Ok(())
    }

    /// Updates the internal accelerometer full-scale value based on the register value.
    fn update_accel_fs(&mut self, sel_val: u8) {
        self.accel_fs = match sel_val & 0b111 {
            0b001 => 16.384, // Default
            0b010 | 0b011 => 32.768,
            0b100 | 0b101 => 2.048,
            0b110 | 0b111 => 4.096,
            _ => 16.384,
        };
        info!("Accelerometer FS set to: {} g", self.accel_fs);
    }

    /// Updates the internal gyroscope full-scale value based on the register value.
    fn update_gyro_fs(&mut self, sel_val: u8) {
        self.gyro_fs = match sel_val & 0b1111 {
            0b0001 => 655.0, // Default
            0b0000 | 0b1111 => 328.0,
            0b0010 | 0b0111 => 1311.0,
            0b0011 => 1966.0,
            _ => 655.0,
        };
        info!("Gyroscope FS set to: {} dps", self.gyro_fs);
    }

    /// Calculates the CRC for a 3-byte slice, matching the C driver's logic.
    fn calculate_crc(&self, data_in: &[u8; 3]) -> u8 {
        let mut crc: u8 = 0xFF;
        let poly: u8 = 0x1D;

        for &byte in data_in {
            let mut current_byte = byte;
            for _ in 0..8 {
                let crc_msb = (crc & 0x80) != 0;
                let data_msb = (current_byte & 0x80) != 0;
                crc <<= 1;
                if data_msb {
                    crc |= 1;
                }
                if crc_msb {
                    crc ^= poly;
                }
                current_byte <<= 1;
            }
        }
        crc ^ 0xFF
    }
}
