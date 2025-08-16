use defmt::{info, warn, Format};
use embassy_stm32::gpio::Output;
use embassy_stm32::spi::Spi;
use embedded_hal_1::spi::{Operation, SpiDevice};

// Register addresses from the datasheet
const GYRO_X_DATA: u8 = 0x00;
const ACCEL_X_DATA: u8 = 0x04;
const WHO_AM_I: u8 = 0x0E; // In Bank 1
const BANK_SELECT: u8 = 0x1F;
const TCODE_STATUS: u8 = 0x19;

// Default full-scale values from the datasheet
const GYRO_FS: f32 = 655.0; // dps
const ACCEL_FS: f32 = 16.384; // g

#[derive(Debug, Format)]
pub enum Error<E> {
    Spi(E),
    WhoAmI,
}

pub struct Iim20670<SPI> {
    spi: SPI,
}

impl<SPI> Iim20670<SPI>
where
    SPI: SpiDevice,
    SPI::Error: core::fmt::Debug,
{
    pub fn new(spi: SPI) -> Self {
        Self { spi }
    }

    /// Initializes the sensor and checks the WHO_AM_I register.
    pub fn init(&mut self) -> Result<(), Error<SPI::Error>> {
        // Unlock bank selection to access registers outside of Bank 0
        self.unlock_bank_select().map_err(Error::Spi)?;

        // Switch to Bank 1 to read the WHO_AM_I register
        self.write_reg(BANK_SELECT, 1).map_err(Error::Spi)?;
        let whoami = self.read_reg(WHO_AM_I).map_err(Error::Spi)?;

        // Switch back to Bank 0 for normal data reading
        self.write_reg(BANK_SELECT, 0).map_err(Error::Spi)?;

        if whoami as u8 != 0xF3 {
            warn!(
                "IIM-20670 WHO_AM_I check failed. Expected 0xF3, got {:#02x}",
                whoami
            );
            return Err(Error::WhoAmI);
        }
        info!("IIM-20670 WHO_AM_I check passed.");
        Ok(())
    }

    /// Unlocks the bank selection register as per datasheet section 6.13
    fn unlock_bank_select(&mut self) -> Result<(), SPI::Error> {
        self.write_reg(TCODE_STATUS, 0b010)?;
        self.write_reg(TCODE_STATUS, 0b001)?;
        self.write_reg(TCODE_STATUS, 0b100)?;
        Ok(())
    }

    /// Reads accelerometer data.
    pub fn read_accel(&mut self) -> Result<[f32; 3], Error<SPI::Error>> {
        let mut data = [0i16; 3];
        for i in 0..3 {
            data[i] = self.read_reg(ACCEL_X_DATA + i as u8).map_err(Error::Spi)? as i16;
        }

        let accel_data = [
            (data[0] as f32 / 32768.0) * ACCEL_FS,
            (data[1] as f32 / 32768.0) * ACCEL_FS,
            (data[2] as f32 / 32768.0) * ACCEL_FS,
        ];
        Ok(accel_data)
    }

    /// Reads gyroscope data.
    pub fn read_gyro(&mut self) -> Result<[f32; 3], Error<SPI::Error>> {
        let mut data = [0i16; 3];
        for i in 0..3 {
            data[i] = self.read_reg(GYRO_X_DATA + i as u8).map_err(Error::Spi)? as i16;
        }

        let gyro_data = [
            (data[0] as f32 / 32768.0) * GYRO_FS,
            (data[1] as f32 / 32768.0) * GYRO_FS,
            (data[2] as f32 / 32768.0) * GYRO_FS,
        ];
        Ok(gyro_data)
    }

    fn read_reg(&mut self, reg: u8) -> Result<u16, SPI::Error> {
        // For a read, RW bit (31) is 0. Address is bits 30-26.
        let command_no_crc = ((reg & 0x1F) as u32) << 26;
        let crc = self.calculate_crc(command_no_crc);
        let command = command_no_crc | (crc as u32);

        let tx_buf = command.to_be_bytes();
        let mut rx_buf = [0u8; 4];

        let mut ops = [Operation::Transfer(&mut rx_buf, &tx_buf)];
        self.spi.transaction(&mut ops)?;

        let response = u32::from_be_bytes(rx_buf);
        // TODO: Optionally, verify response CRC and status bits [25:24]
        let data = (response >> 8) as u16;
        Ok(data)
    }

    fn write_reg(&mut self, reg: u8, data: u16) -> Result<(), SPI::Error> {
        // For a write, RW bit (31) is 1.
        let command_no_crc = (1 << 31) | (((reg & 0x1F) as u32) << 26) | ((data as u32) << 8);
        let crc = self.calculate_crc(command_no_crc);
        let command = command_no_crc | (crc as u32);

        let tx_buf = command.to_be_bytes();
        let mut ops = [Operation::Write(&tx_buf)];
        self.spi.transaction(&mut ops)
    }

    /// Calculates the CRC for a 24-bit word as per the datasheet (page 21)
    fn calculate_crc(&self, data: u32) -> u8 {
        let mut crc: u8 = 0xFF;
        // The data to be CRC'd is the top 24 bits of the 32-bit word (bits 31 down to 8)
        for i in (8..32).rev() {
            let input_bit = ((data >> i) & 1) as u8;
            let crc7 = (crc >> 7) & 1;

            let mut crc_new: u8 = 0;
            crc_new |= ((crc >> 6) & 1) << 7; // new 7 is old 6
            crc_new |= ((crc >> 5) & 1) << 6; // new 6 is old 5
            crc_new |= ((crc >> 4) & 1) << 5; // new 5 is old 4
            crc_new |= (((crc >> 3) & 1) ^ crc7) << 4; // new 4 is old 3 ^ old 7
            crc_new |= (((crc >> 2) & 1) ^ crc7) << 3; // new 3 is old 2 ^ old 7
            crc_new |= (((crc >> 1) & 1) ^ crc7) << 2; // new 2 is old 1 ^ old 7
            crc_new |= (crc & 1) << 1;             // new 1 is old 0
            crc_new |= input_bit ^ crc7;           // new 0 is input ^ old 7

            crc = crc_new;
        }
        !crc // Inverted result
    }
}
