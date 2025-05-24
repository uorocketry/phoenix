//! Driver for the IIM20670 Inertial Measurement Unit (IMU) Sensor (implementation based on DS-000183 Rev 1.0)
use embedded_hal::{
    blocking::{
        delay::DelayUs,
        spi::{Transfer, Write},
    },
    digital::v2::OutputPin,
};

/// SPI Commands (Section 6 "Register Descriptions")
mod command {
    pub const GYRO_X_DATA: u8       = 0x00;
    pub const GYRO_Y_DATA: u8       = 0x01;
    pub const GYRO_Z_DATA: u8       = 0x02;
    pub const TEMP1_DATA: u8        = 0x03;
    pub const ACCEL_X_DATA: u8      = 0x04;
    pub const ACCEL_Y_DATA: u8      = 0x05;
    pub const ACCEL_Z_DATA: u8      = 0x06;
    pub const TEMP2_DATA: u8        = 0x07;
    pub const ACCEL_X_DATA_LR: u8   = 0x08;
    pub const ACCEL_Y_DATA_LR: u8   = 0x09;
    pub const ACCEL_Z_DATA_LR: u8   = 0x0A;
    pub const FIXED_VALUE: u8       = 0x0B;
    pub const FILTER_Y_Z: u8        = 0x0C;
    pub const FILTER_X: u8          = 0x0E;
    pub const TEMP12_DELTA: u8      = 0x0F;
    pub const SELF_TEST: u8         = 0x16;
    pub const RESET_CONTROL: u8     = 0x18;
    pub const MODE: u8              = 0x19;
    pub const BANK_SELECT: u8       = 0x1F;
    pub const ACCEL_FS_SEL: u8      = 0x14;
    pub const GYRO_FS_SEL: u8       = 0x15;
    pub const EN_ACCEL_SELFTEST: u8 = 0x11;
    pub const EN_GYRO_SELFTEST: u8  = 0x12;
}

/// Device validation constants
pub const DEVICE_ID: u8 = 0xF3; // WHO_AM_I register value (Section 6.14)
pub const FIXED_VALUE_EXPECTED: u16 = 0xAA55; // Fixed value register expected value (Section 6.6)

/// Gyroscope full-scale options (Table 18) - Complete implementation
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum GyroFullScale {
    Dps328,      // 0b0000: ±328 dps, 100 LSB/dps
    Dps655,      // 0b0001: ±655 dps, 50 LSB/dps
    Dps1311,     // 0b0010: ±1311 dps, 25 LSB/dps
    Dps1966,     // 0b0011: ±1966 dps, 16.67 LSB/dps
    Dps218,      // 0b0100: ±218 dps, 150 LSB/dps
    Dps437,      // 0b0101: ±437 dps, 75 LSB/dps
    Dps874,      // 0b0110: ±874 dps, 37.5 LSB/dps
    Dps1311Alt,  // 0b0111: ±1311 dps, 25 LSB/dps (alternative)
    Dps61,       // 0b1000: ±61 dps, 533.34 LSB/dps
    Dps123,      // 0b1001: ±123 dps, 266.67 LSB/dps
    Dps246,      // 0b1010: ±246 dps, 133.33 LSB/dps
    Dps492,      // 0b1011: ±492 dps, 66.67 LSB/dps
    Dps41,       // 0b1100: ±41 dps, 800 LSB/dps
    Dps82,       // 0b1101: ±82 dps, 400 LSB/dps
    Dps164,      // 0b1110: ±164 dps, 200 LSB/dps
    Dps328Alt,   // 0b1111: ±328 dps, 100 LSB/dps (alternative)
}

impl GyroFullScale {
    fn to_register_value(&self) -> u8 {
        match self {
            GyroFullScale::Dps328     => 0b0000,
            GyroFullScale::Dps655     => 0b0001,
            GyroFullScale::Dps1311    => 0b0010,
            GyroFullScale::Dps1966    => 0b0011,
            GyroFullScale::Dps218     => 0b0100,
            GyroFullScale::Dps437     => 0b0101,
            GyroFullScale::Dps874     => 0b0110,
            GyroFullScale::Dps1311Alt => 0b0111,
            GyroFullScale::Dps61      => 0b1000,
            GyroFullScale::Dps123     => 0b1001,
            GyroFullScale::Dps246     => 0b1010,
            GyroFullScale::Dps492     => 0b1011,
            GyroFullScale::Dps41      => 0b1100,
            GyroFullScale::Dps82      => 0b1101,
            GyroFullScale::Dps164     => 0b1110,
            GyroFullScale::Dps328Alt  => 0b1111,
        }
    }

    pub fn get_sensitivity(&self) -> f32 {
        match self {
            GyroFullScale::Dps328     => 100.0,
            GyroFullScale::Dps655     => 50.0,
            GyroFullScale::Dps1311    => 25.0,
            GyroFullScale::Dps1966    => 16.67,
            GyroFullScale::Dps218     => 150.0,
            GyroFullScale::Dps437     => 75.0,
            GyroFullScale::Dps874     => 37.5,
            GyroFullScale::Dps1311Alt => 25.0,
            GyroFullScale::Dps61      => 533.34,
            GyroFullScale::Dps123     => 266.67,
            GyroFullScale::Dps246     => 133.33,
            GyroFullScale::Dps492     => 66.67,
            GyroFullScale::Dps41      => 800.0,
            GyroFullScale::Dps82      => 400.0,
            GyroFullScale::Dps164     => 200.0,
            GyroFullScale::Dps328Alt  => 100.0,
        }
    }
}

/// Accelerometer full-scale options (Table 17)
#[derive(Copy, Clone, PartialEq, Eq)]
pub enum AccelFullScale {
    G16_384,
    G16_384_LR65,
    G32_768,
    G32_768_LR65,
    G2_048,
    G2_048_LR16,
    G4_096,
    G4_096_LR8,
}

impl AccelFullScale {
    fn to_register_value(&self) -> u8 {
        match self {
            AccelFullScale::G16_384      => 0b000,
            AccelFullScale::G16_384_LR65 => 0b001,
            AccelFullScale::G32_768      => 0b010,
            AccelFullScale::G32_768_LR65 => 0b011,
            AccelFullScale::G2_048       => 0b100,
            AccelFullScale::G2_048_LR16  => 0b101,
            AccelFullScale::G4_096       => 0b110,
            AccelFullScale::G4_096_LR8   => 0b111,
        }
    }

    pub fn get_sensitivity(&self) -> f32 {
        match self {
            AccelFullScale::G16_384      => 2000.0,
            AccelFullScale::G16_384_LR65 => 2000.0,
            AccelFullScale::G32_768      => 1000.0,
            AccelFullScale::G32_768_LR65 => 1000.0,
            AccelFullScale::G2_048       => 16000.0,
            AccelFullScale::G2_048_LR16  => 16000.0,
            AccelFullScale::G4_096       => 8000.0,
            AccelFullScale::G4_096_LR8   => 8000.0,
        }
    }

    pub fn get_lr_sensitivity(&self) -> f32 {
        match self {
            AccelFullScale::G16_384      => 1000.0,
            AccelFullScale::G16_384_LR65 => 500.0,
            AccelFullScale::G32_768      => 1000.0,
            AccelFullScale::G32_768_LR65 => 500.0,
            AccelFullScale::G2_048       => 2000.0,
            AccelFullScale::G2_048_LR16  => 2000.0,
            AccelFullScale::G4_096       => 4000.0,
            AccelFullScale::G4_096_LR8   => 4000.0,
        }
    }
}

/// Filter cutoffs (Tables 14–16) - Complete implementation
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum FilterCutoff { 
    Hz10,
    Hz12_5,
    Hz27,
    Hz30,
    Hz46,
    Hz60,
    Hz250,
    Hz300,
    Hz400,
}

/// Configuration struct
pub struct ImuConfig {
    pub gyro_scale: GyroFullScale,
    pub accel_scale: AccelFullScale,
    pub gyro_x_filter: FilterCutoff,
    pub gyro_y_filter: FilterCutoff,
    pub gyro_z_filter: FilterCutoff,
    pub accel_x_filter: FilterCutoff,
    pub accel_y_filter: FilterCutoff,
    pub accel_z_filter: FilterCutoff,
}

impl Default for ImuConfig {
    fn default() -> Self {
        ImuConfig {
            gyro_scale: GyroFullScale::Dps655,
            accel_scale: AccelFullScale::G16_384_LR65,
            gyro_x_filter: FilterCutoff::Hz60,
            gyro_y_filter: FilterCutoff::Hz60,
            gyro_z_filter: FilterCutoff::Hz60,
            accel_x_filter: FilterCutoff::Hz60,
            accel_y_filter: FilterCutoff::Hz60,
            accel_z_filter: FilterCutoff::Hz60,
        }
    }
}

/// IMU data in engineering units
#[derive(Debug, Clone, Copy)]
pub struct ImuData {
    pub gyro_x: f32,
    pub gyro_y: f32,
    pub gyro_z: f32,
    pub accel_x: f32,
    pub accel_y: f32,
    pub accel_z: f32,
    pub temp: f32,
}

/// Driver errors
#[derive(Debug)]
pub enum Error<SPIE, CSE> {
    Spi(SPIE),
    Cs(CSE),
    InvalidBank,
    SelfTestFailed,
    CrcError,
    InvalidDeviceId,
    InvalidFixedValue,
    SpiTransferInProgress,
    SpiTransferError,
}

/// Driver state
pub struct Iim20670<SPI, CS, DELAY> {
    spi: SPI,
    cs: CS,
    delay: DELAY,
    gyro_scale: GyroFullScale,
    accel_scale: AccelFullScale,
    banks_unlocked: bool,
}

// CS macro for SPI transactions
macro_rules! with_cs {
    ($s:expr, $blk:expr) => {{
        $s.cs.set_low().map_err(Error::Cs)?;
        $s.delay.delay_us(1);
        let res = $blk;
        $s.cs.set_high().map_err(Error::Cs)?;
        res
    }};
}

impl<SPI, CS, DELAY, SPIE, CSE> Iim20670<SPI, CS, DELAY>
where
    SPI: Transfer<u8, Error = SPIE> + Write<u8, Error = SPIE>,
    CS: OutputPin<Error = CSE>,
    DELAY: DelayUs<u32>,
{
    /// Create driver and initialize with default configuration
    pub fn new(spi: SPI, mut cs: CS, mut delay: DELAY) -> Result<Self, Error<SPIE, CSE>> {
        cs.set_high().map_err(Error::Cs)?;
        delay.delay_us(10_000);
        
        let mut imu = Self { 
            spi, 
            cs, 
            delay, 
            gyro_scale: GyroFullScale::Dps655, 
            accel_scale: AccelFullScale::G16_384_LR65, 
            banks_unlocked: false 
        };
        
        imu.reset()?;
        imu.configure(&ImuConfig::default())?;
        Ok(imu)
    }

    /// Create driver with validation checks
    pub fn new_with_validation(spi: SPI, mut cs: CS, mut delay: DELAY) -> Result<Self, Error<SPIE, CSE>> {
        cs.set_high().map_err(Error::Cs)?;
        delay.delay_us(10_000);
        
        let mut imu = Self { 
            spi, 
            cs, 
            delay, 
            gyro_scale: GyroFullScale::Dps655, 
            accel_scale: AccelFullScale::G16_384_LR65, 
            banks_unlocked: false 
        };
        
        imu.reset()?;
        imu.verify_device_id()?;
        imu.verify_fixed_value()?;
        imu.configure(&ImuConfig::default())?;
        Ok(imu)
    }

    /// Verify device ID matches expected WHO_AM_I value (Section 6.14)
    pub fn verify_device_id(&mut self) -> Result<(), Error<SPIE, CSE>> {
        let id = self.read_device_id()?;
        if id != DEVICE_ID {
            return Err(Error::InvalidDeviceId);
        }
        Ok(())
    }

    /// Verify fixed value register contains expected value (Section 6.6)
    pub fn verify_fixed_value(&mut self) -> Result<(), Error<SPIE, CSE>> {
        let fixed_val = self.read_fixed_value()?;
        if fixed_val != FIXED_VALUE_EXPECTED {
            return Err(Error::InvalidFixedValue);
        }
        Ok(())
    }

    /// Check device integrity by verifying both device ID and fixed value
    pub fn check_device_integrity(&mut self) -> Result<(), Error<SPIE, CSE>> {
        self.verify_device_id()?;
        self.verify_fixed_value()?;
        Ok(())
    }

    /// Perform soft reset (Section 6.12)
    pub fn reset(&mut self) -> Result<(), Error<SPIE, CSE>> {
        with_cs!(self, {
            self.write_register_spi(command::RESET_CONTROL, 0x0002)?;
            self.delay.delay_us(200_000);
            Ok(())
        })
    }

    /// Perform hard reset (Section 6.12)
    pub fn hard_reset(&mut self) -> Result<(), Error<SPIE, CSE>> {
        with_cs!(self, {
            self.write_register_spi(command::RESET_CONTROL, 0x0004)?;
            self.delay.delay_us(3_000);
            Ok(())
        })
    }

    /// Calculate CRC8 for data integrity (Section 5.2)
    fn calculate_crc(&self, data: &[u8]) -> u8 {
        let mut rem = 0xFF;
        for &b in data {
            rem ^= b;
            for _ in 0..8 {
                if rem & 0x80 != 0 {
                    rem = (rem << 1) ^ 0x1D;
                } else {
                    rem <<= 1;
                }
            }
        }
        // Final inversion = CRC byte = !remainder (Section 5.2)
        !rem
    }

    /// Check return status bits from SPI response (Table 13)
    fn check_return_status(&self, status_byte: u8) -> Result<(), Error<SPIE, CSE>> {
        let rs_bits = status_byte & 0x03;
        match rs_bits {
            0b00 => Err(Error::SpiTransferError), // Reserved (treat as error)
            0b01 => Ok(()), // Successful register read/write
            0b10 => Err(Error::SpiTransferInProgress), // Transfer in progress or self-test enabled
            0b11 => Err(Error::SpiTransferError), // Error
            _ => unreachable!(), // Only 2 bits, can't have other values
        }
    }

    /// Read register via SPI with 32-bit frame
    fn read_register_spi(&mut self, addr: u8) -> Result<u16, Error<SPIE, CSE>> {
        with_cs!(self, {
            let mut buf = [(addr & 0x1F), 0, 0, 0];
            buf[3] = self.calculate_crc(&buf[..3]);
            self.spi.transfer(&mut buf).map_err(Error::Spi)?;
            
            // Check return status bits (RS1:RS0 in bits [25:24] of MISO frame)
            // These are in bits [1:0] of the first response byte after the address echo
            self.check_return_status(buf[0])?;
            
            let data = ((buf[1] as u16) << 8) | (buf[2] as u16);
            let crc = buf[3];
            
            if crc != self.calculate_crc(&buf[..3]) {
                return Err(Error::CrcError);
            }
            
            Ok(data)
        })
    }

    /// Write register via SPI with 32-bit frame
    fn write_register_spi(&mut self, addr: u8, val: u16) -> Result<(), Error<SPIE, CSE>> {
        with_cs!(self, {
            let mut buf = [0x80 | (addr & 0x1F), (val >> 8) as u8, val as u8, 0u8];
            buf[3] = self.calculate_crc(&buf[..3]);
            self.spi.write(&buf).map_err(Error::Spi)?;
            
            // For write operations, we need to do a transfer to get the response and check status
            let mut response_buf = [0u8; 4];
            self.spi.transfer(&mut response_buf).map_err(Error::Spi)?;
            
            // Check return status bits from the response
            self.check_return_status(response_buf[0])?;
            
            Ok(())
        })
    }

    /// Unlock bank access (Section 6.13)
    fn unlock_banks(&mut self) -> Result<(), Error<SPIE, CSE>> {
        self.write_register_spi(command::MODE, 0x0002)?;
        self.write_register_spi(command::MODE, 0x0001)?;
        self.write_register_spi(command::MODE, 0x0004)?;
        self.banks_unlocked = true;
        Ok(())
    }

    /// Select register bank (Section 6.16)
    fn set_bank(&mut self, bank: u8) -> Result<(), Error<SPIE, CSE>> {
        if bank > 7 { 
            return Err(Error::InvalidBank); 
        }
        
        if bank != 0 && !self.banks_unlocked { 
            self.unlock_banks()?; 
        }
        
        self.write_register_spi(command::BANK_SELECT, bank as u16)?;
        self.delay.delay_us(100);
        Ok(())
    }

    /// Set gyroscope full-scale range (Section 6.17)
    fn set_gyro_full_scale(&mut self, scale: GyroFullScale) -> Result<(), Error<SPIE, CSE>> {
        self.set_bank(7)?;
        let cur = self.read_register_spi(command::GYRO_FS_SEL)?;
        let new_val = (cur & 0xFFF0) | (scale.to_register_value() as u16 & 0x000F);
        self.write_register_spi(command::GYRO_FS_SEL, new_val)?;
        self.gyro_scale = scale;
        Ok(())
    }

    /// Set accelerometer full-scale range (Section 6.17)
    fn set_accel_full_scale(&mut self, scale: AccelFullScale) -> Result<(), Error<SPIE, CSE>> {
        self.set_bank(6)?;
        let cur = self.read_register_spi(command::ACCEL_FS_SEL)?;
        let new_val = (cur & 0xFFF8) | (scale.to_register_value() as u16 & 0x0007);
        self.write_register_spi(command::ACCEL_FS_SEL, new_val)?;
        self.accel_scale = scale;
        Ok(())
    }

    /// Encode filter bits for Y and Z axes based on datasheet Tables 14-15
    fn encode_filter_bits_yz(&self, gyro: FilterCutoff, accel: FilterCutoff) -> u8 {
        match (gyro, accel) {
            (FilterCutoff::Hz10, FilterCutoff::Hz10)    => 0b000001,
            (FilterCutoff::Hz10, FilterCutoff::Hz46)    => 0b000010,
            (FilterCutoff::Hz10, FilterCutoff::Hz60)    => 0b000011,
            (FilterCutoff::Hz10, FilterCutoff::Hz250)   => 0b000100,
            (FilterCutoff::Hz10, FilterCutoff::Hz300)   => 0b000101,
            (FilterCutoff::Hz10, FilterCutoff::Hz400)   => 0b000110,
            (FilterCutoff::Hz12_5, FilterCutoff::Hz10)  => 0b000111,
            (FilterCutoff::Hz12_5, FilterCutoff::Hz46)  => 0b001000,
            (FilterCutoff::Hz12_5, FilterCutoff::Hz60)  => 0b001001,
            (FilterCutoff::Hz12_5, FilterCutoff::Hz250) => 0b001010,
            (FilterCutoff::Hz12_5, FilterCutoff::Hz300) => 0b001011,
            (FilterCutoff::Hz12_5, FilterCutoff::Hz400) => 0b001100,
            (FilterCutoff::Hz27, FilterCutoff::Hz10)    => 0b001101,
            (FilterCutoff::Hz27, FilterCutoff::Hz46)    => 0b001110,
            (FilterCutoff::Hz27, FilterCutoff::Hz60)    => 0b001111,
            (FilterCutoff::Hz27, FilterCutoff::Hz250)   => 0b010011,
            (FilterCutoff::Hz27, FilterCutoff::Hz300)   => 0b010100,
            (FilterCutoff::Hz27, FilterCutoff::Hz400)   => 0b010101,
            (FilterCutoff::Hz30, FilterCutoff::Hz10)    => 0b010110,
            (FilterCutoff::Hz30, FilterCutoff::Hz46)    => 0b010111,
            (FilterCutoff::Hz30, FilterCutoff::Hz60)    => 0b011000,
            (FilterCutoff::Hz30, FilterCutoff::Hz250)   => 0b011001,
            (FilterCutoff::Hz30, FilterCutoff::Hz300)   => 0b011010,
            (FilterCutoff::Hz30, FilterCutoff::Hz400)   => 0b011011,
            (FilterCutoff::Hz46, FilterCutoff::Hz10)    => 0b011100,
            (FilterCutoff::Hz46, FilterCutoff::Hz46)    => 0b011101,
            (FilterCutoff::Hz46, FilterCutoff::Hz60)    => 0b011110,
            (FilterCutoff::Hz46, FilterCutoff::Hz250)   => 0b011111,
            (FilterCutoff::Hz46, FilterCutoff::Hz300)   => 0b100011,
            (FilterCutoff::Hz46, FilterCutoff::Hz400)   => 0b100100,
            (FilterCutoff::Hz60, FilterCutoff::Hz60)    => 0b100000,
            (FilterCutoff::Hz60, FilterCutoff::Hz10)    => 0b100001,
            (FilterCutoff::Hz60, FilterCutoff::Hz46)    => 0b100110,
            (FilterCutoff::Hz60, FilterCutoff::Hz250)   => 0b101000,
            (FilterCutoff::Hz60, FilterCutoff::Hz300)   => 0b101001,
            (FilterCutoff::Hz60, FilterCutoff::Hz400)   => 0b101010,
            // Default to a safe combination if not found
            _ => 0b100000, // 60Hz gyro, 60Hz accel
        }
    }

    /// Encode filter bits for X axis based on datasheet Table 16 (Section 6.8)
    /// Note: Table 16 shows multiple bit patterns for some (gyro,accel) combinations.
    /// We select the first occurrence for each unique combination.
    fn encode_filter_bits_x(&self, gyro: FilterCutoff, accel: FilterCutoff) -> u8 {
        match (gyro, accel) {
            // D13 D12 D11 D10 D9 D8   gyro   accel
            (FilterCutoff::Hz10,  FilterCutoff::Hz10 )  => 0b000001, // Row 1
            (FilterCutoff::Hz10,  FilterCutoff::Hz46 )  => 0b000010, // Row 2
            (FilterCutoff::Hz10,  FilterCutoff::Hz60 )  => 0b000011, // Row 3
            (FilterCutoff::Hz10,  FilterCutoff::Hz250)  => 0b000100, // Row 4
            (FilterCutoff::Hz10,  FilterCutoff::Hz300)  => 0b000101, // Row 5
            (FilterCutoff::Hz10,  FilterCutoff::Hz400)  => 0b000110, // Row 6

            (FilterCutoff::Hz12_5,FilterCutoff::Hz10 )  => 0b000111, // Row 7
            (FilterCutoff::Hz12_5,FilterCutoff::Hz46 )  => 0b001000, // Row 8
            (FilterCutoff::Hz12_5,FilterCutoff::Hz60 )  => 0b001001, // Row 9
            (FilterCutoff::Hz12_5,FilterCutoff::Hz250)  => 0b001010, // Row 10
            (FilterCutoff::Hz12_5,FilterCutoff::Hz300)  => 0b001011, // Row 11
            (FilterCutoff::Hz12_5,FilterCutoff::Hz400)  => 0b001100, // Row 12

            (FilterCutoff::Hz27,  FilterCutoff::Hz10 )  => 0b001101, // Row 13
            (FilterCutoff::Hz27,  FilterCutoff::Hz46 )  => 0b001110, // Row 14
            (FilterCutoff::Hz27,  FilterCutoff::Hz60 )  => 0b001111, // Row 15
            (FilterCutoff::Hz27,  FilterCutoff::Hz250)  => 0b010011, // Row 19
            (FilterCutoff::Hz27,  FilterCutoff::Hz300)  => 0b010100, // Row 20
            (FilterCutoff::Hz27,  FilterCutoff::Hz400)  => 0b010101, // Row 21

            (FilterCutoff::Hz30,  FilterCutoff::Hz10 )  => 0b010110, // Row 22
            (FilterCutoff::Hz30,  FilterCutoff::Hz46 )  => 0b010111, // Row 23
            (FilterCutoff::Hz30,  FilterCutoff::Hz60 )  => 0b011000, // Row 24
            (FilterCutoff::Hz30,  FilterCutoff::Hz250)  => 0b011001, // Row 25
            (FilterCutoff::Hz30,  FilterCutoff::Hz300)  => 0b011010, // Row 26
            (FilterCutoff::Hz30,  FilterCutoff::Hz400)  => 0b011011, // Row 27

            (FilterCutoff::Hz46,  FilterCutoff::Hz10 )  => 0b011100, // Row 28
            (FilterCutoff::Hz46,  FilterCutoff::Hz46 )  => 0b011101, // Row 29
            (FilterCutoff::Hz46,  FilterCutoff::Hz60 )  => 0b011110, // Row 30
            (FilterCutoff::Hz46,  FilterCutoff::Hz250)  => 0b011111, // Row 31
            (FilterCutoff::Hz46,  FilterCutoff::Hz300)  => 0b100011, // Row 35
            (FilterCutoff::Hz46,  FilterCutoff::Hz400)  => 0b100100, // Row 36

            (FilterCutoff::Hz60,  FilterCutoff::Hz60 )  => 0b100000, // Row 32
            (FilterCutoff::Hz10,  FilterCutoff::Hz60 )  => 0b100001, // Row 33
            (FilterCutoff::Hz60,  FilterCutoff::Hz10 )  => 0b100010, // Row 34
            (FilterCutoff::Hz46,  FilterCutoff::Hz300)  => 0b100011, // Row 35 (alt)
            (FilterCutoff::Hz46,  FilterCutoff::Hz400)  => 0b100100, // Row 36 (alt)
            (FilterCutoff::Hz60,  FilterCutoff::Hz10 )  => 0b100101, // Row 37
            (FilterCutoff::Hz60,  FilterCutoff::Hz46 )  => 0b100110, // Row 38
            (FilterCutoff::Hz60,  FilterCutoff::Hz60 )  => 0b100111, // Row 39
            (FilterCutoff::Hz60,  FilterCutoff::Hz250)  => 0b101000, // Row 40
            (FilterCutoff::Hz60,  FilterCutoff::Hz300)  => 0b101001, // Row 41
            (FilterCutoff::Hz60,  FilterCutoff::Hz400)  => 0b101010, // Row 42

            // Any reserved or unsupported combination—best to fall back safely to 60 Hz/60 Hz
            _ => 0b100000,
        }
    }


    /// Configure digital filters (Section 6.7–6.8)
    fn configure_filters(&mut self,
        gx: FilterCutoff, gy: FilterCutoff, gz: FilterCutoff,
        ax: FilterCutoff, ay: FilterCutoff, az: FilterCutoff
    ) -> Result<(), Error<SPIE, CSE>> {
        self.set_bank(0)?;
        
        // Configure Y and Z filters using proper encoding
        let y_bits = self.encode_filter_bits_yz(gy, ay);
        let z_bits = self.encode_filter_bits_yz(gz, az);
        let yz_value = ((z_bits as u16) << 6) | (y_bits as u16 & 0x3F);
        self.write_register_spi(command::FILTER_Y_Z, yz_value)?;
        
        // Configure X filter using dedicated X-axis encoding
        let x_bits = self.encode_filter_bits_x(gx, ax);
        let x_value = (x_bits as u16) << 8;
        self.write_register_spi(command::FILTER_X, x_value)?;
        
        Ok(())
    }

    /// Apply configuration and lock writes
    pub fn configure(&mut self, cfg: &ImuConfig) -> Result<(), Error<SPIE, CSE>> {
        self.set_gyro_full_scale(cfg.gyro_scale)?;
        self.set_accel_full_scale(cfg.accel_scale)?;
        self.configure_filters(
            cfg.gyro_x_filter, cfg.gyro_y_filter, cfg.gyro_z_filter,
            cfg.accel_x_filter, cfg.accel_y_filter, cfg.accel_z_filter
        )?;
        
        // Lock writes (Section 6.13)
        self.set_bank(0)?;
        let mode_reg = self.read_register_spi(command::MODE)?;
        self.write_register_spi(command::MODE, mode_reg | 0x8000)?;
        Ok(())
    }

    /// Read raw sensor data functions
    fn read_raw_gyro_x(&mut self) -> Result<i16, Error<SPIE, CSE>> { 
        Ok(self.read_register_spi(command::GYRO_X_DATA)? as i16) 
    }
    
    fn read_raw_gyro_y(&mut self) -> Result<i16, Error<SPIE, CSE>> { 
        Ok(self.read_register_spi(command::GYRO_Y_DATA)? as i16) 
    }
    
    fn read_raw_gyro_z(&mut self) -> Result<i16, Error<SPIE, CSE>> { 
        Ok(self.read_register_spi(command::GYRO_Z_DATA)? as i16) 
    }
    
    fn read_raw_accel_x(&mut self) -> Result<i16, Error<SPIE, CSE>> { 
        Ok(self.read_register_spi(command::ACCEL_X_DATA)? as i16) 
    }
    
    fn read_raw_accel_y(&mut self) -> Result<i16, Error<SPIE, CSE>> { 
        Ok(self.read_register_spi(command::ACCEL_Y_DATA)? as i16) 
    }
    
    fn read_raw_accel_z(&mut self) -> Result<i16, Error<SPIE, CSE>> { 
        Ok(self.read_register_spi(command::ACCEL_Z_DATA)? as i16) 
    }
    
    fn read_raw_temp(&mut self) -> Result<i16, Error<SPIE, CSE>> { 
        Ok(self.read_register_spi(command::TEMP1_DATA)? as i16) 
    }
    
    fn read_raw_temp2(&mut self) -> Result<i16, Error<SPIE, CSE>> { 
        Ok(self.read_register_spi(command::TEMP2_DATA)? as i16) 
    }

    /// Read all raw sensor data at once
    fn read_raw_all(&mut self) -> Result<(i16, i16, i16, i16, i16, i16, i16), Error<SPIE, CSE>> {
        self.set_bank(0)?;
        let gx = self.read_raw_gyro_x()?;
        let gy = self.read_raw_gyro_y()?;
        let gz = self.read_raw_gyro_z()?;
        let ax = self.read_raw_accel_x()?;
        let ay = self.read_raw_accel_y()?;
        let az = self.read_raw_accel_z()?;
        let temp = self.read_raw_temp()?;
        Ok((gx, gy, gz, ax, ay, az, temp))
    }

    /// Data conversion functions (Section 4)
    fn convert_gyro(&self, raw: i16) -> f32 { 
        raw as f32 / self.gyro_scale.get_sensitivity() 
    }
    
    fn convert_accel(&self, raw: i16) -> f32 { 
        raw as f32 / self.accel_scale.get_sensitivity() 
    }
    
    fn convert_temp(&self, raw: i16) -> f32 { 
        25.0 + (raw as f32 / 20.0) 
    }

    /// Read all sensor data in engineering units
    pub fn read_imu_data(&mut self) -> Result<ImuData, Error<SPIE, CSE>> {
        let (gx, gy, gz, ax, ay, az, temp) = self.read_raw_all()?;
        
        Ok(ImuData { 
            gyro_x: self.convert_gyro(gx), 
            gyro_y: self.convert_gyro(gy), 
            gyro_z: self.convert_gyro(gz), 
            accel_x: self.convert_accel(ax), 
            accel_y: self.convert_accel(ay), 
            accel_z: self.convert_accel(az), 
            temp: self.convert_temp(temp),
        })
    }

    /// Read gyroscope data only
    pub fn read_gyro(&mut self) -> Result<(f32, f32, f32), Error<SPIE, CSE>> { 
        let (gx, gy, gz, _, _, _, _) = self.read_raw_all()?; 
        Ok((
            self.convert_gyro(gx),
            self.convert_gyro(gy),
            self.convert_gyro(gz)
        )) 
    }
    
    /// Read accelerometer data only
    pub fn read_accel(&mut self) -> Result<(f32, f32, f32), Error<SPIE, CSE>> { 
        let (_, _, _, ax, ay, az, _) = self.read_raw_all()?; 
        Ok((
            self.convert_accel(ax),
            self.convert_accel(ay),
            self.convert_accel(az)
        )) 
    }
    
    /// Read temperature sensor 1
    pub fn read_temp(&mut self) -> Result<f32, Error<SPIE, CSE>> { 
        let (_, _, _, _, _, _, temp) = self.read_raw_all()?; 
        Ok(self.convert_temp(temp)) 
    }
    
    /// Read temperature sensor 2
    pub fn read_temp2(&mut self) -> Result<f32, Error<SPIE, CSE>> { 
        let temp2 = self.read_raw_temp2()?; 
        Ok(self.convert_temp(temp2)) 
    }
    
    /// Read temperature difference between sensors
    pub fn read_temp_difference(&mut self) -> Result<f32, Error<SPIE, CSE>> { 
        let raw_delta = self.read_register_spi(command::TEMP12_DELTA)? as i16;
        // Raw delta in 1/20 °C counts
        Ok(raw_delta as f32 / 20.0)
    }

    /// Read device ID
    pub fn read_device_id(&mut self) -> Result<u8, Error<SPIE, CSE>> { 
        self.set_bank(1)?; 
        Ok(self.read_register_spi(0x0E)? as u8) 
    }
    
    /// Read fixed value register (should return 0xAA55)
    pub fn read_fixed_value(&mut self) -> Result<u16, Error<SPIE, CSE>> { 
        self.set_bank(0)?; 
        self.read_register_spi(command::FIXED_VALUE) 
    }

    /// Set capture mode
    pub fn set_capture_mode(&mut self, enable: bool) -> Result<(), Error<SPIE, CSE>> { 
        self.set_bank(0)?; 
        let mode_reg = self.read_register_spi(command::MODE)?; 
        let new_mode = if enable {
            mode_reg | 0x0008
        } else {
            mode_reg & !0x0008
        }; 
        self.write_register_spi(command::MODE, new_mode)?; 
        Ok(()) 
    }

    /// Put device to sleep (Section 6.13)
    pub fn sleep(&mut self) -> Result<(), Error<SPIE, CSE>> { 
        self.set_bank(0)?; 
        let mode_reg = self.read_register_spi(command::MODE)?; 
        self.write_register_spi(command::MODE, mode_reg | 0x8000)?; 
        Ok(()) 
    }
    
    /// Wake device from sleep (Section 6.13)
    pub fn wake(&mut self) -> Result<(), Error<SPIE, CSE>> { 
        self.set_bank(0)?; 
        let mode_reg = self.read_register_spi(command::MODE)?; 
        self.write_register_spi(command::MODE, mode_reg & !0x8000)?; 
        self.delay.delay_us(1_000); 
        Ok(()) 
    }

    /// Read low-resolution accelerometer data
    pub fn read_accel_lr(&mut self) -> Result<(f32, f32, f32), Error<SPIE, CSE>> { 
        self.set_bank(0)?; 
        let x = self.read_register_spi(command::ACCEL_X_DATA_LR)? as i16; 
        let y = self.read_register_spi(command::ACCEL_Y_DATA_LR)? as i16; 
        let z = self.read_register_spi(command::ACCEL_Z_DATA_LR)? as i16; 
        
        Ok((
            x as f32 / self.accel_scale.get_lr_sensitivity(), 
            y as f32 / self.accel_scale.get_lr_sensitivity(), 
            z as f32 / self.accel_scale.get_lr_sensitivity()
        )) 
    }

    /// Get current gyroscope scale setting
    pub fn get_gyro_scale(&self) -> GyroFullScale {
        self.gyro_scale
    }
    
    /// Get current accelerometer scale setting
    pub fn get_accel_scale(&self) -> AccelFullScale {
        self.accel_scale
    }
    
    /// Check if banks are unlocked
    pub fn banks_unlocked(&self) -> bool {
        self.banks_unlocked
    }
}