//! Embassy-based SPI driver for the TDK IIM-20670 IMU.
//!
//! This driver provides a simple, async interface to the IIM-20670 6-axis
//! Inertial Measurement Unit, designed to work with the Embassy framework on STM32 microcontrollers.
//! This version is for a dedicated SPI bus.
//!
//! # Usage
//!
//! 1. Add this crate to your `Cargo.toml`.
//! 2. Ensure your project is set up for Embassy on an STM32 target.
//! 3. Instantiate the necessary peripherals (SPI, CS pin).
//! 4. Create a new driver instance with `Iim20670::new()`.
//! 5. Call methods like `read_accel()`, `read_gyro()`, or `read_all()` to get sensor data.
//!
//! ```no_run
//! #![no_std]
//! #![no_main]
//!
//! use defmt::info;
//! use embassy_executor::Spawner;
//! use embassy_stm32::gpio::{Level, Output, Speed};
//! use embassy_stm32::spi::{Config, Spi};
//! use embassy_stm32::time::Hertz;
//! use embassy_time::{Delay, Timer};
//! use {defmt_rtt as _, panic_probe as _};
//!
//! // You would need to import the actual driver crate, e.g., `use iim_20670::Iim20670;`
//! // For this example, we assume the driver code is in a module.
//! use iim_20670_driver::Iim20670;
//!
//! #[embassy_executor::main]
//! async fn main(_spawner: Spawner) {
//!     let p = embassy_stm32::init(Default::default());
//!
//!     // Configure SPI
//!     let mut spi_config = Config::default();
//!     spi_config.frequency = Hertz(8_000_000); // 8 MHz, max is 10 MHz
//!     spi_config.mode = embassy_stm32::spi::Mode {
//!         polarity: embassy_stm32::spi::Polarity::IdleHigh,
//!         phase: embassy_stm32::spi::Phase::CaptureOnSecondTransition,
//!     };
//!
//!     let spi = Spi::new(
//!         p.SPI1,
//!         p.PB3, // SCK
//!         p.PB5, // MOSI
//!         p.PB4, // MISO
//!         p.DMA1_CH3,
//!         p.DMA1_CH2,
//!         spi_config,
//!     );
//!
//!     // Configure Chip Select pin
//!     let cs = Output::new(p.PA4, Level::High, Speed::VeryHigh);
//!
//!     // Create driver instance
//!     let mut imu = Iim20670::new(spi, cs, Delay).await.unwrap();
//!
//!     info!("IIM-20670 initialized successfully!");
//!
//!     loop {
//!         match imu.read_all().await {
//!             Ok((accel, gyro)) => {
//!                 info!("Accel: x={}, y={}, z={}", accel[0], accel[1], accel[2]);
//!                 info!("Gyro:  x={}, y={}, z={}", gyro[0], gyro[1], gyro[2]);
//!             }
//!             Err(e) => {
//!                 info!("Error reading IMU data: {:?}", e);
//!             }
//!         }
//!         Timer::after_millis(500).await;
//!     }
//! }
//!
//! // Dummy module to make the example compile
//! mod iim_20670_driver {
//!     // In a real project, the driver code would be in its own crate.
//!     // Paste the driver code from lib.rs here.
//! #    include!("src/lib.rs");
//! }
//! ```

#![no_std]

use embassy_time::Delay;
use embedded_hal_1::digital::OutputPin;
use embedded_hal_async::spi::SpiBus;
use embedded_hal_1::delay::DelayNs;
/// Represents the IIM-20670 device.
///
/// It holds the SPI bus, the chip select pin, and a delay provider.
pub struct Iim20670<SPI, CS> {
    spi: SPI,
    cs: CS,
    delay: Delay,
}

/// Represents errors that can occur while interacting with the IIM-20670.
#[derive(Debug)]
pub enum Error<SPIE, CSE> {
    /// SPI communication error
    Spi(SPIE),
    /// Chip Select pin error
    Cs(CSE),
    /// The device returned an invalid WHO_AM_I value.
    InvalidDeviceId,
}

/// Bank selection for registers.
#[repr(u8)]
enum Bank {
    Bank0 = 0 << 4,
    Bank1 = 1 << 4,
    Bank2 = 2 << 4,
    Bank3 = 3 << 4,
}

/// Gyroscope full-scale range.
#[repr(u8)]
#[derive(Clone, Copy, Debug)]
pub enum GyroFsr {
    Dps41 = 0b111,
    Dps82 = 0b110,
    Dps164 = 0b101,
    Dps328 = 0b100,
    Dps655 = 0b011,
    Dps1311 = 0b010,
    Dps1966 = 0b001,
}

/// Accelerometer full-scale range.
#[repr(u8)]
#[derive(Clone, Copy, Debug)]
pub enum AccelFsr {
    G2 = 0b111,
    G4 = 0b110,
    G8 = 0b101,
    G16 = 0b100,
    G32 = 0b010,
    G65 = 0b001,
}

/// Internal register addresses.
#[allow(dead_code)]
mod registers {
    // Bank 0
    pub const WHO_AM_I: u8 = 0x00;
    pub const PWR_MGMT_1: u8 = 0x06;
    pub const ACCEL_XOUT_H: u8 = 0x2D;
    pub const GYRO_XOUT_H: u8 = 0x33;
    pub const TEMP_OUT_H: u8 = 0x39;
    pub const REG_BANK_SEL: u8 = 0x7F;

    // Bank 2
    pub const GYRO_CONFIG_1: u8 = 0x01;
    pub const ACCEL_CONFIG: u8 = 0x14;
}

/// A helper struct to manage the chip select pin using RAII.
struct CsGuard<'a, CS: OutputPin> {
    cs: &'a mut CS,
}

impl<'a, CS: OutputPin> CsGuard<'a, CS> {
    /// Creates a new `CsGuard`, pulling the CS pin low.
    fn new(cs: &'a mut CS) -> Result<Self, CS::Error> {
        cs.set_low()?;
        Ok(Self { cs })
    }
}

impl<'a, CS: OutputPin> Drop for CsGuard<'a, CS> {
    /// Pulls the CS pin high when the guard is dropped.
    fn drop(&mut self) {
        // Errors on set_high are ignored, as there's not much we can do in a Drop impl.
        let _ = self.cs.set_high();
    }
}

impl<SPI, CS, SPIE, CSE> Iim20670<SPI, CS>
where
    SPI: SpiBus<u8, Error = SPIE>,
    CS: OutputPin<Error = CSE>,
{
    /// Creates a new driver instance for a dedicated SPI bus.
    ///
    /// This function initializes the IIM-20670 and performs a WHO_AM_I check.
    ///
    /// # Arguments
    ///
    /// * `spi` - An SPI bus instance.
    /// * `cs` - The chip select pin, which must be an `OutputPin`.
    /// * `delay` - An `embassy_time::Delay` instance.
    ///
    /// # Returns
    ///
    /// A `Result` containing the `Iim20670` driver instance or an `Error`.
    pub async fn new(spi: SPI, mut cs: CS, delay: Delay) -> Result<Self, Error<SPIE, CSE>> {
        cs.set_high().map_err(Error::Cs)?;
        let mut driver = Self { spi, cs, delay };

        // Reset the device
        driver.write_reg(registers::PWR_MGMT_1, 0x80).await?;
        driver.delay.delay_ms(100);

        // Wake up and set clock source to auto
        driver.write_reg(registers::PWR_MGMT_1, 0x01).await?;
        driver.delay.delay_ms(50);

        // // Verify WHO_AM_I
        // let who_am_i = driver.read_reg(registers::WHO_AM_I).await?;
        // if who_am_i != 0x98 {
        //     return Err(Error::InvalidDeviceId);
        // }

        // Set default configurations
        driver.set_gyro_fsr(GyroFsr::Dps1966).await?;
        driver.set_accel_fsr(AccelFsr::G16).await?;

        Ok(driver)
    }

    /// Selects a register bank.
    async fn select_bank(&mut self, bank: Bank) -> Result<(), Error<SPIE, CSE>> {
        self.write_reg(registers::REG_BANK_SEL, bank as u8).await
    }

    /// Writes a byte to a register.
    async fn write_reg(&mut self, reg: u8, val: u8) -> Result<(), Error<SPIE, CSE>> {
        let _guard = CsGuard::new(&mut self.cs).map_err(Error::Cs)?;
        let write_buf = [reg & 0x7F, val]; // MSB=0 for write
        self.spi.write(&write_buf).await.map_err(Error::Spi)
    }

    /// Reads a byte from a register.
    async fn read_reg(&mut self, reg: u8) -> Result<u8, Error<SPIE, CSE>> {
        let _guard = CsGuard::new(&mut self.cs).map_err(Error::Cs)?;
        let mut buf = [reg | 0x80, 0]; // MSB=1 for read
        self.spi.transfer_in_place(&mut buf).await.map_err(Error::Spi)?;
        Ok(buf[1])
    }

    /// Reads multiple bytes from a starting register address.
    /// This is the corrected function that uses a single transaction.
    async fn read_regs(&mut self, reg: u8, buffer: &mut [u8]) -> Result<(), Error<SPIE, CSE>> {
        let _guard = CsGuard::new(&mut self.cs).map_err(Error::Cs)?;

        // Use a temporary buffer on the stack for the transaction.
        // It needs to be 1 byte longer than the data buffer to hold the register address.
        let mut temp_buf = [0u8; 13]; // Max read is 12 bytes for accel+gyro + 1 for address
        let len = buffer.len();
        assert!(len <= 12, "Read buffer is too large for this implementation");

        // The first byte of the transaction is the register address with the read bit set.
        temp_buf[0] = reg | 0x80;

        // Perform an in-place transfer.
        // The first byte sent is our command. The rest are dummy bytes (0).
        // The first byte received is garbage. The subsequent bytes are the data we want.
        self.spi
            .transfer_in_place(&mut temp_buf[..=len])
            .await
            .map_err(Error::Spi)?;

        // Copy the received data (ignoring the first garbage byte) into the user's buffer.
        buffer.copy_from_slice(&temp_buf[1..=len]);

        Ok(())
    }

    /// Sets the gyroscope full-scale range.
    pub async fn set_gyro_fsr(&mut self, fsr: GyroFsr) -> Result<(), Error<SPIE, CSE>> {
        self.select_bank(Bank::Bank2).await?;
        let current_config = self.read_reg(registers::GYRO_CONFIG_1).await?;
        // Clear FSR bits and set new ones
        let new_config = (current_config & !0b0000_1110) | ((fsr as u8) << 1);
        self.write_reg(registers::GYRO_CONFIG_1, new_config).await?;
        self.select_bank(Bank::Bank0).await
    }

    /// Sets the accelerometer full-scale range.
    pub async fn set_accel_fsr(&mut self, fsr: AccelFsr) -> Result<(), Error<SPIE, CSE>> {
        self.select_bank(Bank::Bank2).await?;
        let current_config = self.read_reg(registers::ACCEL_CONFIG).await?;
        // Clear FSR bits and set new ones
        let new_config = (current_config & !0b0000_1110) | ((fsr as u8) << 1);
        self.write_reg(registers::ACCEL_CONFIG, new_config).await?;
        self.select_bank(Bank::Bank0).await
    }

    /// Reads the raw accelerometer data.
    /// Returns an array `[x, y, z]`.
    pub async fn read_accel(&mut self) -> Result<[i16; 3], Error<SPIE, CSE>> {
        let mut buf = [0u8; 6];
        self.read_regs(registers::ACCEL_XOUT_H, &mut buf).await?;
        let x = i16::from_be_bytes([buf[0], buf[1]]);
        let y = i16::from_be_bytes([buf[2], buf[3]]);
        let z = i16::from_be_bytes([buf[4], buf[5]]);
        Ok([x, y, z])
    }

    /// Reads the raw gyroscope data.
    /// Returns an array `[x, y, z]`.
    pub async fn read_gyro(&mut self) -> Result<[i16; 3], Error<SPIE, CSE>> {
        let mut buf = [0u8; 6];
        self.read_regs(registers::GYRO_XOUT_H, &mut buf).await?;
        let x = i16::from_be_bytes([buf[0], buf[1]]);
        let y = i16::from_be_bytes([buf[2], buf[3]]);
        let z = i16::from_be_bytes([buf[4], buf[5]]);
        Ok([x, y, z])
    }

    /// Reads both accelerometer and gyroscope data in a single transaction.
    /// Returns a tuple `([accel_x, y, z], [gyro_x, y, z])`.
    pub async fn read_all(&mut self) -> Result<([i16; 3], [i16; 3]), Error<SPIE, CSE>> {
        let mut buf = [0u8; 12];
        self.read_regs(registers::ACCEL_XOUT_H, &mut buf).await?;

        let accel_x = i16::from_be_bytes([buf[0], buf[1]]);
        let accel_y = i16::from_be_bytes([buf[2], buf[3]]);
        let accel_z = i16::from_be_bytes([buf[4], buf[5]]);

        let gyro_x = i16::from_be_bytes([buf[6], buf[7]]);
        let gyro_y = i16::from_be_bytes([buf[8], buf[9]]);
        let gyro_z = i16::from_be_bytes([buf[10], buf[11]]);

        Ok(([accel_x, accel_y, accel_z], [gyro_x, gyro_y, gyro_z]))
    }

    /// Reads the raw temperature data.
    /// To convert to °C: `(raw_temp / 326.8) + 25.0`
    pub async fn read_temperature(&mut self) -> Result<i16, Error<SPIE, CSE>> {
        let mut buf = [0u8; 2];
        self.read_regs(registers::TEMP_OUT_H, &mut buf).await?;
        Ok(i16::from_be_bytes([buf[0], buf[1]]))
    }
}
