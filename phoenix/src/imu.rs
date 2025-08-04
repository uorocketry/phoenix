// // //! Blocking SPI driver for the TDK IIM-20670 IMU.
// // //!
// // //! This driver provides a simple, blocking interface to the IIM-20670 6-axis
// // //! Inertial Measurement Unit, using standard `embedded-hal` traits.
// // //!
// // //! # Usage
// // //!
// // //! 1. Add this crate and your target's `embedded-hal` implementation (e.g., `stm32f4xx-hal`) to your `Cargo.toml`.
// // //! 2. Instantiate the necessary peripherals (SPI, CS pin, Delay).
// // //! 3. Create a new driver instance with `Iim20670::new()`.
// // //! 4. Call methods like `read_accel()`, `read_gyro()`, or `read_all()` to get sensor data.
// // //!
// // //! ```no_run
// // //! #![no_std]
// // //! #![no_main]
// // //!
// // //! use cortex_m_rt::entry;
// // //! use stm32f4xx_hal::{
// // //!     pac,
// // //!     prelude::*,
// // //!     spi::{Mode, Phase, Polarity, Spi},
// // //! };
// // //! use defmt::info;
// // //! use {defmt_rtt as _, panic_probe as _};
// // //!
// // //! // Import the synchronous driver from its module or crate
// // //! use iim_20670_driver::Iim20670;
// // //!
// // //! #[entry]
// // //! fn main() -> ! {
// // //!     let dp = pac::Peripherals::take().unwrap();
// // //!     let cp = cortex_m::Peripherals::take().unwrap();
// // //!
// // //!     let rcc = dp.RCC.constrain();
// // //!     let clocks = rcc.cfgr.use_hse(8.MHz()).sysclk(100.MHz()).freeze();
// // //!
// // //!     let mut delay = cp.SYST.delay(&clocks);
// // //!
// // //!     let gpioa = dp.GPIOA.split();
// // //!     let gpiob = dp.GPIOB.split();
// // //!
// // //!     // Configure SPI pins
// // //!     let sck = gpiob.pb3.into_alternate();
// // //!     let miso = gpiob.pb4.into_alternate();
// // //!     let mosi = gpiob.pb5.into_alternate();
// // //!
// // //!     // Configure Chip Select pin
// // //!     let mut cs = gpioa.pa4.into_push_pull_output();
// // //!     cs.set_high(); // Deselect device
// // //!
// // //!     // Configure SPI peripheral
// // //!     let spi = Spi::new(
// // //!         dp.SPI1,
// // //!         (sck, miso, mosi),
// // //!         Mode {
// // //!             polarity: Polarity::IdleHigh,
// // //!             phase: Phase::CaptureOnSecondTransition,
// // //!         },
// // //!         8.MHz(),
// // //!         &clocks,
// // //!     );
// // //!
// // //!     // Create driver instance
// // //!     let mut imu = Iim20670::new(spi, cs, &mut delay).unwrap();
// // //!
// // //!     info!("IIM-20670 initialized successfully!");
// // //!
// // //!     loop {
// // //!         match imu.read_all() {
// // //!             Ok((accel, gyro)) => {
// // //!                 info!("Accel: x={}, y={}, z={}", accel[0], accel[1], accel[2]);
// // //!                 info!("Gyro:  x={}, y={}, z={}", gyro[0], gyro[1], gyro[2]);
// // //!             }
// // //!             Err(_e) => {
// // //!                 info!("Error reading IMU data!");
// // //!             }
// // //!         }
// // //!         delay.delay_ms(500);
// // //!     }
// // //! }
// // //!
// // //! // Dummy module to make the example compile as a standalone file
// // //! mod iim_20670_driver {
// // //!     // In a real project, this would be `use iim_20670;`
// // //!     #include "src/lib.rs"
// // //! }
// // //! ```

// // #![no_std]

// // use embedded_hal_1::delay::DelayNs;
// // use embedded_hal_1::digital::OutputPin;
// // use embedded_hal_1::spi::SpiBus;

// // /// Represents the IIM-20670 device.
// // ///
// // /// It holds the SPI bus, the chip select pin, and a delay provider.
// // pub struct Iim20670<SPI, CS, DELAY> {
// //     spi: SPI,
// //     cs: CS,
// //     delay: DELAY,
// // }

// // /// Represents errors that can occur while interacting with the IIM-20670.
// // #[derive(Debug)]
// // pub enum Error<SPIE, CSE> {
// //     /// SPI communication error
// //     Spi(SPIE),
// //     /// Chip Select pin error
// //     Cs(CSE),
// //     /// The device returned an invalid WHO_AM_I value.
// //     InvalidDeviceId,
// // }

// // /// Bank selection for registers.
// // #[repr(u8)]
// // enum Bank {
// //     Bank0 = 0 << 4,
// //     Bank1 = 1 << 4,
// //     Bank2 = 2 << 4,
// //     Bank3 = 3 << 4,
// // }

// // /// Gyroscope full-scale range.
// // #[repr(u8)]
// // #[derive(Clone, Copy, Debug)]
// // pub enum GyroFsr {
// //     Dps41 = 0b111,
// //     Dps82 = 0b110,
// //     Dps164 = 0b101,
// //     Dps328 = 0b100,
// //     Dps655 = 0b011,
// //     Dps1311 = 0b010,
// //     Dps1966 = 0b001,
// // }

// // /// Accelerometer full-scale range.
// // #[repr(u8)]
// // #[derive(Clone, Copy, Debug)]
// // pub enum AccelFsr {
// //     G2 = 0b111,
// //     G4 = 0b110,
// //     G8 = 0b101,
// //     G16 = 0b100,
// //     G32 = 0b010,
// //     G65 = 0b001,
// // }

// // /// Internal register addresses.
// // #[allow(dead_code)]
// // mod registers {
// //     // Bank 0
// //     pub const WHO_AM_I: u8 = 0x00;
// //     pub const PWR_MGMT_1: u8 = 0x06;
// //     pub const ACCEL_XOUT_H: u8 = 0x2D;
// //     pub const GYRO_XOUT_H: u8 = 0x33;
// //     pub const TEMP_OUT_H: u8 = 0x39;
// //     pub const REG_BANK_SEL: u8 = 0x7F;

// //     // Bank 2
// //     pub const GYRO_CONFIG_1: u8 = 0x01;
// //     pub const ACCEL_CONFIG: u8 = 0x14;
// // }

// // /// A helper struct to manage the chip select pin using RAII.
// // struct CsGuard<'a, CS: OutputPin> {
// //     cs: &'a mut CS,
// // }

// // impl<'a, CS: OutputPin> CsGuard<'a, CS> {
// //     /// Creates a new `CsGuard`, pulling the CS pin low.
// //     fn new(cs: &'a mut CS) -> Result<Self, CS::Error> {
// //         cs.set_low()?;
// //         Ok(Self { cs })
// //     }
// // }

// // impl<'a, CS: OutputPin> Drop for CsGuard<'a, CS> {
// //     /// Pulls the CS pin high when the guard is dropped.
// //     fn drop(&mut self) {
// //         // Errors on set_high are ignored, as there's not much we can do in a Drop impl.
// //         let _ = self.cs.set_high();
// //     }
// // }

// // impl<SPI, CS, DELAY, SPIE, CSE> Iim20670<SPI, CS, DELAY>
// // where
// //     SPI: SpiBus<u8, Error = SPIE>,
// //     CS: OutputPin<Error = CSE>,
// //     DELAY: DelayNs,
// // {
// //     /// Creates a new driver instance.
// //     ///
// //     /// This function initializes the IIM-20670 and performs a WHO_AM_I check.
// //     ///
// //     /// # Arguments
// //     ///
// //     /// * `spi` - An SPI bus instance that implements `embedded_hal::spi::SpiBus`.
// //     /// * `cs` - The chip select pin, which must implement `embedded_hal::digital::OutputPin`.
// //     /// * `delay` - A delay provider that implements `embedded_hal::delay::DelayNs`.
// //     ///
// //     /// # Returns
// //     ///
// //     /// A `Result` containing the `Iim20670` driver instance or an `Error`.
// //     pub fn new(spi: SPI, mut cs: CS, mut delay: DELAY) -> Result<Self, Error<SPIE, CSE>> {
// //         cs.set_high().map_err(Error::Cs)?;
// //         let mut driver = Self { spi, cs, delay };

// //         // Reset the device
// //         driver.write_reg(registers::PWR_MGMT_1, 0x80)?;
// //         driver.delay.delay_ms(100);

// //         // Wake up and set clock source to auto
// //         driver.write_reg(registers::PWR_MGMT_1, 0x01)?;
// //         driver.delay.delay_ms(50);

// //         // Verify WHO_AM_I
// //         // let who_am_i = driver.read_reg(registers::WHO_AM_I)?;
// //         // if who_am_i != 0x98 {
// //         //     return Err(Error::InvalidDeviceId);
// //         // }

// //         // Set default configurations
// //         driver.set_gyro_fsr(GyroFsr::Dps1966)?;
// //         driver.set_accel_fsr(AccelFsr::G16)?;

// //         Ok(driver)
// //     }

// //     /// Selects a register bank.
// //     fn select_bank(&mut self, bank: Bank) -> Result<(), Error<SPIE, CSE>> {
// //         self.write_reg(registers::REG_BANK_SEL, bank as u8)
// //     }

// //     /// Writes a byte to a register.
// //     fn write_reg(&mut self, reg: u8, val: u8) -> Result<(), Error<SPIE, CSE>> {
// //         let _guard = CsGuard::new(&mut self.cs).map_err(Error::Cs)?;
// //         let write_buf = [reg & 0x7F, val]; // MSB=0 for write
// //         self.spi.write(&write_buf).map_err(Error::Spi)
// //     }

// //     /// Reads a byte from a register.
// //     fn read_reg(&mut self, reg: u8) -> Result<u8, Error<SPIE, CSE>> {
// //         let _guard = CsGuard::new(&mut self.cs).map_err(Error::Cs)?;
// //         let mut buf = [reg | 0x80, 0]; // MSB=1 for read
// //         self.spi.transfer_in_place(&mut buf).map_err(Error::Spi)?;
// //         Ok(buf[1])
// //     }

// //     /// Reads multiple bytes from a starting register address.
// //     fn read_regs(&mut self, reg: u8, buffer: &mut [u8]) -> Result<(), Error<SPIE, CSE>> {
// //         let _guard = CsGuard::new(&mut self.cs).map_err(Error::Cs)?;

// //         let mut temp_buf = [0u8; 13]; // Max read is 12 bytes for accel+gyro + 1 for address
// //         let len = buffer.len();
// //         assert!(len <= 12, "Read buffer is too large for this implementation");

// //         temp_buf[0] = reg | 0x80;

// //         self.spi
// //             .transfer_in_place(&mut temp_buf[..=len])
// //             .map_err(Error::Spi)?;

// //         buffer.copy_from_slice(&temp_buf[1..=len]);

// //         Ok(())
// //     }

// //     /// Sets the gyroscope full-scale range.
// //     pub fn set_gyro_fsr(&mut self, fsr: GyroFsr) -> Result<(), Error<SPIE, CSE>> {
// //         self.select_bank(Bank::Bank2)?;
// //         let current_config = self.read_reg(registers::GYRO_CONFIG_1)?;
// //         // Clear FSR bits and set new ones
// //         let new_config = (current_config & !0b0000_1110) | ((fsr as u8) << 1);
// //         self.write_reg(registers::GYRO_CONFIG_1, new_config)?;
// //         self.select_bank(Bank::Bank0)
// //     }

// //     /// Sets the accelerometer full-scale range.
// //     pub fn set_accel_fsr(&mut self, fsr: AccelFsr) -> Result<(), Error<SPIE, CSE>> {
// //         self.select_bank(Bank::Bank2)?;
// //         let current_config = self.read_reg(registers::ACCEL_CONFIG)?;
// //         // Clear FSR bits and set new ones
// //         let new_config = (current_config & !0b0000_1110) | ((fsr as u8) << 1);
// //         self.write_reg(registers::ACCEL_CONFIG, new_config)?;
// //         self.select_bank(Bank::Bank0)
// //     }

// //     /// Reads the raw accelerometer data.
// //     /// Returns an array `[x, y, z]`.
// //     pub fn read_accel(&mut self) -> Result<[i16; 3], Error<SPIE, CSE>> {
// //         let mut buf = [0u8; 6];
// //         self.read_regs(registers::ACCEL_XOUT_H, &mut buf)?;
// //         let x = i16::from_be_bytes([buf[0], buf[1]]);
// //         let y = i16::from_be_bytes([buf[2], buf[3]]);
// //         let z = i16::from_be_bytes([buf[4], buf[5]]);
// //         Ok([x, y, z])
// //     }

// //     /// Reads the raw gyroscope data.
// //     /// Returns an array `[x, y, z]`.
// //     pub fn read_gyro(&mut self) -> Result<[i16; 3], Error<SPIE, CSE>> {
// //         let mut buf = [0u8; 6];
// //         self.read_regs(registers::GYRO_XOUT_H, &mut buf)?;
// //         let x = i16::from_be_bytes([buf[0], buf[1]]);
// //         let y = i16::from_be_bytes([buf[2], buf[3]]);
// //         let z = i16::from_be_bytes([buf[4], buf[5]]);
// //         Ok([x, y, z])
// //     }

// //     /// Reads both accelerometer and gyroscope data in a single transaction.
// //     /// Returns a tuple `([accel_x, y, z], [gyro_x, y, z])`.
// //     pub fn read_all(&mut self) -> Result<([i16; 3], [i16; 3]), Error<SPIE, CSE>> {
// //         let mut buf = [0u8; 12];
// //         self.read_regs(registers::ACCEL_XOUT_H, &mut buf)?;

// //         let accel_x = i16::from_be_bytes([buf[0], buf[1]]);
// //         let accel_y = i16::from_be_bytes([buf[2], buf[3]]);
// //         let accel_z = i16::from_be_bytes([buf[4], buf[5]]);

// //         let gyro_x = i16::from_be_bytes([buf[6], buf[7]]);
// //         let gyro_y = i16::from_be_bytes([buf[8], buf[9]]);
// //         let gyro_z = i16::from_be_bytes([buf[10], buf[11]]);

// //         Ok(([accel_x, accel_y, accel_z], [gyro_x, gyro_y, gyro_z]))
// //     }

// //     /// Reads the raw temperature data.
// //     /// To convert to °C: `(raw_temp / 326.8) + 25.0`
// //     pub fn read_temperature(&mut self) -> Result<i16, Error<SPIE, CSE>> {
// //         let mut buf = [0u8; 2];
// //         self.read_regs(registers::TEMP_OUT_H, &mut buf)?;
// //         Ok(i16::from_be_bytes([buf[0], buf[1]]))
// //     }
// // }


// //! Blocking SPI driver for the TDK IIM-20670 IMU.
// //!
// //! This driver provides a simple, blocking interface to the IIM-20670 6-axis
// //! Inertial Measurement Unit, using standard `embedded-hal` traits.
// //!
// //! # Usage
// //!
// //! 1. Add this crate and your target's `embedded-hal` implementation (e.g., `stm32f4xx-hal`) to your `Cargo.toml`.
// //! 2. Instantiate the necessary peripherals (SPI, CS pin, nReset pin, Delay).
// //! 3. Create a new driver instance with `Iim20670::new()`.
// //! 4. Call methods like `read_accel()`, `read_gyro()`, or `read_all()` to get sensor data.
// //!
// //! ```no_run
// //! #![no_std]
// //! #![no_main]
// //!
// //! use cortex_m_rt::entry;
// //! use stm32f4xx_hal::{
// //!     pac,
// //!     prelude::*,
// //!     spi::{Mode, Phase, Polarity, Spi},
// //! };
// //! use defmt::info;
// //! use {defmt_rtt as _, panic_probe as _};
// //!
// //! // Import the synchronous driver from its module or crate
// //! use iim_20670_driver::Iim20670;
// //!
// //! #[entry]
// //! fn main() -> ! {
// //!     let dp = pac::Peripherals::take().unwrap();
// //!     let cp = cortex_m::Peripherals::take().unwrap();
// //!
// //!     let rcc = dp.RCC.constrain();
// //!     let clocks = rcc.cfgr.use_hse(8.MHz()).sysclk(100.MHz()).freeze();
// //!
// //!     let mut delay = cp.SYST.delay(&clocks);
// //!
// //!     let gpioa = dp.GPIOA.split();
// //!     let gpiob = dp.GPIOB.split();
// //!
// //!     // Configure SPI pins
// //!     let sck = gpiob.pb3.into_alternate();
// //!     let miso = gpiob.pb4.into_alternate();
// //!     let mosi = gpiob.pb5.into_alternate();
// //!
// //!     // Configure Chip Select and nReset pins
// //!     let mut cs = gpioa.pa4.into_push_pull_output();
// //!     cs.set_high(); // Deselect device
// //!     let mut nreset = gpioa.pa5.into_push_pull_output(); // Example pin
// //!
// //!     // Configure SPI peripheral
// //!     let spi = Spi::new(
// //!         dp.SPI1,
// //!         (sck, miso, mosi),
// //!         Mode { // SPI Mode 0
// //!             polarity: Polarity::IdleLow,
// //!             phase: Phase::CaptureOnFirstTransition,
// //!         },
// //!         8.MHz(),
// //!         &clocks,
// //!     );
// //!
// //!     // Create driver instance with the reset pin
// //!     let mut imu = Iim20670::new(spi, cs, Some(nreset), &mut delay).unwrap();
// //!
// //!     info!("IIM-20670 initialized successfully!");
// //!
// //!     loop {
// //!         match imu.read_all() {
// //!             Ok((accel, gyro)) => {
// //!                 info!("Accel: x={}, y={}, z={}", accel[0], accel[1], accel[2]);
// //!                 info!("Gyro:  x={}, y={}, z={}", gyro[0], gyro[1], gyro[2]);
// //!             }
// //!             Err(_e) => {
// //!                 info!("Error reading IMU data!");
// //!             }
// //!         }
// //!         delay.delay_ms(500);
// //!     }
// //! }
// //!
// //! // Dummy module to make the example compile as a standalone file
// //! mod iim_20670_driver {
// //!     // In a real project, this would be `use iim_20670;`
// //!     #include "src/lib.rs"
// //! }
// //! ```

// #![no_std]

// use defmt::info;
// use embedded_hal_1::delay::DelayNs;
// use embedded_hal_1::digital::OutputPin;
// use embedded_hal_1::spi::SpiBus;

// /// Represents the IIM-20670 device.
// ///
// /// It holds the SPI bus, chip select pin, reset pin (optional), and a delay provider.
// pub struct Iim20670<SPI, CS, NRESET, DELAY> {
//     spi: SPI,
//     cs: CS,
//     nreset: Option<NRESET>,
//     delay: DELAY,
// }

// /// Represents errors that can occur while interacting with the IIM-20670.
// #[derive(Debug)]
// pub enum Error<SPIE, CSE, RESETE> {
//     /// SPI communication error
//     Spi(SPIE),
//     /// Chip Select pin error
//     Cs(CSE),
//     /// nReset pin error
//     Reset(RESETE),
//     /// The device returned an invalid WHO_AM_I value.
//     InvalidDeviceId,
// }

// /// Bank selection for registers.
// #[repr(u8)]
// enum Bank {
//     Bank0 = 0 << 4,
//     Bank1 = 1 << 4,
//     Bank2 = 2 << 4,
//     Bank3 = 3 << 4,
// }

// /// Gyroscope full-scale range.
// #[repr(u8)]
// #[derive(Clone, Copy, Debug)]
// pub enum GyroFsr {
//     Dps41 = 0b111,    Dps82 = 0b110,    Dps164 = 0b101,    Dps328 = 0b100,
//     Dps655 = 0b011,    Dps1311 = 0b010,    Dps1966 = 0b001,
// }

// /// Accelerometer full-scale range.
// #[repr(u8)]
// #[derive(Clone, Copy, Debug)]
// pub enum AccelFsr {
//     G2 = 0b111,    G4 = 0b110,    G8 = 0b101,    G16 = 0b100,
//     G32 = 0b010,    G65 = 0b001,
// }

// /// Internal register addresses.
// #[allow(dead_code)]
// mod registers {
//     // Bank 0
//     pub const WHO_AM_I: u8 = 0x00;
//     pub const PWR_MGMT_1: u8 = 0x06;
//     pub const ACCEL_XOUT_H: u8 = 0x2D;
//     pub const GYRO_XOUT_H: u8 = 0x33;
//     pub const TEMP_OUT_H: u8 = 0x39;
//     pub const REG_BANK_SEL: u8 = 0x7F;

//     // Bank 2
//     pub const GYRO_CONFIG_1: u8 = 0x01;
//     pub const ACCEL_CONFIG: u8 = 0x14;
// }

// /// A helper struct to manage the chip select pin using RAII.
// struct CsGuard<'a, CS: OutputPin> {
//     cs: &'a mut CS,
// }

// impl<'a, CS: OutputPin> CsGuard<'a, CS> {
//     /// Creates a new `CsGuard`, pulling the CS pin low.
//     fn new(cs: &'a mut CS) -> Result<Self, CS::Error> {
//         cs.set_low()?;
//         Ok(Self { cs })
//     }
// }

// impl<'a, CS: OutputPin> Drop for CsGuard<'a, CS> {
//     /// Pulls the CS pin high when the guard is dropped.
//     fn drop(&mut self) {
//         let _ = self.cs.set_high();
//     }
// }

// impl<SPI, CS, NRESET, DELAY, SPIE, CSE, RESETE> Iim20670<SPI, CS, NRESET, DELAY>
// where
//     SPI: SpiBus<u8, Error = SPIE>,
//     CS: OutputPin<Error = CSE>,
//     NRESET: OutputPin<Error = RESETE>,
//     DELAY: DelayNs,
// {
//     /// Creates a new driver instance.
//     ///
//     /// This function initializes the IIM-20670 and performs a WHO_AM_I check.
//     ///
//     /// # Arguments
//     ///
//     /// * `spi` - An SPI bus instance.
//     /// * `cs` - The chip select pin.
//     /// * `nreset` - An optional active-low reset pin.
//     /// * `delay` - A delay provider.
//     pub fn new(
//         spi: SPI,
//         mut cs: CS,
//         mut nreset: Option<NRESET>,
//         mut delay: DELAY,
//     ) -> Result<Self, Error<SPIE, CSE, RESETE>> {
//         cs.set_high().map_err(Error::Cs)?;
        
//         // Perform hardware reset if the pin is provided
//         // if let Some(ref mut reset_pin) = nreset {
//         //     reset_pin.set_high().map_err(Error::Reset)?;
//         //     delay.delay_ms(1);
//         //     reset_pin.set_low().map_err(Error::Reset)?;
//         //     delay.delay_ms(40); // Datasheet specifies >30ms
//         //     reset_pin.set_high().map_err(Error::Reset)?;
//         //     delay.delay_ms(100); // Wait for the device to boot up
//         // }

//         let mut driver = Self { spi, cs, nreset, delay };

//         // If no hardware reset pin was provided, perform a software reset.
//         if driver.nreset.is_none() {
//             driver.write_reg(registers::PWR_MGMT_1, 0x80)?;
//             driver.delay.delay_ms(100);
//         }

//         // Wake up and set clock source to auto
//         driver.write_reg(registers::PWR_MGMT_1, 0x01)?;
//         driver.delay.delay_ms(50);

//         // Verify WHO_AM_I
//         // let who_am_i = driver.read_reg(registers::WHO_AM_I)?;
//         // if who_am_i != 0x98 {
//         //     return Err(Error::InvalidDeviceId);
//         // }
//         info!("Verifying WHO_AM_I...");
//         let who_am_i = driver.read_reg(registers::WHO_AM_I)?;
//         info!("WHO_AM_I value: {=u8}", who_am_i);

//         // Set default configurations
//         driver.set_gyro_fsr(GyroFsr::Dps1966)?;
//         driver.set_accel_fsr(AccelFsr::G16)?;

//         Ok(driver)
//     }

//     /// Selects a register bank.
//     fn select_bank(&mut self, bank: Bank) -> Result<(), Error<SPIE, CSE, RESETE>> {
//         self.write_reg(registers::REG_BANK_SEL, bank as u8)
//     }

//     /// Writes a byte to a register.
//     fn write_reg(&mut self, reg: u8, val: u8) -> Result<(), Error<SPIE, CSE, RESETE>> {
//         let _guard = CsGuard::new(&mut self.cs).map_err(Error::Cs)?;
//         let write_buf = [reg & 0x7F, val]; // MSB=0 for write
//         self.spi.write(&write_buf).map_err(Error::Spi)
//     }

//     /// Reads a byte from a register.
//     fn read_reg(&mut self, reg: u8) -> Result<u8, Error<SPIE, CSE, RESETE>> {
//         let _guard = CsGuard::new(&mut self.cs).map_err(Error::Cs)?;
//         let mut buf = [reg | 0x80, 0]; // MSB=1 for read
//         self.spi.transfer_in_place(&mut buf).map_err(Error::Spi)?;
//         Ok(buf[1])
//     }

//     /// Reads multiple bytes from a starting register address.
//     fn read_regs(&mut self, reg: u8, buffer: &mut [u8]) -> Result<(), Error<SPIE, CSE, RESETE>> {
//         let _guard = CsGuard::new(&mut self.cs).map_err(Error::Cs)?;
//         let mut temp_buf = [0u8; 13];
//         let len = buffer.len();
//         assert!(len <= 12, "Read buffer is too large");
//         temp_buf[0] = reg | 0x80;
//         self.spi
//             .transfer_in_place(&mut temp_buf[..=len])
//             .map_err(Error::Spi)?;
//         buffer.copy_from_slice(&temp_buf[1..=len]);
//         Ok(())
//     }

//     /// Sets the gyroscope full-scale range.
//     pub fn set_gyro_fsr(&mut self, fsr: GyroFsr) -> Result<(), Error<SPIE, CSE, RESETE>> {
//         self.select_bank(Bank::Bank2)?;
//         let current_config = self.read_reg(registers::GYRO_CONFIG_1)?;
//         let new_config = (current_config & !0b0000_1110) | ((fsr as u8) << 1);
//         self.write_reg(registers::GYRO_CONFIG_1, new_config)?;
//         self.select_bank(Bank::Bank0)
//     }

//     /// Sets the accelerometer full-scale range.
//     pub fn set_accel_fsr(&mut self, fsr: AccelFsr) -> Result<(), Error<SPIE, CSE, RESETE>> {
//         self.select_bank(Bank::Bank2)?;
//         let current_config = self.read_reg(registers::ACCEL_CONFIG)?;
//         let new_config = (current_config & !0b0000_1110) | ((fsr as u8) << 1);
//         self.write_reg(registers::ACCEL_CONFIG, new_config)?;
//         self.select_bank(Bank::Bank0)
//     }

//     /// Reads the raw accelerometer data.
//     pub fn read_accel(&mut self) -> Result<[i16; 3], Error<SPIE, CSE, RESETE>> {
//         let mut buf = [0u8; 6];
//         self.read_regs(registers::ACCEL_XOUT_H, &mut buf)?;
//         let x = i16::from_be_bytes([buf[0], buf[1]]);
//         let y = i16::from_be_bytes([buf[2], buf[3]]);
//         let z = i16::from_be_bytes([buf[4], buf[5]]);
//         Ok([x, y, z])
//     }

//     /// Reads the raw gyroscope data.
//     pub fn read_gyro(&mut self) -> Result<[i16; 3], Error<SPIE, CSE, RESETE>> {
//         let mut buf = [0u8; 6];
//         self.read_regs(registers::GYRO_XOUT_H, &mut buf)?;
//         info!("Read Gyro: {:?}", buf.clone());
//         let x = i16::from_be_bytes([buf[0], buf[1]]);
//         let y = i16::from_be_bytes([buf[2], buf[3]]);
//         let z = i16::from_be_bytes([buf[4], buf[5]]);
//         Ok([x, y, z])
//     }

//     /// Reads both accelerometer and gyroscope data in a single transaction.
//     pub fn read_all(&mut self) -> Result<([i16; 3], [i16; 3]), Error<SPIE, CSE, RESETE>> {
//         let mut buf = [0u8; 12];
//         self.read_regs(registers::ACCEL_XOUT_H, &mut buf)?;
//         let accel_x = i16::from_be_bytes([buf[0], buf[1]]);
//         let accel_y = i16::from_be_bytes([buf[2], buf[3]]);
//         let accel_z = i16::from_be_bytes([buf[4], buf[5]]);
//         let gyro_x = i16::from_be_bytes([buf[6], buf[7]]);
//         let gyro_y = i16::from_be_bytes([buf[8], buf[9]]);
//         let gyro_z = i16::from_be_bytes([buf[10], buf[11]]);
//         Ok(([accel_x, accel_y, accel_z], [gyro_x, gyro_y, gyro_z]))
//     }

//     /// Reads the raw temperature data.
//     pub fn read_temperature(&mut self) -> Result<i16, Error<SPIE, CSE, RESETE>> {
//         let mut buf = [0u8; 2];
//         self.read_regs(registers::TEMP_OUT_H, &mut buf)?;
//         Ok(i16::from_be_bytes([buf[0], buf[1]]))
//     }
// }


// //! Blocking SPI driver for the TDK IIM-20670 IMU.

// #![no_std]

// use defmt::info;
// use embedded_hal_1::delay::DelayNs;
// use embedded_hal_1::digital::OutputPin;
// use embedded_hal_1::spi::SpiBus;

// /// Represents the IIM-20670 device.
// pub struct Iim20670<SPI, CS, NRESET, DELAY> {
//     spi: SPI,
//     cs: CS,
//     nreset: Option<NRESET>,
//     delay: DELAY,
// }

// /// Represents errors that can occur while interacting with the IIM-20670.
// #[derive(Debug)]
// pub enum Error<SPIE, CSE, RESETE> {
//     Spi(SPIE),
//     Cs(CSE),
//     Reset(RESETE),
//     InvalidDeviceId,
// }

// // CORRECTED: Register addresses according to IIM-20670 datasheet.
// #[allow(dead_code)]
// mod registers {
//     // Bank 0
//     pub const GYRO_X_DATA: u8 = 0x00;
//     pub const GYRO_Y_DATA: u8 = 0x01;
//     pub const GYRO_Z_DATA: u8 = 0x02;
//     pub const TEMP1_DATA: u8 = 0x03;
//     pub const ACCEL_X_DATA: u8 = 0x04;
//     pub const ACCEL_Y_DATA: u8 = 0x05;
//     pub const ACCEL_Z_DATA: u8 = 0x06;
//     pub const RESET_CTRL: u8 = 0x18;
//     pub const BANK_SELECT: u8 = 0x1F;

//     // Bank 6 & 7 (for sensitivity config)
//     pub const SENSITIVITY_CONFIG: u8 = 0x14;
// }

// // Bank selection now uses direct values from the datasheet.
// #[repr(u16)]
// enum Bank {
//     Bank0 = 0x0000,
//     Bank6 = 0x0006,
//     Bank7 = 0x0007,
// }

// /// A helper struct to manage the chip select pin using RAII.
// struct CsGuard<'a, CS: OutputPin> {
//     cs: &'a mut CS,
// }

// impl<'a, CS: OutputPin> CsGuard<'a, CS> {
//     fn new(cs: &'a mut CS) -> Result<Self, CS::Error> {
//         cs.set_low()?;
//         Ok(Self { cs })
//     }
// }

// impl<'a, CS: OutputPin> Drop for CsGuard<'a, CS> {
//     fn drop(&mut self) {
//         let _ = self.cs.set_high();
//     }
// }

// impl<SPI, CS, NRESET, DELAY, SPIE, CSE, RESETE> Iim20670<SPI, CS, NRESET, DELAY>
// where
//     SPI: SpiBus<u8, Error = SPIE>,
//     CS: OutputPin<Error = CSE>,
//     NRESET: OutputPin<Error = RESETE>,
//     DELAY: DelayNs,
// {
//     pub fn new(
//         spi: SPI,
//         mut cs: CS,
//         mut nreset: Option<NRESET>,
//         mut delay: DELAY,
//     ) -> Result<Self, Error<SPIE, CSE, RESETE>> {
//         cs.set_high().map_err(Error::Cs)?;
        
//         if let Some(ref mut reset_pin) = nreset {
//             reset_pin.set_high().map_err(Error::Reset)?;
//             delay.delay_ms(1);
//             reset_pin.set_low().map_err(Error::Reset)?;
//             delay.delay_ms(40);
//             reset_pin.set_high().map_err(Error::Reset)?;
//             delay.delay_ms(100);
//         }

//         let mut driver = Self { spi, cs, nreset, delay };
        
//         // Software reset after hardware reset for a clean slate.
//         info!("Performing software reset...");
//         driver.write_reg_16(registers::RESET_CTRL, 1 << 1)?; // Set soft_reset bit
//         driver.delay.delay_ms(100);


//         Ok(driver)
//     }

//     // CORRECTED: The IIM-20670 uses a 32-bit SPI protocol.
//     // [RW(1) | ADDR(5) | RS(2) | DATA(16) | CRC(8)]
//     // This helper function constructs the 32-bit word for a write.
//     fn write_reg_16(&mut self, reg: u8, data: u16) -> Result<(), Error<SPIE, CSE, RESETE>> {
//         let _guard = CsGuard::new(&mut self.cs).map_err(Error::Cs)?;
        
//         // Construct the 24-bit command part (RW=1, ADDR, RS=00, DATA)
//         let command_part = (1u32 << 31) | ((reg as u32) << 26) | (data as u32);
//         let crc = Self::calculate_crc(command_part >> 8); // CRC is on the first 24 bits
        
//         let tx_word = command_part | crc as u32;
//         let tx_bytes = tx_word.to_be_bytes();

//         self.spi.write(&tx_bytes).map_err(Error::Spi)
//     }

//     // CORRECTED: This helper function handles a 32-bit read.
//     fn read_reg_16(&mut self, reg: u8) -> Result<u16, Error<SPIE, CSE, RESETE>> {
//         let _guard = CsGuard::new(&mut self.cs).map_err(Error::Cs)?;

//         // Construct the 24-bit command part for a read (RW=0, ADDR, RS=00, DATA=0)
//         let command_part = (reg as u32) << 26;
//         let crc = Self::calculate_crc(command_part >> 8);

//         let tx_word = command_part | crc as u32;
//         let mut buffer = tx_word.to_be_bytes();
        
//         self.spi.transfer_in_place(&mut buffer).map_err(Error::Spi)?;
        
//         // Extract 16-bit data from the response
//         let data = u16::from_be_bytes([buffer[2], buffer[3]]);
//         Ok(data)
//     }

//     fn calculate_crc(data_to_encode: u32) -> u8 {
//         let mut crc: u8 = 0xFF;
//         for i in (0..24).rev() {
//             let mut crc_new = 0u8;
//             let bit = (data_to_encode >> i) & 1;
//             let crc7 = (crc >> 7) & 1;

//             crc_new |= ((crc >> 6) & 1) << 7;
//             crc_new |= ((crc >> 5) & 1) << 6;
//             crc_new |= ((crc >> 4) & 1) << 5;
//             crc_new |= (((crc >> 3) & 1) ^ crc7) << 4;
//             crc_new |= (((crc >> 2) & 1) ^ crc7) << 3;
//             crc_new |= (((crc >> 1) & 1) ^ crc7) << 2;
//             crc_new |= (crc & 1) << 1;
//             crc_new |= (bit as u8) ^ crc7;
            
//             crc = crc_new;
//         }
//         !crc // Final inversion
//     }

//     // CORRECTED: Reads from the correct, separate registers for each gyro axis.
//     pub fn read_gyro(&mut self) -> Result<[i16; 3], Error<SPIE, CSE, RESETE>> {
//         let x = self.read_reg_16(registers::GYRO_X_DATA)? as i16;
//         let y = self.read_reg_16(registers::GYRO_Y_DATA)? as i16;
//         let z = self.read_reg_16(registers::GYRO_Z_DATA)? as i16;
//         Ok([x, y, z])
//     }

//     // CORRECTED: Reads from the correct, separate registers for each accel axis.
//     pub fn read_accel(&mut self) -> Result<[i16; 3], Error<SPIE, CSE, RESETE>> {
//         let x = self.read_reg_16(registers::ACCEL_X_DATA)? as i16;
//         let y = self.read_reg_16(registers::ACCEL_Y_DATA)? as i16;
//         let z = self.read_reg_16(registers::ACCEL_Z_DATA)? as i16;
//         Ok([x, y, z])
//     }
// }
//! Blocking SPI driver for the TDK IIM-20670 IMU with unit conversions.

#![no_std]

use defmt::info;
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
    pub const RESET_CTRL: u8 = 0x18;
    pub const MODE_CTRL: u8 = 0x19;
    pub const BANK_SELECT: u8 = 0x1F;

    // Bank 6 & 7 (for sensitivity config)
    pub const SENSITIVITY_CONFIG: u8 = 0x14;
}

#[repr(u16)]
enum Bank {
    Bank0 = 0x0000,
    Bank6 = 0x0006,
    Bank7 = 0x0007,
}

#[repr(u16)]
#[derive(Clone, Copy, Debug)]
pub enum GyroFsr {
    Dps1966 = 0b0011, // Corresponds to FS_SEL = 0011 for +/- 1966 dps
}

impl GyroFsr {
    fn as_f32(&self) -> f32 {
        match self {
            GyroFsr::Dps1966 => 1966.0,
        }
    }
}

#[repr(u16)]
#[derive(Clone, Copy, Debug)]
pub enum AccelFsr {
    G16 = 0b001, // Corresponds to FS_SEL = 001, which gives +/-16.384g
}

impl AccelFsr {
    fn as_f32(&self) -> f32 {
        match self {
            AccelFsr::G16 => 16.384,
        }
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
        cs.set_high().map_err(Error::Cs)?;
        
        if let Some(ref mut reset_pin) = nreset {
            reset_pin.set_high().map_err(Error::Reset)?;
            delay.delay_ms(1);
            reset_pin.set_low().map_err(Error::Reset)?;
            delay.delay_ms(40);
            reset_pin.set_high().map_err(Error::Reset)?;
            delay.delay_ms(100);
        }

        let mut driver = Self {
            spi, cs, nreset, delay,
            accel_fsr: AccelFsr::G16,
            gyro_fsr: GyroFsr::Dps1966,
        };
        
        driver.write_reg_16(registers::RESET_CTRL, 1 << 1)?;
        driver.delay.delay_ms(100);

        info!("Unlocking bank switching...");
        driver.unlock_banks()?;
        driver.delay.delay_ms(10);

        info!("Unlocking Full-Scale Range registers...");
        driver.unlock_fsr()?;
        driver.delay.delay_ms(10);

        info!("Configuring sensor ranges...");
        driver.set_accel_fsr(AccelFsr::G16)?;
        driver.set_gyro_fsr(GyroFsr::Dps1966)?;

        Ok(driver)
    }

    fn spi_transaction(&mut self, reg: u8, data: u16, is_write: bool) -> Result<u16, Error<SPIE, CSE, RESETE>> {
        let _guard = CsGuard::new(&mut self.cs).map_err(Error::Cs)?;
        
        let rw_bit = if is_write { 1u32 } else { 0u32 };
        // *** FIX: Shift data by 8 bits to place it in the correct position [23:8] ***
        let command_part = (rw_bit << 31) | ((reg as u32) << 26) | ((data as u32) << 8);
        let crc = Self::calculate_crc(command_part >> 8);
        let tx_word = command_part | crc as u32;
        let mut buffer = tx_word.to_be_bytes();
        
        self.spi.transfer_in_place(&mut buffer).map_err(Error::Spi)?;
        
        let response_word = u32::from_be_bytes(buffer);
        let status = ((response_word >> 24) & 0b11) as u8;

        if status != 1 {
            return Err(Error::ImuError(status));
        }

        Ok(((response_word >> 8) & 0xFFFF) as u16)
    }

    fn send_raw_command(&mut self, command: u32) -> Result<(), Error<SPIE, CSE, RESETE>> {
        let _guard = CsGuard::new(&mut self.cs).map_err(Error::Cs)?;
        let tx_bytes = command.to_be_bytes();
        self.spi.write(&tx_bytes).map_err(Error::Spi)
    }

    fn unlock_banks(&mut self) -> Result<(), Error<SPIE, CSE, RESETE>> {
        self.write_reg_16(registers::MODE_CTRL, 0b010)?;
        self.write_reg_16(registers::MODE_CTRL, 0b001)?;
        self.write_reg_16(registers::MODE_CTRL, 0b100)?;
        Ok(())
    }

    fn unlock_fsr(&mut self) -> Result<(), Error<SPIE, CSE, RESETE>> {
        self.send_raw_command(0xE4000288)?;
        self.send_raw_command(0xE400018B)?;
        self.send_raw_command(0xE400048E)?;
        self.send_raw_command(0xE40300AD)?;
        self.send_raw_command(0xE4018017)?;
        self.send_raw_command(0xE4028030)?;
        Ok(())
    }

    fn write_reg_16(&mut self, reg: u8, data: u16) -> Result<(), Error<SPIE, CSE, RESETE>> {
        self.spi_transaction(reg, data, true)?;
        Ok(())
    }

    fn read_reg_16(&mut self, reg: u8) -> Result<u16, Error<SPIE, CSE, RESETE>> {
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
        self.write_reg_16(registers::BANK_SELECT, bank as u16)
    }

    pub fn set_gyro_fsr(&mut self, fsr: GyroFsr) -> Result<(), Error<SPIE, CSE, RESETE>> {
        self.select_bank(Bank::Bank7)?;
        self.write_reg_16(registers::SENSITIVITY_CONFIG, fsr as u16)?;
        self.gyro_fsr = fsr;
        self.select_bank(Bank::Bank0)
    }

    pub fn set_accel_fsr(&mut self, fsr: AccelFsr) -> Result<(), Error<SPIE, CSE, RESETE>> {
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
    
    pub fn read_all_converted(&mut self) -> Result<(Acceleration, AngularRate), Error<SPIE, CSE, RESETE>> {
        let accel = self.read_accel_g()?;
        let gyro = self.read_gyro_dps()?;
        Ok((accel, gyro))
    }
}
