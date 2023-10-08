//! RTC support for the Neotron Pico BIOS
//!
//! We could have one of two RTCs fitted. This module abstracts that away.

// -----------------------------------------------------------------------------
// Licence Statement
// -----------------------------------------------------------------------------
// Copyright (c) Jonathan 'theJPster' Pallant and the Neotron Developers, 2023
//
// This program is free software: you can redistribute it and/or modify it under
// the terms of the GNU General Public License as published by the Free Software
// Foundation, either version 3 of the License, or (at your option) any later
// version.
//
// This program is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
// details.
//
// You should have received a copy of the GNU General Public License along with
// this program.  If not, see <https://www.gnu.org/licenses/>.
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
// Sub-modules
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
// Imports
// -----------------------------------------------------------------------------

pub use ds1307::{DateTimeAccess, NaiveDateTime};

// -----------------------------------------------------------------------------
// Types
// -----------------------------------------------------------------------------

/// The ways this module can fail
pub enum Error<E> {
	Bus(E),
	DriverBug,
	NoRtcFound,
}

/// The kinds of RTC we support.
pub enum Rtc {
	Ds1307,
	Mcp7940n,
	None,
}

impl Rtc {
	/// Create a new RTC object.
	///
	/// The `bus` is dropped at the end of the function, so use a `&mut`
	/// reference, or a disposable shared-bus proxy.
	pub fn new<T, E>(bus: T) -> Rtc
	where
		T: embedded_hal::blocking::i2c::WriteRead<Error = E>
			+ embedded_hal::blocking::i2c::Write<Error = E>,
	{
		let mut mcp7940n = mcp794xx::Mcp794xx::new_mcp7940n(bus);

		match mcp7940n.is_oscillator_running() {
			Ok(true) => {
				defmt::info!("MCP7940N found and ticking");
				return Rtc::Mcp7940n;
			}
			Ok(false) => {
				defmt::info!("MCP7940N found, and has started ticking");
				let _ = mcp7940n.enable_external_oscillator();
				let _ = mcp7940n.enable();
				return Rtc::Mcp7940n;
			}
			Err(_e) => defmt::info!("MCP7940N not found"),
		}

		let bus = mcp7940n.destroy();

		let mut ds1307 = ds1307::Ds1307::new(bus);
		match ds1307.running() {
			Ok(true) => {
				defmt::info!("DS1307 found and ticking");
				return Rtc::Ds1307;
			}
			Ok(false) => {
				defmt::info!("DS1307 found, and has started ticking");
				let _ = ds1307.set_running();
				return Rtc::Ds1307;
			}
			Err(_e) => defmt::info!("DS1307 not found"),
		}

		Rtc::None
	}

	/// Get the current wall-time.
	///
	/// We don't care about timezones - this is basic Gregorian naive wall-clock date/time.
	///
	/// You get `Err()` if the RTC doesn't respond or wasn't found on start-up.
	pub fn get_time<T, E>(&mut self, bus: T) -> Result<NaiveDateTime, Error<E>>
	where
		T: embedded_hal::blocking::i2c::WriteRead<Error = E>
			+ embedded_hal::blocking::i2c::Write<Error = E>,
	{
		match self {
			Self::Ds1307 => {
				let mut driver = ds1307::Ds1307::new(bus);
				driver.datetime().map_err(|e| match e {
					ds1307::Error::I2C(bus_error) => Error::Bus(bus_error),
					ds1307::Error::InvalidInputData => Error::DriverBug,
				})
			}
			Self::Mcp7940n => {
				let mut driver = mcp794xx::Mcp794xx::new_mcp7940n(bus);
				driver.datetime().map_err(|e| match e {
					mcp794xx::Error::Comm(bus_error) => Error::Bus(bus_error),
					mcp794xx::Error::InvalidInputData => Error::DriverBug,
				})
			}
			Self::None => Err(Error::NoRtcFound),
		}
	}

	/// Set the current wall-time
	///
	/// We don't care about timezones - this is basic Gregorian naive wall-clock date/time.
	///
	/// You get `Err()` if the RTC doesn't respond or wasn't found on start-up.
	pub fn set_time<T, E>(&mut self, bus: T, new_time: NaiveDateTime) -> Result<(), Error<E>>
	where
		T: embedded_hal::blocking::i2c::WriteRead<Error = E>
			+ embedded_hal::blocking::i2c::Write<Error = E>,
	{
		match self {
			Self::Ds1307 => {
				let mut driver = ds1307::Ds1307::new(bus);
				driver.set_datetime(&new_time).map_err(|e| match e {
					ds1307::Error::I2C(bus_error) => Error::Bus(bus_error),
					ds1307::Error::InvalidInputData => Error::DriverBug,
				})
			}
			Self::Mcp7940n => {
				let mut driver = mcp794xx::Mcp794xx::new_mcp7940n(bus);
				driver.set_datetime(&new_time).map_err(|e| match e {
					mcp794xx::Error::Comm(bus_error) => Error::Bus(bus_error),
					mcp794xx::Error::InvalidInputData => Error::DriverBug,
				})
			}
			Self::None => Err(Error::NoRtcFound),
		}
	}

	/// Store configuration in the RTC's RAM
	pub fn configuration_set<T, E>(&mut self, bus: T, data: &[u8]) -> Result<(), Error<E>>
	where
		T: embedded_hal::blocking::i2c::WriteRead<Error = E>
			+ embedded_hal::blocking::i2c::Write<Error = E>,
	{
		match self {
			Self::Ds1307 => {
				let mut driver = ds1307::Ds1307::new(bus);
				driver.write_ram(1, data).map_err(|e| match e {
					ds1307::Error::I2C(bus_error) => Error::Bus(bus_error),
					ds1307::Error::InvalidInputData => Error::DriverBug,
				})?;
				driver
					.write_ram(0, &[data.len() as u8])
					.map_err(|e| match e {
						ds1307::Error::I2C(bus_error) => Error::Bus(bus_error),
						ds1307::Error::InvalidInputData => Error::DriverBug,
					})
			}
			Self::Mcp7940n => {
				let mut driver = mcp794xx::Mcp794xx::new_mcp7940n(bus);
				driver.write_sram_data(1, data).map_err(|e| match e {
					mcp794xx::Error::Comm(bus_error) => Error::Bus(bus_error),
					mcp794xx::Error::InvalidInputData => Error::DriverBug,
				})?;
				driver
					.write_sram_byte(0, data.len() as u8)
					.map_err(|e| match e {
						mcp794xx::Error::Comm(bus_error) => Error::Bus(bus_error),
						mcp794xx::Error::InvalidInputData => Error::DriverBug,
					})
			}
			Self::None => Err(Error::NoRtcFound),
		}
	}

	/// Get configuration from the RTC's RAM
	pub fn configuration_get<T, E>(&mut self, bus: T, mut data: &mut [u8]) -> Result<u8, Error<E>>
	where
		T: embedded_hal::blocking::i2c::WriteRead<Error = E>
			+ embedded_hal::blocking::i2c::Write<Error = E>,
	{
		match self {
			Self::Ds1307 => {
				let mut driver = ds1307::Ds1307::new(bus);
				if data.len() > 55 {
					// This chip can only read 56 bytes (inc our count byte)
					data = &mut data[0..55];
				}
				let mut count = [0u8; 1];
				driver.read_ram(0, &mut count).map_err(|e| match e {
					ds1307::Error::I2C(bus_error) => Error::Bus(bus_error),
					ds1307::Error::InvalidInputData => Error::DriverBug,
				})?;
				driver.read_ram(1, data).map_err(|e| match e {
					ds1307::Error::I2C(bus_error) => Error::Bus(bus_error),
					ds1307::Error::InvalidInputData => Error::DriverBug,
				})?;
				if usize::from(count[0]) <= data.len() {
					Ok(count[0])
				} else {
					Ok(0)
				}
			}
			Self::Mcp7940n => {
				if data.len() > 63 {
					// This chip can only read 64 bytes (inc our count byte)
					data = &mut data[0..63];
				}
				let mut driver = mcp794xx::Mcp794xx::new_mcp7940n(bus);
				let count = driver.read_sram_byte(0).map_err(|e| match e {
					mcp794xx::Error::Comm(bus_error) => Error::Bus(bus_error),
					mcp794xx::Error::InvalidInputData => Error::DriverBug,
				})?;
				driver.read_sram_data(1, data).map_err(|e| match e {
					mcp794xx::Error::Comm(bus_error) => Error::Bus(bus_error),
					mcp794xx::Error::InvalidInputData => Error::DriverBug,
				})?;
				if usize::from(count) <= data.len() {
					Ok(count)
				} else {
					Ok(0)
				}
			}
			Self::None => Err(Error::NoRtcFound),
		}
	}
}

// -----------------------------------------------------------------------------
// Static and Const Data
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
// Functions
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
// End of file
// -----------------------------------------------------------------------------
