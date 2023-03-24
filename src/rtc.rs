//! RTC support for the Neotron Pico BIOS
//!
//! We could have one of two RTCs fitted. This module abstracts that away.

pub use ds1307::{DateTimeAccess, NaiveDateTime};

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
	pub fn get_time<T, E>(&self, bus: T) -> Result<NaiveDateTime, ()>
	where
		T: embedded_hal::blocking::i2c::WriteRead<Error = E>
			+ embedded_hal::blocking::i2c::Write<Error = E>,
	{
		match self {
			Self::Ds1307 => {
				let mut driver = ds1307::Ds1307::new(bus);
				driver.datetime().map_err(|_e| ())
			}
			Self::Mcp7940n => {
				let mut driver = mcp794xx::Mcp794xx::new_mcp7940n(bus);
				driver.datetime().map_err(|_e| ())
			}
			Self::None => Err(()),
		}
	}
}
