//! SD Card support

use atomic_polyfill::{AtomicBool, Ordering};

use super::Hardware;

pub(crate) struct FakeSpi<'a>(pub(crate) &'a mut Hardware, pub bool);

pub(crate) struct FakeCs();

pub(crate) struct FakeDelayer();

/// How fast can the SD Card SPI CLK input go?
const CLOCK_SD_CARD: fugit::Rate<u32, 1, 1> = fugit::Rate::<u32, 1, 1>::Hz(10_000_000);

/// How fast can the SD Card SPI CLK input go during the card init phase?
const CLOCK_SD_CARD_INIT: fugit::Rate<u32, 1, 1> = fugit::Rate::<u32, 1, 1>::Hz(400_000);

static IS_CS_LOW: AtomicBool = AtomicBool::new(false);

impl<'a> embedded_hal::blocking::spi::Transfer<u8> for FakeSpi<'a> {
	type Error = core::convert::Infallible;
	fn transfer<'w>(&mut self, words: &'w mut [u8]) -> Result<&'w [u8], Self::Error> {
		let result = if IS_CS_LOW.load(Ordering::SeqCst) {
			defmt::debug!("SD > {:02x}", words);
			self.0.with_bus_cs(
				1,
				if self.1 {
					CLOCK_SD_CARD_INIT
				} else {
					CLOCK_SD_CARD
				},
				|spi, _buffer| {
					spi.transfer(words).unwrap();
				},
			);
			defmt::debug!("SD < {:02x}", words);
			words
		} else {
			// Select a slot we don't use so the SD card won't be activated
			self.0.with_bus_cs(7, CLOCK_SD_CARD_INIT, |spi, _buffer| {
				spi.transfer(words).unwrap();
			});
			words
		};
		Ok(result)
	}
}

impl<'a> embedded_hal::blocking::spi::Write<u8> for FakeSpi<'a> {
	type Error = core::convert::Infallible;
	fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
		if IS_CS_LOW.load(Ordering::SeqCst) {
			defmt::debug!("SD > {:02x}", words);
			self.0.with_bus_cs(
				1,
				if self.1 {
					CLOCK_SD_CARD_INIT
				} else {
					CLOCK_SD_CARD
				},
				|spi, _buffer| {
					spi.write(words).unwrap();
				},
			);
		} else {
			// Select a slot we don't use so the SD card won't be activated
			self.0.with_bus_cs(7, CLOCK_SD_CARD_INIT, |spi, _buffer| {
				spi.write(words).unwrap();
			});
		}
		Ok(())
	}
}

impl embedded_hal::digital::v2::OutputPin for FakeCs {
	type Error = core::convert::Infallible;
	fn set_low(&mut self) -> Result<(), Self::Error> {
		IS_CS_LOW.store(true, Ordering::SeqCst);
		Ok(())
	}

	fn set_high(&mut self) -> Result<(), Self::Error> {
		IS_CS_LOW.store(true, Ordering::SeqCst);
		Ok(())
	}
}

impl embedded_hal::blocking::delay::DelayUs<u8> for FakeDelayer {
	fn delay_us(&mut self, us: u8) {
		// It doesn't have to be accurate. Just within an order of magnitude is fine.
		cortex_m::asm::delay(u32::from(us) * 150)
	}
}
