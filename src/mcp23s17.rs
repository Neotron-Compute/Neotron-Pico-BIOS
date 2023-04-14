//! MCP23S17 Control Functions

use embedded_hal::blocking::spi::{Transfer, Write};

use super::SpiBus;

/// The Registers on the MCP23S17
///
/// Assumes the device is in `BANK = 0` mode.
#[derive(defmt::Format, Debug, Copy, Clone, PartialEq, Eq)]
#[repr(u8)]
pub enum Register {
	/// Data Direction Register A
	DDRA = 0x00,
	/// GPIO Input Polarity Register B
	IPOLB = 0x03,
	/// Interrupt-on-Change Control Register B
	GPINTENB = 0x05,
	/// Interrupt Control Register B
	DEFVALB = 0x07,
	/// Interrupt Control Register B
	INTCONB = 0x09,
	/// GPIO Pull-Up Register B
	GPPUB = 0x0D,
	/// GPIO Data Register A
	GPIOA = 0x12,
	/// GPIO Data Register B
	GPIOB = 0x13,
}

/// The Control Byte prefix
const CONTROL_PREFIX: u8 = 0b0100;

/// The value for Device Address 000
const ADDRESS_0: u8 = 0;

/// Perform a write on the MCP23S17
///
/// Is of the format `0 1 0 0 A2 A1 A0 R/W`
const WRITE_COMMAND: u8 = (CONTROL_PREFIX << 4) | (ADDRESS_0 << 1) | 0;

/// Perform a write on the MCP23S17
///
/// Is of the format `0 1 0 0 A2 A1 A0 R/W`
const READ_COMMAND: u8 = (CONTROL_PREFIX << 4) | (ADDRESS_0 << 1) | 1;

/// Write to a register on the MCP23S17.
///
/// Assumes the Chip Select has been asserted already.
pub fn write_register(spi: &mut SpiBus, register: Register, value: u8) {
	spi.write(&[WRITE_COMMAND, register as u8, value]).unwrap()
}

/// Write to a register on the MCP23S17.
///
/// Assumes the Chip Select has been asserted already.
pub fn read_register(spi: &mut SpiBus, register: Register) -> u8 {
	// Starts with outbound, is replaced with inbound
	let mut buffer = [READ_COMMAND, register as u8, 0x00];
	spi.transfer(&mut buffer).unwrap();
	buffer[2]
}
