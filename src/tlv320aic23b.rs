//! # TLV230AIC23B Driver
//!
//! This is driver for the Texas Instruments TLV320AIC23B audio CODEC / amplifier.
//!
//! Specifically, this driver is for setting the registers in the CODEC over I²C - this driver
//! does not handle the digital audio interface (I²S, or similar).
//!
//! The CODEC has the following inputs and outputs:
//!
//! * Stereo analog Line-level Input
//! * Mono analog Microphone Input, with optional +20dB boost
//! * Stereo analog Line-level Output
//! * Stereo analog Amplified Headphone Output
//! * Stereo digital Output
//! * Stereo digital Input

//
// Public Types
//

/// The CODEC has one of two I²C addresses, depending on whether the CS pin is
/// pulled high or low.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum BusAddress {
	/// The address when the CS pin is high
	CsHigh = 0x1B,
	/// The address when the CS pin is low
	CsLow = 0x1A,
}

/// Selects either only the left channel, only the right channel, or both
/// channels together.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum Channel {
	/// Just the left channel
	Left,
	/// Just the right channel
	Right,
	/// Both channels
	Both,
}

/// Represents parts of the CODEC that we can turn on and off
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum Subsystem {
	AnalogDigitalConverter = 1 << 2,
	MicrophoneInput = 1 << 1,
	LineInput = 1 << 0,
}

/// Represents the state inside our CODEC chip.
pub struct Codec {
	bus_address: u8,
	register_cache: [(bool, u16); NUM_REGISTERS],
}

/// Describes the possible sample rates. Note that not all sample rates are available for all MCLK frequencies.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum SampleRate {
	/// 48,000 Hz
	R48k = 4,
}

/// Describes the possible MCLK frequencies
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum Mclk {
	/// USB mode - 12.0000 MHz clock input
	Usb = 0,
}

/// Describes a specific configuration in terms of MCLK, ADC Sample Rate, and DAC Sample Rate
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ConfigParams {
	adc: SampleRate,
	dac: SampleRate,
	mclk: Mclk,
	src: u8,
	bosr: u8,
}

/// Whether the CODEC generates or receives clock signals.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum Mode {
	/// CODEC receives the BCLK and LRCLK signals. The documentation uses the archaic term 'Slave'.
	Secondary = 0,
}

/// The size, in bits, of each sample
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum WordLength {
	B16 = 0b00,
}

/// How the data is sent over the digital bus
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum DataFormat {
	/// I²S Format (i.e. Most Significant Bit first, `left-1` aligned)
	I2s = 0b10,
}

//
// Private Types
//

/// The set of registers in the CODEC
#[derive(Copy, Clone, Debug)]
enum Register {
	LeftLineInputChannelVolumeControl = 0,
	RightLineInputChannelVolumeControl = 1,
	LeftChannelHeadphoneVolumeControl = 2,
	RightChannelHeadphoneVolumeControl = 3,
	AnalogAudioPathControl = 4,
	DigitalAudioPathControl = 5,
	PowerDownControl = 6,
	DigitalAudioInterfaceFormat = 7,
	SampleRateControl = 8,
	DigitalInterfaceActivation = 9,
	Reset = 15,
}

//
// Public Data
//

/// Configuration for USB Mode (12 MHz), ADC @ 48 kHz, DAC @ 48 kHz
pub static CONFIG_USB_48K: ConfigParams = ConfigParams {
	mclk: Mclk::Usb,
	adc: SampleRate::R48k,
	dac: SampleRate::R48k,
	src: 0b0000,
	bosr: 0,
};

//
// Private Data
//

const NUM_REGISTERS: usize = 10;

//
// impls on Public Types
//

impl From<BusAddress> for u8 {
	fn from(addr: BusAddress) -> u8 {
		addr as u8
	}
}

impl Codec {
	/// Create a new CODEC proxy object.
	///
	/// You can call methods on this object to set various parameters on the
	/// CODEC. However, they won't take effect until you call the [Codec::sync]
	/// method.
	pub fn new(bus_address: BusAddress) -> Codec {
		Codec {
			bus_address: bus_address.into(),
			register_cache: [(false, 0); NUM_REGISTERS],
		}
	}

	/// Resets internal register cache to CODEC defaults, as per the datasheet.
	fn set_register_defaults(&mut self) {
		// LeftLineInputChannelVolumeControl - Left Input muted
		self.register_cache[Register::LeftLineInputChannelVolumeControl as usize] =
			(false, 0b0_1_00_10111);
		// RightLineInputChannelVolumeControl - Right Input muted
		self.register_cache[Register::RightLineInputChannelVolumeControl as usize] =
			(false, 0b0_1_00_10111);
		// LeftChannelHeadphoneVolumeControl - Left Output 0dB, zero-cross enabled
		self.register_cache[Register::LeftChannelHeadphoneVolumeControl as usize] =
			(false, 0b0_1_1111001);
		// RightChannelHeadphoneVolumeControl - Right Output 0dB, zero-cross enabled
		self.register_cache[Register::RightChannelHeadphoneVolumeControl as usize] =
			(false, 0b0_1_1111001);
		// AnalogAudioPathControl - No sidetone, DAC off, bypass on, line in selected, mic muted
		self.register_cache[Register::AnalogAudioPathControl as usize] = (false, 0b0_0000_1010);
		// DigitalAudioPathControl - DAC soft mute, de-emphasis disabled, ADC high-pass filter on
		self.register_cache[Register::DigitalAudioPathControl as usize] = (false, 0b0_0000_1000);
		// PowerDownControl - Line In, Mic and ADC all off
		self.register_cache[Register::PowerDownControl as usize] = (false, 0b0_0000_0111);
		// DigitalAudioInterfaceFormat - MSB-first/left-aligned, 16-bit, lrc-high=right, lr-swap off, slave mode
		self.register_cache[Register::DigitalAudioInterfaceFormat as usize] =
			(false, 0b0_0000_0001);
		// SampleRateControl - Normal mode, 256fs, no clock divider, 44.1 kHz in/out
		self.register_cache[Register::SampleRateControl as usize] = (false, 0b0_0010_0000);
		// DigitalInterfaceActivation - Digital interface disabled
		self.register_cache[Register::DigitalInterfaceActivation as usize] = (false, 0b0_0000_0000);
	}

	/// Update one of the internal registers
	fn set_register_bits(&mut self, register: Register, value: u16, mask: u16) {
		// Clear the bits we want to change
		self.register_cache[register as usize].1 &= !mask;
		// Set any bits as necessary, but only in the cleared section
		self.register_cache[register as usize].1 |= value & mask;
		// Mark as dirty
		self.register_cache[register as usize].0 = true;
	}

	/// Read back one of the internal registers
	///
	/// Reads from the cache because the CODEC is write-only.
	fn get_register_bits(&mut self, register: Register, mask: u16) -> u16 {
		// Get the bits, after selecting only those bits in the mask
		self.register_cache[register as usize].1 & mask
	}

	/// Set whether line-input is muted
	///
	/// Call [Codec::sync] to have this change take effect.
	pub fn set_line_input_mute(&mut self, mute: bool, channel: Channel) {
		const MUTE_BIT: u16 = 1 << 7;
		let value = if mute { MUTE_BIT } else { 0 };
		if channel == Channel::Left || channel == Channel::Both {
			self.set_register_bits(Register::LeftLineInputChannelVolumeControl, value, MUTE_BIT);
		}
		if channel == Channel::Right || channel == Channel::Both {
			self.set_register_bits(
				Register::RightLineInputChannelVolumeControl,
				value,
				MUTE_BIT,
			);
		}
	}

	/// Set line-input volume
	///
	/// Call [Codec::sync] to have this change take effect.
	pub fn set_line_input_volume_steps(&mut self, steps: u8, channel: Channel) {
		const MASK: u16 = 0b11111;
		if channel == Channel::Left || channel == Channel::Both {
			self.set_register_bits(
				Register::LeftLineInputChannelVolumeControl,
				steps as u16,
				MASK,
			);
		}
		if channel == Channel::Right || channel == Channel::Both {
			self.set_register_bits(
				Register::RightLineInputChannelVolumeControl,
				steps as u16,
				MASK,
			);
		}
	}

	/// Get line-input volume in integer steps, for both channels.
	///
	/// Goes from 0 (which is -34.5 dB) to 31 (which is +12 dB).
	///
	/// You get 0 if the channel is muted, regardless of the value in the register.
	pub fn get_line_input_volume_steps(&mut self) -> (u8, u8) {
		let steps_left =
			self.get_register_bits(Register::LeftLineInputChannelVolumeControl, 0b0_0001_1111);
		let steps_right =
			self.get_register_bits(Register::RightLineInputChannelVolumeControl, 0b0_0001_1111);
		let mute_left =
			self.get_register_bits(Register::LeftLineInputChannelVolumeControl, 0b0_1000_0000);
		let mute_right =
			self.get_register_bits(Register::RightLineInputChannelVolumeControl, 0b0_1000_0000);
		(
			if mute_left == 0 { steps_left as u8 } else { 0 },
			if mute_right == 0 {
				steps_right as u8
			} else {
				0
			},
		)
	}

	/// Set headphone output volume
	///
	/// Call [Codec::sync] to have this change take effect.
	pub fn set_headphone_output_volume_steps(&mut self, steps: u8, channel: Channel) {
		const MASK: u16 = 0b1111111;
		if channel == Channel::Left || channel == Channel::Both {
			self.set_register_bits(
				Register::LeftChannelHeadphoneVolumeControl,
				steps as u16,
				MASK,
			);
		}
		if channel == Channel::Right || channel == Channel::Both {
			self.set_register_bits(
				Register::RightChannelHeadphoneVolumeControl,
				steps as u16,
				MASK,
			);
		}
	}

	/// Get line-input volume in integer steps, for both channels.
	///
	/// Goes from 48 (which is -73 dB) to 127 (which is +6 dB). Anything below 48 is 'muted.
	pub fn get_headphone_output_volume_steps(&mut self) -> (u8, u8) {
		const MASK: u16 = 0b1111111;
		let steps_left =
			self.get_register_bits(Register::LeftChannelHeadphoneVolumeControl, MASK) as u8;
		let steps_right =
			self.get_register_bits(Register::RightChannelHeadphoneVolumeControl, MASK) as u8;
		(steps_left, steps_right)
	}

	/// Enable or disable the DAC.
	///
	/// Call [Codec::sync] to have this change take effect.
	pub fn set_dac_enable(&mut self, dac_enabled: bool) {
		const MASK: u16 = 1 << 4;
		self.set_register_bits(
			Register::AnalogAudioPathControl,
			if dac_enabled { MASK } else { 0 },
			MASK,
		);
	}

	/// Mute or unmute the DAC.
	///
	/// Call [Codec::sync] to have this change take effect.
	pub fn set_dac_mute(&mut self, dac_muted: bool) {
		const MASK: u16 = 1 << 3;
		self.set_register_bits(
			Register::DigitalAudioPathControl,
			if dac_muted { MASK } else { 0 },
			MASK,
		);
	}

	/// Enable bypass mode.
	///
	/// In bypass mode, the line input is routed to the line output, bypassing the ADC and DAC.
	///
	/// If you want to silence the line output, you need bypass to be off.
	///
	/// Call [Codec::sync] to have this change take effect.
	pub fn set_bypass(&mut self, bypass_enabled: bool) {
		const MASK: u16 = 1 << 3;
		self.set_register_bits(
			Register::AnalogAudioPathControl,
			if bypass_enabled { MASK } else { 0 },
			MASK,
		);
	}

	/// Control which parts of the CODEC are powered on.
	///
	/// Call [Codec::sync] to have this change take effect.
	pub fn set_powered_on(&mut self, subsystem: Subsystem, enabled: bool) {
		// 0 bit means on, 1 bit means off (it's the power *down* control)
		self.set_register_bits(
			Register::PowerDownControl,
			if enabled { 0 } else { 0xFFFF },
			subsystem as u16,
		);
	}

	/// Configure the CODEC for a specific DAC and ADC sample rate, given a specific MCLK frequency.
	///
	/// Call [Codec::sync] to have this change take effect.
	pub fn set_sample_rate(
		&mut self,
		params: &ConfigParams,
		mode: Mode,
		word_length: WordLength,
		format: DataFormat,
	) {
		let usb_mode = if params.mclk == Mclk::Usb { 1 } else { 0 };
		self.set_register_bits(
			Register::DigitalAudioInterfaceFormat,
			((mode as u16) << 6) | ((word_length as u16) << 2) | (format as u16),
			0b00_1_0_0_11_11,
		);
		self.set_register_bits(
			Register::SampleRateControl,
			(params.src as u16) << 2 | (params.bosr as u16) << 1 | usb_mode,
			0b000_1111_1_1,
		);
	}

	/// Turn the I²S interface on or off.
	///
	/// Call [Codec::sync] to have this change take effect.
	pub fn set_digital_interface_enabled(&mut self, enabled: bool) {
		// 1 bit means on, 0 bit means off
		self.set_register_bits(
			Register::DigitalInterfaceActivation,
			if enabled { 0xFFFF } else { 0 },
			0b0_0000_0001,
		);
	}

	/// Resets the CODEC and puts all the registers back to defaults
	pub fn reset<B>(&mut self, bus: &mut B) -> Result<(), B::Error>
	where
		B: embedded_hal::blocking::i2c::Write,
	{
		let byte1 = (Register::Reset as u8) << 1;
		let byte2 = 0;
		let buffer = [byte1, byte2];
		bus.write(self.bus_address, &buffer)?;
		self.set_register_defaults();
		Ok(())
	}

	/// Transfer all registers from this proxy object to the actual chip, over I²C.
	pub fn sync<B>(&mut self, bus: &mut B) -> Result<(), B::Error>
	where
		B: embedded_hal::blocking::i2c::Write,
	{
		for (address, (dirty, register_value)) in self.register_cache.iter_mut().enumerate() {
			if *dirty {
				let byte1 = (address << 1) as u8 | ((*register_value >> 8) & 1) as u8;
				let byte2 = (*register_value & 0xFF) as u8;
				let buffer = [byte1, byte2];
				defmt::debug!(
					"Setting CODEC 0x{:02x} to 0x{:03x}",
					address,
					register_value
				);
				bus.write(self.bus_address, &buffer)?;
				*dirty = false;
			}
		}
		Ok(())
	}
}

//
// impls on Private Types
//

// None

//
// End of file
//
