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

use neotron_common_bios::audio::Config;

/// Errors you can get from this module
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum Error {
	VolumeTooHigh,
}

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
	Clock = 1 << 6,
	Oscillator = 1 << 5,
	Outputs = 1 << 4,
	DigitalAnalogConverter = 1 << 3,
	AnalogDigitalConverter = 1 << 2,
	MicrophoneInput = 1 << 1,
	LineInput = 1 << 0,
}

/// Represents the state inside our CODEC chip.
pub struct Codec {
	bus_address: u8,
	register_cache: [u16; NUM_REGISTERS],
}

/// A volume in decibels.
#[derive(Debug, Copy, Clone, PartialEq, PartialOrd)]
pub struct Volume(f32);

/// Describes the possible sample rates. Note that not all sample rates are available for all MCLK frequencies.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum SampleRate {
	/// 8,000 Hz
	R8k,
	/// 8,021 Hz
	R8k021,
	/// 32,000 Hz
	R32k,
	/// 44,100 Hz
	R44k1,
	/// 48,000 Hz
	R48k,
	/// 88,200 Hz
	R88k2,
	/// 96,000 Hz
	R96k,
}

/// Describes the possible MCLK frequencies
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum Mclk {
	/// USB mode - 12.0000 MHz clock input
	Usb,
	/// Normal mode - 12.2880 MHz Crystal
	Xtal12288000,
	/// Normal mode - 11.2896 MHz Crystal
	Xtal11289600,
	/// Normal mode - 18.4320 MHz Crystal
	Xtal18432000,
	/// Normal mode - 16.9344 MHz Crystal
	Xtal16934400,
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
	/// CODEC generates the BCLK and LRCLK signals. The documentation uses the archaic term 'Master'.
	Main,
	/// CODEC receives the BCLK and LRCLK signals. The documentation uses the archaic term 'Slave.
	Secondary,
}

/// The size, in bits, of each sample
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum WordLength {
	B16 = 0b00,
	B20 = 0b01,
	B24 = 0b10,
	B32 = 0b11,
}

/// How the data is sent over the digital bus
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum DataFormat {
	/// DSP format (frame sync then two data words)
	Dsp = 0b11,
	/// I²S Format (i.e. Most Significant Bit first, `left-1` aligned)
	I2s = 0b10,
	/// Most Significant Bit first, left-aligned
	MsbFirstLeftAligned = 0b01,
	/// Most Significant Bit first, right-aligned
	MsbFirstRightAligned = 0b00,
}

/// If this is enabled, right-channel audio comes out of the left channel, and vice-versa.
///
/// `LeftRightSwap::Disabled` is the default.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum LeftRightSwap {
	Disabled = 0,
	Enabled = 1,
}

/// Set this according to how your companion digital audio device expects the left and right audio to be aligned relative to the LRCLK.
///
/// `LeftRightPhase::RightOnLrcHigh` is the default.
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum LeftRightPhase {
	RightOnLrcHigh = 0,
	RightOnLrcLow = 1,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum AnalogInput {
	/// Stereo line-input selected. Don't forget to call
	/// [Codec::set_line_input_mute], [Codec::set_line_input_volume]  and
	/// [Codec::set_bypass] as required.
	Line = 0,
	/// Mono microphone input selected. Don't forget to call
	/// [Codec::set_microphone_mute], [Codec::set_microphone_boost] and
	/// [Codec::set_sidetone_volume] as required.
	Microphone = 1,
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

/// Configuration for USB Mode (12 MHz), ADC @ 96 kHz, DAC @ 96 kHz
pub static CONFIG_USB_96K: ConfigParams = ConfigParams {
	mclk: Mclk::Usb,
	adc: SampleRate::R96k,
	dac: SampleRate::R96k,
	src: 0b0111,
	bosr: 0,
};

/// Configuration for USB Mode (12 MHz), ADC @ 88.2 kHz, DAC @ 88.2 kHz
pub static CONFIG_USB_88K2: ConfigParams = ConfigParams {
	mclk: Mclk::Usb,
	adc: SampleRate::R88k2,
	dac: SampleRate::R88k2,
	src: 0b1111,
	bosr: 1,
};

/// Configuration for USB Mode (12 MHz), ADC @ 48 kHz, DAC @ 48 kHz
pub static CONFIG_USB_48K: ConfigParams = ConfigParams {
	mclk: Mclk::Usb,
	adc: SampleRate::R48k,
	dac: SampleRate::R48k,
	src: 0b0000,
	bosr: 0,
};

/// Configuration for USB Mode (12 MHz), ADC @ 44.1 kHz, DAC @ 44.1 kHz
pub static CONFIG_USB_44K1: ConfigParams = ConfigParams {
	mclk: Mclk::Usb,
	adc: SampleRate::R44k1,
	dac: SampleRate::R44k1,
	src: 0b1000,
	bosr: 1,
};

/// Configuration for USB Mode (12 MHz), ADC @ 32 kHz, DAC @ 32 kHz
pub static CONFIG_USB_32K: ConfigParams = ConfigParams {
	mclk: Mclk::Usb,
	adc: SampleRate::R32k,
	dac: SampleRate::R32k,
	src: 0b0110,
	bosr: 0,
};

/// Configuration for USB Mode (12 MHz), ADC @ 8.021 kHz, DAC @ 8.021 kHz
pub static CONFIG_USB_8K021: ConfigParams = ConfigParams {
	mclk: Mclk::Usb,
	adc: SampleRate::R8k021,
	dac: SampleRate::R8k021,
	src: 0b1011,
	bosr: 1,
};

/// Configuration for USB Mode (12 MHz), ADC @ 8 kHz, DAC @ 8 kHz
pub static CONFIG_USB_8K: ConfigParams = ConfigParams {
	mclk: Mclk::Usb,
	adc: SampleRate::R8k,
	dac: SampleRate::R8k,
	src: 0b0001,
	bosr: 0,
};

/// Configuration for USB Mode (12 MHz), ADC @ 44.1 kHz, DAC @ 8.021 kHz
pub static CONFIG_USB_44K1_8K021: ConfigParams = ConfigParams {
	mclk: Mclk::Usb,
	adc: SampleRate::R44k1,
	dac: SampleRate::R8k021,
	src: 0b1001,
	bosr: 1,
};

/// Configuration for USB Mode (12 MHz), ADC @ 8 kHz, DAC @ 48 kHz
pub static CONFIG_USB_8K_48K: ConfigParams = ConfigParams {
	mclk: Mclk::Usb,
	adc: SampleRate::R8k,
	dac: SampleRate::R48k,
	src: 0b0010,
	bosr: 0,
};

/// Configuration for USB Mode (12 MHz), ADC @ 8.021 kHz, DAC @ 44.1 kHz
pub static CONFIG_USB_8K021_44K1: ConfigParams = ConfigParams {
	mclk: Mclk::Usb,
	adc: SampleRate::R8k021,
	dac: SampleRate::R44k1,
	src: 0b1010,
	bosr: 1,
};

/// Configuration for 12.2880 MHz Crystal, ADC @ 96 kHz, DAC @ 96 kHz
pub static CONFIG_XTAL12288000_96K: ConfigParams = ConfigParams {
	mclk: Mclk::Xtal12288000,
	adc: SampleRate::R96k,
	dac: SampleRate::R96k,
	src: 0b0111,
	bosr: 0,
};

/// Configuration for 12.2880 MHz Crystal, ADC @ 48 kHz, DAC @ 48 kHz
pub static CONFIG_XTAL12288000_48K: ConfigParams = ConfigParams {
	mclk: Mclk::Xtal12288000,
	adc: SampleRate::R48k,
	dac: SampleRate::R48k,
	src: 0b0000,
	bosr: 0,
};

/// Configuration for 12.2880 MHz Crystal, ADC @ 32 kHz, DAC @ 32 kHz
pub static CONFIG_XTAL12288000_32K: ConfigParams = ConfigParams {
	mclk: Mclk::Xtal12288000,
	adc: SampleRate::R32k,
	dac: SampleRate::R32k,
	src: 0b0110,
	bosr: 0,
};

/// Configuration for 12.2880 MHz Crystal, ADC @ 8 kHz, DAC @ 8 kHz
pub static CONFIG_XTAL12288000_8K: ConfigParams = ConfigParams {
	mclk: Mclk::Xtal12288000,
	adc: SampleRate::R8k,
	dac: SampleRate::R8k,
	src: 0b0011,
	bosr: 0,
};

/// Configuration for 12.2880 MHz Crystal, ADC @ 48 kHz, DAC @ 8 kHz
pub static CONFIG_XTAL12288000_48K_8K: ConfigParams = ConfigParams {
	mclk: Mclk::Xtal12288000,
	adc: SampleRate::R48k,
	dac: SampleRate::R8k,
	src: 0b0001,
	bosr: 0,
};

/// Configuration for 12.2880 MHz Crystal, ADC @ 8 kHz, DAC @ 48 kHz
pub static CONFIG_XTAL12288000_8K_48K: ConfigParams = ConfigParams {
	mclk: Mclk::Xtal12288000,
	adc: SampleRate::R8k,
	dac: SampleRate::R48k,
	src: 0b0010,
	bosr: 0,
};

/// Configuration for 11.2896 MHz Crystal, ADC @ 88.2 kHz, DAC @ 88.2 kHz
pub static CONFIG_XTAL11289600_88K2: ConfigParams = ConfigParams {
	mclk: Mclk::Xtal11289600,
	adc: SampleRate::R88k2,
	dac: SampleRate::R88k2,
	src: 0b1111,
	bosr: 0,
};

/// Configuration for 11.2896 MHz Crystal, ADC @ 44.1 kHz, DAC @ 44.1 kHz
pub static CONFIG_XTAL11289600_44K1: ConfigParams = ConfigParams {
	mclk: Mclk::Xtal11289600,
	adc: SampleRate::R44k1,
	dac: SampleRate::R44k1,
	src: 0b1000,
	bosr: 0,
};

/// Configuration for 11.2896 MHz Crystal, ADC @ 8.021 kHz, DAC @ 8.021 kHz
pub static CONFIG_XTAL11289600_8K021: ConfigParams = ConfigParams {
	mclk: Mclk::Xtal11289600,
	adc: SampleRate::R8k021,
	dac: SampleRate::R8k021,
	src: 0b1011,
	bosr: 0,
};

/// Configuration for 11.2896 MHz Crystal, ADC @ 44.1 kHz, DAC @ 8.021 kHz
pub static CONFIG_XTAL11289600_44K1_8K021: ConfigParams = ConfigParams {
	mclk: Mclk::Xtal11289600,
	adc: SampleRate::R44k1,
	dac: SampleRate::R8k021,
	src: 0b1001,
	bosr: 0,
};

/// Configuration for 11.2896 MHz Crystal, ADC @ 8.021 kHz, DAC @ 44.1 kHz
pub static CONFIG_XTAL11289600_8K021_44K1: ConfigParams = ConfigParams {
	mclk: Mclk::Xtal11289600,
	adc: SampleRate::R8k021,
	dac: SampleRate::R44k1,
	src: 0b1010,
	bosr: 0,
};

/// Configuration for 18.4320 MHz Crystal, ADC @ 96 kHz, DAC @ 96 kHz
pub static CONFIG_XTAL18432000_96K: ConfigParams = ConfigParams {
	mclk: Mclk::Xtal18432000,
	adc: SampleRate::R96k,
	dac: SampleRate::R96k,
	src: 0b0111,
	bosr: 1,
};

/// Configuration for 18.4320 MHz Crystal, ADC @ 48 kHz, DAC @ 48 kHz
pub static CONFIG_XTAL18432000_48K: ConfigParams = ConfigParams {
	mclk: Mclk::Xtal18432000,
	adc: SampleRate::R48k,
	dac: SampleRate::R48k,
	src: 0b0000,
	bosr: 1,
};

/// Configuration for 18.4320 MHz Crystal, ADC @ 32 kHz, DAC @ 32 kHz
pub static CONFIG_XTAL18432000_32K: ConfigParams = ConfigParams {
	mclk: Mclk::Xtal18432000,
	adc: SampleRate::R32k,
	dac: SampleRate::R32k,
	src: 0b0110,
	bosr: 1,
};

/// Configuration for 18.4320 MHz Crystal, ADC @ 8 kHz, DAC @ 8 kHz
pub static CONFIG_XTAL18432000_8K: ConfigParams = ConfigParams {
	mclk: Mclk::Xtal18432000,
	adc: SampleRate::R8k,
	dac: SampleRate::R8k,
	src: 0b0011,
	bosr: 1,
};

/// Configuration for 18.4320 MHz Crystal, ADC @ 48 kHz, DAC @ 8 kHz
pub static CONFIG_XTAL18432000_48K_8K: ConfigParams = ConfigParams {
	mclk: Mclk::Xtal18432000,
	adc: SampleRate::R48k,
	dac: SampleRate::R8k,
	src: 0b0001,
	bosr: 1,
};

/// Configuration for 18.4320 MHz Crystal, ADC @ 8 kHz, DAC @ 48 kHz
pub static CONFIG_XTAL18432000_8K_48K: ConfigParams = ConfigParams {
	mclk: Mclk::Xtal18432000,
	adc: SampleRate::R8k,
	dac: SampleRate::R48k,
	src: 0b0010,
	bosr: 1,
};

/// Configuration for 16.9344 MHz Crystal, ADC @ 88.2 kHz, DAC @ 88.2 kHz
pub static CONFIG_XTAL16934400_88K2: ConfigParams = ConfigParams {
	mclk: Mclk::Xtal16934400,
	adc: SampleRate::R88k2,
	dac: SampleRate::R88k2,
	src: 0b1111,
	bosr: 1,
};

/// Configuration for 16.9344 MHz Crystal, ADC @ 44.1 kHz, DAC @ 44.1 kHz
pub static CONFIG_XTAL16934400_44K1: ConfigParams = ConfigParams {
	mclk: Mclk::Xtal16934400,
	adc: SampleRate::R44k1,
	dac: SampleRate::R44k1,
	src: 0b1000,
	bosr: 1,
};

/// Configuration for 16.9344 MHz Crystal, ADC @ 8.021 kHz, DAC @ 8.021 kHz
pub static CONFIG_XTAL16934400_8K021: ConfigParams = ConfigParams {
	mclk: Mclk::Xtal16934400,
	adc: SampleRate::R8k021,
	dac: SampleRate::R8k021,
	src: 0b1011,
	bosr: 1,
};

/// Configuration for 16.9344 MHz Crystal, ADC @ 44.1 kHz, DAC @ 8.021 kHz
pub static CONFIG_XTAL16934400_44K1_8K021: ConfigParams = ConfigParams {
	mclk: Mclk::Xtal16934400,
	adc: SampleRate::R44k1,
	dac: SampleRate::R8k021,
	src: 0b1001,
	bosr: 1,
};

/// Configuration for 16.9344 MHz Crystal, ADC @ 8.021 kHz, DAC @ 44.1 kHz
pub static CONFIG_XTAL16934400_8K021_44K1: ConfigParams = ConfigParams {
	mclk: Mclk::Xtal16934400,
	adc: SampleRate::R8k021,
	dac: SampleRate::R44k1,
	src: 0b1010,
	bosr: 1,
};

//
// Private Data
//

const NUM_REGISTERS: usize = 10;

//
// impls on Public Types
//

impl Volume {
	/// Loudest possible Line In setting
	pub const LINE_IN_MAX: f32 = 12.0;
	/// Quietest possible Line In setting
	pub const LINE_IN_MIN: f32 = -34.5;

	/// Loudest possible Headphone Out setting
	pub const HP_OUT_MAX: f32 = 6.0;
	/// Quietest possible Headphone Out setting
	pub const HP_OUT_MIN: f32 = -73.0;

	/// Loudest possible sidetone setting
	pub const SIDETONE_MAX: f32 = 0.0;

	/// Quietest possible sidetone setting
	pub const SIDETONE_MIN: f32 = -18.0;

	/// Create a new volume from a value in dB.
	pub fn new(db: f32) -> Volume {
		Volume(db)
	}

	/// Convert a volume (in decibels) into a value the CODEC understands for line-input volume
	fn to_line(&self) -> Result<u16, Error> {
		if self.0 > Self::LINE_IN_MAX {
			Err(Error::VolumeTooHigh)
		} else if self.0 < Self::LINE_IN_MIN {
			Ok(0)
		} else {
			let steps: i16 = (self.0 / 1.5) as i16;
			// 0b10111 is 0.0dB
			let level = (steps + 0b10111) as u16;
			Ok(level)
		}
	}

	/// Convert line-input step value from the CODEC into a volume (in decibels)
	fn from_line(&self, steps: u8) -> Volume {
		let steps = steps & 0b11111;
		Volume((steps as f32 * 1.5) - 34.5)
	}

	/// Convert a volume (in decibels) into a value the CODEC understands for sidetone
	///
	/// The steps are:
	///
	/// * Disabled
	/// * -18 dB
	/// * -12 dB
	/// * -9 dB
	/// * -6 dB
	/// * 0 dB
	///
	/// The input value is rounded down.
	fn to_sidetone(&self) -> Result<u16, Error> {
		if self.0 >= Self::SIDETONE_MAX {
			// Bits are: STA2 STA1 STA0 STE
			Ok(0b100_1)
		} else if self.0 >= -6.0 {
			Ok(0b000_1)
		} else if self.0 >= -9.0 {
			Ok(0b001_1)
		} else if self.0 >= -12.0 {
			Ok(0b010_1)
		} else if self.0 >= -18.0 {
			Ok(0b011_1)
		} else {
			Ok(0b000_0)
		}
	}

	/// Convert a volume (in decibels) into a value the CODEC understands for headphone volume
	fn to_headphone(&self) -> Result<u16, Error> {
		if self.0 > Self::HP_OUT_MAX {
			Err(Error::VolumeTooHigh)
		} else if self.0 < Self::HP_OUT_MIN {
			// Lowest possible volume for headphones out
			Ok(0b0110000)
		} else {
			let steps: i16 = self.0 as i16;
			// 0b1111001 is 0.0dB
			let level = (steps + 0b1111001) as u16;
			Ok(level)
		}
	}
}

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
			register_cache: [0; NUM_REGISTERS],
		}
	}

	/// Resets internal register cache to CODEC defaults, as per the datasheet.
	fn set_register_defaults(&mut self) {
		// LeftLineInputChannelVolumeControl - Left Input muted
		self.register_cache[Register::LeftLineInputChannelVolumeControl as usize] = 0b0_1_00_10111;
		// RightLineInputChannelVolumeControl - Right Input muted
		self.register_cache[Register::RightLineInputChannelVolumeControl as usize] = 0b0_1_00_10111;
		// LeftChannelHeadphoneVolumeControl - Left Output 0dB, zero-cross enabled
		self.register_cache[Register::LeftChannelHeadphoneVolumeControl as usize] = 0b0_1_1111001;
		// RightChannelHeadphoneVolumeControl - Right Output 0dB, zero-cross enabled
		self.register_cache[Register::RightChannelHeadphoneVolumeControl as usize] = 0b0_1_1111001;
		// AnalogAudioPathControl - No sidetone, DAC off, bypass on, line in selected, mic muted
		self.register_cache[Register::AnalogAudioPathControl as usize] = 0b0_0000_1010;
		// DigitalAudioPathControl - DAC soft mute, de-emphasis disabled, ADC high-pass filter on
		self.register_cache[Register::DigitalAudioPathControl as usize] = 0b0_0000_1000;
		// PowerDownControl - Line In, Mic and ADC all off
		self.register_cache[Register::PowerDownControl as usize] = 0b0_0000_0111;
		// DigitalAudioInterfaceFormat - MSB-first/left-aligned, 16-bit, lrc-high=right, lr-swap off, slave mode
		self.register_cache[Register::DigitalAudioInterfaceFormat as usize] = 0b0_0000_0001;
		// SampleRateControl - Normal mode, 256fs, no clock divider, 44.1 kHz in/out
		self.register_cache[Register::SampleRateControl as usize] = 0b0_0010_0000;
		// DigitalInterfaceActivation - Digital interface disabled
		self.register_cache[Register::DigitalInterfaceActivation as usize] = 0b0_0000_0000;
	}

	/// Update one of the internal registers
	fn set_register_bits(&mut self, register: Register, value: u16, mask: u16) {
		// Clear the bits we want to change
		self.register_cache[register as usize] &= !mask;
		// Set any bits as necessary, but only in the cleared section
		self.register_cache[register as usize] |= value & mask;
	}

	/// Read back one of the internal registers
	///
	/// Reads from the cache because the CODEC is write-only.
	fn get_register_bits(&mut self, register: Register, mask: u16) -> u16 {
		// Get the bits, after selecting only those bits in the mask
		self.register_cache[register as usize] & mask
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
	pub fn set_line_input_volume<T>(&mut self, volume: T, channel: Channel) -> Result<(), Error>
	where
		T: core::convert::Into<Volume>,
	{
		let volume = volume.into();
		let steps = volume.to_line()?;
		const MASK: u16 = 0b11111;
		if channel == Channel::Left || channel == Channel::Both {
			self.set_register_bits(Register::LeftLineInputChannelVolumeControl, steps, MASK);
		}
		if channel == Channel::Right || channel == Channel::Both {
			self.set_register_bits(Register::RightLineInputChannelVolumeControl, steps, MASK);
		}
		Ok(())
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
	pub fn set_headphone_output_volume<T>(
		&mut self,
		volume: T,
		channel: Channel,
	) -> Result<(), Error>
	where
		T: core::convert::Into<Volume>,
	{
		let volume = volume.into();
		let steps = volume.to_headphone()?;
		const MASK: u16 = 0b1111111;
		if channel == Channel::Left || channel == Channel::Both {
			self.set_register_bits(Register::LeftChannelHeadphoneVolumeControl, steps, MASK);
		}
		if channel == Channel::Right || channel == Channel::Both {
			self.set_register_bits(Register::RightChannelHeadphoneVolumeControl, steps, MASK);
		}
		Ok(())
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

	/// Set sidetone volume.
	///
	/// Sidetone is where the microphone-in signal is added to the output
	/// signal, so you can hear yourself speak. This provides audible
	/// feedback that your words are being picked up by the microphone, and
	/// is often used in telephony applications.
	///
	/// Call [Codec::sync] to have this change take effect.
	pub fn set_sidetone_volume<T>(&mut self, volume: T, channel: Channel) -> Result<(), Error>
	where
		T: core::convert::Into<Volume>,
	{
		let volume = volume.into();
		let steps = volume.to_sidetone()? << 5;
		const MASK: u16 = 0b1111 << 5;
		if channel == Channel::Left || channel == Channel::Both {
			self.set_register_bits(Register::AnalogAudioPathControl, steps, MASK);
		}
		if channel == Channel::Right || channel == Channel::Both {
			self.set_register_bits(Register::AnalogAudioPathControl, steps, MASK);
		}
		Ok(())
	}

	/// Select which analog input gets digitised.
	///
	/// There is only one ADC, but you can pick which analog input it is connected to.
	///
	/// Call [Codec::sync] to have this change take effect.
	pub fn set_analog_input(&mut self, analog_input: AnalogInput) {
		const MASK: u16 = 1 << 2;
		self.set_register_bits(
			Register::AnalogAudioPathControl,
			(analog_input as u16) << 2,
			MASK,
		);
	}

	/// Enable the `+20dB` microphone input boost.
	///
	/// Call [Codec::sync] to have this change take effect.
	pub fn set_microphone_boost(&mut self, boost_enabled: bool) {
		const MASK: u16 = 1 << 0;
		self.set_register_bits(
			Register::AnalogAudioPathControl,
			if boost_enabled { MASK } else { 0 },
			MASK,
		);
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

	/// Mute the microphone input.
	///
	/// Call [Codec::sync] to have this change take effect.
	pub fn set_microphone_mute(&mut self, mute_enabled: bool) {
		const MASK: u16 = 1 << 1;
		self.set_register_bits(
			Register::AnalogAudioPathControl,
			if mute_enabled { MASK } else { 0 },
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
	pub fn set_sample_rate(&mut self, params: &ConfigParams, mode: Mode, word_length: WordLength) {
		let usb_mode = if params.mclk == Mclk::Usb { 1 } else { 0 };
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
	pub fn sync<B>(&self, bus: &mut B) -> Result<(), B::Error>
	where
		B: embedded_hal::blocking::i2c::Write,
	{
		for (address, register_value) in self.register_cache.iter().enumerate() {
			let byte1 = (address << 1) as u8 | ((register_value >> 8) & 1) as u8;
			let byte2 = (register_value & 0xFF) as u8;
			let buffer = [byte1, byte2];
			bus.write(self.bus_address, &buffer)?;
		}
		Ok(())
	}
}

//
// impls on Private Types
//

// None

//
// Tests
//

#[cfg(test)]
mod test {
	use super::*;

	#[test]
	fn line_volume_from_db_ott() {
		let v = Volume::new(13.5);
		// See datasheet page 3-2, section 3.1.3
		assert_eq!(Err(Error::VolumeTooHigh), v.to_line());
	}

	#[test]
	fn line_volume_from_db_max() {
		let v = Volume::new(12.0);
		// See datasheet page 3-2, section 3.1.3
		assert_eq!(Ok(0b11111), v.to_line());
	}

	#[test]
	fn line_volume_from_db_min() {
		let v = Volume::new(-34.5);
		// See datasheet page 3-2, section 3.1.3
		assert_eq!(Ok(0b00000), v.to_line());
	}

	#[test]
	fn line_volume_from_db_zero() {
		let v = Volume::new(0.0);
		// See datasheet page 3-2, section 3.1.3
		assert_eq!(Ok(0b10111), v.to_line());
	}

	#[test]
	fn hp_volume_from_db_ott() {
		let v = Volume::new(7.0);
		// See datasheet page 3-3, section 3.1.3
		assert_eq!(Err(Error::VolumeTooHigh), v.to_headphone());
	}

	#[test]
	fn hp_volume_from_db_max() {
		let v = Volume::new(6.0);
		// See datasheet page 3-3, section 3.1.3
		assert_eq!(Ok(0b1111111), v.to_headphone());
	}

	#[test]
	fn hp_volume_from_db_min() {
		let v = Volume::new(-73.0);
		// See datasheet page 3-3, section 3.1.3
		assert_eq!(Ok(0b0110000), v.to_headphone());
	}

	#[test]
	fn hp_volume_from_db_zero() {
		let v = Volume::new(0.0);
		// See datasheet page 3-3, section 3.1.3
		assert_eq!(Ok(0b1111001), v.to_headphone());
	}
}

//
// End of file
//
