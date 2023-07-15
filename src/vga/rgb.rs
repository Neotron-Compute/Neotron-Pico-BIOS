//! Code for handling RGB colours.

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

// -----------------------------------------------------------------------------
// Types
// -----------------------------------------------------------------------------

/// Represents two `RGBColour` pixels packed together.
///
/// The `first` pixel is packed in the lower 16-bits. This is because the PIO
/// shifts-right.
#[repr(transparent)]
#[derive(Copy, Clone, PartialEq, Eq, Default)]
pub struct RGBPair(u32);

impl RGBPair {
	pub const fn new() -> RGBPair {
		RGBPair(0)
	}

	pub const fn from_pixels(first: RGBColour, second: RGBColour) -> RGBPair {
		let first: u32 = first.0 as u32;
		let second: u32 = second.0 as u32;
		RGBPair((second << 16) | first)
	}
}

/// Represents a 12-bit colour value.
///
/// Each channel has four-bits, and they are packed in `BGR` format. This is
/// so the PIO can shift them out right-first, and we have RED0 assigned to
/// the lowest GPIO pin.
#[repr(transparent)]
#[derive(Copy, Clone, PartialEq, Eq)]
pub struct RGBColour(pub u16);

impl RGBColour {
	/// Black (all bits off)
	pub(crate) const BLACK: RGBColour = RGBColour::from_24bit(0x00, 0x00, 0x00);

	/// White
	pub(crate) const WHITE: RGBColour = RGBColour::from_24bit(0xF0, 0xF0, 0xF0);

	/// Make an [`RGBColour`] from a 24-bit RGB triplet.
	///
	/// Only the top 4 bits of each colour channel are retained, as RGB colour
	/// is a 12-bit value.
	pub const fn from_24bit(red: u8, green: u8, blue: u8) -> RGBColour {
		let red4: u16 = ((red >> 4) & 0x00F) as u16;
		let green4: u16 = ((green >> 4) & 0x00F) as u16;
		let blue4: u16 = ((blue >> 4) & 0x00F) as u16;
		RGBColour((blue4 << 8) | (green4 << 4) | red4)
	}

	/// Make an [`RGBColour`] from a 12-bit RGB triplet.
	///
	/// Only the bottom 4 bits of each colour channel are retained, as RGB colour
	/// is a 12-bit value.
	pub const fn from_12bit(red: u8, green: u8, blue: u8) -> RGBColour {
		let red4: u16 = (red & 0x00F) as u16;
		let green4: u16 = (green & 0x00F) as u16;
		let blue4: u16 = (blue & 0x00F) as u16;
		RGBColour((blue4 << 8) | (green4 << 4) | red4)
	}

	/// Get the red component as an 8-bit value
	pub const fn red8(self) -> u8 {
		let red4 = self.0 & 0x0F;
		(red4 << 4) as u8
	}

	/// Get the green component as an 8-bit value
	pub const fn green8(self) -> u8 {
		let green4 = (self.0 >> 4) & 0x0F;
		(green4 << 4) as u8
	}

	/// Get the blue component as an 8-bit value
	pub const fn blue8(self) -> u8 {
		let blue4 = (self.0 >> 8) & 0x0F;
		(blue4 << 4) as u8
	}
}

impl From<RGBColour> for crate::common::video::RGBColour {
	fn from(val: RGBColour) -> crate::common::video::RGBColour {
		let red = val.red8();
		let green = val.green8();
		let blue = val.blue8();
		crate::common::video::RGBColour::from_rgb(red, green, blue)
	}
}

impl From<crate::common::video::RGBColour> for RGBColour {
	fn from(val: crate::common::video::RGBColour) -> RGBColour {
		let red = val.red();
		let green = val.green();
		let blue = val.blue();
		RGBColour::from_24bit(red, green, blue)
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
