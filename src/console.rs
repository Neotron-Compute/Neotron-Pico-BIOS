//! Code for handling a text-buffer.

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

use core::sync::atomic::{AtomicPtr, AtomicU16, AtomicU8, Ordering};

use neotron_common_bios::video::{
	Attr, Glyph, GlyphAttr, TextBackgroundColour, TextForegroundColour,
};

// -----------------------------------------------------------------------------
// Types
// -----------------------------------------------------------------------------

/// Holds some data necessary to present a text console.
///
/// Used by Core 0 to control writes to a shared text-buffer.
pub struct TextConsole {
	current_col: AtomicU16,
	current_row: AtomicU16,
	text_buffer: AtomicPtr<GlyphAttr>,
	current_attr: AtomicU8,
}

impl TextConsole {
	/// Create a TextConsole.
	///
	/// Has no buffer associated with it
	pub const fn new() -> TextConsole {
		TextConsole {
			current_row: AtomicU16::new(0),
			current_col: AtomicU16::new(0),
			text_buffer: AtomicPtr::new(core::ptr::null_mut()),
			// White on Black, with the default palette
			current_attr: AtomicU8::new(
				Attr::new(
					TextForegroundColour::WHITE,
					TextBackgroundColour::BLACK,
					false,
				)
				.0,
			),
		}
	}

	/// Update the text buffer we are using.
	///
	/// Will reset the cursor. The screen is not cleared.
	pub fn set_text_buffer(
		&self,
		text_buffer: &'static mut [GlyphAttr;
			             crate::vga::MAX_TEXT_ROWS * crate::vga::MAX_TEXT_COLS],
	) {
		self.text_buffer
			.store(text_buffer.as_mut_ptr(), Ordering::Relaxed)
	}

	/// Place a single Code Page 850 encoded 8-bit character on the screen.
	///
	/// Adjusts the current row and column automatically. Also understands
	/// Carriage Return and New Line bytes.
	pub fn write_font_glyph(&self, glyph: Glyph) {
		// Load from global state
		let mut row = self.current_row.load(Ordering::Relaxed);
		let mut col = self.current_col.load(Ordering::Relaxed);
		let buffer = self.text_buffer.load(Ordering::Relaxed);

		if !buffer.is_null() {
			self.write_at(glyph, buffer, &mut row, &mut col);
			// Push back to global state
			self.current_row.store(row, Ordering::Relaxed);
			self.current_col.store(col, Ordering::Relaxed);
		}
	}

	/// Moves the text cursor to the specified row and column.
	///
	/// If a value is out of bounds, the cursor is not moved in that axis.
	pub fn move_to(&self, row: u16, col: u16) {
		let mode = crate::vga::VIDEO_MODE.get_mode();
		let num_rows = mode.text_height().unwrap_or(0);
		let num_cols = mode.text_width().unwrap_or(0);
		if row < num_rows {
			self.current_row.store(row, Ordering::Relaxed);
		}
		if col < num_cols {
			self.current_col.store(col, Ordering::Relaxed);
		}
	}

	/// Convert a Unicode Scalar Value to a font glyph.
	///
	/// Zero-width and modifier Unicode Scalar Values (e.g. `U+0301 COMBINING,
	/// ACCENT`) are not supported. Normalise your Unicode before calling
	/// this function.
	fn map_char_to_glyph(input: char) -> Glyph {
		// This fixed table only works for the default font. When we support
		// changing font, we will need to plug-in a different table for each font.
		let index = match input {
			'\u{0000}'..='\u{007F}' => input as u8,
			'\u{00A0}' => 255, // NBSP
			'\u{00A1}' => 173, // ¡
			'\u{00A2}' => 189, // ¢
			'\u{00A3}' => 156, // £
			'\u{00A4}' => 207, // ¤
			'\u{00A5}' => 190, // ¥
			'\u{00A6}' => 221, // ¦
			'\u{00A7}' => 245, // §
			'\u{00A8}' => 249, // ¨
			'\u{00A9}' => 184, // ©
			'\u{00AA}' => 166, // ª
			'\u{00AB}' => 174, // «
			'\u{00AC}' => 170, // ¬
			'\u{00AD}' => 240, // SHY
			'\u{00AE}' => 169, // ®
			'\u{00AF}' => 238, // ¯
			'\u{00B0}' => 248, // °
			'\u{00B1}' => 241, // ±
			'\u{00B2}' => 253, // ²
			'\u{00B3}' => 252, // ³
			'\u{00B4}' => 239, // ´
			'\u{00B5}' => 230, // µ
			'\u{00B6}' => 244, // ¶
			'\u{00B7}' => 250, // ·
			'\u{00B8}' => 247, // ¸
			'\u{00B9}' => 251, // ¹
			'\u{00BA}' => 167, // º
			'\u{00BB}' => 175, // »
			'\u{00BC}' => 172, // ¼
			'\u{00BD}' => 171, // ½
			'\u{00BE}' => 243, // ¾
			'\u{00BF}' => 168, // ¿
			'\u{00C0}' => 183, // À
			'\u{00C1}' => 181, // Á
			'\u{00C2}' => 182, // Â
			'\u{00C3}' => 199, // Ã
			'\u{00C4}' => 142, // Ä
			'\u{00C5}' => 143, // Å
			'\u{00C6}' => 146, // Æ
			'\u{00C7}' => 128, // Ç
			'\u{00C8}' => 212, // È
			'\u{00C9}' => 144, // É
			'\u{00CA}' => 210, // Ê
			'\u{00CB}' => 211, // Ë
			'\u{00CC}' => 222, // Ì
			'\u{00CD}' => 214, // Í
			'\u{00CE}' => 215, // Î
			'\u{00CF}' => 216, // Ï
			'\u{00D0}' => 209, // Ð
			'\u{00D1}' => 165, // Ñ
			'\u{00D2}' => 227, // Ò
			'\u{00D3}' => 224, // Ó
			'\u{00D4}' => 226, // Ô
			'\u{00D5}' => 229, // Õ
			'\u{00D6}' => 153, // Ö
			'\u{00D7}' => 158, // ×
			'\u{00D8}' => 157, // Ø
			'\u{00D9}' => 235, // Ù
			'\u{00DA}' => 233, // Ú
			'\u{00DB}' => 234, // Û
			'\u{00DC}' => 154, // Ü
			'\u{00DD}' => 237, // Ý
			'\u{00DE}' => 232, // Þ
			'\u{00DF}' => 225, // ß
			'\u{00E0}' => 133, // à
			'\u{00E1}' => 160, // á
			'\u{00E2}' => 131, // â
			'\u{00E3}' => 198, // ã
			'\u{00E4}' => 132, // ä
			'\u{00E5}' => 134, // å
			'\u{00E6}' => 145, // æ
			'\u{00E7}' => 135, // ç
			'\u{00E8}' => 138, // è
			'\u{00E9}' => 130, // é
			'\u{00EA}' => 136, // ê
			'\u{00EB}' => 137, // ë
			'\u{00EC}' => 141, // ì
			'\u{00ED}' => 161, // í
			'\u{00EE}' => 140, // î
			'\u{00EF}' => 139, // ï
			'\u{00F0}' => 208, // ð
			'\u{00F1}' => 164, // ñ
			'\u{00F2}' => 149, // ò
			'\u{00F3}' => 162, // ó
			'\u{00F4}' => 147, // ô
			'\u{00F5}' => 228, // õ
			'\u{00F6}' => 148, // ö
			'\u{00F7}' => 246, // ÷
			'\u{00F8}' => 155, // ø
			'\u{00F9}' => 151, // ù
			'\u{00FA}' => 163, // ú
			'\u{00FB}' => 150, // û
			'\u{00FC}' => 129, // ü
			'\u{00FD}' => 236, // ý
			'\u{00FE}' => 231, // þ
			'\u{00FF}' => 152, // ÿ
			'\u{0131}' => 213, // ı
			'\u{0192}' => 159, // ƒ
			'\u{2017}' => 242, // ‗
			'\u{2500}' => 196, // ─
			'\u{2502}' => 179, // │
			'\u{250C}' => 218, // ┌
			'\u{2510}' => 191, // ┐
			'\u{2514}' => 192, // └
			'\u{2518}' => 217, // ┘
			'\u{251C}' => 195, // ├
			'\u{2524}' => 180, // ┤
			'\u{252C}' => 194, // ┬
			'\u{2534}' => 193, // ┴
			'\u{253C}' => 197, // ┼
			'\u{2550}' => 205, // ═
			'\u{2551}' => 186, // ║
			'\u{2554}' => 201, // ╔
			'\u{2557}' => 187, // ╗
			'\u{255A}' => 200, // ╚
			'\u{255D}' => 188, // ╝
			'\u{2560}' => 204, // ╠
			'\u{2563}' => 185, // ╣
			'\u{2566}' => 203, // ╦
			'\u{2569}' => 202, // ╩
			'\u{256C}' => 206, // ╬
			'\u{2580}' => 223, // ▀
			'\u{2584}' => 220, // ▄
			'\u{2588}' => 219, // █
			'\u{2591}' => 176, // ░
			'\u{2592}' => 177, // ▒
			'\u{2593}' => 178, // ▓
			'\u{25A0}' => 254, // ■
			_ => b'?',
		};
		Glyph(index)
	}

	/// Put a single character at a specified point on screen.
	///
	/// The character is relative to the current font, but newline and carriage
	/// return will be interpreted appropriately. The given `row` and `col` are
	/// updated.
	fn write_at(&self, glyph: Glyph, buffer: *mut GlyphAttr, row: &mut u16, col: &mut u16) {
		let mode = crate::vga::VIDEO_MODE.get_mode();
		let num_rows = mode.text_height().unwrap_or(0) as usize;
		let num_cols = mode.text_width().unwrap_or(0) as usize;
		let attr = Attr(self.current_attr.load(Ordering::Relaxed));

		if glyph.0 == b'\r' {
			*col = 0;
		} else if glyph.0 == b'\n' {
			*col = 0;
			*row += 1;
		} else {
			let offset = (*col as usize) + (num_cols * (*row as usize));
			// Note (safety): This is safe as we bound `col` and `row`
			unsafe {
				buffer
					.add(offset)
					.write_volatile(GlyphAttr::new(glyph, attr))
			};
			*col += 1;
		}
		if *col == (num_cols as u16) {
			*col = 0;
			*row += 1;
		}

		if *row == (num_rows as u16) {
			// Stay on last line
			*row = (num_rows - 1) as u16;

			unsafe { core::ptr::copy(buffer.add(num_cols), buffer, num_cols * (num_rows - 1)) };

			for blank_col in 0..num_cols {
				let offset = blank_col + (num_cols * (*row as usize));
				unsafe {
					buffer
						.add(offset)
						.write_volatile(GlyphAttr::new(Glyph(b' '), attr))
				};
			}
		}
	}

	/// Store a new attribute to be used for subsequent characters.
	pub fn change_attr(&self, attr: Attr) {
		let value = attr.0;
		self.current_attr.store(value, Ordering::Relaxed);
	}
}

unsafe impl Sync for TextConsole {}

impl core::fmt::Write for &TextConsole {
	/// Allows us to call `writeln!(some_text_console, "hello")`
	fn write_str(&mut self, s: &str) -> core::fmt::Result {
		// Load from global state
		let mut row = self.current_row.load(Ordering::Relaxed);
		let mut col = self.current_col.load(Ordering::Relaxed);
		let buffer = self.text_buffer.load(Ordering::Relaxed);

		if !buffer.is_null() {
			for ch in s.chars() {
				let b = TextConsole::map_char_to_glyph(ch);
				self.write_at(b, buffer, &mut row, &mut col);
			}

			// Push back to global state
			self.current_row.store(row, Ordering::Relaxed);
			self.current_col.store(col, Ordering::Relaxed);
		}

		Ok(())
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
