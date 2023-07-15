//! # VGA Driver for the Neotron Pico
//!
//! VGA output on the Neotron Pico uses 14 GPIO pins and two PIO state machines.
//!
//! It can generate 640x480@60Hz and 640x400@70Hz standard VGA video, with a
//! 25.2 MHz pixel clock. The spec is 25.175 MHz, so we are 0.1% off). The
//! assumption is that the CPU is clocked at 151.2 MHz, i.e. 6x the pixel clock.
//! All of the PIO code relies on this assumption!
//!
//! Currently 80x25, 80x30, 80x50 and 80x60 modes are supported in colour.

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

mod font16;
mod font8;
mod rgb;

// -----------------------------------------------------------------------------
// Imports
// -----------------------------------------------------------------------------

use core::sync::atomic::{AtomicBool, AtomicU16, AtomicU8, Ordering};
use defmt::{debug, trace};
use neotron_common_bios::video::{Attr, GlyphAttr, TextBackgroundColour, TextForegroundColour};
use rp_pico::{hal::pio::PIOExt, pac::interrupt};

pub use rgb::{RGBColour, RGBPair};

// -----------------------------------------------------------------------------
// Types
// -----------------------------------------------------------------------------

/// A font
pub struct Font<'a> {
	data: &'a [u8],
}

/// Holds a video mode, but as an atomic value suitable for use in a `static`.
pub struct AtomicModeWrapper {
	value: AtomicU8,
}

impl AtomicModeWrapper {
	/// Construct a new [`AtomicModeWrapper`] with the given value.
	const fn new(mode: crate::common::video::Mode) -> AtomicModeWrapper {
		AtomicModeWrapper {
			value: AtomicU8::new(mode.as_u8()),
		}
	}

	/// Set a new video mode.
	pub fn set_mode(&self, mode: crate::common::video::Mode) {
		self.value.store(mode.as_u8(), Ordering::SeqCst);
	}

	/// Get the current video mode.
	pub fn get_mode(&self) -> crate::common::video::Mode {
		let value = self.value.load(Ordering::SeqCst);
		// Safety: the 'set_mode' function ensure this is always valid.
		unsafe { crate::common::video::Mode::from_u8(value) }
	}
}

/// Describes the polarity of a sync pulse.
///
/// Some pulses are positive (active-high), some are negative (active-low).
enum SyncPolarity {
	/// An active-high pulse
	Positive,
	/// An active-low pulse
	Negative,
}

impl SyncPolarity {
	const fn enabled(&self) -> bool {
		match self {
			SyncPolarity::Positive => true,
			SyncPolarity::Negative => false,
		}
	}

	const fn disabled(&self) -> bool {
		match self {
			SyncPolarity::Positive => false,
			SyncPolarity::Negative => true,
		}
	}
}

/// Holds some data necessary to run the Video.
///
/// This structure is owned entirely by the main thread (or the drawing
/// thread). Data handled under interrupt is stored in various other places.
struct RenderEngine {
	/// How many frames have been drawn
	frame_count: u32,
	/// The current video mode
	current_video_mode: crate::common::video::Mode,
	/// How many rows of text are we showing right now
	num_text_rows: usize,
	/// How many columns of text are we showing right now
	num_text_cols: usize,
}

impl RenderEngine {
	// Initialise the main-thread resources
	pub fn new() -> RenderEngine {
		RenderEngine {
			frame_count: 0,
			// Should match the default value of TIMING_BUFFER and VIDEO_MODE
			current_video_mode: crate::common::video::Mode::new(
				crate::common::video::Timing::T640x480,
				crate::common::video::Format::Text8x16,
			),
			// Should match the mode above
			num_text_cols: 80,
			num_text_rows: 30,
		}
	}

	/// Do the start-of-frame setup
	fn frame_start(&mut self) {
		trace!("Frame {}", self.frame_count);
		self.frame_count += 1;

		// Update video mode only on first line of video
		if VIDEO_MODE.get_mode() != self.current_video_mode {
			self.current_video_mode = VIDEO_MODE.get_mode();
			match self.current_video_mode.timing() {
				crate::common::video::Timing::T640x400 => unsafe {
					TIMING_BUFFER = TimingBuffer::make_640x400();
				},
				crate::common::video::Timing::T640x480 => unsafe {
					TIMING_BUFFER = TimingBuffer::make_640x480();
				},
				crate::common::video::Timing::T800x600 => {
					panic!("Can't do 800x600");
				}
			}
			self.num_text_cols = self.current_video_mode.text_width().unwrap_or(0) as usize;
			self.num_text_rows = self.current_video_mode.text_height().unwrap_or(0) as usize;
		}
	}

	/// Draw a line of pixels into the relevant pixel buffer (either
	/// [`PIXEL_DATA_BUFFER_ODD`] or [`PIXEL_DATA_BUFFER_EVEN`]).
	///
	/// The `current_line_num` goes from `0..NUM_LINES`.
	#[link_section = ".data"]
	pub fn draw_next_line(&mut self, current_line_num: u16) {
		// Pick a buffer to render into based on the line number we are drawing.
		// It's safe to write to this buffer because it's the the other one that
		// is currently being DMA'd out to the Pixel SM.
		let scan_line_buffer = unsafe {
			if (current_line_num & 1) == 0 {
				&mut PIXEL_DATA_BUFFER_EVEN
			} else {
				&mut PIXEL_DATA_BUFFER_ODD
			}
		};

		match self.current_video_mode.format() {
			crate::common::video::Format::Text8x16 => {
				self.draw_next_line_text::<16>(&font16::FONT, scan_line_buffer, current_line_num)
			}
			crate::common::video::Format::Text8x8 => {
				self.draw_next_line_text::<8>(&font8::FONT, scan_line_buffer, current_line_num)
			}
			_ => {
				// Draw nothing
			}
		};
	}

	/// Draw a line of pixels into the relevant pixel buffer (either
	/// [`PIXEL_DATA_BUFFER_ODD`] or [`PIXEL_DATA_BUFFER_EVEN`]) using the 8x16
	/// font.
	///
	/// The `current_line_num` goes from `0..NUM_LINES`.
	#[link_section = ".data"]
	pub fn draw_next_line_text<const GLYPH_HEIGHT: usize>(
		&mut self,
		font: &Font,
		scan_line_buffer: &mut LineBuffer,
		current_line_num: u16,
	) {
		// Convert our position in scan-lines to a text row, and a line within each glyph on that row
		let text_row = current_line_num as usize / GLYPH_HEIGHT;
		let font_row = current_line_num as usize % GLYPH_HEIGHT;

		if text_row < self.num_text_rows {
			// Note (unsafe): We could stash the char array inside `self`
			// but at some point we are going to need one CPU rendering
			// the text, and the other CPU running code and writing to
			// the buffer. This might be Undefined Behaviour, but
			// unfortunately real-time video is all about shared mutable
			// state. At least our platform is fixed, so we can simply
			// test if it works, for some given version of the Rust compiler.
			let row_slice = unsafe {
				&GLYPH_ATTR_ARRAY
					[(text_row * self.num_text_cols)..((text_row + 1) * self.num_text_cols)]
			};
			// Every font look-up we are about to do for this row will
			// involve offsetting by the row within each glyph. As this
			// is the same for every glyph on this row, we calculate a
			// new pointer once, in advance, and save ourselves an
			// addition each time around the loop.
			let font_ptr = unsafe { font.data.as_ptr().add(font_row) };

			// Get a pointer into our scan-line buffer
			let mut scan_line_buffer_ptr = scan_line_buffer.pixels.as_mut_ptr();

			// Convert from characters to coloured pixels, using the font as a look-up table.

			for glyphattr in row_slice.iter() {
				let index = (glyphattr.glyph().0 as usize) * GLYPH_HEIGHT;
				let attr = glyphattr.attr();

				unsafe {
					// Note (unsafe): We use pointer arithmetic here because we
					// can't afford a bounds-check on an array. This is safe
					// because the font is `256 * width` bytes long and we can't
					// index more than `255 * width` bytes into it.
					let mono_pixels = *font_ptr.add(index);

					// 0bXX------
					let pair = TEXT_COLOUR_LOOKUP.lookup(attr, mono_pixels >> 6);
					scan_line_buffer_ptr.write(pair);
					// 0b--XX----
					let pair = TEXT_COLOUR_LOOKUP.lookup(attr, mono_pixels >> 4);
					scan_line_buffer_ptr.offset(1).write(pair);
					// 0b----XX--
					let pair = TEXT_COLOUR_LOOKUP.lookup(attr, mono_pixels >> 2);
					scan_line_buffer_ptr.offset(2).write(pair);
					// 0b------XX
					let pair = TEXT_COLOUR_LOOKUP.lookup(attr, mono_pixels);
					scan_line_buffer_ptr.offset(3).write(pair);

					scan_line_buffer_ptr = scan_line_buffer_ptr.offset(4);
				}
			}
		}
	}
}

impl Default for RenderEngine {
	fn default() -> Self {
		RenderEngine::new()
	}
}

/// Describes one scan-line's worth of pixels, including the length word required by the Pixel FIFO.
#[repr(C, align(16))]
struct LineBuffer {
	/// Must be one less than the number of pixel-pairs in `pixels`
	length: u32,
	/// Pixels to be displayed, grouped into pairs (to save FIFO space and reduce DMA bandwidth)
	pixels: [RGBPair; MAX_NUM_PIXEL_PAIRS_PER_LINE],
}

impl LineBuffer {
	/// Convert the line buffer to a 32-bit address that the DMA engine understands.
	fn as_ptr(&self) -> u32 {
		self as *const _ as usize as u32
	}
}

/// Holds the four scan-line timing FIFO words we need for one scan-line.
///
/// See `make_timing` for a function which can generate these words. We DMA
/// them into the timing FIFO, so they must sit on a 16-byte boundary.
#[repr(C, align(16))]
struct ScanlineTimingBuffer {
	data: [u32; 4],
}

impl ScanlineTimingBuffer {
	const CLOCKS_PER_PIXEL: u32 = 6;

	/// Create a timing buffer for each scan-line in the V-Sync visible portion.
	///
	/// The timings are in the order (front-porch, sync, back-porch, visible) and are in pixel clocks.
	const fn new_v_visible(
		hsync: SyncPolarity,
		vsync: SyncPolarity,
		timings: (u32, u32, u32, u32),
	) -> ScanlineTimingBuffer {
		ScanlineTimingBuffer {
			data: [
				// Front porch (as per the spec)
				Self::make_timing(
					timings.0 * Self::CLOCKS_PER_PIXEL,
					hsync.disabled(),
					vsync.disabled(),
					Some(1),
				),
				// Sync pulse (as per the spec)
				Self::make_timing(
					timings.1 * Self::CLOCKS_PER_PIXEL,
					hsync.enabled(),
					vsync.disabled(),
					None,
				),
				// Back porch. Adjusted by a few clocks to account for interrupt +
				// PIO SM start latency.
				Self::make_timing(
					(timings.2 * Self::CLOCKS_PER_PIXEL) - 5,
					hsync.disabled(),
					vsync.disabled(),
					None,
				),
				// Visible portion. It also triggers the IRQ to start pixels
				// moving. Adjusted to compensate for changes made to previous
				// period to ensure scan-line remains at correct length.
				Self::make_timing(
					(timings.3 * Self::CLOCKS_PER_PIXEL) + 5,
					hsync.disabled(),
					vsync.disabled(),
					Some(0),
				),
			],
		}
	}

	/// Create a timing buffer for each scan-line in the V-Sync front-porch and back-porch
	const fn new_v_porch(
		hsync: SyncPolarity,
		vsync: SyncPolarity,
		timings: (u32, u32, u32, u32),
	) -> ScanlineTimingBuffer {
		ScanlineTimingBuffer {
			data: [
				// Front porch (as per the spec)
				Self::make_timing(
					timings.0 * Self::CLOCKS_PER_PIXEL,
					hsync.disabled(),
					vsync.disabled(),
					Some(1),
				),
				// Sync pulse (as per the spec)
				Self::make_timing(
					timings.1 * Self::CLOCKS_PER_PIXEL,
					hsync.enabled(),
					vsync.disabled(),
					None,
				),
				// Back porch.
				Self::make_timing(
					timings.2 * Self::CLOCKS_PER_PIXEL,
					hsync.disabled(),
					vsync.disabled(),
					None,
				),
				// Visible portion.
				Self::make_timing(
					timings.3 * Self::CLOCKS_PER_PIXEL,
					hsync.disabled(),
					vsync.disabled(),
					None,
				),
			],
		}
	}

	/// Create a timing buffer for each scan-line in the V-Sync pulse
	const fn new_v_pulse(
		hsync: SyncPolarity,
		vsync: SyncPolarity,
		timings: (u32, u32, u32, u32),
	) -> ScanlineTimingBuffer {
		ScanlineTimingBuffer {
			data: [
				// Front porch (as per the spec)
				Self::make_timing(
					timings.0 * Self::CLOCKS_PER_PIXEL,
					hsync.disabled(),
					vsync.enabled(),
					Some(1),
				),
				// Sync pulse (as per the spec)
				Self::make_timing(
					timings.1 * Self::CLOCKS_PER_PIXEL,
					hsync.enabled(),
					vsync.enabled(),
					None,
				),
				// Back porch.
				Self::make_timing(
					timings.2 * Self::CLOCKS_PER_PIXEL,
					hsync.disabled(),
					vsync.enabled(),
					None,
				),
				// Visible portion.
				Self::make_timing(
					timings.3 * Self::CLOCKS_PER_PIXEL,
					hsync.disabled(),
					vsync.enabled(),
					None,
				),
			],
		}
	}

	/// Generate a 32-bit value we can send to the Timing FIFO.
	///
	/// * `period` - The length of this portion of the scan-line, in system clock ticks
	/// * `hsync` - true if the H-Sync pin should be high during this period, else false
	/// * `vsync` - true if the H-Sync pin should be high during this period, else false
	/// * `raise_irq` - true the timing statemachine should raise an IRQ at the start of this period
	///
	/// Returns a 32-bit value you can post to the Timing FIFO.
	const fn make_timing(period: u32, hsync: bool, vsync: bool, raise_irq: Option<u8>) -> u32 {
		let command = if let Some(irq_index) = raise_irq {
			// This command sets IRQ 0
			pio::InstructionOperands::IRQ {
				clear: false,
				wait: false,
				index: irq_index,
				relative: false,
			}
			.encode()
		} else {
			// This command is a no-op (it moves Y into Y)
			pio::InstructionOperands::MOV {
				destination: pio::MovDestination::Y,
				op: pio::MovOperation::None,
				source: pio::MovSource::Y,
			}
			.encode()
		} as u32;
		let mut value: u32 = 0;
		if hsync {
			value |= 1 << 0;
		}
		if vsync {
			value |= 1 << 1;
		}
		value |= (period - 6) << 2;
		value | command << 16
	}
}

/// Holds the different kinds of scan-line timing buffers we need for various
/// portions of the screen.
struct TimingBuffer {
	/// We use this when there are visible pixels on screen
	visible_line: ScanlineTimingBuffer,
	/// We use this during the v-sync front-porch and v-sync back-porch
	vblank_porch_buffer: ScanlineTimingBuffer,
	/// We use this during the v-sync sync pulse
	vblank_sync_buffer: ScanlineTimingBuffer,
	/// The last visible scan-line,
	visible_lines_ends_at: u16,
	/// The last scan-line of the front porch
	front_porch_end_at: u16,
	/// The last scan-line of the sync pulse
	sync_pulse_ends_at: u16,
	/// The last scan-line of the back-porch (and the frame)
	back_porch_ends_at: u16,
}

impl TimingBuffer {
	/// Make a timing buffer suitable for 640 x 400 @ 70 Hz
	const fn make_640x400() -> TimingBuffer {
		TimingBuffer {
			visible_line: ScanlineTimingBuffer::new_v_visible(
				SyncPolarity::Negative,
				SyncPolarity::Positive,
				(16, 96, 48, 640),
			),
			vblank_porch_buffer: ScanlineTimingBuffer::new_v_porch(
				SyncPolarity::Negative,
				SyncPolarity::Positive,
				(16, 96, 48, 640),
			),
			vblank_sync_buffer: ScanlineTimingBuffer::new_v_pulse(
				SyncPolarity::Negative,
				SyncPolarity::Positive,
				(16, 96, 48, 640),
			),
			visible_lines_ends_at: 399,
			front_porch_end_at: 399 + 12,
			sync_pulse_ends_at: 399 + 12 + 2,
			back_porch_ends_at: 399 + 12 + 2 + 35,
		}
	}

	/// Make a timing buffer suitable for 640 x 480 @ 60 Hz
	pub const fn make_640x480() -> TimingBuffer {
		TimingBuffer {
			visible_line: ScanlineTimingBuffer::new_v_visible(
				SyncPolarity::Negative,
				SyncPolarity::Negative,
				(16, 96, 48, 640),
			),
			vblank_porch_buffer: ScanlineTimingBuffer::new_v_porch(
				SyncPolarity::Negative,
				SyncPolarity::Negative,
				(16, 96, 48, 640),
			),
			vblank_sync_buffer: ScanlineTimingBuffer::new_v_pulse(
				SyncPolarity::Negative,
				SyncPolarity::Negative,
				(16, 96, 48, 640),
			),
			visible_lines_ends_at: 479,
			front_porch_end_at: 479 + 10,
			sync_pulse_ends_at: 479 + 10 + 2,
			back_porch_ends_at: 479 + 10 + 2 + 33,
		}
	}
}

/// See [`TEXT_COLOUR_LOOKUP`]
struct TextColourLookup {
	entries: [RGBPair; 512],
}

impl TextColourLookup {
	const fn blank() -> TextColourLookup {
		TextColourLookup {
			entries: [RGBPair::new(); 512],
		}
	}

	fn init(&mut self, palette: &[AtomicU16]) {
		for (fg, fg_colour) in palette.iter().take(16).enumerate() {
			for (bg, bg_colour) in palette.iter().take(8).enumerate() {
				let attr = unsafe {
					Attr::new(
						TextForegroundColour::new_unchecked(fg as u8),
						TextBackgroundColour::new_unchecked(bg as u8),
						false,
					)
				};
				for pixels in 0..=3 {
					let index: usize = (((attr.0 & 0x7F) as usize) << 2) | (pixels & 0x03) as usize;
					let pair = RGBPair::from_pixels(
						if pixels & 0x02 == 0x02 {
							RGBColour(fg_colour.load(Ordering::Relaxed))
						} else {
							RGBColour(bg_colour.load(Ordering::Relaxed))
						},
						if pixels & 0x01 == 0x01 {
							RGBColour(fg_colour.load(Ordering::Relaxed))
						} else {
							RGBColour(bg_colour.load(Ordering::Relaxed))
						},
					);
					self.entries[index] = pair;
				}
			}
		}
	}

	#[inline]
	fn lookup(&self, attr: Attr, pixels: u8) -> RGBPair {
		let index: usize = (((attr.0 & 0x7F) as usize) << 2) | (pixels & 0x03) as usize;
		unsafe { core::ptr::read(self.entries.as_ptr().add(index)) }
	}
}

// -----------------------------------------------------------------------------
// Static and Const Data
// -----------------------------------------------------------------------------

/// How many pixels per scan-line.
///
/// Adjust the pixel PIO program to run at the right speed to the screen is
/// filled. For example, if this is only 320 but you are aiming at 640x480,
/// make the pixel PIO take twice as long per pixel.
pub const MAX_NUM_PIXELS_PER_LINE: usize = 640;

/// Maximum number of lines on screen.
pub const MAX_NUM_LINES: usize = 480;

/// The highest number of columns in any text mode.
pub const MAX_TEXT_COLS: usize = MAX_NUM_PIXELS_PER_LINE / 8;

/// The highest number of rows in any text mode.
pub const MAX_TEXT_ROWS: usize = MAX_NUM_LINES / 8;

/// This is our text buffer.
///
/// This is arranged as `MAX_TEXT_ROWS` rows of `MAX_TEXT_COLS` columns. Each
/// item is an index into `font16::FONT_DATA` plus an 8-bit attribute.
///
/// Written to by Core 0, and read from by `RenderEngine` running on Core 1.
pub static mut GLYPH_ATTR_ARRAY: [GlyphAttr; MAX_TEXT_COLS * MAX_TEXT_ROWS] =
	[GlyphAttr(0); MAX_TEXT_COLS * MAX_TEXT_ROWS];

/// Stores our current video mode, or the mode we change into on the next frame.
///
/// We boot in 80x30 mode.
pub static VIDEO_MODE: AtomicModeWrapper = AtomicModeWrapper::new(crate::common::video::Mode::new(
	crate::common::video::Timing::T640x480,
	crate::common::video::Format::Text8x16,
));

/// Holds the 256-entry palette for indexed colour modes.
///
/// Note, the first eight entries should match
/// [`neotron_common_bios::video::TextBackgroundColour`] and the first 16 entries
/// should match [`neotron_common_bios::video::TextForegroundColour`].
pub static VIDEO_PALETTE: [AtomicU16; 256] = [
	// Index 000: 0x000 (Black)
	AtomicU16::new(RGBColour::from_12bit(0x0, 0x0, 0x0).0),
	// Index 001: 0x800 (Dark Red)
	AtomicU16::new(RGBColour::from_12bit(0x8, 0x0, 0x0).0),
	// Index 002: 0x080 (Dark Green)
	AtomicU16::new(RGBColour::from_12bit(0x0, 0x8, 0x0).0),
	// Index 003: 0x880 (Orange)
	AtomicU16::new(RGBColour::from_12bit(0x8, 0x8, 0x0).0),
	// Index 004: 0x008 (Blue)
	AtomicU16::new(RGBColour::from_12bit(0x0, 0x0, 0x8).0),
	// Index 005: 0x808 (Dark Magenta)
	AtomicU16::new(RGBColour::from_12bit(0x8, 0x0, 0x8).0),
	// Index 006: 0x088 (Dark Cyan)
	AtomicU16::new(RGBColour::from_12bit(0x0, 0x8, 0x8).0),
	// Index 007: 0xcc0 (Yellow)
	AtomicU16::new(RGBColour::from_12bit(0xc, 0xc, 0x0).0),
	// Index 008: 0x888 (Grey)
	AtomicU16::new(RGBColour::from_12bit(0x8, 0x8, 0x8).0),
	// Index 009: 0xf00 (Bright Red)
	AtomicU16::new(RGBColour::from_12bit(0xf, 0x0, 0x0).0),
	// Index 010: 0x0f0 (Bright Green)
	AtomicU16::new(RGBColour::from_12bit(0x0, 0xf, 0x0).0),
	// Index 011: 0xff0 (Bright Yellow)
	AtomicU16::new(RGBColour::from_12bit(0xf, 0xf, 0x0).0),
	// Index 012: 0x00f (Bright Blue)
	AtomicU16::new(RGBColour::from_12bit(0x0, 0x0, 0xf).0),
	// Index 013: 0xf0f (Bright Magenta)
	AtomicU16::new(RGBColour::from_12bit(0xf, 0x0, 0xf).0),
	// Index 014: 0x0ff (Bright Cyan)
	AtomicU16::new(RGBColour::from_12bit(0x0, 0xf, 0xf).0),
	// Index 015: 0xfff (White)
	AtomicU16::new(RGBColour::from_12bit(0xf, 0xf, 0xf).0),
	// Index 016: 0x003
	AtomicU16::new(RGBColour::from_12bit(0x0, 0x0, 0x3).0),
	// Index 017: 0x006
	AtomicU16::new(RGBColour::from_12bit(0x0, 0x0, 0x6).0),
	// Index 018: 0x00c
	AtomicU16::new(RGBColour::from_12bit(0x0, 0x0, 0xc).0),
	// Index 019: 0x020
	AtomicU16::new(RGBColour::from_12bit(0x0, 0x2, 0x0).0),
	// Index 020: 0x023
	AtomicU16::new(RGBColour::from_12bit(0x0, 0x2, 0x3).0),
	// Index 021: 0x026
	AtomicU16::new(RGBColour::from_12bit(0x0, 0x2, 0x6).0),
	// Index 022: 0x028
	AtomicU16::new(RGBColour::from_12bit(0x0, 0x2, 0x8).0),
	// Index 023: 0x02c
	AtomicU16::new(RGBColour::from_12bit(0x0, 0x2, 0xc).0),
	// Index 024: 0x02f
	AtomicU16::new(RGBColour::from_12bit(0x0, 0x2, 0xf).0),
	// Index 025: 0x040
	AtomicU16::new(RGBColour::from_12bit(0x0, 0x4, 0x0).0),
	// Index 026: 0x043
	AtomicU16::new(RGBColour::from_12bit(0x0, 0x4, 0x3).0),
	// Index 027: 0x046
	AtomicU16::new(RGBColour::from_12bit(0x0, 0x4, 0x6).0),
	// Index 028: 0x048
	AtomicU16::new(RGBColour::from_12bit(0x0, 0x4, 0x8).0),
	// Index 029: 0x04c
	AtomicU16::new(RGBColour::from_12bit(0x0, 0x4, 0xc).0),
	// Index 030: 0x04f
	AtomicU16::new(RGBColour::from_12bit(0x0, 0x4, 0xf).0),
	// Index 031: 0x083
	AtomicU16::new(RGBColour::from_12bit(0x0, 0x8, 0x3).0),
	// Index 032: 0x086
	AtomicU16::new(RGBColour::from_12bit(0x0, 0x8, 0x6).0),
	// Index 033: 0x08c
	AtomicU16::new(RGBColour::from_12bit(0x0, 0x8, 0xc).0),
	// Index 034: 0x08f
	AtomicU16::new(RGBColour::from_12bit(0x0, 0x8, 0xf).0),
	// Index 035: 0x0a0
	AtomicU16::new(RGBColour::from_12bit(0x0, 0xa, 0x0).0),
	// Index 036: 0x0a3
	AtomicU16::new(RGBColour::from_12bit(0x0, 0xa, 0x3).0),
	// Index 037: 0x0a6
	AtomicU16::new(RGBColour::from_12bit(0x0, 0xa, 0x6).0),
	// Index 038: 0x0a8
	AtomicU16::new(RGBColour::from_12bit(0x0, 0xa, 0x8).0),
	// Index 039: 0x0ac
	AtomicU16::new(RGBColour::from_12bit(0x0, 0xa, 0xc).0),
	// Index 040: 0x0af
	AtomicU16::new(RGBColour::from_12bit(0x0, 0xa, 0xf).0),
	// Index 041: 0x0e0
	AtomicU16::new(RGBColour::from_12bit(0x0, 0xe, 0x0).0),
	// Index 042: 0x0e3
	AtomicU16::new(RGBColour::from_12bit(0x0, 0xe, 0x3).0),
	// Index 043: 0x0e6
	AtomicU16::new(RGBColour::from_12bit(0x0, 0xe, 0x6).0),
	// Index 044: 0x0e8
	AtomicU16::new(RGBColour::from_12bit(0x0, 0xe, 0x8).0),
	// Index 045: 0x0ec
	AtomicU16::new(RGBColour::from_12bit(0x0, 0xe, 0xc).0),
	// Index 046: 0x0ef
	AtomicU16::new(RGBColour::from_12bit(0x0, 0xe, 0xf).0),
	// Index 047: 0x0f3
	AtomicU16::new(RGBColour::from_12bit(0x0, 0xf, 0x3).0),
	// Index 048: 0x0f6
	AtomicU16::new(RGBColour::from_12bit(0x0, 0xf, 0x6).0),
	// Index 049: 0x0f8
	AtomicU16::new(RGBColour::from_12bit(0x0, 0xf, 0x8).0),
	// Index 050: 0x0fc
	AtomicU16::new(RGBColour::from_12bit(0x0, 0xf, 0xc).0),
	// Index 051: 0x300
	AtomicU16::new(RGBColour::from_12bit(0x3, 0x0, 0x0).0),
	// Index 052: 0x303
	AtomicU16::new(RGBColour::from_12bit(0x3, 0x0, 0x3).0),
	// Index 053: 0x306
	AtomicU16::new(RGBColour::from_12bit(0x3, 0x0, 0x6).0),
	// Index 054: 0x308
	AtomicU16::new(RGBColour::from_12bit(0x3, 0x0, 0x8).0),
	// Index 055: 0x30c
	AtomicU16::new(RGBColour::from_12bit(0x3, 0x0, 0xc).0),
	// Index 056: 0x30f
	AtomicU16::new(RGBColour::from_12bit(0x3, 0x0, 0xf).0),
	// Index 057: 0x320
	AtomicU16::new(RGBColour::from_12bit(0x3, 0x2, 0x0).0),
	// Index 058: 0x323
	AtomicU16::new(RGBColour::from_12bit(0x3, 0x2, 0x3).0),
	// Index 059: 0x326
	AtomicU16::new(RGBColour::from_12bit(0x3, 0x2, 0x6).0),
	// Index 060: 0x328
	AtomicU16::new(RGBColour::from_12bit(0x3, 0x2, 0x8).0),
	// Index 061: 0x32c
	AtomicU16::new(RGBColour::from_12bit(0x3, 0x2, 0xc).0),
	// Index 062: 0x32f
	AtomicU16::new(RGBColour::from_12bit(0x3, 0x2, 0xf).0),
	// Index 063: 0x340
	AtomicU16::new(RGBColour::from_12bit(0x3, 0x4, 0x0).0),
	// Index 064: 0x343
	AtomicU16::new(RGBColour::from_12bit(0x3, 0x4, 0x3).0),
	// Index 065: 0x346
	AtomicU16::new(RGBColour::from_12bit(0x3, 0x4, 0x6).0),
	// Index 066: 0x348
	AtomicU16::new(RGBColour::from_12bit(0x3, 0x4, 0x8).0),
	// Index 067: 0x34c
	AtomicU16::new(RGBColour::from_12bit(0x3, 0x4, 0xc).0),
	// Index 068: 0x34f
	AtomicU16::new(RGBColour::from_12bit(0x3, 0x4, 0xf).0),
	// Index 069: 0x380
	AtomicU16::new(RGBColour::from_12bit(0x3, 0x8, 0x0).0),
	// Index 070: 0x383
	AtomicU16::new(RGBColour::from_12bit(0x3, 0x8, 0x3).0),
	// Index 071: 0x386
	AtomicU16::new(RGBColour::from_12bit(0x3, 0x8, 0x6).0),
	// Index 072: 0x388
	AtomicU16::new(RGBColour::from_12bit(0x3, 0x8, 0x8).0),
	// Index 073: 0x38c
	AtomicU16::new(RGBColour::from_12bit(0x3, 0x8, 0xc).0),
	// Index 074: 0x38f
	AtomicU16::new(RGBColour::from_12bit(0x3, 0x8, 0xf).0),
	// Index 075: 0x3a0
	AtomicU16::new(RGBColour::from_12bit(0x3, 0xa, 0x0).0),
	// Index 076: 0x3a3
	AtomicU16::new(RGBColour::from_12bit(0x3, 0xa, 0x3).0),
	// Index 077: 0x3a6
	AtomicU16::new(RGBColour::from_12bit(0x3, 0xa, 0x6).0),
	// Index 078: 0x3a8
	AtomicU16::new(RGBColour::from_12bit(0x3, 0xa, 0x8).0),
	// Index 079: 0x3ac
	AtomicU16::new(RGBColour::from_12bit(0x3, 0xa, 0xc).0),
	// Index 080: 0x3af
	AtomicU16::new(RGBColour::from_12bit(0x3, 0xa, 0xf).0),
	// Index 081: 0x3e0
	AtomicU16::new(RGBColour::from_12bit(0x3, 0xe, 0x0).0),
	// Index 082: 0x3e3
	AtomicU16::new(RGBColour::from_12bit(0x3, 0xe, 0x3).0),
	// Index 083: 0x3e6
	AtomicU16::new(RGBColour::from_12bit(0x3, 0xe, 0x6).0),
	// Index 084: 0x3e8
	AtomicU16::new(RGBColour::from_12bit(0x3, 0xe, 0x8).0),
	// Index 085: 0x3ec
	AtomicU16::new(RGBColour::from_12bit(0x3, 0xe, 0xc).0),
	// Index 086: 0x3ef
	AtomicU16::new(RGBColour::from_12bit(0x3, 0xe, 0xf).0),
	// Index 087: 0x3f0
	AtomicU16::new(RGBColour::from_12bit(0x3, 0xf, 0x0).0),
	// Index 088: 0x3f3
	AtomicU16::new(RGBColour::from_12bit(0x3, 0xf, 0x3).0),
	// Index 089: 0x3f6
	AtomicU16::new(RGBColour::from_12bit(0x3, 0xf, 0x6).0),
	// Index 090: 0x3f8
	AtomicU16::new(RGBColour::from_12bit(0x3, 0xf, 0x8).0),
	// Index 091: 0x3fc
	AtomicU16::new(RGBColour::from_12bit(0x3, 0xf, 0xc).0),
	// Index 092: 0x3ff
	AtomicU16::new(RGBColour::from_12bit(0x3, 0xf, 0xf).0),
	// Index 093: 0x600
	AtomicU16::new(RGBColour::from_12bit(0x6, 0x0, 0x0).0),
	// Index 094: 0x603
	AtomicU16::new(RGBColour::from_12bit(0x6, 0x0, 0x3).0),
	// Index 095: 0x606
	AtomicU16::new(RGBColour::from_12bit(0x6, 0x0, 0x6).0),
	// Index 096: 0x608
	AtomicU16::new(RGBColour::from_12bit(0x6, 0x0, 0x8).0),
	// Index 097: 0x60c
	AtomicU16::new(RGBColour::from_12bit(0x6, 0x0, 0xc).0),
	// Index 098: 0x60f
	AtomicU16::new(RGBColour::from_12bit(0x6, 0x0, 0xf).0),
	// Index 099: 0x620
	AtomicU16::new(RGBColour::from_12bit(0x6, 0x2, 0x0).0),
	// Index 100: 0x623
	AtomicU16::new(RGBColour::from_12bit(0x6, 0x2, 0x3).0),
	// Index 101: 0x626
	AtomicU16::new(RGBColour::from_12bit(0x6, 0x2, 0x6).0),
	// Index 102: 0x628
	AtomicU16::new(RGBColour::from_12bit(0x6, 0x2, 0x8).0),
	// Index 103: 0x62c
	AtomicU16::new(RGBColour::from_12bit(0x6, 0x2, 0xc).0),
	// Index 104: 0x62f
	AtomicU16::new(RGBColour::from_12bit(0x6, 0x2, 0xf).0),
	// Index 105: 0x640
	AtomicU16::new(RGBColour::from_12bit(0x6, 0x4, 0x0).0),
	// Index 106: 0x643
	AtomicU16::new(RGBColour::from_12bit(0x6, 0x4, 0x3).0),
	// Index 107: 0x646
	AtomicU16::new(RGBColour::from_12bit(0x6, 0x4, 0x6).0),
	// Index 108: 0x648
	AtomicU16::new(RGBColour::from_12bit(0x6, 0x4, 0x8).0),
	// Index 109: 0x64c
	AtomicU16::new(RGBColour::from_12bit(0x6, 0x4, 0xc).0),
	// Index 110: 0x64f
	AtomicU16::new(RGBColour::from_12bit(0x6, 0x4, 0xf).0),
	// Index 111: 0x680
	AtomicU16::new(RGBColour::from_12bit(0x6, 0x8, 0x0).0),
	// Index 112: 0x683
	AtomicU16::new(RGBColour::from_12bit(0x6, 0x8, 0x3).0),
	// Index 113: 0x686
	AtomicU16::new(RGBColour::from_12bit(0x6, 0x8, 0x6).0),
	// Index 114: 0x688
	AtomicU16::new(RGBColour::from_12bit(0x6, 0x8, 0x8).0),
	// Index 115: 0x68c
	AtomicU16::new(RGBColour::from_12bit(0x6, 0x8, 0xc).0),
	// Index 116: 0x68f
	AtomicU16::new(RGBColour::from_12bit(0x6, 0x8, 0xf).0),
	// Index 117: 0x6a0
	AtomicU16::new(RGBColour::from_12bit(0x6, 0xa, 0x0).0),
	// Index 118: 0x6a3
	AtomicU16::new(RGBColour::from_12bit(0x6, 0xa, 0x3).0),
	// Index 119: 0x6a6
	AtomicU16::new(RGBColour::from_12bit(0x6, 0xa, 0x6).0),
	// Index 120: 0x6a8
	AtomicU16::new(RGBColour::from_12bit(0x6, 0xa, 0x8).0),
	// Index 121: 0x6ac
	AtomicU16::new(RGBColour::from_12bit(0x6, 0xa, 0xc).0),
	// Index 122: 0x6af
	AtomicU16::new(RGBColour::from_12bit(0x6, 0xa, 0xf).0),
	// Index 123: 0x6e0
	AtomicU16::new(RGBColour::from_12bit(0x6, 0xe, 0x0).0),
	// Index 124: 0x6e3
	AtomicU16::new(RGBColour::from_12bit(0x6, 0xe, 0x3).0),
	// Index 125: 0x6e6
	AtomicU16::new(RGBColour::from_12bit(0x6, 0xe, 0x6).0),
	// Index 126: 0x6e8
	AtomicU16::new(RGBColour::from_12bit(0x6, 0xe, 0x8).0),
	// Index 127: 0x6ec
	AtomicU16::new(RGBColour::from_12bit(0x6, 0xe, 0xc).0),
	// Index 128: 0x6ef
	AtomicU16::new(RGBColour::from_12bit(0x6, 0xe, 0xf).0),
	// Index 129: 0x6f0
	AtomicU16::new(RGBColour::from_12bit(0x6, 0xf, 0x0).0),
	// Index 130: 0x6f3
	AtomicU16::new(RGBColour::from_12bit(0x6, 0xf, 0x3).0),
	// Index 131: 0x6f6
	AtomicU16::new(RGBColour::from_12bit(0x6, 0xf, 0x6).0),
	// Index 132: 0x6f8
	AtomicU16::new(RGBColour::from_12bit(0x6, 0xf, 0x8).0),
	// Index 133: 0x6fc
	AtomicU16::new(RGBColour::from_12bit(0x6, 0xf, 0xc).0),
	// Index 134: 0x6ff
	AtomicU16::new(RGBColour::from_12bit(0x6, 0xf, 0xf).0),
	// Index 135: 0x803
	AtomicU16::new(RGBColour::from_12bit(0x8, 0x0, 0x3).0),
	// Index 136: 0x806
	AtomicU16::new(RGBColour::from_12bit(0x8, 0x0, 0x6).0),
	// Index 137: 0x80c
	AtomicU16::new(RGBColour::from_12bit(0x8, 0x0, 0xc).0),
	// Index 138: 0x80f
	AtomicU16::new(RGBColour::from_12bit(0x8, 0x0, 0xf).0),
	// Index 139: 0x820
	AtomicU16::new(RGBColour::from_12bit(0x8, 0x2, 0x0).0),
	// Index 140: 0x823
	AtomicU16::new(RGBColour::from_12bit(0x8, 0x2, 0x3).0),
	// Index 141: 0x826
	AtomicU16::new(RGBColour::from_12bit(0x8, 0x2, 0x6).0),
	// Index 142: 0x828
	AtomicU16::new(RGBColour::from_12bit(0x8, 0x2, 0x8).0),
	// Index 143: 0x82c
	AtomicU16::new(RGBColour::from_12bit(0x8, 0x2, 0xc).0),
	// Index 144: 0x82f
	AtomicU16::new(RGBColour::from_12bit(0x8, 0x2, 0xf).0),
	// Index 145: 0x840
	AtomicU16::new(RGBColour::from_12bit(0x8, 0x4, 0x0).0),
	// Index 146: 0x843
	AtomicU16::new(RGBColour::from_12bit(0x8, 0x4, 0x3).0),
	// Index 147: 0x846
	AtomicU16::new(RGBColour::from_12bit(0x8, 0x4, 0x6).0),
	// Index 148: 0x848
	AtomicU16::new(RGBColour::from_12bit(0x8, 0x4, 0x8).0),
	// Index 149: 0x84c
	AtomicU16::new(RGBColour::from_12bit(0x8, 0x4, 0xc).0),
	// Index 150: 0x84f
	AtomicU16::new(RGBColour::from_12bit(0x8, 0x4, 0xf).0),
	// Index 151: 0x883
	AtomicU16::new(RGBColour::from_12bit(0x8, 0x8, 0x3).0),
	// Index 152: 0x886
	AtomicU16::new(RGBColour::from_12bit(0x8, 0x8, 0x6).0),
	// Index 153: 0x88c
	AtomicU16::new(RGBColour::from_12bit(0x8, 0x8, 0xc).0),
	// Index 154: 0x88f
	AtomicU16::new(RGBColour::from_12bit(0x8, 0x8, 0xf).0),
	// Index 155: 0x8a0
	AtomicU16::new(RGBColour::from_12bit(0x8, 0xa, 0x0).0),
	// Index 156: 0x8a3
	AtomicU16::new(RGBColour::from_12bit(0x8, 0xa, 0x3).0),
	// Index 157: 0x8a6
	AtomicU16::new(RGBColour::from_12bit(0x8, 0xa, 0x6).0),
	// Index 158: 0x8a8
	AtomicU16::new(RGBColour::from_12bit(0x8, 0xa, 0x8).0),
	// Index 159: 0x8ac
	AtomicU16::new(RGBColour::from_12bit(0x8, 0xa, 0xc).0),
	// Index 160: 0x8af
	AtomicU16::new(RGBColour::from_12bit(0x8, 0xa, 0xf).0),
	// Index 161: 0x8e0
	AtomicU16::new(RGBColour::from_12bit(0x8, 0xe, 0x0).0),
	// Index 162: 0x8e3
	AtomicU16::new(RGBColour::from_12bit(0x8, 0xe, 0x3).0),
	// Index 163: 0x8e6
	AtomicU16::new(RGBColour::from_12bit(0x8, 0xe, 0x6).0),
	// Index 164: 0x8e8
	AtomicU16::new(RGBColour::from_12bit(0x8, 0xe, 0x8).0),
	// Index 165: 0x8ec
	AtomicU16::new(RGBColour::from_12bit(0x8, 0xe, 0xc).0),
	// Index 166: 0x8ef
	AtomicU16::new(RGBColour::from_12bit(0x8, 0xe, 0xf).0),
	// Index 167: 0x8f0
	AtomicU16::new(RGBColour::from_12bit(0x8, 0xf, 0x0).0),
	// Index 168: 0x8f3
	AtomicU16::new(RGBColour::from_12bit(0x8, 0xf, 0x3).0),
	// Index 169: 0x8f6
	AtomicU16::new(RGBColour::from_12bit(0x8, 0xf, 0x6).0),
	// Index 170: 0x8f8
	AtomicU16::new(RGBColour::from_12bit(0x8, 0xf, 0x8).0),
	// Index 171: 0x8fc
	AtomicU16::new(RGBColour::from_12bit(0x8, 0xf, 0xc).0),
	// Index 172: 0x8ff
	AtomicU16::new(RGBColour::from_12bit(0x8, 0xf, 0xf).0),
	// Index 173: 0xc00
	AtomicU16::new(RGBColour::from_12bit(0xc, 0x0, 0x0).0),
	// Index 174: 0xc03
	AtomicU16::new(RGBColour::from_12bit(0xc, 0x0, 0x3).0),
	// Index 175: 0xc06
	AtomicU16::new(RGBColour::from_12bit(0xc, 0x0, 0x6).0),
	// Index 176: 0xc08
	AtomicU16::new(RGBColour::from_12bit(0xc, 0x0, 0x8).0),
	// Index 177: 0xc0c
	AtomicU16::new(RGBColour::from_12bit(0xc, 0x0, 0xc).0),
	// Index 178: 0xc0f
	AtomicU16::new(RGBColour::from_12bit(0xc, 0x0, 0xf).0),
	// Index 179: 0xc20
	AtomicU16::new(RGBColour::from_12bit(0xc, 0x2, 0x0).0),
	// Index 180: 0xc23
	AtomicU16::new(RGBColour::from_12bit(0xc, 0x2, 0x3).0),
	// Index 181: 0xc26
	AtomicU16::new(RGBColour::from_12bit(0xc, 0x2, 0x6).0),
	// Index 182: 0xc28
	AtomicU16::new(RGBColour::from_12bit(0xc, 0x2, 0x8).0),
	// Index 183: 0xc2c
	AtomicU16::new(RGBColour::from_12bit(0xc, 0x2, 0xc).0),
	// Index 184: 0xc2f
	AtomicU16::new(RGBColour::from_12bit(0xc, 0x2, 0xf).0),
	// Index 185: 0xc40
	AtomicU16::new(RGBColour::from_12bit(0xc, 0x4, 0x0).0),
	// Index 186: 0xc43
	AtomicU16::new(RGBColour::from_12bit(0xc, 0x4, 0x3).0),
	// Index 187: 0xc46
	AtomicU16::new(RGBColour::from_12bit(0xc, 0x4, 0x6).0),
	// Index 188: 0xc48
	AtomicU16::new(RGBColour::from_12bit(0xc, 0x4, 0x8).0),
	// Index 189: 0xc4c
	AtomicU16::new(RGBColour::from_12bit(0xc, 0x4, 0xc).0),
	// Index 190: 0xc4f
	AtomicU16::new(RGBColour::from_12bit(0xc, 0x4, 0xf).0),
	// Index 191: 0xc80
	AtomicU16::new(RGBColour::from_12bit(0xc, 0x8, 0x0).0),
	// Index 192: 0xc83
	AtomicU16::new(RGBColour::from_12bit(0xc, 0x8, 0x3).0),
	// Index 193: 0xc86
	AtomicU16::new(RGBColour::from_12bit(0xc, 0x8, 0x6).0),
	// Index 194: 0xc88
	AtomicU16::new(RGBColour::from_12bit(0xc, 0x8, 0x8).0),
	// Index 195: 0xc8c
	AtomicU16::new(RGBColour::from_12bit(0xc, 0x8, 0xc).0),
	// Index 196: 0xc8f
	AtomicU16::new(RGBColour::from_12bit(0xc, 0x8, 0xf).0),
	// Index 197: 0xca0
	AtomicU16::new(RGBColour::from_12bit(0xc, 0xa, 0x0).0),
	// Index 198: 0xca3
	AtomicU16::new(RGBColour::from_12bit(0xc, 0xa, 0x3).0),
	// Index 199: 0xca6
	AtomicU16::new(RGBColour::from_12bit(0xc, 0xa, 0x6).0),
	// Index 200: 0xca8
	AtomicU16::new(RGBColour::from_12bit(0xc, 0xa, 0x8).0),
	// Index 201: 0xcac
	AtomicU16::new(RGBColour::from_12bit(0xc, 0xa, 0xc).0),
	// Index 202: 0xcaf
	AtomicU16::new(RGBColour::from_12bit(0xc, 0xa, 0xf).0),
	// Index 203: 0xce0
	AtomicU16::new(RGBColour::from_12bit(0xc, 0xe, 0x0).0),
	// Index 204: 0xce3
	AtomicU16::new(RGBColour::from_12bit(0xc, 0xe, 0x3).0),
	// Index 205: 0xce6
	AtomicU16::new(RGBColour::from_12bit(0xc, 0xe, 0x6).0),
	// Index 206: 0xce8
	AtomicU16::new(RGBColour::from_12bit(0xc, 0xe, 0x8).0),
	// Index 207: 0xcec
	AtomicU16::new(RGBColour::from_12bit(0xc, 0xe, 0xc).0),
	// Index 208: 0xcef
	AtomicU16::new(RGBColour::from_12bit(0xc, 0xe, 0xf).0),
	// Index 209: 0xcf0
	AtomicU16::new(RGBColour::from_12bit(0xc, 0xf, 0x0).0),
	// Index 210: 0xcf3
	AtomicU16::new(RGBColour::from_12bit(0xc, 0xf, 0x3).0),
	// Index 211: 0xcf6
	AtomicU16::new(RGBColour::from_12bit(0xc, 0xf, 0x6).0),
	// Index 212: 0xcf8
	AtomicU16::new(RGBColour::from_12bit(0xc, 0xf, 0x8).0),
	// Index 213: 0xcfc
	AtomicU16::new(RGBColour::from_12bit(0xc, 0xf, 0xc).0),
	// Index 214: 0xcff
	AtomicU16::new(RGBColour::from_12bit(0xc, 0xf, 0xf).0),
	// Index 215: 0xf03
	AtomicU16::new(RGBColour::from_12bit(0xf, 0x0, 0x3).0),
	// Index 216: 0xf06
	AtomicU16::new(RGBColour::from_12bit(0xf, 0x0, 0x6).0),
	// Index 217: 0xf08
	AtomicU16::new(RGBColour::from_12bit(0xf, 0x0, 0x8).0),
	// Index 218: 0xf0c
	AtomicU16::new(RGBColour::from_12bit(0xf, 0x0, 0xc).0),
	// Index 219: 0xf20
	AtomicU16::new(RGBColour::from_12bit(0xf, 0x2, 0x0).0),
	// Index 220: 0xf23
	AtomicU16::new(RGBColour::from_12bit(0xf, 0x2, 0x3).0),
	// Index 221: 0xf26
	AtomicU16::new(RGBColour::from_12bit(0xf, 0x2, 0x6).0),
	// Index 222: 0xf28
	AtomicU16::new(RGBColour::from_12bit(0xf, 0x2, 0x8).0),
	// Index 223: 0xf2c
	AtomicU16::new(RGBColour::from_12bit(0xf, 0x2, 0xc).0),
	// Index 224: 0xf2f
	AtomicU16::new(RGBColour::from_12bit(0xf, 0x2, 0xf).0),
	// Index 225: 0xf40
	AtomicU16::new(RGBColour::from_12bit(0xf, 0x4, 0x0).0),
	// Index 226: 0xf43
	AtomicU16::new(RGBColour::from_12bit(0xf, 0x4, 0x3).0),
	// Index 227: 0xf46
	AtomicU16::new(RGBColour::from_12bit(0xf, 0x4, 0x6).0),
	// Index 228: 0xf48
	AtomicU16::new(RGBColour::from_12bit(0xf, 0x4, 0x8).0),
	// Index 229: 0xf4c
	AtomicU16::new(RGBColour::from_12bit(0xf, 0x4, 0xc).0),
	// Index 230: 0xf4f
	AtomicU16::new(RGBColour::from_12bit(0xf, 0x4, 0xf).0),
	// Index 231: 0xf80
	AtomicU16::new(RGBColour::from_12bit(0xf, 0x8, 0x0).0),
	// Index 232: 0xf83
	AtomicU16::new(RGBColour::from_12bit(0xf, 0x8, 0x3).0),
	// Index 233: 0xf86
	AtomicU16::new(RGBColour::from_12bit(0xf, 0x8, 0x6).0),
	// Index 234: 0xf88
	AtomicU16::new(RGBColour::from_12bit(0xf, 0x8, 0x8).0),
	// Index 235: 0xf8c
	AtomicU16::new(RGBColour::from_12bit(0xf, 0x8, 0xc).0),
	// Index 236: 0xf8f
	AtomicU16::new(RGBColour::from_12bit(0xf, 0x8, 0xf).0),
	// Index 237: 0xfa0
	AtomicU16::new(RGBColour::from_12bit(0xf, 0xa, 0x0).0),
	// Index 238: 0xfa3
	AtomicU16::new(RGBColour::from_12bit(0xf, 0xa, 0x3).0),
	// Index 239: 0xfa6
	AtomicU16::new(RGBColour::from_12bit(0xf, 0xa, 0x6).0),
	// Index 240: 0xfa8
	AtomicU16::new(RGBColour::from_12bit(0xf, 0xa, 0x8).0),
	// Index 241: 0xfac
	AtomicU16::new(RGBColour::from_12bit(0xf, 0xa, 0xc).0),
	// Index 242: 0xfaf
	AtomicU16::new(RGBColour::from_12bit(0xf, 0xa, 0xf).0),
	// Index 243: 0xfe0
	AtomicU16::new(RGBColour::from_12bit(0xf, 0xe, 0x0).0),
	// Index 244: 0xfe3
	AtomicU16::new(RGBColour::from_12bit(0xf, 0xe, 0x3).0),
	// Index 245: 0xfe6
	AtomicU16::new(RGBColour::from_12bit(0xf, 0xe, 0x6).0),
	// Index 246: 0xfe8
	AtomicU16::new(RGBColour::from_12bit(0xf, 0xe, 0x8).0),
	// Index 247: 0xfec
	AtomicU16::new(RGBColour::from_12bit(0xf, 0xe, 0xc).0),
	// Index 248: 0xfef
	AtomicU16::new(RGBColour::from_12bit(0xf, 0xe, 0xf).0),
	// Index 249: 0xff3
	AtomicU16::new(RGBColour::from_12bit(0xf, 0xf, 0x3).0),
	// Index 250: 0xff6
	AtomicU16::new(RGBColour::from_12bit(0xf, 0xf, 0x6).0),
	// Index 251: 0xff8
	AtomicU16::new(RGBColour::from_12bit(0xf, 0xf, 0x8).0),
	// Index 252: 0xffc
	AtomicU16::new(RGBColour::from_12bit(0xf, 0xf, 0xc).0),
	// Index 253: 0xbbb
	AtomicU16::new(RGBColour::from_12bit(0xb, 0xb, 0xb).0),
	// Index 254: 0x333
	AtomicU16::new(RGBColour::from_12bit(0x3, 0x3, 0x3).0),
	// Index 255: 0x777
	AtomicU16::new(RGBColour::from_12bit(0x7, 0x7, 0x7).0),
];

/// How many pixel pairs we send out.
///
/// Each pixel is two 12-bit values packed into one 32-bit word(an `RGBPair`).
/// This is to make more efficient use of DMA and FIFO resources.
const MAX_NUM_PIXEL_PAIRS_PER_LINE: usize = MAX_NUM_PIXELS_PER_LINE / 2;

/// Stores our timing data which we DMA into the timing PIO State Machine.
///
/// Default value must match [`RenderEngine::new`]
static mut TIMING_BUFFER: TimingBuffer = TimingBuffer::make_640x480();

/// Tracks which scan-line will be shown next.
///
/// This is for timing purposes, therefore it goes from
/// `0..TIMING_BUFFER.back_porch_ends_at`.
///
/// Set by the PIO IRQ.
static NEXT_SCAN_LINE: AtomicU16 = AtomicU16::new(0);

/// Indicates that we should draw the current scan-line given by [`NEXT_SCAN_LINE`].
///
/// Set by the PIO IRQ.
static DRAW_THIS_LINE: AtomicBool = AtomicBool::new(false);

/// DMA channel for the timing FIFO
const TIMING_DMA_CHAN: usize = 0;

/// DMA channel for the pixel FIFO
const PIXEL_DMA_CHAN: usize = 1;

/// One scan-line's worth of 12-bit pixels, used for the even scan-lines (0, 2, 4 ... NUM_LINES-2).
///
/// Gets read by DMA, which pushes them into the pixel state machine's FIFO.
///
/// Gets written to by `RenderEngine` running on Core 1.
static mut PIXEL_DATA_BUFFER_EVEN: LineBuffer = LineBuffer {
	length: (MAX_NUM_PIXEL_PAIRS_PER_LINE as u32) - 1,
	pixels: [RGBPair::from_pixels(RGBColour::WHITE, RGBColour::BLACK);
		MAX_NUM_PIXEL_PAIRS_PER_LINE],
};

/// One scan-line's worth of 12-bit pixels, used for the odd scan-lines (1, 3, 5 ... NUM_LINES-1).
///
/// Gets read by DMA, which pushes them into the pixel state machine's FIFO.
///
/// Gets written to by `RenderEngine` running on Core 1.
static mut PIXEL_DATA_BUFFER_ODD: LineBuffer = LineBuffer {
	length: (MAX_NUM_PIXEL_PAIRS_PER_LINE as u32) - 1,
	pixels: [RGBPair::from_pixels(RGBColour::BLACK, RGBColour::WHITE);
		MAX_NUM_PIXEL_PAIRS_PER_LINE],
};

/// Holds the colour look-up table for text mode.
///
/// The input is a 9-bit value comprised of the 4-bit foreground colour index,
/// the 3-bit background colour index, and a two mono pixels. The output is a
/// 32-bit RGB Colour Pair, containing two RGB pixels.
///
/// ```
/// +-----+-----+-----+-----+-----+-----+-----+-----+-----+
/// | FG3 | FG2 | FG1 | FG0 | BG2 | BG1 | BG0 | PX1 | PX0 |
/// +-----+-----+-----+-----+-----+-----+-----+-----+-----+
/// ```
static mut TEXT_COLOUR_LOOKUP: TextColourLookup = TextColourLookup::blank();

// -----------------------------------------------------------------------------
// Functions
// -----------------------------------------------------------------------------

/// Initialise all the static data and peripherals we need for our video display.
///
/// We need to keep `pio` and `dma` to run the video. We need `resets` to set
/// things up, so we only borrow that.
pub fn init(
	pio: super::pac::PIO0,
	dma: super::pac::DMA,
	resets: &mut super::pac::RESETS,
	ppb: &mut crate::pac::PPB,
	fifo: &mut rp_pico::hal::sio::SioFifo,
	psm: &mut crate::pac::PSM,
) {
	// Grab PIO0 and the state machines it contains
	let (mut pio, sm0, sm1, _sm2, _sm3) = pio.split(resets);

	// This program runs the timing loop. We post timing data (i.e. the length
	// of each period, along with what the H-Sync and V-Sync pins should do)
	// and it sets the GPIO pins and busy-waits the appropriate amount of
	// time. It also takes an extra 'instruction' which we can use to trigger
	// the appropriate interrupts.
	//
	// Post <value:32> where value: <clock_cycles:14> <hsync:1> <vsync:1>
	// <instruction:16>
	//
	// The SM will execute the instruction (typically either a NOP or an IRQ),
	// set the H-Sync and V-Sync pins as desired, then wait the given number
	// of clock cycles.
	//
	// Note: autopull should be set to 32-bits, OSR is set to shift right.
	let timing_program = pio_proc::pio_asm!(
		".wrap_target"
		// Step 1. Push next 2 bits of OSR into `pins`, to set H-Sync and V-Sync
		"out pins, 2"
		// Step 2. Push last 14 bits of OSR into X for the timing loop.
		"out x, 14"
		// Step 3. Execute bottom 16-bits of OSR as an instruction. This take two cycles.
		"out exec, 16"
		// Spin until X is zero
		"loop0:"
			"jmp x-- loop0"
		".wrap"
	);

	// This is the video pixels program. It waits for an IRQ (posted by the
	// timing loop) then pulls pixel data from the FIFO. We post the number of
	// pixels for that line, then the pixel data.
	//
	// Post <num_pixels> <pixel1> <pixel2> ... <pixelN>; each <pixelX> maps to
	// the RGB output pins. On a Neotron Pico, there are 12 (4 Red, 4 Green and
	// 4 Blue) - each value should be 12-bits long in the bottom of a 16-bit
	// word.
	//
	// Currently the FIFO supplies only the pixels, not the length value. When
	// we read the length from the FIFO as well, all hell breaks loose.
	//
	// Note autopull should be set to 32-bits, OSR is set to shift right.
	let pixel_program = pio_proc::pio_asm!(
		".wrap_target"
		// Wait for timing state machine to start visible line
		"wait 1 irq 0"
		// Read the line length (in pixel-pairs)
		"out x, 32"
		"loop1:"
			// Write out first pixel - takes 5 clocks per pixel
			"out pins, 16 [5]"
			// Write out second pixel - takes 5 clocks per pixel (allowing one clock for the jump)
			"out pins, 16 [4]"
			// Repeat until all pixel pairs sent
			"jmp x-- loop1"
		// Clear all pins after visible section
		"mov pins null"
		".wrap"
	);

	// These two state machines run thus:
	//
	// | Clock | Timing PIOSM | Pixel PIOSM      |
	// |:------|:-------------|:-----------------|
	// | 1     | out pins, 2  | wait 1 irq 0     |
	// | 2     | out x, 14    | wait 1 irq 0     |
	// | 3     | out exec, 16 | wait 1 irq 0     |
	// | 4     | <exec irq>   | wait 1 irq 0     |
	// | 5     | jmp x--      | wait 1 irq 0     |
	// | 6     |              | out x, 32        |
	// | 7     |              | out pins, 16 [4] |
	// | 8     |              | ..               |
	// | 9     |              | ..               |
	// | 10    |              | ..               |
	// | 11    |              | ..               |
	// | 12    |              | out pins, 16 [3] |
	// | 13    |              | ..               |
	// | 14    |              | ..               |
	// | 15    |              | ..               |
	// | 16    |              | jump x--         |
	//
	// Note: Credit to
	// https://gregchadwick.co.uk/blog/playing-with-the-pico-pt5/ who had a
	// very similar idea to me, but wrote it up far better than I ever could.

	let timing_installed = pio.install(&timing_program.program).unwrap();
	let (mut timing_sm, _, timing_fifo) =
		rp_pico::hal::pio::PIOBuilder::from_program(timing_installed)
			.buffers(rp_pico::hal::pio::Buffers::OnlyTx)
			.out_pins(0, 2) // H-Sync is GPIO0, V-Sync is GPIO1
			.autopull(true)
			.out_shift_direction(rp_pico::hal::pio::ShiftDirection::Right)
			.pull_threshold(32)
			.build(sm0);
	timing_sm.set_pindirs([
		(0, rp_pico::hal::pio::PinDir::Output),
		(1, rp_pico::hal::pio::PinDir::Output),
	]);

	// Important notes!
	//
	// You must not set a clock_divider (other than 1.0) on the pixel state
	// machine. You might want the pixels to be twice as wide (or mode), but
	// enabling a clock divider adds a lot of jitter (i.e. the start each
	// each line differs by some number of 151.2 MHz clock cycles).

	let pixels_installed = pio.install(&pixel_program.program).unwrap();
	let (mut pixel_sm, _, pixel_fifo) =
		rp_pico::hal::pio::PIOBuilder::from_program(pixels_installed)
			.buffers(rp_pico::hal::pio::Buffers::OnlyTx)
			.out_pins(2, 12) // Red0 is GPIO2, Blue3 is GPIO13
			.autopull(true)
			.out_shift_direction(rp_pico::hal::pio::ShiftDirection::Right)
			.pull_threshold(32) // We read all 32-bits in each FIFO word
			.build(sm1);
	pixel_sm.set_pindirs((2..=13).map(|x| (x, rp_pico::hal::pio::PinDir::Output)));

	pio.irq1().enable_sm_interrupt(1);

	// Read from the timing buffer and write to the timing FIFO. We get an
	// IRQ when the transfer is complete (i.e. when line has been fully
	// loaded).
	dma.ch[TIMING_DMA_CHAN].ch_ctrl_trig.write(|w| {
		w.data_size().size_word();
		w.incr_read().set_bit();
		w.incr_write().clear_bit();
		unsafe { w.treq_sel().bits(timing_fifo.dreq_value()) };
		unsafe { w.chain_to().bits(TIMING_DMA_CHAN as u8) };
		unsafe { w.ring_size().bits(0) };
		w.ring_sel().clear_bit();
		w.bswap().clear_bit();
		w.irq_quiet().clear_bit();
		w.en().set_bit();
		w.sniff_en().clear_bit();
		w
	});
	dma.ch[TIMING_DMA_CHAN]
		.ch_read_addr
		.write(|w| unsafe { w.bits(TIMING_BUFFER.visible_line.data.as_ptr() as usize as u32) });
	dma.ch[TIMING_DMA_CHAN]
		.ch_write_addr
		.write(|w| unsafe { w.bits(timing_fifo.fifo_address() as usize as u32) });
	dma.ch[TIMING_DMA_CHAN]
		.ch_trans_count
		.write(|w| unsafe { w.bits(TIMING_BUFFER.visible_line.data.len() as u32) });

	// Read from the pixel buffer (even first) and write to the pixel FIFO
	dma.ch[PIXEL_DMA_CHAN].ch_ctrl_trig.write(|w| {
		w.data_size().size_word();
		w.incr_read().set_bit();
		w.incr_write().clear_bit();
		unsafe { w.treq_sel().bits(pixel_fifo.dreq_value()) };
		unsafe { w.chain_to().bits(PIXEL_DMA_CHAN as u8) };
		unsafe { w.ring_size().bits(0) };
		w.ring_sel().clear_bit();
		w.bswap().clear_bit();
		w.irq_quiet().clear_bit();
		w.en().set_bit();
		w.sniff_en().clear_bit();
		w
	});
	dma.ch[PIXEL_DMA_CHAN]
		.ch_read_addr
		.write(|w| unsafe { w.bits(PIXEL_DATA_BUFFER_EVEN.as_ptr()) });
	dma.ch[PIXEL_DMA_CHAN]
		.ch_write_addr
		.write(|w| unsafe { w.bits(pixel_fifo.fifo_address() as usize as u32) });
	dma.ch[PIXEL_DMA_CHAN]
		.ch_trans_count
		.write(|w| unsafe { w.bits(PIXEL_DATA_BUFFER_EVEN.pixels.len() as u32 + 1) });

	// Enable the DMA
	dma.multi_chan_trigger
		.write(|w| unsafe { w.bits((1 << PIXEL_DMA_CHAN) | (1 << TIMING_DMA_CHAN)) });

	debug!("DMA set-up complete");

	timing_sm.start();
	pixel_sm.start();

	debug!("State Machines running");

	// We drop our state-machine and PIO objects here - this means the video
	// cannot be reconfigured at a later time, but they do keep on running
	// as-is.

	unsafe {
		for b in super::CORE1_STACK.iter_mut() {
			*b = super::CORE1_STACK_PAINT_WORD;
		}
	}

	debug!(
		"Core 1 stack: {:08x}, {} bytes",
		unsafe { super::CORE1_STACK.as_ptr() },
		unsafe { super::CORE1_STACK.len() * core::mem::size_of::<usize>() }
	);

	// No-one else is looking at this right now.
	unsafe {
		TEXT_COLOUR_LOOKUP.init(&VIDEO_PALETTE);
	}

	crate::multicore::launch_core1_with_stack(
		core1_main,
		unsafe { &mut super::CORE1_STACK },
		ppb,
		fifo,
		psm,
	);

	debug!("Core 1 running");
}

/// Gets the current video mode
pub fn get_video_mode() -> crate::common::video::Mode {
	VIDEO_MODE.get_mode()
}

/// Sets the current video mode
pub fn set_video_mode(mode: crate::common::video::Mode) -> bool {
	if test_video_mode(mode) {
		VIDEO_MODE.set_mode(mode);
		true
	} else {
		false
	}
}

/// Sets the current video mode
pub fn test_video_mode(mode: crate::common::video::Mode) -> bool {
	match (
		mode.timing(),
		mode.format(),
		mode.is_horiz_2x(),
		mode.is_vert_2x(),
	) {
		(
			crate::common::video::Timing::T640x480,
			crate::common::video::Format::Text8x16 | crate::common::video::Format::Text8x8,
			false,
			false,
		) => true,
		(
			crate::common::video::Timing::T640x400,
			crate::common::video::Format::Text8x16 | crate::common::video::Format::Text8x8,
			false,
			false,
		) => true,
		_ => false,
	}
}

/// Get the current scan line.
pub fn get_scan_line() -> u16 {
	NEXT_SCAN_LINE.load(Ordering::Relaxed)
}

/// Get how many visible lines there currently are
pub fn get_num_scan_lines() -> u16 {
	let mode = get_video_mode();
	mode.vertical_lines()
}

/// This function runs the video processing loop on Core 1.
///
/// It keeps the odd/even scan-line buffers updated, as per the contents of
/// the text buffer.
///
/// # Safety
///
/// Only run this function on Core 1.
#[link_section = ".data"]
unsafe extern "C" fn core1_main() -> u32 {
	let mut video = RenderEngine::new();

	// The LED pin was called `_pico_led` over in the `Hardware::build`
	// function that ran on Core 1. Rather than try and move the pin over to
	// this core, we just unsafely poke the GPIO registers to set/clear the
	// relevant pin.
	let gpio_out_set = 0xd000_0014 as *mut u32;
	let gpio_out_clr = 0xd000_0018 as *mut u32;

	// Enable the interrupts (DMA has to first be set by Core 0)
	cortex_m::interrupt::enable();
	// We are on Core 1, so these interrupts will run on Core 1
	crate::pac::NVIC::unpend(crate::pac::Interrupt::PIO0_IRQ_1);
	crate::pac::NVIC::unmask(crate::pac::Interrupt::PIO0_IRQ_1);

	loop {
		// Wait for a free DMA buffer. Can't do a compare-and-swap on ARMv6-M :/
		while !DRAW_THIS_LINE.load(Ordering::Acquire) {
			cortex_m::asm::wfe();
		}
		DRAW_THIS_LINE.store(false, Ordering::Relaxed);

		// The one we draw *now* is the one that is *shown* next
		let this_line = NEXT_SCAN_LINE.load(Ordering::Relaxed);

		if this_line == 0 {
			video.frame_start();
		}

		unsafe {
			// Turn on LED
			gpio_out_set.write(1 << 25);
		}

		// This function currently consumes about 70% CPU (or rather, 90% CPU
		// on each of visible lines, and 0% CPU on the other lines)
		video.draw_next_line(this_line);

		unsafe {
			// Turn off LED
			gpio_out_clr.write(1 << 25);
		}
	}
}

/// This function is called whenever the Timing PIO starts a scan-line.
///
/// Timing wise, we should be at the start of the back-porch.
///
/// # Safety
///
/// Only call this from the PIO IRQ handler.
#[link_section = ".data"]
#[interrupt]
unsafe fn PIO0_IRQ_1() {
	let pio = unsafe { &*crate::pac::PIO0::ptr() };
	let dma = unsafe { &*crate::pac::DMA::ptr() };

	// Clear the interrupt
	pio.irq.write_with_zero(|w| w.irq().bits(1 << 1));

	// This is now the line we are currently playing
	let current_timing_line = NEXT_SCAN_LINE.load(Ordering::Relaxed);
	// This is the line we should cue up to play next
	let next_timing_line = if current_timing_line == TIMING_BUFFER.back_porch_ends_at {
		// Wrap around
		0
	} else {
		// Keep going
		current_timing_line + 1
	};

	// Are we in the visible portion *right* now? If so, copy some pixels into
	// the Pixel SM FIFO using DMA. Hopefully the main thread has them ready for
	// us (though we're playing them, ready or not).
	if current_timing_line <= TIMING_BUFFER.visible_lines_ends_at {
		if (current_timing_line & 1) == 1 {
			// Load the odd line into the Pixel SM FIFO for immediate playback
			dma.ch[PIXEL_DMA_CHAN]
				.ch_al3_read_addr_trig
				.write(|w| w.bits(PIXEL_DATA_BUFFER_ODD.as_ptr()))
		} else {
			// Load the even line into the Pixel SM FIFO for immediate playback
			dma.ch[PIXEL_DMA_CHAN]
				.ch_al3_read_addr_trig
				.write(|w| w.bits(PIXEL_DATA_BUFFER_EVEN.as_ptr()))
		}
		// The data will start pouring into the FIFO, but the output is corked until
		// the timing SM generates the second interrupt, just before the visible
		// portion.
	}

	// Set this before we set the `DRAW_THIS_LINE` flag.
	NEXT_SCAN_LINE.store(next_timing_line, Ordering::Relaxed);

	// Work out what sort of sync pulses we need on the *next* scan-line, and
	// also tell the main thread what to draw ready for the *next* scan-line.
	let buffer = if next_timing_line <= TIMING_BUFFER.visible_lines_ends_at {
		// A visible line is *up next* so start drawing it *right now*.
		DRAW_THIS_LINE.store(true, Ordering::Release);
		&TIMING_BUFFER.visible_line
	} else if next_timing_line <= TIMING_BUFFER.front_porch_end_at {
		// VGA front porch before VGA sync pulse
		&TIMING_BUFFER.vblank_porch_buffer
	} else if next_timing_line <= TIMING_BUFFER.sync_pulse_ends_at {
		// Sync pulse
		&TIMING_BUFFER.vblank_sync_buffer
	} else {
		// VGA back porch following VGA sync pulse.
		&TIMING_BUFFER.vblank_porch_buffer
	};
	// Start transferring the next block of timing info into the FIFO, ready for
	// the next line. We will be back in this interrupt once it starts actually
	// playing.
	dma.ch[TIMING_DMA_CHAN]
		.ch_al3_read_addr_trig
		.write(|w| w.bits(buffer as *const _ as usize as u32));
}

// -----------------------------------------------------------------------------
// End of file
// -----------------------------------------------------------------------------
