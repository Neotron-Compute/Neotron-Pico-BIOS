//! # VGA Driver for the Neotron Pico
//!
//! VGA output on the Neotron Pico uses 14 GPIO pins and two PIO state machines.
//!
//! It can generate 640x480@60Hz and 640x400@70Hz standard VGA video, with a
//! 25.2 MHz pixel clock. The spec is 25.175 MHz, so we are 0.1% off). The
//! assumption is that the CPU is clocked at 126 MHz, i.e. 5x the pixel
//! clock. All of the PIO code relies on this assumption!
//!
//! Currently only an 80x25 two-colour text-mode is supported. Other modes will be
//! added in the future.

// -----------------------------------------------------------------------------
// Licence Statement
// -----------------------------------------------------------------------------
// Copyright (c) Jonathan 'theJPster' Pallant and the Neotron Developers, 2021
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

// -----------------------------------------------------------------------------
// Imports
// -----------------------------------------------------------------------------

use core::sync::atomic::{AtomicBool, AtomicPtr, AtomicU16, AtomicUsize, Ordering};
use defmt::{debug, trace};
use rp_pico::hal::pio::PIOExt;

// -----------------------------------------------------------------------------
// Types
// -----------------------------------------------------------------------------

/// Holds some data necessary to run the Video.
///
/// This structure is owned entirely by the main thread (or the drawing
/// thread). Data handled under interrupt is stored in various other places.
struct RenderEngine {
	/// How many frames have been drawn
	frame_count: u32,
	/// Look-up table mapping two 1-bpp pixels to two 12-bit RGB values (packed into one 32-bit word).
	///
	/// You can adjust this table to convert text to different colours.
	lookup: [RGBPair; 4],
}

/// A font
pub struct Font<'a> {
	height: usize,
	data: &'a [u8],
}

/// Holds some data necessary to present a text console.
///
/// Used by Core 0 to control writes to a shared text-buffer.
pub struct TextConsole {
	current_col: AtomicU16,
	current_row: AtomicU16,
	text_buffer: AtomicPtr<GlyphAttr>,
}

/// Describes one scan-line's worth of pixels, including the length word required by the Pixel FIFO.
#[repr(C, align(16))]
struct LineBuffer {
	/// Must be one less than the number of pixel-pairs in `pixels`
	length: u32,
	/// Pixels to be displayed, grouped into pairs (to save FIFO space and reduce DMA bandwidth)
	pixels: [RGBPair; MAX_NUM_PIXEL_PAIRS_PER_LINE],
}

/// Describes the polarity of a sync pulse.
///
/// Some pulses are positive (active-high), some are negative (active-low).
pub enum SyncPolarity {
	/// An active-high pulse
	Positive,
	/// An active-low pulse
	Negative,
}

/// Holds the four scan-line timing FIFO words we need for one scan-line.
///
/// See `make_timing` for a function which can generate these words. We DMA
/// them into the timing FIFO, so they must sit on a 16-byte boundary.
#[repr(C, align(16))]
struct ScanlineTimingBuffer {
	data: [u32; 4],
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

/// Represents a 12-bit colour value.
///
/// Each channel has four-bits, and they are packed in `BGR` format. This is
/// so the PIO can shift them out right-first, and we have RED0 assigned to
/// the lowest GPIO pin.
#[repr(transparent)]
#[derive(Copy, Clone, PartialEq, Eq)]
pub struct RGBColour(u16);

/// Represents two `RGBColour` pixels packed together.
///
/// The `first` pixel is packed in the lower 16-bits. This is because the PIO
/// shifts-right.
#[repr(transparent)]
#[derive(Copy, Clone, PartialEq, Eq)]
pub struct RGBPair(u32);

/// Represents a glyph in the current font.
#[repr(transparent)]
#[derive(Copy, Clone, PartialEq, Eq)]
pub struct Glyph(u8);

/// Represents VGA format foreground/background attributes.
#[repr(transparent)]
#[derive(Copy, Clone, PartialEq, Eq)]
pub struct Attr(u8);

/// Represents a glyph/attribute pair. This is what out text console is made
/// out of. They work in exactly the same way as IBM PC VGA.
#[repr(transparent)]
#[derive(Copy, Clone, PartialEq, Eq, Default)]
pub struct GlyphAttr(u16);

// -----------------------------------------------------------------------------
// Static and Const Data
// -----------------------------------------------------------------------------

/// How many pixels per scan-line.
///
/// Adjust the pixel PIO program to run at the right speed to the screen is
/// filled. For example, if this is only 320 but you are aiming at 640x480,
/// make the pixel PIO take twice as long per pixel.
const MAX_NUM_PIXELS_PER_LINE: usize = 640;

/// Maximum number of lines on screen.
const MAX_NUM_LINES: usize = 480;

/// How many pixel pairs we send out.
///
/// Each pixel is two 12-bit values packed into one 32-bit word(an `RGBPair`).
/// This is to make more efficient use of DMA and FIFO resources.
const MAX_NUM_PIXEL_PAIRS_PER_LINE: usize = MAX_NUM_PIXELS_PER_LINE / 2;

/// The highest number of columns in any text mode.
pub const MAX_TEXT_COLS: usize = MAX_NUM_PIXELS_PER_LINE / 8;

/// The highest number of rows in any text mode.
pub const MAX_TEXT_ROWS: usize = MAX_NUM_LINES as usize / 8;

/// Current number of visible columns.
///
/// Must be `<= MAX_TEXT_COLS`
pub static NUM_TEXT_COLS: AtomicUsize = AtomicUsize::new(80);

/// Current number of visible rows.
///
/// Must be `<= MAX_TEXT_ROWS`
pub static NUM_TEXT_ROWS: AtomicUsize = AtomicUsize::new(30);

/// Used to signal when Core 1 has started
static CORE1_START_FLAG: AtomicBool = AtomicBool::new(false);

/// Stores our timing data which we DMA into the timing PIO State Machine
static mut TIMING_BUFFER: TimingBuffer = TimingBuffer::make_640x480();

/// Stores which mode we are in
static mut VIDEO_MODE: crate::common::video::Mode = crate::common::video::Mode::new(
	crate::common::video::Timing::T640x480,
	crate::common::video::Format::Text8x16,
);

/// Tracks which scan-line we are currently on (for timing purposes => it goes 0..`TIMING_BUFFER.back_porch_ends_at`)
static CURRENT_TIMING_LINE: AtomicU16 = AtomicU16::new(0);

/// Tracks which scan-line we are currently on (for pixel purposes => it goes 0..NUM_LINES)
static CURRENT_DISPLAY_LINE: AtomicU16 = AtomicU16::new(0);

/// Set to `true` when DMA of previous line is complete and next line is scheduled.
static DMA_READY: AtomicBool = AtomicBool::new(false);

/// Somewhere to stash the DMA controller object, so the IRQ can find it
static mut DMA_PERIPH: Option<super::pac::DMA> = None;

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
	pixels: [RGBPair::from_pixels(colours::WHITE, colours::BLACK); MAX_NUM_PIXEL_PAIRS_PER_LINE],
};

/// One scan-line's worth of 12-bit pixels, used for the odd scan-lines (1, 3, 5 ... NUM_LINES-1).
///
/// Gets read by DMA, which pushes them into the pixel state machine's FIFO.
///
/// Gets written to by `RenderEngine` running on Core 1.
static mut PIXEL_DATA_BUFFER_ODD: LineBuffer = LineBuffer {
	length: (MAX_NUM_PIXEL_PAIRS_PER_LINE as u32) - 1,
	pixels: [RGBPair::from_pixels(colours::BLACK, colours::WHITE); MAX_NUM_PIXEL_PAIRS_PER_LINE],
};

/// This is our text buffer.
///
/// This is arranged as `NUM_TEXT_ROWS` rows of `NUM_TEXT_COLS` columns. Each
/// item is an index into `font16::FONT_DATA` plus an 8-bit attribute.
///
/// Written to by Core 0, and read from by `RenderEngine` running on Core 1.
pub static mut GLYPH_ATTR_ARRAY: [GlyphAttr; MAX_TEXT_COLS * MAX_TEXT_ROWS] =
	[GlyphAttr(0); MAX_TEXT_COLS * MAX_TEXT_ROWS];

/// Core 1 entry function.
///
/// This is a naked function I have pre-compiled to thumb-2 instructions. I
/// could use inline assembler, but then I'd have to make you install
/// arm-none-eabi-as or arm-none-eabi-gcc, or wait until `llvm_asm!` is
/// stablised.
///
/// We want this function to load the three parameters (which we placed in
/// Core 1's stack) into registers. We then jump to the third of these, which
/// effectively makes a function call with the first two values as function
/// arguments.
static CORE1_ENTRY_FUNCTION: [u16; 2] = [
	0xbd03, // pop {r0, r1, pc}
	0x46c0, // nop - pad this out to 32-bits long
];

/// A set of useful constants representing common RGB colours.
pub mod colours {
	pub const BLACK: super::RGBColour = super::RGBColour::from_24bit(0x00, 0x00, 0x00);
	pub const DARK_GRAY: super::RGBColour = super::RGBColour::from_24bit(0x80, 0x80, 0x80);
	pub const BLUE: super::RGBColour = super::RGBColour::from_24bit(0x00, 0x00, 0x80);
	pub const LIGHT_BLUE: super::RGBColour = super::RGBColour::from_24bit(0x00, 0x00, 0xF0);
	pub const GREEN: super::RGBColour = super::RGBColour::from_24bit(0x00, 0x80, 0x00);
	pub const LIGHT_GREEN: super::RGBColour = super::RGBColour::from_24bit(0x00, 0xF0, 0x00);
	pub const CYAN: super::RGBColour = super::RGBColour::from_24bit(0x00, 0x80, 0x80);
	pub const LIGHT_CYAN: super::RGBColour = super::RGBColour::from_24bit(0x00, 0xF0, 0xF0);
	pub const RED: super::RGBColour = super::RGBColour::from_24bit(0x80, 0x00, 0x00);
	pub const LIGHT_RED: super::RGBColour = super::RGBColour::from_24bit(0xF0, 0x00, 0x00);
	pub const MAGENTA: super::RGBColour = super::RGBColour::from_24bit(0x80, 0x00, 0x80);
	pub const LIGHT_MAGENTA: super::RGBColour = super::RGBColour::from_24bit(0xF0, 0x00, 0xF0);
	pub const BROWN: super::RGBColour = super::RGBColour::from_24bit(0x80, 0x80, 0x00);
	pub const YELLOW: super::RGBColour = super::RGBColour::from_24bit(0xF0, 0xF0, 0x00);
	pub const LIGHT_GRAY: super::RGBColour = super::RGBColour::from_24bit(0xA0, 0xA0, 0xA0);
	pub const WHITE: super::RGBColour = super::RGBColour::from_24bit(0xF0, 0xF0, 0xF0);
}

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

	// This is the video pixels program. It waits for an IRQ
	// (posted by the timing loop) then pulls pixel data from the FIFO. We post
	// the number of pixels for that line, then the pixel data.
	//
	// Post <num_pixels> <pixel1> <pixel2> ... <pixelN>; each <pixelX> maps to
	// the RGB output pins. On a Neotron Pico, there are 12 (4 Red, 4 Green and
	// 4 Blue) - so we set autopull to 12, and each value should be 12-bits long.
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
			"out pins, 16 [4]"
			// Write out second pixel - takes 5 clocks per pixel (allowing one clock for the jump)
			"out pins, 16 [3]"
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
	// each line differs by some number of 126 MHz clock cycles).

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
	dma.inte0.write(|w| unsafe {
		w.inte0()
			.bits((1 << PIXEL_DMA_CHAN) | (1 << TIMING_DMA_CHAN))
	});

	// Enable the DMA
	dma.multi_chan_trigger
		.write(|w| unsafe { w.bits((1 << PIXEL_DMA_CHAN) | (1 << TIMING_DMA_CHAN)) });

	debug!("DMA enabled");

	unsafe {
		// Hand off the DMA peripheral to the interrupt
		DMA_PERIPH = Some(dma);
		// We don't enable the interrupts here - we enable them on core 1
	}

	debug!("DMA set-up complete");

	timing_sm.start();
	pixel_sm.start();

	debug!("State Machines running");

	// We drop our state-machine and PIO objects here - this means the video
	// cannot be reconfigured at a later time, but they do keep on running
	// as-is.

	let core1_stack: &'static mut [usize] = unsafe {
		extern "C" {
			static mut _core1_stack_bottom: usize;
			static mut _core1_stack_len: usize;
		}
		core::slice::from_raw_parts_mut(
			&mut _core1_stack_bottom as *mut _,
			&mut _core1_stack_len as *const _ as usize / 4,
		)
	};

	debug!(
		"Core 1 stack: {:08x}, {} bytes",
		core1_stack.as_ptr(),
		core1_stack.len()
	);

	multicore_launch_core1_with_stack(core1_main, core1_stack, ppb, fifo, psm);

	debug!("Core 1 running");
}

/// The bootrom code will call this function on core1 to perform any set-up, before the
/// entry function is called.
extern "C" fn core1_wrapper(entry_func: extern "C" fn() -> u32, _stack_base: *mut u32) -> u32 {
	entry_func()
}

/// Starts core 1 running the given function, with the given stack.
fn multicore_launch_core1_with_stack(
	main_func: unsafe extern "C" fn() -> u32,
	stack: &mut [usize],
	ppb: &mut crate::pac::PPB,
	fifo: &mut rp_pico::hal::sio::SioFifo,
	psm: &mut crate::pac::PSM,
) {
	debug!("Resetting CPU1...");

	psm.frce_off.modify(|_, w| w.proc1().set_bit());
	while !psm.frce_off.read().proc1().bit_is_set() {
		cortex_m::asm::nop();
	}
	psm.frce_off.modify(|_, w| w.proc1().clear_bit());

	debug!("Setting up stack...");

	// Gets popped into `r0` by CORE1_ENTRY_FUNCTION. This is the `main`
	// function we want to run. It appears in the call to `core1_wrapper` as
	// the first argument.
	stack[stack.len() - 3] = main_func as *const () as usize;
	// Gets popped into `r1` by CORE1_ENTRY_FUNCTION. This is the top of stack
	// for Core 1. It appears in the call to `core1_wrapper` as the second
	// argument.
	stack[stack.len() - 2] = stack.as_ptr() as *const _ as usize;
	// Gets popped into `pc` by CORE1_ENTRY_FUNCTION. This is the function
	// `CORE1_ENTRY_FUNCTION` will jump to, passing the above two values as
	// arguments.
	stack[stack.len() - 1] = core1_wrapper as *const () as usize;
	// Point into the top of the stack (so there are three values pushed onto
	// it, i.e. at/above it)
	let stack_ptr = unsafe { stack.as_mut_ptr().add(stack.len() - 3) };

	debug!("Stack ptr is 0x{:x}", stack_ptr);
	debug!("Stack bottom is 0x{:x}", stack.as_ptr());
	debug!("Stack top is 0x{:x}", &stack[stack.len() - 4..stack.len()]);

	// This is the launch sequence we send to core1, to get it to leave the
	// boot ROM and run our code.
	let cmd_sequence: [u32; 6] = [
		0,
		0,
		1,
		ppb.vtor.read().bits() as usize as u32,
		stack_ptr as usize as u32,
		// Have to add 1 to convert from an array pointer to a thumb instruction pointer
		(CORE1_ENTRY_FUNCTION.as_ptr() as usize as u32) + 1,
	];

	let enabled = crate::pac::NVIC::is_enabled(crate::pac::Interrupt::SIO_IRQ_PROC0);
	crate::pac::NVIC::mask(crate::pac::Interrupt::SIO_IRQ_PROC0);

	'outer: loop {
		for cmd in cmd_sequence.iter() {
			debug!("Sending command {:x}...", *cmd);

			// we drain before sending a 0
			if *cmd == 0 {
				debug!("Draining FIFO...");
				fifo.drain();
				// core 1 may be waiting for fifo space
				cortex_m::asm::sev();
			}
			debug!("Pushing to FIFO...");
			fifo.write_blocking(*cmd);

			debug!("Getting response from FIFO...");
			let response = loop {
				if let Some(x) = fifo.read() {
					break x;
				} else {
					debug!("ST is {:x}", fifo.status());
				}
			};

			// move to next state on correct response otherwise start over
			debug!("Got {:x}", response);
			if *cmd != response {
				continue 'outer;
			}
		}
		break;
	}

	if enabled {
		unsafe { crate::pac::NVIC::unmask(crate::pac::Interrupt::SIO_IRQ_PROC0) };
	}

	debug!("Waiting for Core 1 to start...");
	while !CORE1_START_FLAG.load(Ordering::Relaxed) {
		cortex_m::asm::nop();
	}
	debug!("Core 1 started!!");
}

/// Gets the current video mode
pub fn get_video_mode() -> crate::common::video::Mode {
	unsafe { VIDEO_MODE }
}

/// Sets the current video mode
pub fn set_video_mode(mode: crate::common::video::Mode) -> bool {
	cortex_m::interrupt::disable();
	let mode_ok = match (
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
		) => {
			unsafe {
				VIDEO_MODE = mode;
				TIMING_BUFFER = TimingBuffer::make_640x480();
			}
			true
		}
		(
			crate::common::video::Timing::T640x400,
			crate::common::video::Format::Text8x16 | crate::common::video::Format::Text8x8,
			false,
			false,
		) => {
			unsafe {
				VIDEO_MODE = mode;
				TIMING_BUFFER = TimingBuffer::make_640x400();
			}
			true
		}
		_ => false,
	};
	if mode_ok {
		NUM_TEXT_COLS.store(mode.text_width().unwrap_or(0) as usize, Ordering::SeqCst);
		NUM_TEXT_ROWS.store(mode.text_height().unwrap_or(0) as usize, Ordering::SeqCst);
	}
	unsafe {
		cortex_m::interrupt::enable();
	}
	mode_ok
}

/// Get the current scan line.
pub fn get_scan_line() -> u16 {
	CURRENT_DISPLAY_LINE.load(Ordering::Relaxed)
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
unsafe extern "C" fn core1_main() -> u32 {
	CORE1_START_FLAG.store(true, Ordering::Relaxed);

	// Enable the interrupts (DMA_PERIPH has to be set first by Core 0)
	cortex_m::interrupt::enable();
	// We are on Core 1, so these interrupts will run on Core 1
	crate::pac::NVIC::unpend(crate::pac::Interrupt::DMA_IRQ_0);
	crate::pac::NVIC::unmask(crate::pac::Interrupt::DMA_IRQ_0);

	debug!("IRQs enabled");

	let mut video = RenderEngine::new();

	loop {
		// This function currently consumes about 70% CPU (or rather, 90% CPU
		// on each of 400 lines, and 0% CPU on the other 50 lines)
		video.poll();
	}
}

/// Call this function whenever the DMA reports that it has completed a transfer.
///
/// We use this as a prompt to either start a transfer or more Timing words,
/// or a transfer or more pixel words.
///
/// # Safety
///
/// Only call this from the DMA IRQ handler.
pub unsafe fn irq() {
	let dma: &mut super::pac::DMA = match DMA_PERIPH.as_mut() {
		Some(dma) => dma,
		None => {
			return;
		}
	};
	let status = dma.ints0.read().bits();

	// Check if this is a DMA interrupt for the sync DMA channel
	let timing_dma_chan_irq = (status & (1 << TIMING_DMA_CHAN)) != 0;

	// Check if this is a DMA interrupt for the line DMA channel
	let pixel_dma_chan_irq = (status & (1 << PIXEL_DMA_CHAN)) != 0;

	if timing_dma_chan_irq {
		// clear timing_dma_chan bit in DMA interrupt bitfield
		dma.ints0.write(|w| w.bits(1 << TIMING_DMA_CHAN));

		let old_timing_line = CURRENT_TIMING_LINE.load(Ordering::Relaxed);
		let next_timing_line = if old_timing_line == TIMING_BUFFER.back_porch_ends_at {
			// Wrap around
			0
		} else {
			// Keep going
			old_timing_line + 1
		};
		CURRENT_TIMING_LINE.store(next_timing_line, Ordering::Relaxed);

		let buffer = if next_timing_line <= TIMING_BUFFER.visible_lines_ends_at {
			// Visible lines
			&TIMING_BUFFER.visible_line
		} else if next_timing_line <= TIMING_BUFFER.front_porch_end_at {
			// VGA front porch before VGA sync pulse
			&TIMING_BUFFER.vblank_porch_buffer
		} else if next_timing_line <= TIMING_BUFFER.sync_pulse_ends_at {
			// Sync pulse
			&TIMING_BUFFER.vblank_sync_buffer
		} else {
			// VGA back porch following VGA sync pulse
			&TIMING_BUFFER.vblank_porch_buffer
		};
		dma.ch[TIMING_DMA_CHAN]
			.ch_al3_read_addr_trig
			.write(|w| w.bits(buffer as *const _ as usize as u32))
	}

	if pixel_dma_chan_irq {
		dma.ints0.write(|w| w.bits(1 << PIXEL_DMA_CHAN));

		// A pixel DMA transfer is now complete. This only fires on visible lines.

		let mut next_display_line = CURRENT_DISPLAY_LINE.load(Ordering::Relaxed) + 1;
		if next_display_line > TIMING_BUFFER.visible_lines_ends_at {
			next_display_line = 0;
		};

		// Set the DMA load address according to which line we are on. We use
		// the 'trigger' alias to restart the DMA at the same time as we
		// write the new read address. The DMA had stopped because the
		// previous line was transferred completely.
		if (next_display_line & 1) == 1 {
			// Odd visible line is next
			dma.ch[PIXEL_DMA_CHAN]
				.ch_al3_read_addr_trig
				.write(|w| w.bits(PIXEL_DATA_BUFFER_ODD.as_ptr()))
		} else {
			// Even visible line is next
			dma.ch[PIXEL_DMA_CHAN]
				.ch_al3_read_addr_trig
				.write(|w| w.bits(PIXEL_DATA_BUFFER_EVEN.as_ptr()))
		}

		CURRENT_DISPLAY_LINE.store(next_display_line, Ordering::Relaxed);
		DMA_READY.store(true, Ordering::Relaxed);
	}
}

impl RenderEngine {
	// Initialise the main-thread resources
	pub fn new() -> RenderEngine {
		RenderEngine {
			frame_count: 0,
			lookup: [
				RGBPair::from_pixels(colours::BLUE, colours::BLUE),
				RGBPair::from_pixels(colours::BLUE, colours::WHITE),
				RGBPair::from_pixels(colours::WHITE, colours::BLUE),
				RGBPair::from_pixels(colours::WHITE, colours::WHITE),
			],
		}
	}

	pub fn poll(&mut self) {
		if DMA_READY.load(Ordering::Relaxed) {
			DMA_READY.store(false, Ordering::Relaxed);
			let current_line_num = CURRENT_DISPLAY_LINE.load(Ordering::Relaxed);
			if current_line_num == 0 {
				trace!("Frame {}", self.frame_count);
				self.frame_count += 1;
			}

			// new line - pick a buffer to draw into (not the one that is currently rendering!)
			let scan_line_buffer = unsafe {
				if (current_line_num & 1) == 0 {
					&mut PIXEL_DATA_BUFFER_ODD
				} else {
					&mut PIXEL_DATA_BUFFER_EVEN
				}
			};

			let font = match unsafe { VIDEO_MODE.format() } {
				crate::common::video::Format::Text8x16 => &font16::FONT,
				crate::common::video::Format::Text8x8 => &font8::FONT,
				_ => {
					return;
				}
			};

			let num_rows = NUM_TEXT_ROWS.load(Ordering::Relaxed);
			let num_cols = NUM_TEXT_COLS.load(Ordering::Relaxed);

			// Convert our position in scan-lines to a text row, and a line within each glyph on that row
			let text_row = current_line_num as usize / font.height;
			let font_row = current_line_num as usize % font.height;

			if text_row < num_rows {
				// Note (unsafe): We could stash the char array inside `self`
				// but at some point we are going to need one CPU rendering
				// the text, and the other CPU running code and writing to
				// the buffer. This might be Undefined Behaviour, but
				// unfortunately real-time video is all about shared mutable
				// state. At least our platform is fixed, so we can simply
				// test if it works, for some given version of the Rust compiler.
				let row_slice = unsafe {
					&GLYPH_ATTR_ARRAY[(text_row * num_cols)..((text_row + 1) * num_cols)]
				};
				// Every font look-up we are about to do for this row will
				// involve offsetting by the row within each glyph. As this
				// is the same for every glyph on this row, we calculate a
				// new pointer once, in advance, and save ourselves an
				// addition each time around the loop.
				let font_ptr = unsafe { font.data.as_ptr().add(font_row) };

				// Get a pointer into our scan-line buffer
				let scan_line_buffer_ptr = scan_line_buffer.pixels.as_mut_ptr();
				let mut px_idx = 0;

				// Convert from characters to coloured pixels, using the font as a look-up table.
				for glyphattr in row_slice.iter() {
					let index = (glyphattr.glyph().0 as isize) * font.height as isize;
					// Note (unsafe): We use pointer arithmetic here because we
					// can't afford a bounds-check on an array. This is safe
					// because the font is `256 * width` bytes long and we can't
					// index more than `255 * width` bytes into it.
					let mono_pixels = unsafe { *font_ptr.offset(index) } as usize;
					// Convert from eight mono pixels in one byte to four RGB
					// pairs. Hopefully the `& 3` elides the panic calls.
					unsafe {
						core::ptr::write_volatile(
							scan_line_buffer_ptr.offset(px_idx),
							self.lookup[(mono_pixels >> 6) & 3],
						);
						core::ptr::write_volatile(
							scan_line_buffer_ptr.offset(px_idx + 1),
							self.lookup[(mono_pixels >> 4) & 3],
						);
						core::ptr::write_volatile(
							scan_line_buffer_ptr.offset(px_idx + 2),
							self.lookup[(mono_pixels >> 2) & 3],
						);
						core::ptr::write_volatile(
							scan_line_buffer_ptr.offset(px_idx + 3),
							self.lookup[mono_pixels & 3],
						);
					}
					px_idx += 4;
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

impl TextConsole {
	/// Create a TextConsole.
	///
	/// Has no buffer associated with it
	pub const fn new() -> TextConsole {
		TextConsole {
			current_row: AtomicU16::new(0),
			current_col: AtomicU16::new(0),
			text_buffer: AtomicPtr::new(core::ptr::null_mut()),
		}
	}

	/// Update the text buffer we are using.
	///
	/// Will reset the cursor. The screen is not cleared.
	pub fn set_text_buffer(
		&self,
		text_buffer: &'static mut [GlyphAttr; MAX_TEXT_ROWS * MAX_TEXT_COLS],
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
			self.current_row.store(row as u16, Ordering::Relaxed);
			self.current_col.store(col as u16, Ordering::Relaxed);
		}
	}

	/// Moves the text cursor to the specified row and column.
	///
	/// If a value is out of bounds, the cursor is not moved in that axis.
	pub fn move_to(&self, row: u16, col: u16) {
		if (row as usize) < NUM_TEXT_ROWS.load(Ordering::Relaxed) {
			self.current_row.store(row, Ordering::Relaxed);
		}
		if (col as usize) < NUM_TEXT_COLS.load(Ordering::Relaxed) {
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
	/// The character is relative to the current font.
	fn write_at(&self, glyph: Glyph, buffer: *mut GlyphAttr, row: &mut u16, col: &mut u16) {
		let num_rows = NUM_TEXT_ROWS.load(Ordering::Relaxed);
		let num_cols = NUM_TEXT_COLS.load(Ordering::Relaxed);

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
					.write_volatile(GlyphAttr::new(glyph, Attr(0)))
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

			unsafe {
				core::ptr::copy(
					buffer.add(num_cols as usize),
					buffer,
					num_cols * (num_rows - 1),
				)
			};

			for blank_col in 0..num_cols {
				let offset = (blank_col as usize) + (num_cols * (*row as usize));
				unsafe {
					buffer
						.add(offset)
						.write_volatile(GlyphAttr::new(Glyph(b' '), Attr(0)))
				};
			}
		}
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
			self.current_row.store(row as u16, Ordering::Relaxed);
			self.current_col.store(col as u16, Ordering::Relaxed);
		}

		Ok(())
	}
}

impl LineBuffer {
	/// Convert the line buffer to a 32-bit address that the DMA engine understands.
	fn as_ptr(&self) -> u32 {
		self as *const _ as usize as u32
	}
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

impl ScanlineTimingBuffer {
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
				Self::make_timing(timings.0 * 5, hsync.disabled(), vsync.disabled(), false),
				// Sync pulse (as per the spec)
				Self::make_timing(timings.1 * 5, hsync.enabled(), vsync.disabled(), false),
				// Back porch. Adjusted by a few clocks to account for interrupt +
				// PIO SM start latency.
				Self::make_timing(
					(timings.2 * 5) - 5,
					hsync.disabled(),
					vsync.disabled(),
					false,
				),
				// Visible portion. It also triggers the IRQ to start pixels
				// moving. Adjusted to compensate for changes made to previous
				// period to ensure scan-line remains at correct length.
				Self::make_timing(
					(timings.3 * 5) + 5,
					hsync.disabled(),
					vsync.disabled(),
					true,
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
				Self::make_timing(timings.0 * 5, hsync.disabled(), vsync.disabled(), false),
				// Sync pulse (as per the spec)
				Self::make_timing(timings.1 * 5, hsync.enabled(), vsync.disabled(), false),
				// Back porch.
				Self::make_timing(timings.2 * 5, hsync.disabled(), vsync.disabled(), false),
				// Visible portion.
				Self::make_timing(timings.3 * 5, hsync.disabled(), vsync.disabled(), false),
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
				Self::make_timing(timings.0 * 5, hsync.disabled(), vsync.enabled(), false),
				// Sync pulse (as per the spec)
				Self::make_timing(timings.1 * 5, hsync.enabled(), vsync.enabled(), false),
				// Back porch.
				Self::make_timing(timings.2 * 5, hsync.disabled(), vsync.enabled(), false),
				// Visible portion.
				Self::make_timing(timings.3 * 5, hsync.disabled(), vsync.enabled(), false),
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
	const fn make_timing(period: u32, hsync: bool, vsync: bool, raise_irq: bool) -> u32 {
		let command = if raise_irq {
			// This command sets IRQ 0. It is the same as:
			//
			// ```
			// pio::InstructionOperands::IRQ {
			// 	clear: false,
			// 	wait: false,
			// 	index: 0,
			// 	relative: false,
			// }.encode()
			// ```
			//
			// Unfortunately encoding this isn't a const-fn, so we cheat:
			0xc000
		} else {
			// This command is a no-op (it moves Y into Y)
			//
			// ```
			// pio::InstructionOperands::MOV {
			// 	destination: pio::MovDestination::Y,
			// 	op: pio::MovOperation::None,
			// 	source: pio::MovSource::Y,
			// }.encode()
			// ```
			//
			// Unfortunately encoding this isn't a const-fn, so we cheat:
			0xa042
		};
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

impl TimingBuffer {
	/// Make a timing buffer suitable for 640 x 400 @ 70 Hz
	pub const fn make_640x400() -> TimingBuffer {
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

impl RGBColour {
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
}

impl RGBPair {
	pub const fn from_pixels(first: RGBColour, second: RGBColour) -> RGBPair {
		let first: u32 = first.0 as u32;
		let second: u32 = second.0 as u32;
		RGBPair((second << 16) | first)
	}
}

impl GlyphAttr {
	/// Make a new glyph/attribute pair.
	pub const fn new(glyph: Glyph, attr: Attr) -> GlyphAttr {
		let value: u16 = (glyph.0 as u16) + ((attr.0 as u16) << 8);
		GlyphAttr(value)
	}

	/// Get the glyph component of this pair.
	pub const fn glyph(self) -> Glyph {
		Glyph(self.0 as u8)
	}

	/// Get the attribute component of this pair.
	pub const fn attr(self) -> Attr {
		Attr((self.0 >> 8) as u8)
	}
}

// -----------------------------------------------------------------------------
// End of file
// -----------------------------------------------------------------------------
