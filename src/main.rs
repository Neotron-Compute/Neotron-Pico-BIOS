//! # Neotron Pico BIOS
//!
//! This is the BIOS for the Neotron Pico. It:
//!
//! * initialises the hardware on the Neotron Pico,
//! * loads the Neotron OS (or any other compatible OS), and
//! * implements the Neotron Common BIOS API, to provide hardware abstraction
//!   services for the OS.
//!
//! The BIOS is started by having standard Cortex-M Interrupt Vector Table at
//! address `0x1000_0100`. This IVT is found and jumped to by the RP2040 boot
//! block(`0x1000_0000` to `0x1000_00FF`).

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

#![no_std]
#![no_main]

// -----------------------------------------------------------------------------
// Imports
// -----------------------------------------------------------------------------

use core::sync::atomic::{AtomicU16, Ordering};
use cortex_m_rt::entry;
use defmt::*;
use defmt_rtt as _;
use embedded_time::rate::*;
use git_version::git_version;
use hal::clocks::Clock;
use panic_probe as _;
use pico::{
	self,
	hal::{
		self,
		pac::{self, interrupt},
		pio::{self as hal_pio, PIOExt},
	},
};

// -----------------------------------------------------------------------------
// Types
// -----------------------------------------------------------------------------

#[repr(C, align(16))]
struct ScanlineTimingBuffer {
	data: [u32; 4],
}

#[repr(C, align(16))]
struct TimingBuffer {
	visible_line: ScanlineTimingBuffer,
	vblank_porch_buffer: ScanlineTimingBuffer,
	vblank_sync_buffer: ScanlineTimingBuffer,
}

const NUM_PIXEL_PAIRS: usize = 32;

#[repr(C, align(16))]
struct LineBuffer {
	pixels: [u32; NUM_PIXEL_PAIRS + 1],
}

// -----------------------------------------------------------------------------
// Static and Const Data
// -----------------------------------------------------------------------------

/// This is the standard RP2040 bootloader. It must be stored in the first 256
/// bytes of the external SPI Flash chip. It will map the external SPI flash
/// chip to address `0x1000_0000` and jump to an Interrupt Vector Table at
/// address `0x1000_0100` (i.e. immediately after the bootloader).
///
/// See `memory.x` for a definition of the `.boot2` section.
#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER;

/// BIOS version
const GIT_VERSION: &str = git_version!();

/// Stores our timing data which we DMA into the timing PIO State Machine
static TIMING_BUFFER: TimingBuffer = TimingBuffer {
	// Note - the order of the arguments to `make_timing` is:
	//
	// * H-Sync Pulse (true = high, false = low)
	// * V-Sync Pulse (true = high, false = low)
	// * Generate pixel start IRQ (true or false)
	visible_line: ScanlineTimingBuffer {
		data: [
			// Front porch (as per the spec)
			make_timing(16 * 5, true, false, false),
			// Sync pulse (as per the spec)
			make_timing(96 * 5, false, false, false),
			// Back porch (shortened by two pixels because the
			// video starts two pixels late)
			make_timing(48 * 5, true, false, false),
			// Visible portion. It also triggers the IRQ to start pixels moving.
			make_timing(640 * 5, true, false, true),
		],
	},
	vblank_porch_buffer: ScanlineTimingBuffer {
		data: [
			// Front porch
			make_timing(16 * 5, true, false, false),
			// Sync pulse
			make_timing(96 * 5, false, false, false),
			// Back porch
			make_timing(48 * 5, true, false, false),
			// 'visible' portion (but it's blank)
			make_timing(640 * 5, true, false, false),
		],
	},
	vblank_sync_buffer: ScanlineTimingBuffer {
		data: [
			// Front porch
			make_timing(16 * 5, true, true, false),
			// Sync pulse
			make_timing(96 * 5, false, true, false),
			// Back porch
			make_timing(48 * 5, true, true, false),
			// Visible portion (but it's blank)
			make_timing(640 * 5, true, true, false),
		],
	},
};

/// Tracks which scanline we are currently on (for timing purposes => it goes 0..=524)
static CURRENT_TIMING_LINE: AtomicU16 = AtomicU16::new(0);

/// Tracks which scanline we are currently on (for pixel purposes => it goes 0..=479)
static CURRENT_DISPLAY_LINE: AtomicU16 = AtomicU16::new(0);

/// Somewhere to stash the DMA controller object, so the IRQ can find it
static mut DMA_PERIPH: Option<pac::DMA> = None;

/// DMA channel for the timing FIFO
static mut TIMING_DMA_CHAN: usize = 0;

/// DMA channel for the pixel FIFO
static mut PIXEL_DMA_CHAN: usize = 1;

/// 12-bit pixels for the even scan-lines (0, 2, 4 ... 478). Defaults to black.
static mut PIXEL_DATA_BUFFER_EVEN: LineBuffer = LineBuffer {
	// The length is one less than the number of pixel pairs because of the
	// way the jmp works in the PIO program (it's a do-while loop).
	pixels: [0; NUM_PIXEL_PAIRS + 1],
};

/// 12-bit pixels for the odd scan-lines (1, 3, 5 ... 479). Defaults to white.
static mut PIXEL_DATA_BUFFER_ODD: LineBuffer = LineBuffer {
	// The length is one less than the number of pixel pairs because of the
	// way the jmp works in the PIO program (it's a do-while loop).
	pixels: [0; NUM_PIXEL_PAIRS + 1],
};

// -----------------------------------------------------------------------------
// Functions
// -----------------------------------------------------------------------------

/// This is the entry-point to the BIOS. It is called by cortex-m-rt once the
/// `.bss` and `.data` sections have been initialised.
#[entry]
fn main() -> ! {
	info!("Neotron BIOS {} starting...", GIT_VERSION);

	// Grab the singleton containing all the RP2040 peripherals
	let mut pac = pac::Peripherals::take().unwrap();
	// Grab the singleton containing all the generic Cortex-M peripherals
	let core = pac::CorePeripherals::take().unwrap();

	// Reset the DMA engine
	pac.RESETS.reset.modify(|_, w| w.dma().clear_bit());
	while pac.RESETS.reset_done.read().dma().bit_is_clear() {}

	// Needed by the clock setup
	let mut watchdog = hal::watchdog::Watchdog::new(pac.WATCHDOG);

	// Run at 126 MHz SYS_PLL, 48 MHz, USB_PLL

	let xosc = hal::xosc::setup_xosc_blocking(pac.XOSC, pico::XOSC_CRYSTAL_FREQ.Hz())
		.map_err(|_x| false)
		.unwrap();

	// Configure watchdog tick generation to tick over every microsecond
	watchdog.enable_tick_generation((pico::XOSC_CRYSTAL_FREQ / 1_000_000) as u8);

	let mut clocks = hal::clocks::ClocksManager::new(pac.CLOCKS);

	let pll_sys = hal::pll::setup_pll_blocking(
		pac.PLL_SYS,
		xosc.operating_frequency().into(),
		hal::pll::PLLConfig {
			vco_freq: Megahertz(1512),
			refdiv: 1,
			post_div1: 6,
			post_div2: 2,
		},
		&mut clocks,
		&mut pac.RESETS,
	)
	.map_err(|_x| false)
	.unwrap();

	let pll_usb = hal::pll::setup_pll_blocking(
		pac.PLL_USB,
		xosc.operating_frequency().into(),
		hal::pll::common_configs::PLL_USB_48MHZ,
		&mut clocks,
		&mut pac.RESETS,
	)
	.map_err(|_x| false)
	.unwrap();

	clocks
		.init_default(&xosc, &pll_sys, &pll_usb)
		.map_err(|_x| false)
		.unwrap();

	info!("Clocks OK");

	// Create an object we can use to busy-wait for specified numbers of
	// milliseconds. For this to work, it needs to know our clock speed.
	let _delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());

	// sio is the *Single-cycle Input/Output* peripheral. It has all our GPIO
	// pins, as well as some mailboxes and other useful things for inter-core
	// communications.
	let sio = hal::sio::Sio::new(pac.SIO);

	// Configure and grab all the RP2040 pins the Pico exposes.
	let pins = pico::Pins::new(
		pac.IO_BANK0,
		pac.PADS_BANK0,
		sio.gpio_bank0,
		&mut pac.RESETS,
	);

	info!("Pins OK");

	// Give H-Sync, V-Sync and 12 RGB colour pins to PIO0 to output video
	let _h_sync = pins.gpio0.into_mode::<hal::gpio::FunctionPio0>();
	let _v_sync = pins.gpio1.into_mode::<hal::gpio::FunctionPio0>();
	let _red0 = pins.gpio2.into_mode::<hal::gpio::FunctionPio0>();
	let _red1 = pins.gpio3.into_mode::<hal::gpio::FunctionPio0>();
	let _red2 = pins.gpio4.into_mode::<hal::gpio::FunctionPio0>();
	let _red3 = pins.gpio5.into_mode::<hal::gpio::FunctionPio0>();
	let _green0 = pins.gpio6.into_mode::<hal::gpio::FunctionPio0>();
	let _green1 = pins.gpio7.into_mode::<hal::gpio::FunctionPio0>();
	let _green2 = pins.gpio8.into_mode::<hal::gpio::FunctionPio0>();
	let _green3 = pins.gpio9.into_mode::<hal::gpio::FunctionPio0>();
	let _blue0 = pins.gpio10.into_mode::<hal::gpio::FunctionPio0>();
	let _blue1 = pins.gpio11.into_mode::<hal::gpio::FunctionPio0>();
	let _blue2 = pins.gpio12.into_mode::<hal::gpio::FunctionPio0>();
	let _blue3 = pins.gpio13.into_mode::<hal::gpio::FunctionPio0>();

	// Set up a checker board in the line buffers
	unsafe {
		PIXEL_DATA_BUFFER_EVEN.pixels[0] = NUM_PIXEL_PAIRS as u32 - 1;
		for px in PIXEL_DATA_BUFFER_EVEN.pixels[1..].iter_mut() {
			*px = 0x0FFF_0000;
		}
		PIXEL_DATA_BUFFER_ODD.pixels[0] = NUM_PIXEL_PAIRS as u32 - 1;
		for px in PIXEL_DATA_BUFFER_ODD.pixels[1..].iter_mut() {
			*px = 0x0000_0FFF;
		}
	}

	// Grab PIO0 and the state machines it contains
	let (mut pio, sm0, sm1, _sm2, _sm3) = pac.PIO0.split(&mut pac.RESETS);

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
	let timing_program = pio_proc::pio!(
		32,
		"
		.wrap_target
		; Step 1. Push next 2 bits of OSR into `pins`, to set H-Sync and V-Sync
		out pins, 2
		; Step 2. Push last 14 bits of OSR into X for the timing loop.
		out x, 14
		; Step 3. Execute bottom 16-bits of OSR as an instruction. This take two cycles.
		out exec, 16
		loop0:
			; Spin until X is zero
			jmp x-- loop0
		.wrap
		"
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
	let pixel_program = pio_proc::pio!(
		32,
		"
		.wrap_target
		; Wait for timing state machine to start visible line
		wait 1 irq 0
		; Read the line length (in pixel-pairs)
		; out x, 32 ; BROKEN
		set x, 31 ; Use fixed value instead
		loop1:
			; Write out first pixel - takes 5 clocks per pixel (4 pixels)
			out pins, 16 [19]
			; Write out second pixel - takes 5 clocks per pixel (4 pixels, allowing one for the jump)
			out pins, 16 [18]
			; Repeat until all pixel pairs sent
			jmp x-- loop1
		; Clear all pins after visible section
		mov pins null
		.wrap
		"
	);

	// These two state machines run thus:
	//
	// | Clock | Timing PIOSM | Pixel PIOSM      |
	// |:------|:-------------|:-----------------|
	// | 1     | out pins, 2  | wait 1 irq 0     |
	// | 2     | out x, 14    | wait 1 irq 0     |
	// | 3     | out exec, 16 | wait 1 irq 0     |
	// | 4     | <irq>        | wait 1 irq 0     |
	// | 5     | jmp x--      | wait 1 irq 0     |
	// | 6     |              | set x, 31        |
	// | 7     |              | out pins, 16 [4] |
	// | 8     |              | ..               |
	// | 9     |              | ..               |
	// | 10    |              | ..               |
	// | 11    |              | ..               |
	// | 12    |              | out pins, 16 [3] |
	// | 13    |              | ..               |
	// | 14    |              | ..               |
	// | 15    |              | ..               |
	// | 16    |              | jump x-- loop1   |
	//
	// Note: Credit to
	// https://gregchadwick.co.uk/blog/playing-with-the-pico-pt5/ who had a
	// very similar idea to me, but wrote it up far better than I ever could.

	let timing_installed = pio.install(&timing_program.program).unwrap();
	let (mut timing_sm, _, timing_fifo) = hal_pio::PIOBuilder::from_program(timing_installed)
		.buffers(hal_pio::Buffers::OnlyTx)
		.out_pins(0, 2)
		.autopull(true)
		.out_shift_direction(hal_pio::ShiftDirection::Right)
		.pull_threshold(32)
		.build(sm0);
	timing_sm.set_pindirs([(0, hal_pio::PinDir::Output), (1, hal_pio::PinDir::Output)]);

	// Important notes!
	//
	// You must not set a clock_divider (other than 1.0) on the pixel state
	// machine. You might want the pixels to be twice as wide (or mode), but
	// enabling a clock divider adds a lot of jitter (i.e. the start each
	// each line differs by some number of 126 MHz clock cycles).

	let pixels_installed = pio.install(&pixel_program.program).unwrap();
	let (mut pixel_sm, _, pixel_fifo) = hal_pio::PIOBuilder::from_program(pixels_installed)
		.buffers(hal_pio::Buffers::OnlyTx)
		.out_pins(2, 12)
		.autopull(true)
		.out_shift_direction(hal_pio::ShiftDirection::Right)
		.pull_threshold(32)
		.build(sm1);
	pixel_sm.set_pindirs((2..=13).map(|x| (x, hal_pio::PinDir::Output)));

	unsafe {
		// Read from the timing buffer and write to the timing FIFO. We get an
		// IRQ when the transfer is complete (i.e. when line has been fully
		// loaded).
		pac.DMA.ch[TIMING_DMA_CHAN].ch_ctrl_trig.write(|w| {
			w.data_size().size_word();
			w.incr_read().set_bit();
			w.incr_write().clear_bit();
			w.treq_sel().bits(timing_fifo.dreq_value());
			w.chain_to().bits(TIMING_DMA_CHAN as u8);
			w.ring_size().bits(0);
			w.ring_sel().clear_bit();
			w.bswap().clear_bit();
			w.irq_quiet().clear_bit();
			w.en().set_bit();
			w.sniff_en().clear_bit();
			w
		});
		pac.DMA.ch[TIMING_DMA_CHAN]
			.ch_read_addr
			.write(|w| w.bits(TIMING_BUFFER.visible_line.data.as_ptr() as usize as u32));
		pac.DMA.ch[TIMING_DMA_CHAN]
			.ch_write_addr
			.write(|w| w.bits(timing_fifo.dma_address()));
		pac.DMA.ch[TIMING_DMA_CHAN]
			.ch_trans_count
			.write(|w| w.bits(4));

		// Read from the pixel buffer (even first) and write to the pixel FIFO
		pac.DMA.ch[PIXEL_DMA_CHAN].ch_ctrl_trig.write(|w| {
			w.data_size().size_word();
			w.incr_read().set_bit();
			w.incr_write().clear_bit();
			w.treq_sel().bits(pixel_fifo.dreq_value());
			w.chain_to().bits(PIXEL_DMA_CHAN as u8);
			w.ring_size().bits(0);
			w.ring_sel().clear_bit();
			w.bswap().clear_bit();
			w.irq_quiet().clear_bit();
			w.en().set_bit();
			w.sniff_en().clear_bit();
			w
		});
		pac.DMA.ch[PIXEL_DMA_CHAN]
			.ch_read_addr
			.write(|w| w.bits(PIXEL_DATA_BUFFER_EVEN.as_ptr()));
		pac.DMA.ch[PIXEL_DMA_CHAN]
			.ch_write_addr
			.write(|w| w.bits(pixel_fifo.dma_address()));
		pac.DMA.ch[PIXEL_DMA_CHAN]
			.ch_trans_count
			.write(|w| w.bits(NUM_PIXEL_PAIRS as u32));
		pac.DMA.inte0.write(|w| {
			w.inte0()
				.bits((1 << PIXEL_DMA_CHAN) | (1 << TIMING_DMA_CHAN))
		});

		// Hand off the DMA peripheral to the interrupt
		DMA_PERIPH = Some(pac.DMA);

		// Enable the interupts (DMA_PERIPH has to be set first)
		pac::NVIC::unpend(pac::Interrupt::DMA_IRQ_0);
		pac::NVIC::unmask(pac::Interrupt::DMA_IRQ_0);

		info!("IRQs enabled");

		// Enable the DMA
		DMA_PERIPH
			.as_mut()
			.unwrap()
			.multi_chan_trigger
			.write(|w| w.bits((1 << PIXEL_DMA_CHAN) | (1 << TIMING_DMA_CHAN)));

		info!("DMA enabled");
	}

	info!("DMA set-up complete");

	timing_sm.start();
	pixel_sm.start();

	info!("State Machines running");

	let mut last_line = 0;
	let mut frame_count = 0;
	loop {
		cortex_m::asm::wfi();
		let try_line = CURRENT_DISPLAY_LINE.load(Ordering::Relaxed);
		if try_line != last_line {
			last_line = try_line;
			let next_line = if last_line == 479 {
				info!("Frame {}", frame_count);
				frame_count += 1;
				0
			} else {
				last_line + 1
			};
			// new line - do some painting!
			let px_buf = unsafe {
				if (next_line & 1) == 1 {
					&mut PIXEL_DATA_BUFFER_ODD
				} else {
					&mut PIXEL_DATA_BUFFER_EVEN
				}
			};
			for px in px_buf.pixels.iter_mut().skip(1) {
				*px = u32::from(next_line) | u32::from(next_line) << 16;
			}
		}
	}
}

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

#[interrupt]
unsafe fn DMA_IRQ_0() {
	let dma: &mut pac::DMA = match DMA_PERIPH.as_mut() {
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
		let timing_line = if old_timing_line == 524 {
			// 524 -> 0
			0
		} else {
			// n -> n + 1
			old_timing_line + 1
		};
		CURRENT_TIMING_LINE.store(timing_line, Ordering::Relaxed);

		let buffer = match timing_line {
			0..=479 => {
				// Visible lines
				&TIMING_BUFFER.visible_line
			}
			480..=489 => {
				// VGA front porch before VGA sync pulse
				&TIMING_BUFFER.vblank_porch_buffer
			}
			490..=491 => {
				// Sync pulse
				&TIMING_BUFFER.vblank_sync_buffer
			}
			492.. => {
				// VGA back porch following VGA sync pulse
				&TIMING_BUFFER.vblank_porch_buffer
			}
		};
		dma.ch[TIMING_DMA_CHAN]
			.ch_al3_read_addr_trig
			.write(|w| w.bits(buffer as *const _ as usize as u32))
	}

	if pixel_dma_chan_irq {
		dma.ints0.write(|w| w.bits(1 << PIXEL_DMA_CHAN));

		// A pixel DMA transfer is now complete. This only fires on lines 0..=480.

		let old_display_line = CURRENT_DISPLAY_LINE.load(Ordering::Relaxed);
		let next_display_line = if old_display_line == 479 {
			// 479 -> 0
			0
		} else {
			// n -> n + 1
			old_display_line + 1
		};
		CURRENT_DISPLAY_LINE.store(next_display_line, Ordering::Relaxed);

		// Set the DMA load address according to which line we are on
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
	}
}

impl LineBuffer {
	/// Convert the line buffer to a 32-bit address that the DMA engine understands.
	fn as_ptr(&self) -> u32 {
		// NB: skip the length field
		self.pixels.as_ptr() as usize as u32 + 4
	}
}

// -----------------------------------------------------------------------------
// End of file
// -----------------------------------------------------------------------------
