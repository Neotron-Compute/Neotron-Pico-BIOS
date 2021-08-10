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

use core::sync::atomic::AtomicBool;
use core::sync::atomic::Ordering;
use core::sync::atomic::AtomicU32;

use embedded_hal::digital::v2::OutputPin;
use cortex_m_rt::entry;
use embedded_time::rate::*;
use hal::clocks::Clock;
use panic_halt as _;
use pico;
use pico::hal;
use pico::hal::pac;

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

// -----------------------------------------------------------------------------
// Types
// -----------------------------------------------------------------------------

// None

// -----------------------------------------------------------------------------
// Functions
// -----------------------------------------------------------------------------

/// This is the entry-point to the BIOS. It is called by cortex-m-rt once the
/// `.bss` and `.data` sections have been initialised.
#[entry]
fn main() -> ! {
	// Grab the singleton containing all the RP2040 peripherals
	let mut pac = pac::Peripherals::take().unwrap();
	// Grab the singleton containing all the generic Cortex-M peripherals
	let core = pac::CorePeripherals::take().unwrap();

	// Needed by the clock setup
	let mut watchdog = hal::watchdog::Watchdog::new(pac.WATCHDOG);

	// Get ourselves up to a decent clock speed.
	let clocks = hal::clocks::init_clocks_and_plls(
		pico::XOSC_CRYSTAL_FREQ,
		pac.XOSC,
		pac.CLOCKS,
		pac.PLL_SYS,
		pac.PLL_USB,
		&mut pac.RESETS,
		&mut watchdog,
	)
	.ok()
	.unwrap();

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

	// Grab the LED pin
	let mut led_pin = pins.led.into_push_pull_output();
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

    scanvideo_setup();

    scanvideo_timing_enable(true);

	// Do some blinky so we can see it work.
	loop {
        led_pin.set_low().unwrap();
        let mut scanline_buffer = scanvideo_begin_scanline_generation(true);
        draw_color_bar(&mut scanline_buffer);
        scanvideo_end_scanline_generation(scanline_buffer);
        led_pin.set_high().unwrap();
	}
}

const VIDEO_CLOCK_FREQUENCY: u32 = 25_000_000;
const VIDEO_VISIBLE_PIXELS: u32 = 640;
const VIDEO_VISIBLE_LINES: u32 = 480;
const VIDEO_H_FRONT_PORCH: u32 = 16;
const VIDEO_H_PULSE: u32 = 64;
const VIDEO_H_TOTAL: u32 = 800;
const VIDEO_H_SYNC_POLARITY: bool = true;
const VIDEO_V_FRONT_PORCH: u32 = 1;
const VIDEO_V_PULSE: u32 = 2;
const VIDEO_V_TOTAL: u32 = 523;
const VIDEO_V_SYNC_POLARITY: bool = true;

const MAX_SCANVIDEO_WORDS: usize = 1 + ((VIDEO_VISIBLE_PIXELS as usize) / 2);
const MAX_SCANVIDEO_BUFFERS: usize = 8;

struct Buffer {
	pio_data: [u32; MAX_SCANVIDEO_WORDS],
}

impl Buffer {
	const fn new() -> Buffer {
		Buffer { 
			pio_data: [0; MAX_SCANVIDEO_WORDS]
		}
	}
}

struct Buffers {
	buffers: [Buffer; MAX_SCANVIDEO_BUFFERS],
}

struct BufferHandle {
	buffer: &'static mut Buffer
}

struct ScanvideoState {
	// scanline.lock
	// dma.lock
	// free_list.lock
	// in_use.lock
	// scanline.last_scanline_id
	last_scanline_id: AtomicU32,
}

static SHARED_STATE: ScanvideoState = ScanvideoState {
	last_scanline_id: AtomicU32::new(0)
};

/// Configure video for 640x480 @ 60 Hz
fn scanvideo_setup() {
	SHARED_STATE.last_scanline_id.store(0xFFFFFFFF, Ordering::Relaxed);

	// video_program_load_offset = pio_add_program(video_pio, &video_24mhz_composable);
    // setup_sm(PICO_SCANVIDEO_SCANLINE_SM, video_program_load_offset);
    // video_htiming_load_offset = pio_add_program(video_pio, &video_timing_program);
    // setup_sm(PICO_SCANVIDEO_TIMING_SM, video_htiming_load_offset);
    // irq_set_priority(PIO0_IRQ_0, 0); // highest priority
    // irq_set_priority(PIO0_IRQ_1, 0x40); // lower priority by 1

}

fn scanvideo_timing_enable(_some_arg: bool) {

}

static mut BUFFERS: Buffers = Buffers {
	 buffers: [
	 	Buffer::new(),
	 	Buffer::new(),
	 	Buffer::new(),
	 	Buffer::new(),
	 	Buffer::new(),
	 	Buffer::new(),
	 	Buffer::new(),
	 	Buffer::new(),
	 ]
};

fn scanvideo_begin_scanline_generation(_some_arg: bool) -> BufferHandle {
	let next_line = SHARED_STATE.last_scanline_id.load(Ordering::Relaxed);
	let buffer = (next_line as usize) % MAX_SCANVIDEO_BUFFERS;
	let buffer = unsafe { &mut BUFFERS.buffers[buffer] };
	BufferHandle {
		buffer
	}
}

fn draw_color_bar(handle: &mut BufferHandle) {
	handle.buffer.pio_data[0] = VIDEO_VISIBLE_PIXELS;
	for p in handle.buffer.pio_data[1..].iter_mut() {
		// Each word is two 16-bit values.
		// Of those, we use the bottom 12-bits as the RGB colour.
		// White pixel, black pixel.
		*p = 0x0FFF_0000;
	}
}

fn scanvideo_end_scanline_generation(_complete_buffer: BufferHandle) {
}

// -----------------------------------------------------------------------------
// End of file
// -----------------------------------------------------------------------------
