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

use cortex_m_rt::entry;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::v2::OutputPin;
use embedded_time::rate::*;
use git_version::git_version;
use hal::clocks::Clock;
use panic_probe as _;
use pico::{
	self,
	hal::{
		self, pac,
		pio::{self as hal_pio, PIOExt},
	},
};

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
	info!("Neotron BIOS {} starting...", GIT_VERSION);

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

	// Need to configure PIO0SM0 to run the timing loop. We post timing data
	// and it busy-waits the appropriate amount of time and trigger the
	// appropriate interrupts.

	// Post <front_porch> <pulse> <back_porch>

	// Need to configure PIO0SM1 to run the pixel loop. It waits for an IRQ
	// (posted by the timing loop) then pulls pixel data from the FIFO. We
	// post the number of pixels for that line, then the pixel data.

	// Post <horizontal pixels> <pixel0> <pixel1> <pixel2>; each <pixelX> is a 12-bit value padded up to 16-bits.

	let (mut pio, sm0, _sm1, _sm2, _sm3) = pac.PIO0.split(&mut pac.RESETS);

	let program = pio_proc::pio!(
		32,
		"
		.wrap_target
		; Note autopull is set to 32-bits
		; output is set to shift right
		out exec, 16  ; Execute top 16-bits as an instruction
		out pins, 2   ; Push bottom 2 bits to PINS
		out x, 14     ; Push next 14 bits into X for the timing loop
		loop0:
		jmp x-- loop0 ; Spin until X is zero
		.wrap
		"
	);

	let installed = pio.install(&program.program).unwrap();
	let (mut sm, _rx_fifo, mut tx_fifo) = hal_pio::PIOBuilder::from_program(installed)
		.buffers(hal_pio::Buffers::OnlyTx)
		.set_pins(0, 2)
		.out_pins(0, 2)
		.clock_divisor(5.0)
		.autopull(true)
		.out_shift_direction(hal_pio::ShiftDirection::Right)
		.pull_threshold(32)
		.build(sm0);
	sm.set_pindirs([(0, hal_pio::PinDir::Output), (1, hal_pio::PinDir::Output)]);

	sm.start();

	let mut x: u8 = 0;
	loop {
		// Flash a pin every 60 frames, or once a second
		x = x.wrapping_add(1);
		if x == 60 {
			led_pin.set_high().unwrap();
			x = 0;
		} else {
			led_pin.set_low().unwrap();
		}
		// This should all be moved off into an IRQ so it doesn't block the main thread
		for scanline in 1..=525 {
			// 640 x 480, negative h-sync - four periods per video line

			let need_vsync = scanline >= 490 && scanline <= 491;
			let _is_visible = scanline <= 480;

			// This if the front porch
			load_timing(&mut tx_fifo, 16, true, need_vsync, false);
			// This is the sync pulse
			load_timing(&mut tx_fifo, 96, false, need_vsync, false);
			// This is the back porch
			load_timing(&mut tx_fifo, 48, true, need_vsync, false);
			// This is the visible portion - trigger the IRQ to start pixels moving
			load_timing(&mut tx_fifo, 640, true, need_vsync, true);
		}
	}
}

fn load_timing(
	fifo: &mut hal_pio::Tx<(pac::PIO0, hal_pio::SM0)>,
	period: u16,
	hsync: bool,
	vsync: bool,
	want_irq: bool,
) {
	let command = if want_irq {
		// This command sets IRQ 0
		pio::InstructionOperands::IRQ {
			clear: false,
			wait: false,
			index: 0,
			relative: false,
		}
	} else {
		// This command is a no-op (it moves Y into Y)
		pio::InstructionOperands::MOV {
			destination: pio::MovDestination::Y,
			op: pio::MovOperation::None,
			source: pio::MovSource::Y,
		}
	};
	let mut value: u32 = u32::from(command.encode());
	if hsync {
		value |= 1 << 16;
	}
	if vsync {
		value |= 1 << 17;
	}
	value |= (u32::from(period) - 4) << 18;
	while !fifo.write(value) {
		// Spin?
	}
}

// -----------------------------------------------------------------------------
// End of file
// -----------------------------------------------------------------------------
