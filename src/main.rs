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
//! block (`0x1000_0000` to `0x1000_00FF`).

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
// Sub-modules
// -----------------------------------------------------------------------------

pub mod vga;

// -----------------------------------------------------------------------------
// Imports
// -----------------------------------------------------------------------------

use common::{video::Attr, video::TextBackgroundColour, video::TextForegroundColour, MemoryRegion};
use core::fmt::Write;
use cortex_m_rt::entry;
use defmt::{debug, info};
use defmt_rtt as _;
use embedded_hal::digital::v2::OutputPin;
use embedded_time::rate::*;
use neotron_common_bios as common;
use panic_probe as _;
use rp_pico::{
	self,
	hal::{
		self,
		pac::{self, interrupt},
		Clock,
	},
};

// -----------------------------------------------------------------------------
// Types
// -----------------------------------------------------------------------------

/// All the things the BIOS callbacks need to do their job.
struct Context {
	delay: cortex_m::delay::Delay,
}

// -----------------------------------------------------------------------------
// Static and Const Data
// -----------------------------------------------------------------------------

/// The BIOS version string
static BIOS_VERSION: &str = concat!("Neotron Pico BIOS version ", env!("BIOS_VERSION"), "\0");

/// This is our Operating System. It must be compiled separately.
///
/// The RP2040 requires an OS linked at `0x1002_0000`, which is the OS binary
/// `flash1002`. Use `objdump` as per the README file to make a `flash1002.bin`.
#[link_section = ".flash_os"]
#[used]
pub static OS_IMAGE: [u8; include_bytes!("flash1002.bin").len()] = *include_bytes!("flash1002.bin");

/// The table of API calls we provide the OS
static BIOS_API: common::Api = common::Api {
	api_version_get,
	bios_version_get,
	serial_get_info,
	serial_configure,
	serial_write,
	serial_read,
	time_get,
	time_set,
	configuration_get,
	configuration_set,
	video_is_valid_mode,
	video_mode_needs_vram,
	video_set_mode,
	video_get_mode,
	video_get_framebuffer,
	video_set_framebuffer,
	video_wait_for_line,
	video_convert_character,
	memory_get_region,
	hid_get_event,
	hid_set_leds,
	video_get_palette,
	video_set_palette,
	video_set_whole_palette,
	i2c_bus_get_info,
	i2c_write_read,
	audio_mixer_channel_get_info,
	audio_mixer_channel_set_level,
	audio_output_set_config,
	audio_output_get_config,
	audio_output_data,
	audio_output_get_space,
	audio_input_set_config,
	audio_input_get_config,
	audio_input_data,
	audio_input_get_count,
	bus_select,
	bus_get_info,
	bus_write_read,
	bus_exchange,
	delay,
};

/// Holds the contextual data we require to operate the callbacks (which don't get a `&self` or similar).
///
/// Has to be an `Option` as we have no boot-time initialisation for static variables.
///
/// Ideally would not be an unsafe `static mut` but instead some kind of `Cell` with run-time checking.
static mut CONTEXT: Option<Context> = None;

extern "C" {
	/// Linker symbol giving the start of the `FLASH_OS` region. This is where we find the start of the Neotron OS.
	static mut _flash_os_start: u32;
	/// Linker symbol giving the size of the `FLASH_OS` region, in bytes.
	///
	/// Take the address of this symbol to get the size value you require.
	static mut _flash_os_len: u32;
	/// Linker symbol giving the start of the `RAM_OS` region. This is where we set up RAM for Neotron OS.
	static mut _ram_os_start: u32;
	/// Linker symbol giving the size of the `RAM_OS` region, in bytes.
	///
	/// Take the address of this symbol to get the size value you require.
	static mut _ram_os_len: u32;
}

// -----------------------------------------------------------------------------
// Functions
// -----------------------------------------------------------------------------

/// This is the entry-point to the BIOS. It is called by cortex-m-rt once the
/// `.bss` and `.data` sections have been initialised.
#[entry]
fn main() -> ! {
	cortex_m::interrupt::disable();

	// BIOS_VERSION has a trailing `\0` as that is what the BIOS/OS API requires.
	info!("{} starting...", &BIOS_VERSION[0..BIOS_VERSION.len() - 1]);

	// Grab the singleton containing all the RP2040 peripherals
	let mut pp = pac::Peripherals::take().unwrap();
	// Grab the singleton containing all the generic Cortex-M peripherals
	let cp = pac::CorePeripherals::take().unwrap();

	// Reset the DMA engine. If we don't do this, starting from probe-run
	// (as opposed to a cold-start) is unreliable.
	reset_dma_engine(&mut pp);

	// Needed by the clock setup
	let mut watchdog = hal::watchdog::Watchdog::new(pp.WATCHDOG);

	// Run at 252 MHz SYS_PLL, 48 MHz, USB_PLL. This is important, we as clock
	// the PIO at ÷ 10, to give 25.2 MHz (which is close enough to the 25.175
	// MHz standard VGA pixel clock).

	// Step 1. Turn on the crystal.
	let xosc = hal::xosc::setup_xosc_blocking(pp.XOSC, rp_pico::XOSC_CRYSTAL_FREQ.Hz())
		.map_err(|_x| false)
		.unwrap();
	// Step 2. Configure watchdog tick generation to tick over every microsecond.
	watchdog.enable_tick_generation((rp_pico::XOSC_CRYSTAL_FREQ / 1_000_000) as u8);
	// Step 3. Create a clocks manager.
	let mut clocks = hal::clocks::ClocksManager::new(pp.CLOCKS);
	// Step 4. Set up the system PLL. We take Crystal Oscillator (=12 MHz),
	// ×252 (=1512 MHz), ÷6 (=252 MHz), ÷1 (=252 MHz)
	let pll_sys = hal::pll::setup_pll_blocking(
		pp.PLL_SYS,
		xosc.operating_frequency().into(),
		hal::pll::PLLConfig {
			vco_freq: Megahertz(1512),
			refdiv: 1,
			post_div1: 6,
			post_div2: 1,
		},
		&mut clocks,
		&mut pp.RESETS,
	)
	.map_err(|_x| false)
	.unwrap();
	// Step 5. Set up a 48 MHz PLL for the USB system.
	let pll_usb = hal::pll::setup_pll_blocking(
		pp.PLL_USB,
		xosc.operating_frequency().into(),
		hal::pll::common_configs::PLL_USB_48MHZ,
		&mut clocks,
		&mut pp.RESETS,
	)
	.map_err(|_x| false)
	.unwrap();
	// Step 6. Set the system to run from the PLLs we just configured.
	clocks
		.init_default(&xosc, &pll_sys, &pll_usb)
		.map_err(|_x| false)
		.unwrap();

	info!("Clocks OK");

	// sio is the *Single-cycle Input/Output* peripheral. It has all our GPIO
	// pins, as well as some mailboxes and other useful things for inter-core
	// communications.
	let mut sio = hal::sio::Sio::new(pp.SIO);

	// Configure and grab all the RP2040 pins the Pico exposes.
	let pins = rp_pico::Pins::new(pp.IO_BANK0, pp.PADS_BANK0, sio.gpio_bank0, &mut pp.RESETS);

	// Disable power save mode to force SMPS into low-efficiency, low-noise mode.
	let mut b_power_save = pins.b_power_save.into_push_pull_output();
	b_power_save.set_high().unwrap();

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

	info!("Pins OK");

	vga::init(
		pp.PIO0,
		pp.DMA,
		&mut pp.RESETS,
		&mut pp.PPB,
		&mut sio.fifo,
		&mut pp.PSM,
	);

	info!("VGA Up");

	// Make a sys-tick based delay object
	let mut delay = cortex_m::delay::Delay::new(cp.SYST, clocks.system_clock.freq().integer());

	// Create a new temporary console for some boot-up messages
	let tc = vga::TextConsole::new();
	tc.set_text_buffer(unsafe { &mut vga::GLYPH_ATTR_ARRAY });

	// A crude way to clear the screen
	for _row in 0..vga::MAX_TEXT_ROWS {
		writeln!(&tc).unwrap();
	}
	// Set up for bright white on black.
	tc.set_attribute(Attr::new(
		TextForegroundColour::WHITE,
		TextBackgroundColour::BLACK,
		false,
	));
	tc.move_to(0, 0);

	// Set up SPI comms
	let spi_csio = pins.gpio17.into_push_pull_output();
	let spi_bufen = pins.gpio21.into_push_pull_output();
	let _spi_cipo = pins.gpio16.into_mode::<hal::gpio::FunctionSpi>();
	let _spi_clk = pins.gpio18.into_mode::<hal::gpio::FunctionSpi>();
	let _spi_copi = pins.gpio19.into_mode::<hal::gpio::FunctionSpi>();
	let spi = hal::Spi::<_, _, 8>::new(pp.SPI0);
	let spi = spi.init(
		&mut pp.RESETS,
		clocks.peripheral_clock.freq(),
		1_000_000.Hz(),
		&embedded_hal::spi::MODE_0,
	);
	talk_to_bus(&tc, spi, spi_csio, spi_bufen, &mut delay);

	// Say hello over VGA (with a bit of a pause)
	sign_on(&tc, &mut delay);

	unsafe {
		CONTEXT = Some(Context { delay });
	}

	info!("Jumping to OS!");

	// Now jump to the OS. We are assuming the first four bytes of OS Flash are the address of the OS start function.
	let code: &common::OsStartFn = unsafe { ::core::mem::transmute(&_flash_os_start) };
	code(&BIOS_API);
}

struct Bus<S, IOCS, BUFEN>
where
	S: embedded_hal::blocking::spi::Transfer<u8>,
	IOCS: embedded_hal::digital::v2::OutputPin,
	BUFEN: embedded_hal::digital::v2::OutputPin,
{
	spi: S,
	iocs: IOCS,
	bufen: BUFEN,
	selected_device: Option<u8>,
}

#[repr(u8)]
enum McpRegister {
	IODIRA = 0x00,
	IODIRB = 0x01,
	IPOLA = 0x02,
	IPOLB = 0x03,
	GPINTENA = 0x04,
	GPINTENB = 0x05,
	DEFVALA = 0x06,
	DEFVALB = 0x07,
	INTCONA = 0x08,
	INTCONB = 0x09,
	IOCONA = 0x0A,
	IOCONB = 0x0B,
	GPPUA = 0x0C,
	GPPUB = 0x0D,
	INTFA = 0x0E,
	INTFB = 0x0F0,
	INTCAPA = 0x10,
	INTCAPB = 0x11,
	GPIOA = 0x12,
	GPIOB = 0x13,
	OLATA = 0x14,
	OLATB = 0x15,
}

impl<S, IOCS, BUFEN> Bus<S, IOCS, BUFEN>
where
	S: embedded_hal::blocking::spi::Transfer<u8>,
	IOCS: embedded_hal::digital::v2::OutputPin,
	BUFEN: embedded_hal::digital::v2::OutputPin,
{
	const READ_HDR: u8 = 0b0100_0001;
	const WRITE_HDR: u8 = 0b0100_0000;

	fn init(&mut self) {
		// Disable the 74HC245 buffer whilst we set things up
		let _ = self.bufen.set_high();
		// Set CS pins as outputs (high) and IRQ pins as inputs
		self.mux_write_register16(McpRegister::GPIOA, 0xFF00);
		self.mux_write_register16(McpRegister::IODIRA, 0x00FF);
		// Select nothing to start off with
		self.select_device(None);
	}

	fn select_device(&mut self, device: Option<u8>) {
		if self.selected_device != device {
			let cs_bit = if let Some(device) = device {
				if device <= 7 {
					!(1 << device)
				} else {
					panic!("Bad device ID {}", device);
				}
			} else {
				0
			};
			// Disable the 74HC245 to de-delect all bus devices (so they don't hear
			// our command to the MCP23S17).
			let _ = self.bufen.set_high();
			// Wait for buffer change to take effect
			for _ in 0..100_000 {
				cortex_m::asm::nop();
			}
			// Change the active CS pin on the MCP23S17
			self.mux_write_register(McpRegister::GPIOA, cs_bit);
			// Re-enable the 74HC245 to pass on the newly active CS pin.
			let _ = self.bufen.set_low();
			// Wait for buffer change to take effect
			for _ in 0..100_000 {
				cortex_m::asm::nop();
			}
			// Remember which CS pin the MCP23S17 has loaded
			self.selected_device = device;
		}
	}

	// Write to the bus mux's registers
	fn mux_write_register(&mut self, reg: McpRegister, value: u8) {
		let mut buffer = [Self::WRITE_HDR, reg as u8, value];
		let _ = self.iocs.set_low();
		let _ = self.spi.transfer(&mut buffer);
		let _ = self.iocs.set_high();
	}

	/// Write a 16-bit value to the bus mux's registers.
	///
	/// You should pass an `xxxA` register here for this to make sense, and have
	/// auto address increment function enabled (which it is by default).
	///
	/// Only call this function when the bus is disconnect (bufen is high).
	fn mux_write_register16(&mut self, reg: McpRegister, value: u16) {
		let mut buffer = [Self::WRITE_HDR, reg as u8, (value >> 8) as u8, value as u8];
		let _ = self.iocs.set_low();
		for _ in 0..100_000 {
			cortex_m::asm::nop();
		}
		let _ = self.spi.transfer(&mut buffer);
		let _ = self.iocs.set_high();
	}

	/// Read from the bus mux's registers
	///
	/// Only call this function when the bus is disconnect (bufen is high).
	fn mux_read_register(&mut self, reg: McpRegister) -> u8 {
		let mut buffer = [Self::READ_HDR, reg as u8, 0xFF];
		let _ = self.iocs.set_low();
		let _ = self.spi.transfer(&mut buffer);
		let _ = self.iocs.set_high();
		// The value we read is in the last place in the buffer
		buffer[2]
	}

	fn transact_on_bus<F>(&mut self, slot_id: u8, func: F)
	where
		F: FnOnce(&mut S),
	{
		self.select_device(Some(slot_id));
		func(&mut self.spi);
		self.select_device(None);
	}
}

/// Talk to the Bus
fn talk_to_bus(
	mut tc: &vga::TextConsole,
	spi: impl embedded_hal::blocking::spi::Transfer<u8>,
	iocs: impl embedded_hal::digital::v2::OutputPin,
	bufen: impl embedded_hal::digital::v2::OutputPin,
	delay: &mut cortex_m::delay::Delay,
) {
	let mut bus = Bus {
		spi,
		iocs,
		bufen,
		selected_device: None,
	};
	bus.init();

	let device = 2;
	writeln!(tc, "Selecting {}", device).unwrap();
	debug!("Selecting {}", device);
	bus.transact_on_bus(device, |spi| {
		let mut card_command = [0b0100_0000, 0x0, 0x0];
		let _ = spi.transfer(&mut card_command);
	});
	loop {
		for n in &[1, 2, 4, 8, 16, 32, 64, 128] {
			bus.transact_on_bus(device, |spi| {
				let mut card_command = [0b0100_0000, 0x12, *n];
				let _ = spi.transfer(&mut card_command);
			});
			delay.delay_ms(250);
		}
	}
}

/// Tell people about the BIOS, using the VGA output.
fn sign_on(mut tc: &vga::TextConsole, delay: &mut cortex_m::delay::Delay) {
	static LICENCE_TEXT: &str = "\
        Copyright © Jonathan 'theJPster' Pallant and the Neotron Developers, 2022\n\
		Licenced under the GNU GPL v3, or later.";
	static LOGO_TEXT: &str = "\
		Neotron \\FB\\B1 This is Yellow on Red \\FF\\B0 This is normal again.\n\
		Neotron \\FF\\B4 This is White on Blue \\FF\\B0 This is normal again.\n\
		";

	#[derive(Copy, Clone, Eq, PartialEq, Debug)]
	enum Escape {
		None,
		Slash,
		Fore,
		Back,
	}

	let mut attr = Attr::new(
		TextForegroundColour::WHITE,
		TextBackgroundColour::BLACK,
		false,
	);
	let mut escape = Escape::None;
	for ch in LOGO_TEXT.chars() {
		match (escape, ch) {
			(Escape::Fore, '0') => {
				attr.set_fg(TextForegroundColour::BLACK);
				escape = Escape::None;
			}
			(Escape::Fore, '1') => {
				attr.set_fg(TextForegroundColour::DARK_RED);
				escape = Escape::None;
			}
			(Escape::Fore, '2') => {
				attr.set_fg(TextForegroundColour::DARK_GREEN);
				escape = Escape::None;
			}
			(Escape::Fore, '3') => {
				attr.set_fg(TextForegroundColour::ORANGE);
				escape = Escape::None;
			}
			(Escape::Fore, '4') => {
				attr.set_fg(TextForegroundColour::BLUE);
				escape = Escape::None;
			}
			(Escape::Fore, '5') => {
				attr.set_fg(TextForegroundColour::DARK_MAGENTA);
				escape = Escape::None;
			}
			(Escape::Fore, '6') => {
				attr.set_fg(TextForegroundColour::DARK_CYAN);
				escape = Escape::None;
			}
			(Escape::Fore, '7') => {
				attr.set_fg(TextForegroundColour::YELLOW);
				escape = Escape::None;
			}
			(Escape::Fore, '8') => {
				attr.set_fg(TextForegroundColour::GREY);
				escape = Escape::None;
			}
			(Escape::Fore, '9') => {
				attr.set_fg(TextForegroundColour::BRIGHT_RED);
				escape = Escape::None;
			}
			(Escape::Fore, 'a' | 'A') => {
				attr.set_fg(TextForegroundColour::BRIGHT_GREEN);
				escape = Escape::None;
			}
			(Escape::Fore, 'b' | 'B') => {
				attr.set_fg(TextForegroundColour::BRIGHT_YELLOW);
				escape = Escape::None;
			}
			(Escape::Fore, 'c' | 'C') => {
				attr.set_fg(TextForegroundColour::BRIGHT_BLUE);
				escape = Escape::None;
			}
			(Escape::Fore, 'd' | 'D') => {
				attr.set_fg(TextForegroundColour::BRIGHT_MAGENTA);
				escape = Escape::None;
			}
			(Escape::Fore, 'e' | 'E') => {
				attr.set_fg(TextForegroundColour::BRIGHT_CYAN);
				escape = Escape::None;
			}
			(Escape::Fore, 'f' | 'F') => {
				attr.set_fg(TextForegroundColour::WHITE);
				escape = Escape::None;
			}
			(Escape::Back, '0') => {
				attr.set_bg(TextBackgroundColour::BLACK);
				escape = Escape::None;
			}
			(Escape::Back, '1') => {
				attr.set_bg(TextBackgroundColour::DARK_RED);
				escape = Escape::None;
			}
			(Escape::Back, '2') => {
				attr.set_bg(TextBackgroundColour::DARK_GREEN);
				escape = Escape::None;
			}
			(Escape::Back, '3') => {
				attr.set_bg(TextBackgroundColour::ORANGE);
				escape = Escape::None;
			}
			(Escape::Back, '4') => {
				attr.set_bg(TextBackgroundColour::BLUE);
				escape = Escape::None;
			}
			(Escape::Back, '5') => {
				attr.set_bg(TextBackgroundColour::DARK_MAGENTA);
				escape = Escape::None;
			}
			(Escape::Back, '6') => {
				attr.set_bg(TextBackgroundColour::DARK_CYAN);
				escape = Escape::None;
			}
			(Escape::Back, '7') => {
				attr.set_bg(TextBackgroundColour::YELLOW);
				escape = Escape::None;
			}
			(Escape::None, '\\') => {
				escape = Escape::Slash;
			}
			(Escape::Slash, 'F') => {
				escape = Escape::Fore;
			}
			(Escape::Slash, 'B') => {
				escape = Escape::Back;
			}
			(Escape::None, _) => {
				write!(tc, "{}", ch).unwrap();
			}
			_ => {
				defmt::panic!("Bad format {} {}", escape as u32, ch);
			}
		}
		tc.set_attribute(attr);
	}

	// Set up for bright white on black.
	tc.set_attribute(Attr::new(
		TextForegroundColour::WHITE,
		TextBackgroundColour::BLACK,
		false,
	));

	// Print the version, skipping the NUL byte at the end.
	writeln!(tc, "{}", &BIOS_VERSION[0..BIOS_VERSION.len() - 1]).unwrap();
	// Print the licence text
	writeln!(tc, "{}", LICENCE_TEXT).unwrap();

	// Dump some stats about render times to the screen
	let mut last_render_count = 0;
	let mut last_clashed_count = 0;
	for i in 0..10 {
		// Wait for 1 second (or 60 frames)
		delay.delay_ms(1000);
		// We have 480 * 60 lines elapsed. How long on average did we wait per line?
		let render_count = vga::RENDER_TIME.load(core::sync::atomic::Ordering::Relaxed);
		let delta_render_count = (render_count - last_render_count) / (480 * 60);
		last_render_count = render_count;
		// Also show how many lines we bungled
		let clashed_count = vga::CLASHED_COUNT.load(core::sync::atomic::Ordering::Relaxed);
		let delta_clashed_count = clashed_count - last_clashed_count;
		last_clashed_count = clashed_count;
		write!(
			tc,
			"{} {} clocks/line {} clashed lines              \r",
			i, delta_render_count, delta_clashed_count
		)
		.unwrap();
	}
}

/// Reset the DMA Peripheral.
fn reset_dma_engine(pp: &mut pac::Peripherals) {
	pp.RESETS.reset.modify(|_r, w| w.dma().set_bit());
	cortex_m::asm::nop();
	pp.RESETS.reset.modify(|_r, w| w.dma().clear_bit());
	while pp.RESETS.reset_done.read().dma().bit_is_clear() {}
}

/// Returns the version number of the BIOS API.
pub extern "C" fn api_version_get() -> common::Version {
	common::API_VERSION
}

/// Returns a pointer to a static string slice containing the BIOS Version.
///
/// This string contains the version number and build string of the BIOS.
/// For C compatibility this string is null-terminated and guaranteed to
/// only contain ASCII characters (bytes with a value 127 or lower). We
/// also pass the length (excluding the null) to make it easy to construct
/// a Rust string. It is unspecified as to whether the string is located
/// in Flash ROM or RAM (but it's likely to be Flash ROM).
pub extern "C" fn bios_version_get() -> common::ApiString<'static> {
	common::ApiString::new(BIOS_VERSION)
}

/// Get information about the Serial ports in the system.
///
/// Serial ports are ordered octet-oriented pipes. You can push octets
/// into them using a 'write' call, and pull bytes out of them using a
/// 'read' call. They have options which allow them to be configured at
/// different speeds, or with different transmission settings (parity
/// bits, stop bits, etc) - you set these with a call to
/// `SerialConfigure`. They may physically be a MIDI interface, an RS-232
/// port or a USB-Serial port. There is no sense of 'open' or 'close' -
/// that is an Operating System level design feature. These APIs just
/// reflect the raw hardware, in a similar manner to the registers exposed
/// by a memory-mapped UART peripheral.
pub extern "C" fn serial_get_info(_device: u8) -> common::Option<common::serial::DeviceInfo> {
	common::Option::None
}

/// Set the options for a given serial device. An error is returned if the
/// options are invalid for that serial device.
pub extern "C" fn serial_configure(
	_device: u8,
	_config: common::serial::Config,
) -> common::Result<()> {
	common::Result::Err(common::Error::Unimplemented)
}

/// Write bytes to a serial port. There is no sense of 'opening' or
/// 'closing' the device - serial devices are always open. If the return
/// value is `Ok(n)`, the value `n` may be less than the size of the given
/// buffer. If so, that means not all of the data could be transmitted -
/// only the first `n` bytes were.
pub extern "C" fn serial_write(
	_device: u8,
	_data: common::ApiByteSlice,
	_timeout: common::Option<common::Timeout>,
) -> common::Result<usize> {
	common::Result::Err(common::Error::Unimplemented)
}

/// Read bytes from a serial port. There is no sense of 'opening' or
/// 'closing' the device - serial devices are always open. If the return value
///  is `Ok(n)`, the value `n` may be less than the size of the given buffer.
///  If so, that means not all of the data could be received - only the
///  first `n` bytes were filled in.
pub extern "C" fn serial_read(
	_device: u8,
	_data: common::ApiBuffer,
	_timeout: common::Option<common::Timeout>,
) -> common::Result<usize> {
	common::Result::Err(common::Error::Unimplemented)
}

/// Get the current wall time.
///
/// The Neotron BIOS does not understand time zones, leap-seconds or the
/// Gregorian calendar. It simply stores time as an incrementing number of
/// seconds since some epoch, and the number of milliseconds since that second
/// began. A day is assumed to be exactly 86,400 seconds long. This is a lot
/// like POSIX time, except we have a different epoch
/// - the Neotron epoch is 2000-01-01T00:00:00Z. It is highly recommend that you
/// store UTC in the BIOS and use the OS to handle time-zones.
///
/// If the BIOS does not have a battery-backed clock, or if that battery has
/// failed to keep time, the system starts up assuming it is the epoch.
pub extern "C" fn time_get() -> common::Time {
	// TODO: Read from the MCP7940N
	common::Time { secs: 0, nsecs: 0 }
}

/// Set the current wall time.
///
/// See `time_get` for a description of now the Neotron BIOS should handle
/// time.
///
/// You only need to call this whenever you get a new sense of the current
/// time (e.g. the user has updated the current time, or if you get a GPS
/// fix). The BIOS should push the time out to the battery-backed Real
/// Time Clock, if it has one.
pub extern "C" fn time_set(_time: common::Time) {
	// TODO: Update the MCP7940N RTC
}

/// Get the configuration data block.
///
/// Configuration data is, to the BIOS, just a block of bytes of a given
/// length. How it stores them is up to the BIOS - it could be EEPROM, or
/// battery-backed SRAM.
pub extern "C" fn configuration_get(_buffer: common::ApiBuffer) -> common::Result<usize> {
	common::Result::Err(common::Error::Unimplemented)
}

/// Set the configuration data block.
///
/// See `configuration_get`.
pub extern "C" fn configuration_set(_buffer: common::ApiByteSlice) -> common::Result<()> {
	common::Result::Err(common::Error::Unimplemented)
}

/// Does this Neotron BIOS support this video mode?
pub extern "C" fn video_is_valid_mode(mode: common::video::Mode) -> bool {
	mode == common::video::Mode::new(
		common::video::Timing::T640x480,
		common::video::Format::Text8x16,
	)
}

/// Switch to a new video mode.
///
/// The contents of the screen are undefined after a call to this function.
///
/// If the BIOS does not have enough reserved RAM (or dedicated VRAM) to
/// support this mode, the change will succeed but a subsequent call to
/// `video_get_framebuffer` will return `null`. You must then supply a
/// pointer to a block of size `Mode::frame_size_bytes()` to
/// `video_set_framebuffer` before any video will appear.
pub extern "C" fn video_set_mode(mode: common::video::Mode) -> common::Result<()> {
	debug!("video_set_mode({})", mode.as_u8());
	// if vga::set_video_mode(mode) {
	common::Result::Ok(())
	// } else {
	// 	common::Result::Err(common::Error::UnsupportedConfiguration(0))
	// }
}

/// Returns the video mode the BIOS is currently in.
///
/// The OS should call this function immediately after start-up and note
/// the value - this is the `default` video mode which can always be
/// serviced without supplying extra RAM.
pub extern "C" fn video_get_mode() -> common::video::Mode {
	vga::get_video_mode()
}

/// Get the framebuffer address.
///
/// We can write through this address to the video framebuffer. The
/// meaning of the data we write, and the size of the region we are
/// allowed to write to, is a function of the current video mode (see
/// `video_get_mode`).
///
/// This function will return `null` if the BIOS isn't able to support the
/// current video mode from its memory reserves. If that happens, you will
/// need to use some OS RAM or Application RAM and provide that as a
/// framebuffer to `video_set_framebuffer`. The BIOS will always be able
/// to provide the 'basic' text buffer experience from reserves, so this
/// function will never return `null` on start-up.
pub extern "C" fn video_get_framebuffer() -> *mut u8 {
	unsafe { vga::GLYPH_ATTR_ARRAY.as_mut_ptr() as *mut u8 }
}

/// Set the framebuffer address.
///
/// Tell the BIOS where it should start fetching pixel or textual data from
/// (depending on the current video mode).
///
/// This value is forgotten after a video mode change and must be re-supplied.
///
/// # Safety
///
/// The pointer must point to enough video memory to handle the current video
/// mode, and any future video mode you set.
pub unsafe extern "C" fn video_set_framebuffer(_buffer: *const u8) -> common::Result<()> {
	common::Result::Err(common::Error::Unimplemented)
}

/// Find out whether the given video mode needs more VRAM than we currently have.
///
/// The answer is no for any currently supported video mode (which is just the four text modes right now).
pub extern "C" fn video_mode_needs_vram(_mode: common::video::Mode) -> bool {
	false
}

/// Find out how large a given region of memory is.
///
/// The first region is the 'main application region' and is defined to always
/// start at address `0x2000_0000` on a standard Cortex-M system. This
/// application region stops just before the BIOS reserved memory, at the top of
/// the internal SRAM. The OS will have been linked to use the first 1 KiB of
/// this region.
///
/// Other regions may be located at other addresses (e.g. external DRAM or
/// PSRAM).
///
/// The OS will always load non-relocatable applications into the bottom of
/// Region 0. It can allocate OS specific structures from any other Region (if
/// any), or from the top of Region 0 (although this reduces the maximum
/// application space available). The OS will prefer lower numbered regions
/// (other than Region 0), so faster memory should be listed first.
///
/// If the region number given is invalid, the function returns `(null, 0)`.
pub extern "C" fn memory_get_region(region: u8) -> common::Result<common::MemoryRegion> {
	match region {
		0 => {
			// Application Region
			common::Result::Ok(MemoryRegion {
				start: unsafe { &mut _ram_os_start as *mut u32 } as *mut u8,
				length: unsafe { &mut _ram_os_len as *const u32 } as usize,
				kind: common::MemoryKind::Ram,
			})
		}
		_ => common::Result::Err(common::Error::InvalidDevice),
	}
}

/// Get the next available HID event, if any.
///
/// This function doesn't block. It will return `Ok(None)` if there is no event ready.
pub extern "C" fn hid_get_event() -> common::Result<common::Option<common::hid::HidEvent>> {
	// TODO: Support some HID events
	common::Result::Ok(common::Option::None)
}

/// Control the keyboard LEDs.
pub extern "C" fn hid_set_leds(_leds: common::hid::KeyboardLeds) -> common::Result<()> {
	common::Result::Err(common::Error::Unimplemented)
}

/// Wait for the next occurence of the specified video scan-line.
///
/// In general we must assume that the video memory is read top-to-bottom
/// as the picture is being drawn on the monitor (e.g. via a VGA video
/// signal). If you modify video memory during this *drawing period*
/// there is a risk that the image on the monitor (however briefly) may
/// contain some parts from before the modification and some parts from
/// after. This can given rise to the *tearing effect* where it looks
/// like the screen has been torn (or ripped) across because there is a
/// discontinuity part-way through the image.
///
/// This function busy-waits until the video drawing has reached a
/// specified scan-line on the video frame.
///
/// There is no error code here. If the line you ask for is beyond the
/// number of visible scan-lines in the current video mode, it waits util
/// the last visible scan-line is complete.
///
/// If you wait for the last visible line until drawing, you stand the
/// best chance of your pixels operations on the video RAM being
/// completed before scan-lines start being sent to the monitor for the
/// next frame.
///
/// You can also use this for a crude `16.7 ms` delay but note that
/// some video modes run at `70 Hz` and so this would then give you a
/// `14.3ms` second delay.
pub extern "C" fn video_wait_for_line(line: u16) {
	let desired_line = line.min(vga::get_num_scan_lines());
	loop {
		let current_line = vga::get_scan_line();
		if current_line == desired_line {
			break;
		}
	}
}

extern "C" fn video_get_palette(index: u8) -> common::Option<common::video::RGBColour> {
	debug!("video_get_palette({})", index);
	unimplemented!();
}

extern "C" fn video_set_palette(index: u8, rgb: common::video::RGBColour) {
	debug!("video_set_palette({}, #{:06x})", index, rgb.as_packed());
	unimplemented!();
}

unsafe extern "C" fn video_set_whole_palette(
	palette: *const common::video::RGBColour,
	length: usize,
) {
	debug!("video_set_whole_palette({:x}, {})", palette, length);
	unimplemented!();
}

extern "C" fn video_convert_character(character: u32) -> common::Option<u8> {
	if let Some(character) = char::from_u32(character) {
		let glyph = vga::TextConsole::map_char_to_glyph(character);
		if let Some(glyph) = glyph {
			return common::Option::Some(glyph.0);
		}
	}
	common::Option::None
}

extern "C" fn i2c_bus_get_info(_i2c_bus: u8) -> common::Option<common::i2c::BusInfo> {
	debug!("i2c_bus_get_info");
	unimplemented!();
}

extern "C" fn i2c_write_read(
	_i2c_bus: u8,
	_i2c_device_address: u8,
	_tx: common::ApiByteSlice,
	_tx2: common::ApiByteSlice,
	_rx: common::ApiBuffer,
) -> common::Result<()> {
	debug!("i2c_write_read");
	unimplemented!();
}

extern "C" fn audio_mixer_channel_get_info(
	_audio_mixer_id: u8,
) -> common::Result<common::audio::MixerChannelInfo> {
	debug!("audio_mixer_channel_get_info");
	unimplemented!();
}

extern "C" fn audio_mixer_channel_set_level(_audio_mixer_id: u8, _level: u8) -> common::Result<()> {
	debug!("audio_mixer_channel_set_level");
	unimplemented!();
}

extern "C" fn audio_output_set_config(_config: common::audio::Config) -> common::Result<()> {
	debug!("audio_output_set_config");
	unimplemented!();
}

extern "C" fn audio_output_get_config() -> common::Result<common::audio::Config> {
	debug!("audio_output_get_config");
	unimplemented!();
}

unsafe extern "C" fn audio_output_data(_samples: common::ApiByteSlice) -> common::Result<usize> {
	debug!("audio_output_data");
	unimplemented!();
}

extern "C" fn audio_output_get_space() -> common::Result<usize> {
	debug!("audio_output_get_space");
	unimplemented!();
}

extern "C" fn audio_input_set_config(_config: common::audio::Config) -> common::Result<()> {
	debug!("audio_input_set_config");
	unimplemented!();
}

extern "C" fn audio_input_get_config() -> common::Result<common::audio::Config> {
	debug!("audio_input_get_config");
	unimplemented!();
}

extern "C" fn audio_input_data(_samples: common::ApiBuffer) -> common::Result<usize> {
	debug!("audio_input_data");
	unimplemented!();
}

extern "C" fn audio_input_get_count() -> common::Result<usize> {
	debug!("audio_input_get_count");
	unimplemented!();
}

extern "C" fn bus_select(_periperal_id: common::Option<u8>) {
	debug!("bus_select");
	unimplemented!();
}

extern "C" fn bus_get_info(_periperal_id: u8) -> common::Option<common::bus::PeripheralInfo> {
	debug!("bus_get_info");
	unimplemented!();
}

extern "C" fn bus_write_read(
	_tx: common::ApiByteSlice,
	_tx2: common::ApiByteSlice,
	_rx: common::ApiBuffer,
) -> common::Result<()> {
	debug!("bus_write_read");
	unimplemented!();
}

extern "C" fn bus_exchange(_buffer: common::ApiBuffer) -> common::Result<()> {
	debug!("bus_exchange");
	unimplemented!();
}

extern "C" fn delay(timeout: common::Timeout) {
	debug!("delay({} ms)", timeout.get_ms());
	if let Some(ctx) = unsafe { CONTEXT.as_mut() } {
		ctx.delay.delay_ms(timeout.get_ms());
	}
}

/// Called when DMA raises IRQ0; i.e. when a DMA transfer to the pixel FIFO or
/// the timing FIFO has completed.
#[interrupt]
#[link_section = ".data"]
fn DMA_IRQ_0() {
	unsafe {
		vga::irq();
	}
}

// -----------------------------------------------------------------------------
// End of file
// -----------------------------------------------------------------------------
