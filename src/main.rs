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

#![no_std]
#![no_main]

// The boot2 feature of rp-pico is temporarily disabled, so the version
// of rp2040-boot2 can be overridden. Therefore, BOOT2_FIRMWARE needs to be
// defined here.
#[link_section = ".boot2"]
#[no_mangle]
#[used]
pub static BOOT2_FIRMWARE: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

// -----------------------------------------------------------------------------
// Sub-modules
// -----------------------------------------------------------------------------

mod console;
mod i2s;
mod mcp23s17;
mod multicore;
mod mutex;
mod rtc;
mod sdcard;
mod vga;

// -----------------------------------------------------------------------------
// Imports
// -----------------------------------------------------------------------------

// Standard Library Stuff
use core::{
	convert::TryFrom,
	fmt::Write,
	sync::atomic::{AtomicBool, AtomicU32, Ordering},
};

// Third Party Stuff
use cortex_m_rt::entry;
use defmt::info;
use defmt_rtt as _;
use ds1307::{Datelike, NaiveDateTime, Timelike};
use embedded_hal::{
	blocking::i2c::{WriteIter as _I2cWriteIter, WriteIterRead as _I2cWriteIterRead},
	blocking::spi::{Transfer as _SpiTransfer, Write as _SpiWrite},
	digital::v2::{InputPin, OutputPin},
};
use fugit::RateExtU32;
use panic_probe as _;
use pc_keyboard::{KeyCode, ScancodeSet};
use rp_pico::{
	self,
	hal::{
		self,
		clocks::ClocksManager,
		gpio::{
			bank0::{self, Gpio16, Gpio18, Gpio19},
			FunctionPio0, FunctionPio1, FunctionSioOutput, FunctionSpi, Pin, PullNone,
		},
		pac::{self, interrupt},
		Clock,
	},
};

// Other Neotron Crates
use mutex::NeoMutex;
use neotron_bmc_commands::Command;
use neotron_bmc_protocol::Receivable;
use neotron_common_bios::{
	self as common,
	hid::HidEvent,
	video::{Attr, TextBackgroundColour, TextForegroundColour},
	ApiResult, Error as CError, FfiBuffer, FfiByteSlice, FfiOption, FfiString, MemoryRegion,
};

// -----------------------------------------------------------------------------
// Types
// -----------------------------------------------------------------------------

/// What we count our system ticks in
type Duration = fugit::Duration<u64, 1, 1_000_000>;

/// The type of our IRQ input pin from the MCP23S17.
type IrqPin = Pin<bank0::Gpio20, rp_pico::hal::gpio::FunctionSioInput, rp_pico::hal::gpio::PullUp>;

type I2cPins = (
	Pin<bank0::Gpio14, rp_pico::hal::gpio::FunctionI2C, rp_pico::hal::gpio::PullNone>,
	Pin<bank0::Gpio15, rp_pico::hal::gpio::FunctionI2C, rp_pico::hal::gpio::PullNone>,
);

type SpiBus = hal::Spi<
	hal::spi::Enabled,
	pac::SPI0,
	(
		Pin<Gpio19, FunctionSpi, PullNone>,
		Pin<Gpio16, FunctionSpi, PullNone>,
		Pin<Gpio18, FunctionSpi, PullNone>,
	),
	8,
>;

/// What state our SD Card can be in
#[derive(defmt::Format, Debug, Clone, PartialEq, Eq)]
enum CardState {
	Unplugged,
	Uninitialised,
	#[allow(unused)]
	Online(CardInfo),
	#[allow(unused)]
	Errored,
}

/// Describes an SD card we've discovered
#[derive(defmt::Format, Debug, Clone, PartialEq, Eq)]
struct CardInfo {
	/// Number of blocks on the SD card
	num_blocks: u64,
	/// Type of card
	card_type: embedded_sdmmc::sdcard::CardType,
}

/// All the hardware we use on the Pico
struct Hardware {
	/// All the pins we use on the Raspberry Pi Pico
	pins: Pins,
	/// The SPI bus connected to all the slots
	spi_bus: SpiBus,
	/// Something to perform small delays with. Uses SysTICK.
	delay: cortex_m::delay::Delay,
	/// Current 5-bit value shown on the LEDs (including the HDD in bit 0).
	led_state: u8,
	/// The last CS pin we selected
	last_cs: u8,
	/// Our keyboard decoder
	keyboard: pc_keyboard::ScancodeSet2,
	/// Our queue of HID events
	event_queue: heapless::Deque<common::hid::HidEvent, 16>,
	/// A place to send/receive bytes to/from the BMC
	bmc_buffer: [u8; 64],
	/// Our last interrupt read from the IO chip. It's inverted, so a bit set
	/// means an interrupt is pending on that slot.
	interrupts_pending: u8,
	/// The number of IRQs we've had
	irq_count: u32,
	/// Our I2C Bus
	i2c: shared_bus::BusManagerSimple<hal::i2c::I2C<pac::I2C1, I2cPins, hal::i2c::Controller>>,
	/// Our External RTC (on the I2C bus)
	rtc: rtc::Rtc,
	/// The time we started up at, in microseconds since the Neotron epoch
	bootup_at: Duration,
	/// A 1 MHz Timer
	timer: hal::timer::Timer,
	/// the state of our SD Card
	card_state: CardState,
	/// Tracks all the clocks in the RP2040
	clocks: ClocksManager,
	/// The watchdog
	watchdog: hal::Watchdog,
	/// The audio CODEC
	audio_codec: tlv320aic23::Codec,
	/// PCM sample output queue
	audio_player: i2s::Player,
	/// How we are currently configured
	audio_config: common::audio::Config,
}

/// Flips between true and false so we always send a unique read request
struct UseAlt(AtomicBool);

impl UseAlt {
	const fn new() -> UseAlt {
		UseAlt(AtomicBool::new(false))
	}

	fn get(&self) -> bool {
		let use_alt = self.0.load(Ordering::Relaxed);
		self.0.store(!use_alt, Ordering::Relaxed);
		use_alt
	}
}

/// All the pins we use on the Pico, in the right mode.
///
/// We ignore the 'unused' lint, because these pins merely need to exist in
/// order to demonstrate the hardware is in the right state.
#[allow(unused)]
struct Pins {
	h_sync: Pin<bank0::Gpio0, FunctionPio0, PullNone>,
	v_sync: Pin<bank0::Gpio1, FunctionPio0, PullNone>,
	red0: Pin<bank0::Gpio2, FunctionPio0, PullNone>,
	red1: Pin<bank0::Gpio3, FunctionPio0, PullNone>,
	red2: Pin<bank0::Gpio4, FunctionPio0, PullNone>,
	red3: Pin<bank0::Gpio5, FunctionPio0, PullNone>,
	green0: Pin<bank0::Gpio6, FunctionPio0, PullNone>,
	green1: Pin<bank0::Gpio7, FunctionPio0, PullNone>,
	green2: Pin<bank0::Gpio8, FunctionPio0, PullNone>,
	green3: Pin<bank0::Gpio9, FunctionPio0, PullNone>,
	blue0: Pin<bank0::Gpio10, FunctionPio0, PullNone>,
	blue1: Pin<bank0::Gpio11, FunctionPio0, PullNone>,
	blue2: Pin<bank0::Gpio12, FunctionPio0, PullNone>,
	blue3: Pin<bank0::Gpio13, FunctionPio0, PullNone>,
	npower_save: Pin<bank0::Gpio23, FunctionSioOutput, PullNone>,
	nspi_cs_io: Pin<bank0::Gpio17, FunctionSioOutput, PullNone>,
	noutput_en: Pin<bank0::Gpio21, FunctionSioOutput, PullNone>,
	i2s_adc_data: Pin<bank0::Gpio22, FunctionPio1, PullNone>,
	i2s_dac_data: Pin<bank0::Gpio26, FunctionPio1, PullNone>,
	i2s_bit_clock: Pin<bank0::Gpio27, FunctionPio1, PullNone>,
	i2s_lr_clock: Pin<bank0::Gpio28, FunctionPio1, PullNone>,
}

// -----------------------------------------------------------------------------
// Static and Const Data
// -----------------------------------------------------------------------------

/// Version string auto-generated in build.rs
static VERSION: &str = include_str!(concat!(env!("OUT_DIR"), "/version.txt"));

/// Ensures we always send a unique read request
static USE_ALT: UseAlt = UseAlt::new();

/// Our global hardware object.
///
/// You need to grab this to do anything with the hardware.
static HARDWARE: NeoMutex<Option<Hardware>> = NeoMutex::new(None);

/// The pin we use for external interrupt input
static IRQ_PIN: NeoMutex<Option<IrqPin>> = NeoMutex::new(None);

/// This is our Operating System. It must be compiled separately.
///
/// The RP2040 requires an OS linked at `0x1002_0000` and compiled for the
/// `thumbv6m-none-eabi` target. You should therefore use the binary
/// `thumbv6m-none-eabi-flash1002-libneotron_os.bin` from
/// <https://github.com/Neotron-Compute/Neotron-OS/releases>
#[link_section = ".flash_os"]
#[used]
pub static OS_IMAGE: [u8; include_bytes!("thumbv6m-none-eabi-flash1002-libneotron_os.bin").len()] =
	*include_bytes!("thumbv6m-none-eabi-flash1002-libneotron_os.bin");

/// Tracks if we have had an IO interrupt.
///
/// Set by the GPIO interrupt routine to reflect the level state of the IRQ input
/// from the IO chip. This this value is `true` if any of `IRQ0` through `IRQ7`
/// is currently active (low).
static INTERRUPT_PENDING: AtomicBool = AtomicBool::new(false);

/// Stack for Core 1
static mut CORE1_STACK: [usize; 256] = [0; 256];

/// The table of API calls we provide the OS
static API_CALLS: common::Api = common::Api {
	api_version_get,
	bios_version_get,
	serial_configure,
	serial_get_info,
	serial_write,
	serial_read,
	time_clock_get,
	time_clock_set,
	configuration_get,
	configuration_set,
	video_is_valid_mode,
	video_set_mode,
	video_get_mode,
	video_get_framebuffer,
	video_set_framebuffer,
	memory_get_region,
	video_mode_needs_vram,
	hid_get_event,
	hid_set_leds,
	video_wait_for_line,
	block_dev_get_info,
	block_write,
	block_read,
	block_verify,
	time_ticks_get,
	time_ticks_per_second,
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
	bus_interrupt_status,
	block_dev_eject,
	power_idle,
	power_control,
	compare_and_swap_bool,
};

/// Seconds between the Neotron Epoch (2000-01-01T00:00:00) and the UNIX Epoch (1970-01-01T00:00:00).
const SECONDS_BETWEEN_UNIX_AND_NEOTRON_EPOCH: i64 = 946684800;

extern "C" {
	static mut _flash_os_start: u32;
	static mut _flash_os_len: u32;
	static mut _ram_os_start: u32;
	static mut _ram_os_len: u32;
}

/// What we paint Core 0's stack with
const CORE0_STACK_PAINT_WORD: usize = 0xBBBB_BBBB;

/// What we paint Core 1's stack with
const CORE1_STACK_PAINT_WORD: usize = 0xCCCC_CCCC;

// -----------------------------------------------------------------------------
// Functions
// -----------------------------------------------------------------------------

/// This is the entry-point to the BIOS. It is called by cortex-m-rt once the
/// `.bss` and `.data` sections have been initialised.
#[entry]
fn main() -> ! {
	cortex_m::interrupt::disable();

	// Grab the singleton containing all the RP2040 peripherals
	let mut pp = pac::Peripherals::take().unwrap();
	// Grab the singleton containing all the generic Cortex-M peripherals
	let cp = pac::CorePeripherals::take().unwrap();

	// Check if stuff is running that shouldn't be. If so, do a full watchdog reboot.
	if stuff_running(&mut pp) {
		watchdog_reboot();
	}

	// Reset the DMA engine. If we don't do this, starting from probe-rs
	// (as opposed to a cold-start) is unreliable.
	reset_dma_engine(&mut pp);

	// Reset the spinlocks.
	pp.SIO.spinlock[31].reset();

	paint_stacks();
	check_stacks();

	// Needed by the clock setup
	let mut watchdog = hal::watchdog::Watchdog::new(pp.WATCHDOG);

	// VERSION has a trailing `\0` as that is what the BIOS/OS API requires.
	info!(
		"Neotron BIOS v{} (git:{}) starting...",
		env!("CARGO_PKG_VERSION"),
		VERSION.trim_matches('\0')
	);

	// Run at 151.2 MHz SYS_PLL, 48 MHz, USB_PLL. This is important, we as clock
	// the PIO at ÷ 6, to give 25.2 MHz (which is close enough to the 25.175
	// MHz standard VGA pixel clock).

	// Step 1. Turn on the crystal.
	let xosc = hal::xosc::setup_xosc_blocking(pp.XOSC, rp_pico::XOSC_CRYSTAL_FREQ.Hz())
		.map_err(|_x| false)
		.unwrap();
	// Step 2. Configure watchdog tick generation to tick over every microsecond.
	watchdog.enable_tick_generation((rp_pico::XOSC_CRYSTAL_FREQ / 1_000_000) as u8);
	// Step 3. Create a clocks manager.
	let mut clocks = hal::clocks::ClocksManager::new(pp.CLOCKS);
	// Step 4. Set up the system PLL.
	//
	// We take the Crystal Oscillator (=12 MHz) with no divider, and ×126 to
	// give a FOUTVCO of 1512 MHz. This must be in the range 750 MHz - 1600 MHz.
	// The factor of 126 is calculated automatically given the desired FOUTVCO.
	//
	// Next we ÷5 on the first post divider to give 302.4 MHz.
	//
	// Finally we ÷2 on the second post divider to give 151.2 MHz.
	//
	// We note from the [RP2040
	// Datasheet](https://datasheets.raspberrypi.com/rp2040/rp2040-datasheet.pdf),
	// Section 2.18.2.1:
	//
	// > Jitter is minimised by running the VCO at the highest possible
	// > frequency, so that higher post-divide values can be used.
	let pll_sys = hal::pll::setup_pll_blocking(
		pp.PLL_SYS,
		xosc.operating_frequency(),
		hal::pll::PLLConfig {
			vco_freq: 1512.MHz(),
			refdiv: 1,
			post_div1: 5,
			post_div2: 2,
		},
		&mut clocks,
		&mut pp.RESETS,
	)
	.map_err(|_x| false)
	.unwrap();
	// Step 5. Set up a 48 MHz PLL for the USB system.
	let pll_usb = hal::pll::setup_pll_blocking(
		pp.PLL_USB,
		xosc.operating_frequency(),
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

	info!("SCKDV: {}", unsafe {
		(*rp_pico::pac::XIP_SSI::ptr()).baudr.read().bits()
	});

	// sio is the *Single-cycle Input/Output* peripheral. It has all our GPIO
	// pins, as well as some mailboxes and other useful things for inter-core
	// communications.
	let mut sio = hal::sio::Sio::new(pp.SIO);

	// This object lets us wait for long periods of time (to make things readable on screen)
	let delay = cortex_m::delay::Delay::new(cp.SYST, clocks.system_clock.freq().to_Hz());

	// Configure and grab all the RP2040 pins the Pico exposes, along with the non-VGA peripherals.
	let (mut hw, nirq_io) = Hardware::build(
		pp.IO_BANK0,
		pp.PADS_BANK0,
		sio.gpio_bank0,
		pp.SPI0,
		pp.I2C1,
		pp.TIMER,
		pp.PIO1,
		clocks,
		delay,
		watchdog,
		&mut pp.RESETS,
	);
	hw.init_io_chip();
	hw.set_hdd_led(false);

	nirq_io.set_interrupt_enabled(hal::gpio::Interrupt::EdgeLow, true);
	nirq_io.set_interrupt_enabled(hal::gpio::Interrupt::EdgeHigh, true);

	info!("Pins OK");

	vga::init(
		pp.PIO0,
		pp.DMA,
		&mut pp.RESETS,
		&mut pp.PPB,
		&mut sio.fifo,
		&mut pp.PSM,
	);

	info!("VGA OK");

	// Did we start with an interrupt pending?
	INTERRUPT_PENDING.store(nirq_io.is_low().unwrap(), Ordering::Relaxed);

	// Check for interrupts on start-up (particularly the SD card being in)
	hw.io_poll_interrupts(true);

	{
		let mut lock = HARDWARE.lock();
		lock.replace(hw);
	}

	{
		// You can only do this before interrupts are enabled. Otherwise you may
		// try and grab the mutex in an ISR whilst it's held in the main thread,
		// and that causes a panic.
		let mut lock = IRQ_PIN.lock();
		lock.replace(nirq_io);
	}

	// Unmask the IO_BANK0 IRQ so that the NVIC interrupt controller
	// will jump to the interrupt function when the interrupt occurs.
	// Then enable interrupts on Core 0.
	//
	// We do this last so that the interrupt can't go off while
	// it is in the middle of being configured.
	unsafe {
		pac::NVIC::unmask(pac::Interrupt::IO_IRQ_BANK0);
		cortex_m::interrupt::enable();
	}

	// Empty the keyboard FIFO
	while let ApiResult::Ok(FfiOption::Some(_x)) = hid_get_event() {
		// Spin
	}

	// Say hello over VGA
	sign_on();

	// Now jump to the OS
	let code: &common::OsStartFn = unsafe { ::core::mem::transmute(&_flash_os_start) };
	code(&API_CALLS);
}

/// Check if the rest of the system appears to be running already.
///
/// If so, returns `true`, else `false`.
///
/// This probably means Core 0 reset but the rest of the system did not, and you
/// should do a hard reset to get everything back into sync.
fn stuff_running(p: &mut pac::Peripherals) -> bool {
	// Look at scratch register 7 and see what we left ourselves. If it's zero,
	// this was a full clean boot-up. If it's 0xDEADC0DE, this means we were
	// running and Core 0 restarted without restarting everything else.
	let scratch = p.WATCHDOG.scratch7.read().bits();
	defmt::info!("WD Scratch is 0x{:08x}", scratch);
	if scratch == 0xDEADC0DE {
		// we need a hard reset
		true
	} else {
		// set the marker so we know Core 0 has booted up
		p.WATCHDOG.scratch7.write(|w| unsafe { w.bits(0xDEADC0DE) });
		false
	}
}

/// Clear the scratch register so we don't force a full watchdog reboot on the
/// next boot.
fn clear_scratch() {
	let p = unsafe { pac::Peripherals::steal() };
	p.WATCHDOG.scratch7.write(|w| unsafe { w.bits(0) });
}

/// Do a full watchdog reboot
fn watchdog_reboot() -> ! {
	use embedded_hal::watchdog::WatchdogEnable;
	clear_scratch();
	if let Some(hw) = HARDWARE.lock().as_mut() {
		hw.watchdog
			.start(fugit::Duration::<u32, 1, 1000000>::millis(10));
	} else {
		// Hardware hasn't been initialised, so make a temporary watchdog driver
		let p = unsafe { pac::Peripherals::steal() };
		let mut watchdog = hal::Watchdog::new(p.WATCHDOG);
		watchdog.start(fugit::Duration::<u32, 1, 1000000>::millis(10));
	}
	loop {
		cortex_m::asm::wfi();
	}
}

/// Print the BIOS boot-up messages on a temporary text console
fn sign_on() {
	static LOGO_ANSI_BYTES: &[u8] = include_bytes!(concat!(env!("OUT_DIR"), "/logo.bytes"));
	static COPYRIGHT_TEXT: &str = "\
		\n\
		Copyright © Jonathan 'theJPster' Pallant and the Neotron Developers, 2023\n\
		This program is free software under GPL v3 (or later)\n\
		You are entitled to the Complete and Corresponding Source for this BIOS\n\
		\n\
		BIOS: \n\
		BMC : \n\
		\n\
		Press ESC to pause...";

	// Create a new temporary console for some boot-up messages
	let tc = console::TextConsole::new();
	tc.set_text_buffer(unsafe { &mut vga::GLYPH_ATTR_ARRAY });

	tc.change_attr(Attr::new(
		TextForegroundColour::WHITE,
		TextBackgroundColour::BLUE,
		false,
	));

	// A crude way to clear the screen
	for _col in 0..vga::MAX_TEXT_ROWS {
		writeln!(&tc, " ").unwrap();
	}

	// Draw the logo
	tc.move_to(0, 0);
	for pair in LOGO_ANSI_BYTES.chunks(2) {
		tc.change_attr(Attr(pair[0]));
		tc.write_font_glyph(neotron_common_bios::video::Glyph(pair[1]));
	}

	tc.change_attr(Attr::new(
		TextForegroundColour::WHITE,
		TextBackgroundColour::BLUE,
		false,
	));

	write!(&tc, "{}", COPYRIGHT_TEXT).unwrap();

	// Draw the BIOS version
	tc.change_attr(Attr::new(
		TextForegroundColour::YELLOW,
		TextBackgroundColour::BLACK,
		false,
	));
	tc.move_to(12, 6);
	write!(
		&tc,
		"v{} (git:{})",
		env!("CARGO_PKG_VERSION"),
		VERSION.trim_matches('\0')
	)
	.unwrap();

	// Draw the BMC version
	tc.move_to(13, 6);
	tc.change_attr(Attr::new(
		TextForegroundColour::WHITE,
		TextBackgroundColour::RED,
		false,
	));
	let bmc_ver = {
		let mut lock = HARDWARE.lock();
		let hw = lock.as_mut().unwrap();
		let ver = hw.bmc_read_firmware_version();
		let _ = hw.play_startup_tune();
		ver
	};
	match bmc_ver {
		Ok(string_bytes) => match core::str::from_utf8(&string_bytes) {
			Ok(s) => {
				write!(&tc, "{}", s.trim_matches('\0')).unwrap();
			}
			Err(_e) => {
				write!(&tc, "Version Unknown").unwrap();
			}
		},
		Err(_e) => {
			write!(&tc, "Error reading version").unwrap();
		}
	}

	tc.change_attr(Attr::new(
		TextForegroundColour::WHITE,
		TextBackgroundColour::BLUE,
		false,
	));

	// This is in 100ms units
	let mut countdown = 15;
	loop {
		{
			let mut lock = HARDWARE.lock();
			let hw = lock.as_mut().unwrap();
			hw.delay.delay_ms(100);
		}

		tc.move_to(15, 0);
		match hid_get_event() {
			ApiResult::Ok(FfiOption::Some(HidEvent::KeyPress(KeyCode::Escape))) => {
				write!(&tc, "Paused - Press SPACE to continue...").unwrap();
				countdown = 0;
			}
			ApiResult::Ok(FfiOption::Some(HidEvent::KeyPress(KeyCode::Spacebar))) => {
				write!(&tc, "Unpaused                           ").unwrap();
				countdown = 10;
			}
			_ => {
				// ignore
			}
		}
		if countdown == 1 {
			break;
		} else {
			countdown -= 1;
		}
	}
}

/// Paint the Core 0 and Core 1 stacks
fn paint_stacks() {
	extern "C" {
		static mut __sheap: usize;
		static mut _stack_start: usize;
	}
	unsafe {
		let stack_len =
			(&_stack_start as *const usize as usize) - (&__sheap as *const usize as usize);
		// But not the top 64 words, because we're using the stack right now!
		let stack = core::slice::from_raw_parts_mut(
			&mut __sheap as *mut usize,
			(stack_len / core::mem::size_of::<usize>()) - 256,
		);
		info!("Painting Core 1 stack: {:?}", stack.as_ptr_range());
		for b in stack.iter_mut() {
			*b = CORE0_STACK_PAINT_WORD;
		}

		info!("Painting Core 1 stack: {:?}", CORE1_STACK.as_ptr_range());
		for b in CORE1_STACK.iter_mut() {
			*b = CORE1_STACK_PAINT_WORD;
		}
	}
}

/// Reset the DMA Peripheral.
fn reset_dma_engine(pp: &mut pac::Peripherals) {
	pp.RESETS.reset.modify(|_r, w| w.dma().set_bit());
	cortex_m::asm::nop();
	pp.RESETS.reset.modify(|_r, w| w.dma().clear_bit());
	while pp.RESETS.reset_done.read().dma().bit_is_clear() {}
}

/// Measure how much stack space remains unused.
#[cfg(feature = "check-stack")]
fn check_stacks() {
	extern "C" {
		static mut __sheap: usize;
		static mut _stack_start: usize;
	}
	let stack_len =
		unsafe { (&_stack_start as *const usize as usize) - (&__sheap as *const usize as usize) };
	check_stack(
		unsafe { &__sheap as *const usize },
		stack_len,
		CORE0_STACK_PAINT_WORD,
	);
	check_stack(
		unsafe { CORE1_STACK.as_ptr() },
		unsafe { CORE1_STACK.len() * core::mem::size_of::<usize>() },
		CORE1_STACK_PAINT_WORD,
	);
}

/// Dummy stack checker that does nothing
#[cfg(not(feature = "check-stack"))]
fn check_stacks() {}

/// Check an individual stack to see how much has been used
#[cfg(feature = "check-stack")]
fn check_stack(start: *const usize, stack_len_bytes: usize, check_word: usize) {
	let mut p: *const usize = start;
	let mut free_bytes = 0;
	loop {
		let value = unsafe { p.read_volatile() };
		if value != check_word {
			break;
		}
		// Safety: We must hit a used stack value somewhere!
		p = unsafe { p.offset(1) };
		free_bytes += core::mem::size_of::<usize>();
	}
	defmt::info!(
		"Stack free at 0x{:08x}: {} bytes used of {} bytes",
		start,
		stack_len_bytes - free_bytes,
		stack_len_bytes
	);
}

// -----------------------------------------------------------------------------
// Type impl blocks
// -----------------------------------------------------------------------------

impl Hardware {
	/// How fast can the I/O chip SPI CLK input go?
	const CLOCK_IO: fugit::Rate<u32, 1, 1> = fugit::Rate::<u32, 1, 1>::Hz(10_000_000);

	/// How fast can the BMC SPI CLK input go?
	const CLOCK_BMC: fugit::Rate<u32, 1, 1> = fugit::Rate::<u32, 1, 1>::Hz(2_000_000);

	/// How many nano seconds per clock cycle (at 126 MHz)?
	const NS_PER_CLOCK_CYCLE: u32 = 1_000_000_000 / 126_000_000;

	/// MCP23S17 CS pin setup time (before transaction). At least 50ns, we give 100ns.
	const CS_IO_SETUP_CPU_CLOCKS: u32 = 100 / Self::NS_PER_CLOCK_CYCLE;

	/// MCP23S17 CS pin hold time (and end of transaction). At least 50ns, we give 100ns.
	const CS_IO_HOLD_CPU_CLOCKS: u32 = 100 / Self::NS_PER_CLOCK_CYCLE;

	/// MCP23S17 CS pin disable time (between transactions). At least 50ns, we give 100ns.
	const CS_IO_DISABLE_CPU_CLOCKS: u32 = 100 / Self::NS_PER_CLOCK_CYCLE;

	/// Give the device 10us to get ready.
	///
	/// This seems to reduce the error rate on the BMC link to an acceptable level.
	const CS_BUS_SETUP_CPU_CLOCKS: u32 = 10_000 / Self::NS_PER_CLOCK_CYCLE;

	/// Give the device 2000ns before we take away CS.
	const CS_BUS_HOLD_CPU_CLOCKS: u32 = 2000 / Self::NS_PER_CLOCK_CYCLE;

	/// Give the device 100ms to sort itself out when we do a retry.
	const BMC_RETRY_CPU_CLOCKS: u32 = 100_000_000 / Self::NS_PER_CLOCK_CYCLE;

	/// Give the BMC 6us to calculate its response
	const BMC_REQUEST_RESPONSE_DELAY_CLOCKS: u32 = 6_000 / Self::NS_PER_CLOCK_CYCLE;

	/// Build all our hardware drivers.
	///
	/// Puts the pins into the right modes, builds the SPI driver, etc.
	#[allow(clippy::too_many_arguments)]
	fn build(
		bank: pac::IO_BANK0,
		pads: pac::PADS_BANK0,
		sio: hal::sio::SioGpioBank0,
		spi: pac::SPI0,
		i2c: pac::I2C1,
		timer: pac::TIMER,
		pio1: pac::PIO1,
		clocks: ClocksManager,
		delay: cortex_m::delay::Delay,
		watchdog: hal::Watchdog,
		resets: &mut pac::RESETS,
	) -> (Hardware, IrqPin) {
		let hal_pins = rp_pico::Pins::new(bank, pads, sio, resets);
		// We construct the pin here and then throw it away. Then Core 1 does
		// some unsafe writes to the GPIO_SET/GPIO_CLEAR registers to set/clear
		// pin 25 to track render loop timing. This avoids trying to 'move' the pin
		// over to Core 1.
		let _pico_led = hal_pins.led.into_push_pull_output();
		let raw_i2c = hal::i2c::I2C::i2c1(
			i2c,
			{
				let mut pin = hal_pins.gpio14.reconfigure();
				pin.set_drive_strength(hal::gpio::OutputDriveStrength::EightMilliAmps);
				pin.set_slew_rate(hal::gpio::OutputSlewRate::Fast);
				pin
			},
			{
				let mut pin = hal_pins.gpio15.reconfigure();
				pin.set_drive_strength(hal::gpio::OutputDriveStrength::EightMilliAmps);
				pin.set_slew_rate(hal::gpio::OutputSlewRate::Fast);
				pin
			},
			100.kHz(),
			resets,
			&clocks.system_clock,
		);
		let i2c = shared_bus::BusManagerSimple::new(raw_i2c);
		let proxy = i2c.acquire_i2c();
		let mut external_rtc = rtc::Rtc::new(proxy);
		let timer = hal::timer::Timer::new(timer, resets, &clocks);
		// Do a conversion from external RTC time (chrono::NaiveDateTime) to a format we can track
		let ticks_at_boot_us = match external_rtc.get_time(i2c.acquire_i2c()) {
			Ok(time) => {
				defmt::info!(
					"Time: {:04}-{:02}-{:02} {:02}:{:02}:{:02}",
					time.year(),
					time.month(),
					time.day(),
					time.hour(),
					time.minute(),
					time.second()
				);
				let ticks_at_boot_us =
					time.timestamp_micros() - (SECONDS_BETWEEN_UNIX_AND_NEOTRON_EPOCH * 1_000_000);
				defmt::info!("Ticks at boot: {}", ticks_at_boot_us);
				ticks_at_boot_us
			}
			Err(_) => {
				defmt::info!("Time: Unknown");
				0
			}
		};

		let pins = Pins {
			// Disable power save mode to force SMPS into low-efficiency, low-noise mode.
			npower_save: {
				let mut pin = hal_pins.b_power_save.reconfigure();
				pin.set_high().unwrap();
				pin
			},
			// Give H-Sync, V-Sync and 12 RGB colour pins to PIO0 to output video
			h_sync: {
				let mut pin = hal_pins.gpio0.reconfigure();
				pin.set_drive_strength(hal::gpio::OutputDriveStrength::EightMilliAmps);
				pin.set_slew_rate(hal::gpio::OutputSlewRate::Fast);
				pin
			},
			v_sync: {
				let mut pin = hal_pins.gpio1.reconfigure();
				pin.set_drive_strength(hal::gpio::OutputDriveStrength::EightMilliAmps);
				pin.set_slew_rate(hal::gpio::OutputSlewRate::Fast);
				pin
			},
			red0: {
				let mut pin = hal_pins.gpio2.reconfigure();
				pin.set_drive_strength(hal::gpio::OutputDriveStrength::EightMilliAmps);
				pin.set_slew_rate(hal::gpio::OutputSlewRate::Fast);
				pin
			},
			red1: {
				let mut pin = hal_pins.gpio3.reconfigure();
				pin.set_drive_strength(hal::gpio::OutputDriveStrength::EightMilliAmps);
				pin.set_slew_rate(hal::gpio::OutputSlewRate::Fast);
				pin
			},
			red2: {
				let mut pin = hal_pins.gpio4.reconfigure();
				pin.set_drive_strength(hal::gpio::OutputDriveStrength::EightMilliAmps);
				pin.set_slew_rate(hal::gpio::OutputSlewRate::Fast);
				pin
			},
			red3: {
				let mut pin = hal_pins.gpio5.reconfigure();
				pin.set_drive_strength(hal::gpio::OutputDriveStrength::EightMilliAmps);
				pin.set_slew_rate(hal::gpio::OutputSlewRate::Fast);
				pin
			},
			green0: {
				let mut pin = hal_pins.gpio6.reconfigure();
				pin.set_drive_strength(hal::gpio::OutputDriveStrength::EightMilliAmps);
				pin.set_slew_rate(hal::gpio::OutputSlewRate::Fast);
				pin
			},
			green1: {
				let mut pin = hal_pins.gpio7.reconfigure();
				pin.set_drive_strength(hal::gpio::OutputDriveStrength::EightMilliAmps);
				pin.set_slew_rate(hal::gpio::OutputSlewRate::Fast);
				pin
			},
			green2: {
				let mut pin = hal_pins.gpio8.reconfigure();
				pin.set_drive_strength(hal::gpio::OutputDriveStrength::EightMilliAmps);
				pin.set_slew_rate(hal::gpio::OutputSlewRate::Fast);
				pin
			},
			green3: {
				let mut pin = hal_pins.gpio9.reconfigure();
				pin.set_drive_strength(hal::gpio::OutputDriveStrength::EightMilliAmps);
				pin.set_slew_rate(hal::gpio::OutputSlewRate::Fast);
				pin
			},
			blue0: {
				let mut pin = hal_pins.gpio10.reconfigure();
				pin.set_drive_strength(hal::gpio::OutputDriveStrength::EightMilliAmps);
				pin.set_slew_rate(hal::gpio::OutputSlewRate::Fast);
				pin
			},
			blue1: {
				let mut pin = hal_pins.gpio11.reconfigure();
				pin.set_drive_strength(hal::gpio::OutputDriveStrength::EightMilliAmps);
				pin.set_slew_rate(hal::gpio::OutputSlewRate::Fast);
				pin
			},
			blue2: {
				let mut pin = hal_pins.gpio12.reconfigure();
				pin.set_drive_strength(hal::gpio::OutputDriveStrength::EightMilliAmps);
				pin.set_slew_rate(hal::gpio::OutputSlewRate::Fast);
				pin
			},
			blue3: {
				let mut pin = hal_pins.gpio13.reconfigure();
				pin.set_drive_strength(hal::gpio::OutputDriveStrength::EightMilliAmps);
				pin.set_slew_rate(hal::gpio::OutputSlewRate::Fast);
				pin
			},
			nspi_cs_io: {
				let mut pin = hal_pins.gpio17.reconfigure();
				pin.set_high().unwrap();
				pin
			},
			noutput_en: {
				let mut pin = hal_pins.gpio21.reconfigure();
				pin.set_high().unwrap();
				pin
			},
			i2s_adc_data: hal_pins.gpio22.reconfigure(),
			i2s_dac_data: hal_pins.gpio26.reconfigure(),
			i2s_bit_clock: hal_pins.gpio27.reconfigure(),
			i2s_lr_clock: hal_pins.gpio28.reconfigure(),
		};

		let mut audio_codec = tlv320aic23::Codec::new(tlv320aic23::BusAddress::CsLow);
		if let Err(hal::i2c::Error::Abort(code)) = audio_codec.reset(&mut i2c.acquire_i2c()) {
			defmt::error!("CODEC did not respond: code {:?}", code);
		}
		audio_codec.set_digital_interface_enabled(true);
		audio_codec.set_dac_selected(true);
		audio_codec.set_dac_mute(false);
		audio_codec.set_bypass(false);
		audio_codec.set_line_input_mute(true, tlv320aic23::Channel::Both);
		audio_codec.set_powered_on(tlv320aic23::Subsystem::AnalogDigitalConverter, true);
		audio_codec.set_powered_on(tlv320aic23::Subsystem::MicrophoneInput, true);
		audio_codec.set_powered_on(tlv320aic23::Subsystem::LineInput, true);
		// Set volume to -6dB. 73 = 0 dB, and it's 1 dB per step.
		audio_codec.set_headphone_output_volume(64, tlv320aic23::Channel::Both);
		audio_codec.set_sample_rate(
			tlv320aic23::CONFIG_USB_48K,
			tlv320aic23::LrSwap::Disabled,
			tlv320aic23::LrPhase::RightOnHigh,
			tlv320aic23::Mode::Controller,
			tlv320aic23::InputBitLength::B16,
			tlv320aic23::DataFormat::I2s,
		);
		let audio_config = common::audio::Config {
			sample_format: common::audio::SampleFormat::SixteenBitStereo,
			sample_rate_hz: 48000,
		};
		if let Err(hal::i2c::Error::Abort(code)) = audio_codec.sync(&mut i2c.acquire_i2c()) {
			defmt::error!("CODEC did not respond: code {:?}", code);
		}

		let audio_player = i2s::init(pio1, resets);

		let spi_copi = {
			let mut pin = hal_pins.gpio19.reconfigure();
			pin.set_drive_strength(hal::gpio::OutputDriveStrength::EightMilliAmps);
			pin.set_slew_rate(hal::gpio::OutputSlewRate::Fast);
			pin
		};
		let spi_cipo = {
			let mut pin = hal_pins.gpio16.reconfigure();
			pin.set_drive_strength(hal::gpio::OutputDriveStrength::EightMilliAmps);
			pin.set_slew_rate(hal::gpio::OutputSlewRate::Fast);
			pin
		};
		let spi_clk = {
			let mut pin = hal_pins.gpio18.reconfigure();
			pin.set_drive_strength(hal::gpio::OutputDriveStrength::EightMilliAmps);
			pin.set_slew_rate(hal::gpio::OutputSlewRate::Fast);
			pin
		};

		(
			Hardware {
				pins,
				// We are in SPI MODE 0. This means we change the COPI pin on the
				// CLK falling edge, and we sample the CIPO pin on the CLK rising
				// edge.
				//
				// SPI clock speed here is irrelevant - we change it depending on the device.
				spi_bus: hal::Spi::new(spi, (spi_copi, spi_cipo, spi_clk)).init(
					resets,
					clocks.peripheral_clock.freq(),
					2_000_000.Hz(),
					embedded_hal::spi::MODE_0,
				),
				delay,
				led_state: 0,
				last_cs: 0,
				keyboard: pc_keyboard::ScancodeSet2::new(),
				event_queue: heapless::Deque::new(),
				bmc_buffer: [0u8; 64],
				interrupts_pending: 0,
				irq_count: 0,
				i2c,
				rtc: external_rtc,
				bootup_at: Duration::from_ticks(ticks_at_boot_us as u64),
				timer,
				card_state: CardState::Unplugged,
				clocks,
				watchdog,
				audio_codec,
				audio_player,
				audio_config,
			},
			hal_pins.gpio20.reconfigure(),
		)
	}

	/// Perform some SPI operation with the I/O chip selected.
	///
	/// You are required to have called `self.release_cs_lines()` previously,
	/// otherwise the I/O chip and your selected bus device will both see a
	/// chip-select signal.
	fn with_io_cs<F, T>(&mut self, func: F) -> T
	where
		F: FnOnce(&mut SpiBus) -> T,
	{
		self.spi_bus
			.set_baudrate(self.clocks.peripheral_clock.freq(), Self::CLOCK_IO);

		// Select MCP23S17
		self.pins.nspi_cs_io.set_low().unwrap();
		// Setup time
		cortex_m::asm::delay(Self::CS_IO_SETUP_CPU_CLOCKS);
		// Do the SPI thing
		let result = func(&mut self.spi_bus);
		// Hold the CS pin a bit longer
		cortex_m::asm::delay(Self::CS_IO_HOLD_CPU_CLOCKS);
		// Release the CS pin
		self.pins.nspi_cs_io.set_high().unwrap();
		// Return the result from the closure
		result
	}

	/// Write to a register on the MCP23S17 I/O chip.
	///
	/// * `register` - the address of the register to write to
	/// * `value` - the value to write
	fn io_chip_write(&mut self, register: mcp23s17::Register, value: u8) {
		// Inter-packet delay
		cortex_m::asm::delay(Self::CS_IO_DISABLE_CPU_CLOCKS);

		// Do the operation with CS pin active
		self.with_io_cs(|spi| {
			mcp23s17::write_register(spi, register, value);
		});

		// Inter-packet delay
		cortex_m::asm::delay(Self::CS_IO_DISABLE_CPU_CLOCKS);

		let read_back = self.io_chip_read(register);
		if read_back != value {
			defmt::panic!(
				"Wrote 0x{:02x} to IO chip register {:?}, got 0x{:02x}",
				value,
				register,
				read_back
			);
		}
	}

	/// Read from a register on the MCP23S17 I/O chip.
	///
	/// * `register` - the address of the register to read from
	fn io_chip_read(&mut self, register: mcp23s17::Register) -> u8 {
		// Inter-packet delay
		cortex_m::asm::delay(Self::CS_IO_DISABLE_CPU_CLOCKS);

		// Do the operation with CS pin active
		self.with_io_cs(|spi| mcp23s17::read_register(spi, register))
	}

	/// Set the four debug LEDs on the PCB.
	///
	/// These are connected to the top 4 bits of GPIOA on the MCP23S17.
	fn set_debug_leds(&mut self, leds: u8) {
		// LEDs are active-low.
		let leds = (leds ^ 0xFF) & 0xF;
		self.led_state = leds << 1 | (self.led_state & 1);
		self.io_chip_write(
			mcp23s17::Register::GPIOA,
			self.led_state << 3 | self.last_cs,
		);
	}

	/// Set the HDD LED on the PCB.
	///
	/// These are connected to the bit 4 of GPIOA on the MCP23S17.
	fn set_hdd_led(&mut self, enabled: bool) {
		// LEDs are active-low.
		self.led_state = (self.led_state & 0x1e) | u8::from(!enabled);
		self.io_chip_write(
			mcp23s17::Register::GPIOA,
			self.led_state << 3 | self.last_cs,
		);
	}

	/// Perform some SPI transaction with a specific bus chip-select pin active.
	///
	/// Activates a specific chip-select line, runs the closure (passing in the
	/// SPI bus object), then de-activates the CS pin.
	fn with_bus_cs<F>(&mut self, cs: u8, clock_speed: fugit::Rate<u32, 1, 1>, func: F)
	where
		F: FnOnce(&mut SpiBus, &mut [u8]),
	{
		// Only CS0..CS7 is valid
		let cs = cs & 0b111;

		if cs != self.last_cs {
			// Set CS Outputs into decoder/buffer
			self.io_chip_write(mcp23s17::Register::GPIOA, self.led_state << 3 | cs);
			self.last_cs = cs;
		}

		// Drive CS lines from decoder/buffer
		self.drive_cs_lines();

		// Setup time
		cortex_m::asm::delay(Self::CS_BUS_SETUP_CPU_CLOCKS);

		self.spi_bus
			.set_baudrate(self.clocks.peripheral_clock.freq(), clock_speed);

		// Call function
		func(&mut self.spi_bus, &mut self.bmc_buffer);

		// Hold the CS pin a bit longer
		cortex_m::asm::delay(Self::CS_BUS_HOLD_CPU_CLOCKS);

		// Undrive CS lines from decoder/buffer
		self.release_cs_lines();
	}

	/// Enable the 74HC138 so it drives one of the CS0..CS7 pins active-low
	/// (according to the 3 input bits).
	fn drive_cs_lines(&mut self) {
		self.pins.noutput_en.set_low().unwrap();
	}

	/// Disable the 74HC138 so it drives none of the CS0..CS7 pins active-low.
	fn release_cs_lines(&mut self) {
		self.pins.noutput_en.set_high().unwrap();
	}

	/// Configure the MCP23S17 correctly.
	///
	/// We have GPIOA as outputs (for the debug LEDs, HDD LED and 3-bit
	/// chip-select number). We have GPIOB as pulled-up inputs (for the eight
	/// interrupts).
	fn init_io_chip(&mut self) {
		// Undrive CS lines from decoder/buffer
		self.release_cs_lines();

		cortex_m::asm::delay(Self::CS_IO_DISABLE_CPU_CLOCKS);

		// Set IODIRA = 0x00 => GPIOA is all outputs
		self.io_chip_write(mcp23s17::Register::DDRA, 0x00);

		// Set GPIOA = 0x00 => GPIOA is all low
		self.io_chip_write(mcp23s17::Register::GPIOA, 0x00);

		// Set GPPUB to = 0xFF => GPIOB is pulled-up
		self.io_chip_write(mcp23s17::Register::GPPUB, 0xFF);

		// Set up interrupts. We want the line to go low if anything on GPIOB is
		// low (at least initially - we may flip the sense on the SD Card IRQ if
		// we find a card is inserted).

		// Set IPOL so all pins active low
		self.io_chip_write(mcp23s17::Register::IPOLB, 0xFF);

		// The INTCON register controls how the associated pin value is compared
		// for the interrupt-on-change feature. All the bits are set, so the
		// corresponding I/O pin is compared against the associated bit in the
		// DEFVAL register.
		self.io_chip_write(mcp23s17::Register::INTCONB, 0xFF);

		// All the bits are set, so the corresponding pins are enabled for
		// interrupt-on-change. The DEFVAL and INTCON registers must also be
		// configured if any pins are enabled for interrupt-on-change.
		self.io_chip_write(mcp23s17::Register::GPINTENB, 0xFF);

		// The default comparison value is configured in the DEFVAL register. If
		// enabled (via GPINTEN and INTCON) to compare against the DEFVAL
		// register, an opposite value on the associated pin will cause an
		// interrupt to occur. This the logical value after the polarity switch
		// in `IPOLB` has been applied.
		self.io_chip_write(mcp23s17::Register::DEFVALB, 0x00);
	}

	/// If the interrupt flag was set by the IRQ handler, read the interrupt
	/// bits from the MCP23S17.
	fn io_poll_interrupts(&mut self, force: bool) {
		if INTERRUPT_PENDING.load(Ordering::Relaxed) || force {
			// We use IOPOL to ensure a `1` bit means `Interrupt Pending`
			self.interrupts_pending = self.io_read_interrupts();
			let pol = self.io_chip_read(mcp23s17::Register::IPOLB);
			defmt::debug!(
				"MCP23S17 IRQ pins: 0b{:08b} (real 0b{:08b})",
				self.interrupts_pending,
				pol ^ self.interrupts_pending
			);
			// Change the debug LEDs so we can see the interrupts
			self.irq_count = self.irq_count.wrapping_add(1);
			self.set_debug_leds((self.irq_count & 0x0F) as u8);

			for irq in 0..8 {
				let irq_bit = 1 << irq;
				if (self.interrupts_pending & irq_bit) != 0 {
					self.io_handle_irq(irq);
				}
			}

			// We sometimes get stuck in a loop, due to bad programming. The IO
			// chip tells us we have an interrupt, but when we ask it which one,
			// it appears to say actually, there's no interrupt. This puts us
			// into an infinite loop, because the INTERRUPT_PENDING value isn't
			// cleared. This logic tries to catch that loop before sixty
			// bazillion lines of defmt are printed, causing the actual error to
			// disappear out of the terminal buffer.
			static BAD_IRQ_COUNT: AtomicU32 = AtomicU32::new(0);
			if self.interrupts_pending == 0 && !force {
				let count = BAD_IRQ_COUNT.load(Ordering::Relaxed) + 1;
				BAD_IRQ_COUNT.store(count, Ordering::Relaxed);
				if count > 5 {
					panic!("Unexpected interrupts in bagging area.");
				}
			} else {
				BAD_IRQ_COUNT.store(0, Ordering::Relaxed);
			}
		}
	}

	/// Handle an IRQ on a slot
	fn io_handle_irq(&mut self, slot: u8) {
		match slot {
			1 => {
				// Slot 1 interrupt, which is the SD Card. Flip the sense, so we
				// get another interrupt when the card next changes state.
				let was_inserted = self.io_flip_input_level(1);
				defmt::info!(
					"SD Card state change: Card {}",
					if was_inserted { "inserted" } else { "removed" }
				);

				self.card_state = match (&self.card_state, was_inserted) {
					// Eject events
					(CardState::Unplugged, false) => {
						defmt::warn!("Spurious unplug event");
						CardState::Unplugged
					}
					(CardState::Errored | CardState::Uninitialised, false) => {
						defmt::info!("SD Card removed");
						CardState::Unplugged
					}
					(CardState::Online(_), false) => {
						defmt::warn!("Active SD Card removed!");
						CardState::Unplugged
					}
					// Insert events
					(CardState::Unplugged, true) => {
						defmt::info!("SD Card inserted, pending activation");
						CardState::Uninitialised
					}
					(_, true) => {
						defmt::warn!("Unexpected card insertion!?");
						CardState::Uninitialised
					}
				}
			}
			_ => {
				defmt::debug!("IRQ on slot {}", slot);
			}
		}
	}

	/// Returns the contents of the GPIOB register, i.e. eight bits, where if
	/// bit N is low, IRQN is active.
	fn io_read_interrupts(&mut self) -> u8 {
		self.io_chip_read(mcp23s17::Register::GPIOB)
	}

	/// Flip which level is expected on an incoming interrupt pin.
	///
	/// Returns the old level
	fn io_flip_input_level(&mut self, slot: u8) -> bool {
		let mut mask = self.io_chip_read(mcp23s17::Register::IPOLB);
		let slot_mask = 1 << slot;
		let current_level = (mask & slot_mask) != 0;
		mask ^= slot_mask;
		self.io_chip_write(mcp23s17::Register::IPOLB, mask);
		current_level
	}

	/// Send a request to the BMC (a register read or a register write) and get
	/// the response.
	///
	/// It will perform a couple of retries, and then panic if the BMC did not
	/// respond correctly. It sets the Chip Select line and controls the IO chip
	/// automatically.
	///
	/// The `buffer` argument may contain a mutable buffer for received data.
	fn bmc_do_request(
		&mut self,
		req: neotron_bmc_protocol::Request,
		buffer: Option<&mut [u8]>,
	) -> Result<(), ()> {
		const MAX_LATENCY: usize = 128;

		if let Some(buffer) = buffer.as_ref() {
			if buffer.len() > self.bmc_buffer.len() {
				defmt::error!("Asked for too much data ({})", buffer.len());
				return Err(());
			}
			if buffer.len() > usize::from(u8::MAX) {
				defmt::error!("Asked for too much data ({})", buffer.len());
				return Err(());
			}
		}

		let req_bytes = req.as_bytes();
		for _retries in 0..4 {
			// Clear the input buffer
			for byte in self.bmc_buffer.iter_mut() {
				*byte = 0xFF;
			}
			let expected_response_len = buffer.as_ref().map(|b| b.len()).unwrap_or(0) + 2;
			defmt::debug!("req: {=[u8; 4]:02x}", req_bytes);
			let mut latency = 0;
			self.with_bus_cs(0, Self::CLOCK_BMC, |spi, borrowed_buffer| {
				// Send the request
				spi.write(&req_bytes).unwrap();
				for retry in 0..MAX_LATENCY {
					cortex_m::asm::delay(Self::BMC_REQUEST_RESPONSE_DELAY_CLOCKS);
					spi.transfer(&mut borrowed_buffer[0..=0]).unwrap();
					if neotron_bmc_protocol::ResponseResult::try_from(borrowed_buffer[0]).is_ok() {
						latency = retry;
						break;
					}
				}
				// Get the rest of the response now we know it's queued up.
				spi.transfer(&mut borrowed_buffer[1..expected_response_len])
					.unwrap();
			});
			defmt::debug!(
				"res: {=[u8]:02x} ({})",
				self.bmc_buffer[0..expected_response_len],
				latency
			);
			// 8 bytes of data requested, plus one bytes of response code and one byte of CRC
			let response_portion = &self.bmc_buffer[0..expected_response_len];
			match neotron_bmc_protocol::Response::from_bytes(response_portion) {
				Ok(res) => {
					if res.result == neotron_bmc_protocol::ResponseResult::Ok {
						if let Some(buffer) = buffer {
							if res.data.len() == buffer.len() {
								buffer.copy_from_slice(res.data);
							} else {
								defmt::warn!(
									"Mismatch between received and expected data: expected {}, got {} bytes - {=[u8]:X}",
									buffer.len(),
									res.data.len(),
									res.data
								);
								return Err(());
							}
						}
						return Ok(());
					} else {
						defmt::warn!(
							"Error executing {:?}: Error from BMC {:?} {=[u8]:x}",
							req,
							res.result,
							response_portion
						);
						// No point retrying - we heardly them perfectly
						return Err(());
					}
				}
				Err(e) => {
					defmt::warn!(
						"Error executing {:?}: Decoding Error {:?} {=[u8]:x}",
						req,
						e,
						response_portion
					);
				}
			}
			// Wait a bit before we try again
			cortex_m::asm::delay(Self::BMC_RETRY_CPU_CLOCKS);
		}
		panic!("Failed to talk to BMC after several retries.");
	}

	/// Make the BMC produce a sequence of beeps using the Speaker
	fn play_startup_tune(&mut self) -> Result<(), ()> {
		// (delay (ms), command, data)
		let seq: &[(u16, Command, u8)] = &[
			(0, Command::SpeakerPeriodLow, 137),
			(0, Command::SpeakerPeriodHigh, 0),
			(0, Command::SpeakerDutyCycle, 127),
			// This triggers the beep to occur
			(70, Command::SpeakerDuration, 7),
			(0, Command::SpeakerPeriodLow, 116),
			// This triggers the beep to occur
			(70, Command::SpeakerDuration, 7),
			(0, Command::SpeakerPeriodLow, 97),
			// This triggers the beep to occur
			(70, Command::SpeakerDuration, 7),
		];

		for (delay, reg, val) in seq {
			if self
				.bmc_do_request(
					neotron_bmc_protocol::Request::new_short_write(
						USE_ALT.get(),
						(*reg).into(),
						*val,
					),
					None,
				)
				.is_err()
			{
				defmt::error!("Failed to play note");
			}
			if *delay > 0 {
				self.delay.delay_ms(*delay as u32);
			}
		}
		Ok(())
	}

	/// Read the BMC firmware version string.
	///
	/// You get 32 bytes of probably UTF-8 data.
	fn bmc_read_firmware_version(&mut self) -> Result<[u8; 32], ()> {
		let mut firmware_version = [0u8; 32];
		self.bmc_do_request(
			neotron_bmc_protocol::Request::new_read(
				USE_ALT.get(),
				Command::FirmwareVersion.into(),
				firmware_version.len() as u8,
			),
			Some(&mut firmware_version),
		)?;
		Ok(firmware_version)
	}

	/// Is there an interrupt pending on the given slot?
	fn is_irq_pending_on_slot(&self, slot: u8) -> bool {
		assert!(slot < 8);
		(self.interrupts_pending & (1 << slot)) != 0
	}

	/// Read the BMC PS/2 keyboard FIFO.
	///
	/// We ask for 8 bytes of data. We get `1` byte of 'length', then `N` bytes of valid data, and `32 - (N + 1)` bytes of padding.
	fn bmc_read_ps2_keyboard_fifo(&mut self, out_buffer: &mut [u8; 8]) -> Result<usize, ()> {
		// Now is a good time to poll for interrupts.
		self.io_poll_interrupts(false);
		if !self.is_irq_pending_on_slot(0) {
			// No point asking, the interrupt isn't set.
			return Ok(0);
		}

		let mut fifo_data = [0u8; 9];
		self.bmc_do_request(
			neotron_bmc_protocol::Request::new_read(
				USE_ALT.get(),
				Command::Ps2KbBuffer.into(),
				fifo_data.len() as u8,
			),
			Some(&mut fifo_data),
		)?;
		let bytes_in_fifo = fifo_data[0];
		if bytes_in_fifo == 0 {
			defmt::trace!("Got no PS/2 bytes");
		} else {
			defmt::debug!("Got PS/2 bytes {=[u8]:x}", &fifo_data[..]);
		}
		for (dest, src) in out_buffer.iter_mut().zip(fifo_data.iter().skip(1)) {
			*dest = *src;
		}
		Ok(bytes_in_fifo as usize)
	}

	fn sdcard_poll(&mut self) {
		match self.card_state {
			CardState::Uninitialised => {
				use embedded_sdmmc::BlockDevice;
				// Downclock SPI to initialisation speed
				let spi = sdcard::FakeSpi(self, true);
				let cs = sdcard::FakeCs();
				let delayer = sdcard::FakeDelayer();
				let options = embedded_sdmmc::sdcard::AcquireOpts {
					use_crc: true,
					acquire_retries: 5,
				};
				let sdcard = embedded_sdmmc::SdCard::new_with_options(spi, cs, delayer, options);
				// Talk to the card to trigger a scan if its type
				let num_blocks = sdcard.num_blocks();
				let card_type = sdcard.get_card_type();
				defmt::info!(
					"Found card size {:?} blocks, type {:?}",
					num_blocks,
					card_type
				);
				if let (Ok(num_blocks), Some(card_type)) = (num_blocks, card_type) {
					self.card_state = CardState::Online(CardInfo {
						num_blocks: num_blocks.0 as u64,
						card_type,
					});
				}
			}
			CardState::Unplugged | CardState::Errored | CardState::Online(_) => {}
		}
	}
}

// -----------------------------------------------------------------------------
// Exported API functions
// -----------------------------------------------------------------------------

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
pub extern "C" fn bios_version_get() -> FfiString<'static> {
	FfiString::new(VERSION)
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
pub extern "C" fn serial_get_info(_device: u8) -> FfiOption<common::serial::DeviceInfo> {
	FfiOption::None
}

/// Set the options for a given serial device. An error is returned if the
/// options are invalid for that serial device.
pub extern "C" fn serial_configure(_device: u8, _config: common::serial::Config) -> ApiResult<()> {
	ApiResult::Err(CError::Unimplemented)
}

/// Write bytes to a serial port. There is no sense of 'opening' or
/// 'closing' the device - serial devices are always open. If the return
/// value is `Ok(n)`, the value `n` may be less than the size of the given
/// buffer. If so, that means not all of the data could be transmitted -
/// only the first `n` bytes were.
pub extern "C" fn serial_write(
	_device: u8,
	_data: FfiByteSlice,
	_timeout: FfiOption<common::Timeout>,
) -> ApiResult<usize> {
	ApiResult::Err(CError::Unimplemented)
}

/// Read bytes from a serial port. There is no sense of 'opening' or
/// 'closing' the device - serial devices are always open. If the return value
///  is `Ok(n)`, the value `n` may be less than the size of the given buffer.
///  If so, that means not all of the data could be received - only the
///  first `n` bytes were filled in.
pub extern "C" fn serial_read(
	_device: u8,
	_data: FfiBuffer,
	_timeout: FfiOption<common::Timeout>,
) -> ApiResult<usize> {
	ApiResult::Err(CError::Unimplemented)
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
pub extern "C" fn time_clock_get() -> common::Time {
	// We track real-time using the RP2040 1 MHz timer. We noted the wall-time at boot-up.

	let ticks_since_epoch = {
		let mut lock = HARDWARE.lock();
		let hw = lock.as_mut().unwrap();
		// We can unwrap this because we set the day-of-week correctly on startup
		let ticks_since_boot = hw.timer.get_counter().duration_since_epoch();
		ticks_since_boot + hw.bootup_at
	};

	let nanos_since_epoch = ticks_since_epoch.to_nanos();

	let secs = (nanos_since_epoch / 1_000_000_000) as u32;
	let nsecs = (nanos_since_epoch % 1_000_000_000) as u32;
	defmt::info!("Time is {}.{:09}s", secs, nsecs);
	common::Time { secs, nsecs }
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
pub extern "C" fn time_clock_set(time: common::Time) {
	let mut lock = HARDWARE.lock();
	let hw = lock.as_mut().unwrap();

	// 1. How long have we been running, in 1 MHz ticks?
	let ticks_since_boot = hw.timer.get_counter().duration_since_epoch();

	// 2. What is the given time as 1 MHz ticks since the epoch?
	let ticks_since_epoch = u64::from(time.secs) * 1_000_000 + u64::from(time.nsecs / 1000);
	let ticks_since_epoch = Duration::from_ticks(ticks_since_epoch);

	// 3. Work backwards, and find the time it must have been when we booted up.
	let ticks_at_boot = ticks_since_epoch
		.checked_sub(ticks_since_boot)
		.unwrap_or_else(|| Duration::from_ticks(0));

	// 4. Store that value
	defmt::info!("Ticks at boot: {}", ticks_at_boot.ticks());
	hw.bootup_at = ticks_at_boot;

	// 5. Convert to calendar time
	if let Some(new_time) = NaiveDateTime::from_timestamp_opt(
		i64::from(time.secs) + SECONDS_BETWEEN_UNIX_AND_NEOTRON_EPOCH,
		time.nsecs,
	) {
		// 6. Update the hardware RTC as well
		match hw.rtc.set_time(hw.i2c.acquire_i2c(), new_time) {
			Ok(_) => {
				defmt::info!("Time set in RTC OK");
			}
			Err(rtc::Error::Bus(_)) => {
				defmt::warn!("Failed to talk to RTC to set time");
			}
			Err(rtc::Error::DriverBug) => {
				defmt::warn!("RTC driver failed");
			}
			Err(rtc::Error::NoRtcFound) => {
				defmt::info!("Not setting time - no RTC found");
			}
		}
	}
}

/// Get the configuration data block.
///
/// Configuration data is, to the BIOS, just a block of bytes of a given
/// length. How it stores them is up to the BIOS - it could be EEPROM, or
/// battery-backed SRAM.
pub extern "C" fn configuration_get(mut buffer: FfiBuffer) -> ApiResult<usize> {
	let mut lock = HARDWARE.lock();
	let hw = lock.as_mut().unwrap();
	let Some(slice) = buffer.as_mut_slice() else {
		return ApiResult::Err(CError::Unimplemented);
	};
	match hw.rtc.configuration_get(hw.i2c.acquire_i2c(), slice) {
		Ok(n) => {
			defmt::info!("Config Read {:x}", &slice[0..usize::from(n)]);
			ApiResult::Ok(usize::from(n))
		}
		Err(rtc::Error::NoRtcFound) => {
			defmt::info!("Can't get config - no RTC present");
			ApiResult::Err(CError::InvalidDevice)
		}
		Err(rtc::Error::DriverBug) => {
			defmt::warn!("Can't get config - Driver Bug");
			ApiResult::Err(CError::DeviceError(0))
		}
		Err(rtc::Error::Bus(e)) => {
			defmt::warn!("Can't get config - bus error {:?}", e);
			ApiResult::Err(CError::DeviceError(1))
		}
	}
}

/// Set the configuration data block.
///
/// See `configuration_get`.
pub extern "C" fn configuration_set(buffer: FfiByteSlice) -> ApiResult<()> {
	defmt::info!("Config save {:x}", buffer.as_slice());
	let mut lock = HARDWARE.lock();
	let hw = lock.as_mut().unwrap();
	match hw
		.rtc
		.configuration_set(hw.i2c.acquire_i2c(), buffer.as_slice())
	{
		Ok(()) => ApiResult::Ok(()),
		Err(rtc::Error::NoRtcFound) => {
			defmt::info!("Can't save config - no RTC present");
			ApiResult::Err(CError::InvalidDevice)
		}
		Err(rtc::Error::DriverBug) => {
			defmt::warn!("Can't save config - Driver Bug");
			ApiResult::Err(CError::DeviceError(0))
		}
		Err(rtc::Error::Bus(e)) => {
			defmt::warn!("Can't save config - bus error {:?}", e);
			ApiResult::Err(CError::DeviceError(0))
		}
	}
}

/// Does this Neotron BIOS support this video mode?
pub extern "C" fn video_is_valid_mode(mode: common::video::Mode) -> bool {
	vga::test_video_mode(mode)
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
pub extern "C" fn video_set_mode(mode: common::video::Mode) -> ApiResult<()> {
	defmt::info!("Changing to mode {}", mode.as_u8());
	if vga::set_video_mode(mode) {
		ApiResult::Ok(())
	} else {
		ApiResult::Err(CError::UnsupportedConfiguration(0))
	}
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
	let ptr = vga::CUSTOM_FB.load(Ordering::Relaxed);
	if ptr.is_null() {
		unsafe { vga::GLYPH_ATTR_ARRAY.as_mut_ptr() as *mut u8 }
	} else {
		ptr
	}
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
pub unsafe extern "C" fn video_set_framebuffer(buffer: *const u8) -> ApiResult<()> {
	vga::CUSTOM_FB.store(buffer as *mut u8, Ordering::Relaxed);
	ApiResult::Ok(())
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
pub extern "C" fn memory_get_region(region: u8) -> FfiOption<common::MemoryRegion> {
	match region {
		0 => {
			// Application Region
			FfiOption::Some(MemoryRegion {
				start: unsafe { &mut _ram_os_start as *mut u32 } as *mut u8,
				length: unsafe { &mut _ram_os_len as *const u32 } as usize,
				kind: common::MemoryKind::Ram,
			})
		}
		_ => FfiOption::None,
	}
}

/// This function doesn't block. It will return `Ok(None)` if there is no event
/// ready.
///
/// The Pico BIOS gets PS/2 scan-codes (in PS/2 Scan Code Set 2 format) from the
/// BMC. The BMC receives them from the PS/2 keyboard (as 11-bit words with
/// stop, stop and parity bits) and buffers them (as raw 8-bit values with the
/// start/stop/parity bits removed). These scan-codes are converted into
/// human-readable labels here in this function. The labels are applied as if
/// you had a US-English keyboard. If you do not have a US-English keyboard, the
/// labels, will be incorrect, but that doesn't matter. It is the OS's job to
/// convert those labels (along with the key up or key down event) into Unicode
/// characters, which is where the country-specific keyboard mapping comes in.
///
/// This is a similar model used to that in the IBM PC. Your PC's BIOS cares not
/// for which country you are; that was MS-DOS's job.
///
/// The reason we don't just pass keyboard scan-codes in Scan Code Set 2 (the
/// power-up default for almost every IBM PS/2 compatible keyboard) is that in
/// the future your BIOS key get the keyboard input from another source. If it
/// came from a USB Keyboard, you would have USB HID Scan Codes. If it came from
/// an SDL2 window under Linux/Windows/macOS, you would have SDL2 specific key
/// codes. So the BIOS must convert this wide and varied set of HID inputs into
/// a single KeyCode enum. Plus, Scan Code Set 2 is a pain, because most of the
/// 'extended' keys they added on the IBM PC/AT actually generate two bytes, not
/// one. It's much nicer when your Scan Codes always have one byte per key.
pub extern "C" fn hid_get_event() -> ApiResult<FfiOption<common::hid::HidEvent>> {
	let mut buffer = [0u8; 8];

	let mut lock = HARDWARE.lock();
	let hw = lock.as_mut().unwrap();

	if let Some(ev) = hw.event_queue.pop_front() {
		// Queued data available, so short-cut
		return ApiResult::Ok(FfiOption::Some(ev));
	}

	loop {
		match hw.bmc_read_ps2_keyboard_fifo(&mut buffer) {
			Ok(n) if n > 0 => {
				let slice = if n >= 8 { &buffer } else { &buffer[0..n] };
				defmt::info!("{} bytes in KB FIFO, got: {=[u8]:x}", n, &slice);
				for b in slice.iter() {
					match hw.keyboard.advance_state(*b) {
						Ok(Some(key_event)) => {
							convert_hid_event(key_event, &mut hw.event_queue);
						}
						Ok(None) => {
							// Need more data
						}
						Err(_e) => {
							defmt::warn!("Keyboard decode error!");
						}
					}
				}
			}
			Ok(_) => {
				// Stop reading, FIFO is empty.
				break;
			}
			Err(e) => {
				defmt::warn!("Read KB error: {:?}", e);
			}
		}
	}

	if let Some(ev) = hw.event_queue.pop_front() {
		// Queued data available, so short-cut
		ApiResult::Ok(FfiOption::Some(ev))
	} else {
		ApiResult::Ok(FfiOption::None)
	}
}

fn convert_hid_event(
	pc_keyboard_ev: pc_keyboard::KeyEvent,
	ev_queue: &mut heapless::Deque<common::hid::HidEvent, 16>,
) {
	match pc_keyboard_ev.state {
		pc_keyboard::KeyState::Down => {
			ev_queue
				.push_back(common::hid::HidEvent::KeyPress(pc_keyboard_ev.code))
				.unwrap();
		}
		pc_keyboard::KeyState::Up => {
			ev_queue
				.push_back(common::hid::HidEvent::KeyRelease(pc_keyboard_ev.code))
				.unwrap();
		}
		pc_keyboard::KeyState::SingleShot => {
			ev_queue
				.push_back(common::hid::HidEvent::KeyPress(pc_keyboard_ev.code))
				.unwrap();
			ev_queue
				.push_back(common::hid::HidEvent::KeyRelease(pc_keyboard_ev.code))
				.unwrap();
		}
	}
}

/// Control the keyboard LEDs.
pub extern "C" fn hid_set_leds(_leds: common::hid::KeyboardLeds) -> ApiResult<()> {
	ApiResult::Err(CError::Unimplemented)
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

/// Read the RGB palette.
extern "C" fn video_get_palette(index: u8) -> FfiOption<common::video::RGBColour> {
	let raw_u16 = vga::VIDEO_PALETTE[index as usize].load(Ordering::Relaxed);
	let our_colour = vga::RGBColour(raw_u16);
	// Convert from our 12-bit colour type to the public 24-bit colour type
	FfiOption::Some(our_colour.into())
}

/// Update the RGB palette.
extern "C" fn video_set_palette(index: u8, rgb: common::video::RGBColour) {
	// Convert from their 24-bit colour type to our 12-bit colour type
	let our_colour: vga::RGBColour = rgb.into();
	// Store it
	vga::VIDEO_PALETTE[index as usize].store(our_colour.0, Ordering::Relaxed);
}

/// Update all the RGB palette
unsafe extern "C" fn video_set_whole_palette(
	palette: *const common::video::RGBColour,
	length: usize,
) {
	// Don't let them set more than 255 entries
	let num_entries = length.min(255);
	for i in 0..num_entries {
		video_set_palette(i as u8, palette.add(i).read())
	}
}

extern "C" fn i2c_bus_get_info(i2c_bus: u8) -> FfiOption<common::i2c::BusInfo> {
	match i2c_bus {
		0 => FfiOption::Some(common::i2c::BusInfo {
			name: "RP2040".into(),
		}),
		1 => FfiOption::Some(common::i2c::BusInfo {
			name: "Pico-BMC".into(),
		}),
		_ => FfiOption::None,
	}
}

extern "C" fn i2c_write_read(
	i2c_bus: u8,
	i2c_device_address: u8,
	tx: FfiByteSlice,
	tx2: FfiByteSlice,
	mut rx: FfiBuffer,
) -> ApiResult<()> {
	let mut lock = HARDWARE.lock();
	let hw = lock.as_mut().unwrap();
	match i2c_bus {
		0 => {
			// Use RP2040 bus
			let mut i2c = hw.i2c.acquire_i2c();
			let tx_bytes = tx.as_slice();
			let tx2_bytes = tx2.as_slice();
			let tx_iter = tx_bytes.iter().chain(tx2_bytes).cloned();
			let r = if let Some(rx_buf) = rx.as_mut_slice() {
				i2c.write_iter_read(i2c_device_address, tx_iter, rx_buf)
			} else {
				i2c.write(i2c_device_address, tx_iter)
			};
			match r {
				Ok(()) => ApiResult::Ok(()),
				Err(e) => {
					defmt::warn!("Error executing I2C: {:?}", e);
					ApiResult::Err(CError::DeviceError(0))
				}
			}
		}
		1 => {
			// TODO talk to Pico-BMC over SPI
			ApiResult::Err(CError::Unimplemented)
		}
		_ => ApiResult::Err(CError::Unimplemented),
	}
}

extern "C" fn audio_mixer_channel_get_info(
	audio_mixer_id: u8,
) -> FfiOption<common::audio::MixerChannelInfo> {
	let mut lock = HARDWARE.lock();
	let hw = lock.as_mut().unwrap();

	match audio_mixer_id {
		0 => FfiOption::Some(common::audio::MixerChannelInfo {
			name: common::FfiString::new("LeftOut"),
			direction: common::audio::Direction::Output,
			max_level: 79,
			current_level: hw.audio_codec.get_headphone_output_volume().0,
		}),
		1 => FfiOption::Some(common::audio::MixerChannelInfo {
			name: common::FfiString::new("RightOut"),
			direction: common::audio::Direction::Output,
			max_level: 79,
			current_level: hw.audio_codec.get_headphone_output_volume().1,
		}),
		2 => FfiOption::Some(common::audio::MixerChannelInfo {
			name: common::FfiString::new("LeftIn"),
			direction: common::audio::Direction::Input,
			max_level: 31,
			current_level: hw.audio_codec.get_line_input_volume_steps().0,
		}),
		3 => FfiOption::Some(common::audio::MixerChannelInfo {
			name: common::FfiString::new("RightIn"),
			direction: common::audio::Direction::Input,
			max_level: 31,
			current_level: hw.audio_codec.get_line_input_volume_steps().1,
		}),
		4 => FfiOption::Some(common::audio::MixerChannelInfo {
			name: common::FfiString::new("LeftInMute"),
			direction: common::audio::Direction::Input,
			max_level: 1,
			current_level: hw.audio_codec.get_line_input_mute().0 as u8,
		}),
		5 => FfiOption::Some(common::audio::MixerChannelInfo {
			name: common::FfiString::new("RightInMute"),
			direction: common::audio::Direction::Input,
			max_level: 1,
			current_level: hw.audio_codec.get_line_input_mute().1 as u8,
		}),
		6 => FfiOption::Some(common::audio::MixerChannelInfo {
			name: common::FfiString::new("Sidetone"),
			direction: common::audio::Direction::Output,
			max_level: 5,
			current_level: match hw.audio_codec.get_sidetone_level() {
				tlv320aic23::Sidetone::ZeroDb => 5,
				tlv320aic23::Sidetone::Minus6 => 4,
				tlv320aic23::Sidetone::Minus9 => 3,
				tlv320aic23::Sidetone::Minus12 => 2,
				tlv320aic23::Sidetone::Minus18 => 1,
				tlv320aic23::Sidetone::Disabled => 0,
			},
		}),
		7 => FfiOption::Some(common::audio::MixerChannelInfo {
			name: common::FfiString::new("MicEnable"),
			direction: common::audio::Direction::Input,
			max_level: 1,
			current_level: hw.audio_codec.get_audio_input() as u8,
		}),
		8 => FfiOption::Some(common::audio::MixerChannelInfo {
			name: common::FfiString::new("MicBoost"),
			direction: common::audio::Direction::Input,
			max_level: 1,
			current_level: hw.audio_codec.get_microphone_boost() as u8,
		}),
		9 => FfiOption::Some(common::audio::MixerChannelInfo {
			name: common::FfiString::new("MicMute"),
			direction: common::audio::Direction::Input,
			max_level: 1,
			current_level: hw.audio_codec.get_microphone_mute() as u8,
		}),
		10 => FfiOption::Some(common::audio::MixerChannelInfo {
			name: common::FfiString::new("Bypass"),
			direction: common::audio::Direction::Input,
			max_level: 1,
			current_level: hw.audio_codec.get_bypass() as u8,
		}),
		11 => FfiOption::Some(common::audio::MixerChannelInfo {
			name: common::FfiString::new("DacEnable"),
			direction: common::audio::Direction::Output,
			max_level: 1,
			current_level: hw.audio_codec.get_dac_selected() as u8,
		}),
		12 => FfiOption::Some(common::audio::MixerChannelInfo {
			name: common::FfiString::new("DacMute"),
			direction: common::audio::Direction::Output,
			max_level: 1,
			current_level: hw.audio_codec.get_dac_mute() as u8,
		}),
		_ => FfiOption::None,
	}
}

extern "C" fn audio_mixer_channel_set_level(audio_mixer_id: u8, level: u8) -> ApiResult<()> {
	let FfiOption::Some(info) = audio_mixer_channel_get_info(audio_mixer_id) else {
		return ApiResult::Err(neotron_common_bios::Error::InvalidDevice);
	};

	let mut lock = HARDWARE.lock();
	let hw = lock.as_mut().unwrap();

	if level > info.max_level {
		return ApiResult::Err(neotron_common_bios::Error::UnsupportedConfiguration(0));
	}

	use tlv320aic23::Channel;

	match audio_mixer_id {
		0 => hw
			.audio_codec
			.set_headphone_output_volume(level, Channel::Left),
		1 => hw
			.audio_codec
			.set_headphone_output_volume(level, Channel::Right),
		2 => hw
			.audio_codec
			.set_line_input_volume_steps(level, Channel::Left),
		3 => hw
			.audio_codec
			.set_line_input_volume_steps(level, Channel::Right),
		4 => hw
			.audio_codec
			.set_line_input_mute(level == 1, Channel::Left),
		5 => hw
			.audio_codec
			.set_line_input_mute(level == 1, Channel::Right),
		6 => hw.audio_codec.set_sidetone_level(match level {
			5 => tlv320aic23::Sidetone::ZeroDb,
			4 => tlv320aic23::Sidetone::Minus6,
			3 => tlv320aic23::Sidetone::Minus9,
			2 => tlv320aic23::Sidetone::Minus12,
			1 => tlv320aic23::Sidetone::Minus18,
			_ => tlv320aic23::Sidetone::Disabled,
		}),
		7 => hw.audio_codec.set_audio_input(match level {
			0 => tlv320aic23::AudioInput::LineInput,
			_ => tlv320aic23::AudioInput::Microphone,
		}),
		8 => hw.audio_codec.set_microphone_boost(level == 1),
		9 => hw.audio_codec.set_microphone_mute(level == 1),
		10 => hw.audio_codec.set_bypass(level == 1),
		11 => hw.audio_codec.set_dac_selected(level == 1),
		12 => hw.audio_codec.set_dac_mute(level == 1),
		_ => {
			return ApiResult::Err(neotron_common_bios::Error::InvalidDevice);
		}
	};
	if hw.audio_codec.sync(&mut hw.i2c.acquire_i2c()).is_ok() {
		neotron_common_bios::FfiResult::Ok(())
	} else {
		ApiResult::Err(neotron_common_bios::Error::DeviceError(0))
	}
}

extern "C" fn audio_output_set_config(config: common::audio::Config) -> ApiResult<()> {
	// We can't support 8-bit samples with this CODEC. See `audio_output_data`
	// if you add any more supported sample formats, so the playback routine
	// does the right thing.
	match config.sample_format {
		common::audio::SampleFormat::SixteenBitStereo
		| common::audio::SampleFormat::SixteenBitMono => {
			// Format Ok
		}
		_ => {
			return ApiResult::Err(CError::UnsupportedConfiguration(0));
		}
	}
	let sample_rate = match config.sample_rate_hz {
		48000 => tlv320aic23::CONFIG_USB_48K,
		44100 => tlv320aic23::CONFIG_USB_44K1,
		32000 => tlv320aic23::CONFIG_USB_32K,
		8000 => tlv320aic23::CONFIG_USB_8K,
		_ => {
			return ApiResult::Err(CError::UnsupportedConfiguration(1));
		}
	};
	let mut lock = HARDWARE.lock();
	let hw = lock.as_mut().unwrap();
	hw.audio_codec.set_sample_rate(
		sample_rate,
		tlv320aic23::LrSwap::Disabled,
		tlv320aic23::LrPhase::RightOnHigh,
		tlv320aic23::Mode::Controller,
		tlv320aic23::InputBitLength::B16,
		tlv320aic23::DataFormat::I2s,
	);
	hw.audio_config = config;
	ApiResult::Ok(())
}

extern "C" fn audio_output_get_config() -> ApiResult<common::audio::Config> {
	let mut lock = HARDWARE.lock();
	let hw = lock.as_mut().unwrap();
	let config = hw.audio_config.clone();
	ApiResult::Ok(config)
}

unsafe extern "C" fn audio_output_data(samples: FfiByteSlice) -> ApiResult<usize> {
	let mut lock = HARDWARE.lock();
	let hw = lock.as_mut().unwrap();
	let samples = samples.as_slice();
	// We only support these two kinds.
	let count = match hw.audio_config.sample_format {
		common::audio::SampleFormat::SixteenBitMono => {
			hw.audio_player.play_samples_16bit_mono(samples)
		}
		common::audio::SampleFormat::SixteenBitStereo => {
			hw.audio_player.play_samples_16bit_stereo(samples)
		}
		_ => {
			return ApiResult::Err(CError::Unimplemented);
		}
	};
	ApiResult::Ok(count)
}

extern "C" fn audio_output_get_space() -> ApiResult<usize> {
	let mut lock = HARDWARE.lock();
	let hw = lock.as_mut().unwrap();
	// The result is in samples, where each sample is 16-bit stereo (i.e.
	// 32-bits) or 16-bit mono (i.e. 16-bit) depending on the audio
	// configuration.
	ApiResult::Ok(hw.audio_player.available_space())
}

extern "C" fn audio_input_set_config(_config: common::audio::Config) -> ApiResult<()> {
	ApiResult::Err(CError::Unimplemented)
}

extern "C" fn audio_input_get_config() -> ApiResult<common::audio::Config> {
	ApiResult::Err(CError::Unimplemented)
}

extern "C" fn audio_input_data(_samples: FfiBuffer) -> ApiResult<usize> {
	ApiResult::Err(CError::Unimplemented)
}

extern "C" fn audio_input_get_count() -> ApiResult<usize> {
	ApiResult::Ok(0)
}

extern "C" fn bus_select(_periperal_id: FfiOption<u8>) {
	// Do nothing
}

extern "C" fn bus_get_info(periperal_id: u8) -> FfiOption<common::bus::PeripheralInfo> {
	match periperal_id {
		0 => FfiOption::Some(common::bus::PeripheralInfo {
			name: "BMC".into(),
			kind: common::bus::PeripheralKind::Reserved,
		}),
		1 => FfiOption::Some(common::bus::PeripheralInfo {
			name: "SdCard".into(),
			kind: common::bus::PeripheralKind::SdCard,
		}),
		2 => FfiOption::Some(common::bus::PeripheralInfo {
			name: "Slot2".into(),
			kind: common::bus::PeripheralKind::Slot,
		}),
		3 => FfiOption::Some(common::bus::PeripheralInfo {
			name: "Slot3".into(),
			kind: common::bus::PeripheralKind::Slot,
		}),
		4 => FfiOption::Some(common::bus::PeripheralInfo {
			name: "Slot4".into(),
			kind: common::bus::PeripheralKind::Slot,
		}),
		5 => FfiOption::Some(common::bus::PeripheralInfo {
			name: "Slot5".into(),
			kind: common::bus::PeripheralKind::Slot,
		}),
		_ => FfiOption::None,
	}
}

extern "C" fn bus_write_read(
	_tx: FfiByteSlice,
	_tx2: FfiByteSlice,
	_rx: FfiBuffer,
) -> ApiResult<()> {
	ApiResult::Err(CError::Unimplemented)
}

extern "C" fn bus_exchange(_buffer: FfiBuffer) -> ApiResult<()> {
	ApiResult::Err(CError::Unimplemented)
}

extern "C" fn bus_interrupt_status() -> u32 {
	0
}

/// Get information about the Block Devices in the system.
///
/// Block Devices are also known as *disk drives*. They can be read from
/// (and often written to) but only in units called *blocks* or *sectors*.
///
/// The BIOS should enumerate removable devices first, followed by fixed
/// devices.
///
/// The set of devices is not expected to change at run-time - removal of
/// media is indicated with a boolean field in the
/// `block_dev::DeviceInfo` structure.
pub extern "C" fn block_dev_get_info(device: u8) -> FfiOption<common::block_dev::DeviceInfo> {
	check_stacks();
	match device {
		0 => {
			let mut lock = HARDWARE.lock();
			let hw = lock.as_mut().unwrap();

			hw.set_hdd_led(true);
			hw.sdcard_poll();
			hw.set_hdd_led(false);

			match &hw.card_state {
				CardState::Unplugged | CardState::Uninitialised | CardState::Errored => {
					FfiOption::Some(common::block_dev::DeviceInfo {
						name: FfiString::new("SdCard0"),
						device_type: common::block_dev::DeviceType::SecureDigitalCard,
						block_size: 0,
						num_blocks: 0,
						ejectable: false,
						removable: true,
						media_present: false,
						read_only: false,
					})
				}
				CardState::Online(info) => FfiOption::Some(common::block_dev::DeviceInfo {
					name: FfiString::new("SdCard0"),
					device_type: common::block_dev::DeviceType::SecureDigitalCard,
					block_size: 512,
					num_blocks: info.num_blocks,
					ejectable: false,
					removable: true,
					media_present: true,
					read_only: false,
				}),
			}
		}
		_ => {
			// Nothing else supported by this BIOS
			FfiOption::None
		}
	}
}

/// Write one or more sectors to a block device.
///
/// The function will block until all data is written. The array pointed
/// to by `data` must be `num_blocks * block_size` in length, where
/// `block_size` is given by `block_dev_get_info`.
///
/// There are no requirements on the alignment of `data` but if it is
/// aligned, the BIOS may be able to use a higher-performance code path.
pub extern "C" fn block_write(
	_device: u8,
	_block: common::block_dev::BlockIdx,
	_num_blocks: u8,
	_data: FfiByteSlice,
) -> ApiResult<()> {
	ApiResult::Err(CError::Unimplemented)
}

/// Read one or more sectors to a block device.
///
/// The function will block until all data is read. The array pointed to by
/// `data` must be `num_blocks * block_size` in length, where `block_size` is
/// given by `block_dev_get_info`.
///
/// The API docs say there are no requirements on the alignment of `data` but we
/// unsafely transmute it into a `[embedded_sdmmc::Block]` so it should match
/// whatever that needs.
pub extern "C" fn block_read(
	device: u8,
	block: common::block_dev::BlockIdx,
	num_blocks: u8,
	data: FfiBuffer,
) -> ApiResult<()> {
	use embedded_sdmmc::BlockDevice;
	check_stacks();
	if data.data_len != usize::from(num_blocks) * 512 {
		return ApiResult::Err(CError::UnsupportedConfiguration(0));
	}
	let mut lock = HARDWARE.lock();
	let hw = lock.as_mut().unwrap();
	hw.set_hdd_led(true);
	let mut inner = || {
		match device {
			0 => {
				hw.sdcard_poll();
				let info = match &hw.card_state {
					CardState::Online(info) => info.clone(),
					_ => return ApiResult::Err(CError::NoMediaFound),
				};
				// Run card at full speed
				let spi = sdcard::FakeSpi(hw, false);
				let cs = sdcard::FakeCs();
				let delayer = sdcard::FakeDelayer();
				let options = embedded_sdmmc::sdcard::AcquireOpts {
					use_crc: true,
					acquire_retries: 5,
				};
				let sdcard = embedded_sdmmc::SdCard::new_with_options(spi, cs, delayer, options);
				unsafe {
					sdcard.mark_card_as_init(info.card_type);
				}
				let blocks = unsafe {
					core::slice::from_raw_parts_mut(
						data.data as *mut embedded_sdmmc::Block,
						data.data_len / 512,
					)
				};
				let start_block_idx = embedded_sdmmc::BlockIdx(block.0 as u32);
				match sdcard.read(blocks, start_block_idx, "bios") {
					Ok(_) => ApiResult::Ok(()),
					Err(e) => {
						defmt::warn!("SD error reading {}: {:?}", block.0, e);
						ApiResult::Err(CError::DeviceError(0))
					}
				}
			}
			_ => {
				// Nothing else supported by this BIOS
				ApiResult::Err(CError::InvalidDevice)
			}
		}
	};
	let result = inner();
	hw.set_hdd_led(false);
	result
}

/// Verify one or more sectors on a block device (that is read them and
/// check they match the given data).
///
/// The function will block until all data is verified. The array pointed
/// to by `data` must be `num_blocks * block_size` in length, where
/// `block_size` is given by `block_dev_get_info`.
///
/// There are no requirements on the alignment of `data` but if it is
/// aligned, the BIOS may be able to use a higher-performance code path.
pub extern "C" fn block_verify(
	_device: u8,
	_block: common::block_dev::BlockIdx,
	_num_blocks: u8,
	_data: FfiByteSlice,
) -> ApiResult<()> {
	ApiResult::Err(CError::Unimplemented)
}

extern "C" fn block_dev_eject(_dev_id: u8) -> ApiResult<()> {
	ApiResult::Ok(())
}

/// Sleep the CPU until the next interrupt.
extern "C" fn power_idle() {
	if !INTERRUPT_PENDING.load(Ordering::Relaxed) {
		unsafe {
			core::arch::asm!("wfe");
		}
	}
}

extern "C" fn time_ticks_get() -> common::Ticks {
	let mut lock = HARDWARE.lock();
	let hw = lock.as_mut().unwrap();
	let now = hw.timer.get_counter();
	common::Ticks(now.ticks())
}

/// Report that we have a 1 MHz timer
extern "C" fn time_ticks_per_second() -> common::Ticks {
	common::Ticks(1_000_000)
}

/// Control the system power.
extern "C" fn power_control(power_mode: common::PowerMode) -> ! {
	match power_mode {
		common::PowerMode::Off => {
			// TODO: Need to send a message to the BMC to turn us off.
			// We'll just reboot for now.
			watchdog_reboot()
		}
		common::PowerMode::Bootloader => {
			// Reboot to USB bootloader with no GPIOs and both USB interfaces
			// enabled.
			hal::rom_data::reset_to_usb_boot(0, 0);
			core::unreachable!()
		}
		_ => {
			// Anything else causes a reset
			watchdog_reboot()
		}
	}
}

/// Performs a compare-and-swap on `value`.
///
/// * If `value == old_value`, sets `value = new_value` and returns `true`
/// * If `value != old_value`, returns `false`
extern "C" fn compare_and_swap_bool(value: &AtomicBool, old_value: bool, new_value: bool) -> bool {
	critical_section::with(|_cs| {
		if value.load(Ordering::Relaxed) == old_value {
			value.store(new_value, Ordering::Relaxed);
			true
		} else {
			false
		}
	})
}

// -----------------------------------------------------------------------------
// Interrupts and Exceptions
// -----------------------------------------------------------------------------

/// Called when we get a SIO interrupt on the main bank of GPIO pins.
///
/// This should only be when the MCP23S17 has driven our IRQ pin low.
#[interrupt]
fn IO_IRQ_BANK0() {
	// The `#[interrupt]` attribute covertly converts this to `&'static mut
	// Option<IrqPin>`
	static mut LOCAL_IRQ_PIN: Option<IrqPin> = None;

	// This is one-time lazy initialisation. We steal the variables given to us
	// via `IRQ_PIN`.
	if LOCAL_IRQ_PIN.is_none() {
		let mut lock = IRQ_PIN.lock();
		*LOCAL_IRQ_PIN = lock.take();
	}
	if let Some(pin) = LOCAL_IRQ_PIN {
		let is_low = pin.is_low().unwrap();
		INTERRUPT_PENDING.store(is_low, Ordering::Relaxed);
		pin.clear_interrupt(hal::gpio::Interrupt::EdgeLow);
		pin.clear_interrupt(hal::gpio::Interrupt::EdgeHigh);
	}
	cortex_m::asm::sev();
}

/// Called when the CPU Hard Faults.
///
/// This probably means something panicked, and the Rust code was compiled to
/// abort-on-panic. Or someone jumped to a bad address.
///
/// Hopefully the debug output we print will be of some use. Think of it as our
/// Blue Screen of Death, or Guru Meditation Error.
#[cortex_m_rt::exception]
unsafe fn HardFault(frame: &cortex_m_rt::ExceptionFrame) -> ! {
	// TODO: check we're actually in text mode
	let tc = console::TextConsole::new();
	tc.set_text_buffer(unsafe { &mut vga::GLYPH_ATTR_ARRAY });
	for _col in 0..vga::MAX_TEXT_ROWS {
		let _ = writeln!(&tc);
	}
	tc.move_to(0, 0);
	tc.change_attr(Attr::new(
		TextForegroundColour::WHITE,
		TextBackgroundColour::RED,
		false,
	));
	let _ = writeln!(&tc, "+------------------------------+");
	let _ = writeln!(&tc, "| System Hardfault             |");
	let _ = writeln!(&tc, "| pc: 0x{:08x}               |", frame.pc());
	let _ = writeln!(&tc, "| lr: 0x{:08x}               |", frame.lr());
	let _ = writeln!(&tc, "| r0: 0x{:08x}               |", frame.r0());
	let _ = writeln!(&tc, "| r1: 0x{:08x}               |", frame.r1());
	let _ = writeln!(&tc, "| r2: 0x{:08x}               |", frame.r2());
	let _ = writeln!(&tc, "| r3: 0x{:08x}               |", frame.r3());
	let _ = writeln!(&tc, "+------------------------------+");
	loop {
		cortex_m::asm::wfi();
	}
}

// -----------------------------------------------------------------------------
// End of file
// -----------------------------------------------------------------------------
