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

// Standard Library Stuff
use core::{
	fmt::Write,
	sync::atomic::{AtomicBool, AtomicU8, Ordering},
};

// Third Party Stuff
use cortex_m_rt::entry;
use critical_section::Mutex;
use defmt::info;
use defmt_rtt as _;
use embedded_hal::{
	blocking::spi::Transfer as _SpiTransfer, blocking::spi::Write as _SpiWrite,
	digital::v2::OutputPin,
};
use fugit::RateExtU32;
use panic_probe as _;
use rp_pico::{
	self,
	hal::{
		self,
		clocks::ClocksManager,
		gpio::{bank0, Function, Input, Output, Pin, PullUp, PushPull},
		pac::{self, interrupt},
		Clock,
	},
};

// Other Neotron Crates
use common::MemoryRegion;
use neotron_bmc_protocol::Receivable;
use neotron_common_bios as common;

// -----------------------------------------------------------------------------
// Types
// -----------------------------------------------------------------------------

/// All the hardware we use on the Pico
struct Hardware {
	/// All the pins we use on the Raspberry Pi Pico
	pins: Pins,
	/// The SPI bus connected to all the slots
	spi_bus: hal::Spi<hal::spi::Enabled, pac::SPI0, 8>,
	/// Something to perform small delays with. Uses SysTICK.
	delay: cortex_m::delay::Delay,
	/// Current 5-bit value shown on the debug LEDs
	debug_leds: u8,
	/// The last CS pin we selected
	last_cs: u8,
	/// Our keyboard decoder
	keyboard: pc_keyboard::Keyboard<pc_keyboard::layouts::Uk105Key, pc_keyboard::ScancodeSet2>,
	/// Our queue of HID events
	event_queue: heapless::Deque<neotron_common_bios::hid::HidEvent, 16>,
}

/// Flips between true and false so we always send a unique read request
struct UseAlt(AtomicBool);

impl UseAlt {
	const fn new() -> UseAlt {
		UseAlt(AtomicBool::new(false))
	}

	fn get(&self) -> bool {
		let use_alt = self.0.load(core::sync::atomic::Ordering::Relaxed);
		self.0
			.store(!use_alt, core::sync::atomic::Ordering::Relaxed);
		use_alt
	}
}

/// All the pins we use on the Pico, in the right mode.
///
/// We ignore the 'unused' lint, because these pins merely need to exist in
/// order to demonstrate the hardware is in the right state.
#[allow(unused)]
struct Pins {
	h_sync: Pin<bank0::Gpio0, Function<pac::PIO0>>,
	v_sync: Pin<bank0::Gpio1, Function<pac::PIO0>>,
	red0: Pin<bank0::Gpio2, Function<pac::PIO0>>,
	red1: Pin<bank0::Gpio3, Function<pac::PIO0>>,
	red2: Pin<bank0::Gpio4, Function<pac::PIO0>>,
	red3: Pin<bank0::Gpio5, Function<pac::PIO0>>,
	green0: Pin<bank0::Gpio6, Function<pac::PIO0>>,
	green1: Pin<bank0::Gpio7, Function<pac::PIO0>>,
	green2: Pin<bank0::Gpio8, Function<pac::PIO0>>,
	green3: Pin<bank0::Gpio9, Function<pac::PIO0>>,
	blue0: Pin<bank0::Gpio10, Function<pac::PIO0>>,
	blue1: Pin<bank0::Gpio11, Function<pac::PIO0>>,
	blue2: Pin<bank0::Gpio12, Function<pac::PIO0>>,
	blue3: Pin<bank0::Gpio13, Function<pac::PIO0>>,
	npower_save: Pin<bank0::Gpio23, Output<PushPull>>,
	i2c_sda: Pin<bank0::Gpio14, Function<hal::gpio::I2C>>,
	i2c_scl: Pin<bank0::Gpio15, Function<hal::gpio::I2C>>,
	spi_cipo: Pin<bank0::Gpio16, Function<hal::gpio::Spi>>,
	nspi_cs_io: Pin<bank0::Gpio17, Output<PushPull>>,
	spi_clk: Pin<bank0::Gpio18, Function<hal::gpio::Spi>>,
	spi_copi: Pin<bank0::Gpio19, Function<hal::gpio::Spi>>,
	nirq_io: Pin<bank0::Gpio20, Input<PullUp>>,
	noutput_en: Pin<bank0::Gpio21, Output<PushPull>>,
	i2s_adc_data: Pin<bank0::Gpio22, Function<pac::PIO1>>,
	i2s_dac_data: Pin<bank0::Gpio26, Function<pac::PIO1>>,
	i2s_bit_clock: Pin<bank0::Gpio27, Function<pac::PIO1>>,
	i2s_lr_clock: Pin<bank0::Gpio28, Function<pac::PIO1>>,
	pico_led: Pin<bank0::Gpio25, Output<PushPull>>,
}

// -----------------------------------------------------------------------------
// Static and Const Data
// -----------------------------------------------------------------------------

/// Version string auto-generated by git.
static VERSION: &str = include_str!(concat!(env!("OUT_DIR"), "/version.txt"));

/// Ensures we always send a unique read request
static USE_ALT: UseAlt = UseAlt::new();

/// Our global hardware object.
///
/// You need to grab this to do anything with the hardware.
static HARDWARE: Mutex<core::cell::RefCell<Option<Hardware>>> =
	Mutex::new(core::cell::RefCell::new(None));

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
};

extern "C" {
	static mut _flash_os_start: u32;
	static mut _flash_os_len: u32;
	static mut _ram_os_start: u32;
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

	// Grab the singleton containing all the RP2040 peripherals
	let mut pp = pac::Peripherals::take().unwrap();
	// Grab the singleton containing all the generic Cortex-M peripherals
	let cp = pac::CorePeripherals::take().unwrap();

	// Reset the DMA engine. If we don't do this, starting from probe-run
	// (as opposed to a cold-start) is unreliable.
	reset_dma_engine(&mut pp);

	// Reset the spinlocks.
	pp.SIO.spinlock[31].reset();

	// Needed by the clock setup
	let mut watchdog = hal::watchdog::Watchdog::new(pp.WATCHDOG);

	// VERSION has a trailing `\0` as that is what the BIOS/OS API requires.
	info!("Neotron BIOS {} starting...", VERSION.trim_matches('\0'));

	// Run at 126 MHz SYS_PLL, 48 MHz, USB_PLL. This is important, we as clock
	// the PIO at ÷ 5, to give 25.2 MHz (which is close enough to the 25.175
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
	// ×126 (=1512 MHz), ÷6 (=252 MHz), ÷2 (=126 MHz)
	let pll_sys = hal::pll::setup_pll_blocking(
		pp.PLL_SYS,
		xosc.operating_frequency(),
		hal::pll::PLLConfig {
			vco_freq: 1512.MHz(),
			refdiv: 1,
			post_div1: 6,
			post_div2: 2,
		},
		&mut clocks,
		&mut pp.RESETS,
	)
	.map_err(|_x| false)
	.unwrap();
	// Step 5. Set up a 48 MHz PLL for the USB system.
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

	// sio is the *Single-cycle Input/Output* peripheral. It has all our GPIO
	// pins, as well as some mailboxes and other useful things for inter-core
	// communications.
	let mut sio = hal::sio::Sio::new(pp.SIO);

	// This object lets us wait for long periods of time (to make things readable on screen)
	let delay = cortex_m::delay::Delay::new(cp.SYST, clocks.system_clock.freq().to_Hz());

	// Configure and grab all the RP2040 pins the Pico exposes, along with the non-VGA peripherals.
	let mut hw = Hardware::build(
		pp.IO_BANK0,
		pp.PADS_BANK0,
		sio.gpio_bank0,
		&mut pp.RESETS,
		pp.SPI0,
		clocks,
		delay,
	);
	hw.init_io_chip();
	hw.set_hdd_led(false);

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

	critical_section::with(|cs| {
		let mut hw_cell = HARDWARE.borrow_ref_mut(cs);
		hw_cell.replace(hw);
	});

	// Say hello over VGA
	sign_on();

	// Now jump to the OS
	let code: &common::OsStartFn = unsafe { ::core::mem::transmute(&_flash_os_start) };
	code(&API_CALLS);
}

impl Hardware {
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

	/// Give the device 10us when we do a retry.
	const SPI_RETRY_CPU_CLOCKS: u32 = 10_000 / Self::NS_PER_CLOCK_CYCLE;

	/// Data Direction Register A on the MCP23S17
	const MCP23S17_DDRA: u8 = 0x00;

	/// GPIO Data Register A on the MCP23S17
	const MCP23S17_GPIOA: u8 = 0x12;

	/// GPIO Pull-Up Register B on the MCP23S17
	const MCP23S17_GPPUB: u8 = 0x0D;

	fn build(
		bank: pac::IO_BANK0,
		pads: pac::PADS_BANK0,
		sio: hal::sio::SioGpioBank0,
		resets: &mut pac::RESETS,
		spi: pac::SPI0,
		clocks: ClocksManager,
		delay: cortex_m::delay::Delay,
	) -> Hardware {
		let hal_pins = rp_pico::Pins::new(bank, pads, sio, resets);

		Hardware {
			pins: Pins {
				// Disable power save mode to force SMPS into low-efficiency, low-noise mode.
				npower_save: {
					let mut pin = hal_pins.b_power_save.into_push_pull_output();
					pin.set_high().unwrap();
					pin
				},
				// Give H-Sync, V-Sync and 12 RGB colour pins to PIO0 to output video
				h_sync: {
					let mut pin = hal_pins.gpio0.into_mode();
					pin.set_drive_strength(hal::gpio::OutputDriveStrength::EightMilliAmps);
					pin.set_slew_rate(hal::gpio::OutputSlewRate::Fast);
					pin
				},
				v_sync: {
					let mut pin = hal_pins.gpio1.into_mode();
					pin.set_drive_strength(hal::gpio::OutputDriveStrength::EightMilliAmps);
					pin.set_slew_rate(hal::gpio::OutputSlewRate::Fast);
					pin
				},
				red0: {
					let mut pin = hal_pins.gpio2.into_mode();
					pin.set_drive_strength(hal::gpio::OutputDriveStrength::EightMilliAmps);
					pin.set_slew_rate(hal::gpio::OutputSlewRate::Fast);
					pin
				},
				red1: {
					let mut pin = hal_pins.gpio3.into_mode();
					pin.set_drive_strength(hal::gpio::OutputDriveStrength::EightMilliAmps);
					pin.set_slew_rate(hal::gpio::OutputSlewRate::Fast);
					pin
				},
				red2: {
					let mut pin = hal_pins.gpio4.into_mode();
					pin.set_drive_strength(hal::gpio::OutputDriveStrength::EightMilliAmps);
					pin.set_slew_rate(hal::gpio::OutputSlewRate::Fast);
					pin
				},
				red3: {
					let mut pin = hal_pins.gpio5.into_mode();
					pin.set_drive_strength(hal::gpio::OutputDriveStrength::EightMilliAmps);
					pin.set_slew_rate(hal::gpio::OutputSlewRate::Fast);
					pin
				},
				green0: {
					let mut pin = hal_pins.gpio6.into_mode();
					pin.set_drive_strength(hal::gpio::OutputDriveStrength::EightMilliAmps);
					pin.set_slew_rate(hal::gpio::OutputSlewRate::Fast);
					pin
				},
				green1: {
					let mut pin = hal_pins.gpio7.into_mode();
					pin.set_drive_strength(hal::gpio::OutputDriveStrength::EightMilliAmps);
					pin.set_slew_rate(hal::gpio::OutputSlewRate::Fast);
					pin
				},
				green2: {
					let mut pin = hal_pins.gpio8.into_mode();
					pin.set_drive_strength(hal::gpio::OutputDriveStrength::EightMilliAmps);
					pin.set_slew_rate(hal::gpio::OutputSlewRate::Fast);
					pin
				},
				green3: {
					let mut pin = hal_pins.gpio9.into_mode();
					pin.set_drive_strength(hal::gpio::OutputDriveStrength::EightMilliAmps);
					pin.set_slew_rate(hal::gpio::OutputSlewRate::Fast);
					pin
				},
				blue0: {
					let mut pin = hal_pins.gpio10.into_mode();
					pin.set_drive_strength(hal::gpio::OutputDriveStrength::EightMilliAmps);
					pin.set_slew_rate(hal::gpio::OutputSlewRate::Fast);
					pin
				},
				blue1: {
					let mut pin = hal_pins.gpio11.into_mode();
					pin.set_drive_strength(hal::gpio::OutputDriveStrength::EightMilliAmps);
					pin.set_slew_rate(hal::gpio::OutputSlewRate::Fast);
					pin
				},
				blue2: {
					let mut pin = hal_pins.gpio12.into_mode();
					pin.set_drive_strength(hal::gpio::OutputDriveStrength::EightMilliAmps);
					pin.set_slew_rate(hal::gpio::OutputSlewRate::Fast);
					pin
				},
				blue3: {
					let mut pin = hal_pins.gpio13.into_mode();
					pin.set_drive_strength(hal::gpio::OutputDriveStrength::EightMilliAmps);
					pin.set_slew_rate(hal::gpio::OutputSlewRate::Fast);
					pin
				},
				i2c_sda: {
					let mut pin = hal_pins.gpio14.into_mode();
					pin.set_drive_strength(hal::gpio::OutputDriveStrength::EightMilliAmps);
					pin.set_slew_rate(hal::gpio::OutputSlewRate::Fast);
					pin
				},
				i2c_scl: {
					let mut pin = hal_pins.gpio15.into_mode();
					pin.set_drive_strength(hal::gpio::OutputDriveStrength::EightMilliAmps);
					pin.set_slew_rate(hal::gpio::OutputSlewRate::Fast);
					pin
				},
				spi_cipo: {
					let mut pin = hal_pins.gpio16.into_mode();
					pin.set_drive_strength(hal::gpio::OutputDriveStrength::EightMilliAmps);
					pin.set_slew_rate(hal::gpio::OutputSlewRate::Fast);
					pin
				},
				nspi_cs_io: {
					let mut pin = hal_pins.gpio17.into_push_pull_output();
					pin.set_high().unwrap();
					pin
				},
				spi_clk: {
					let mut pin = hal_pins.gpio18.into_mode();
					pin.set_drive_strength(hal::gpio::OutputDriveStrength::EightMilliAmps);
					pin.set_slew_rate(hal::gpio::OutputSlewRate::Fast);
					pin
				},
				spi_copi: {
					let mut pin = hal_pins.gpio19.into_mode();
					pin.set_drive_strength(hal::gpio::OutputDriveStrength::EightMilliAmps);
					pin.set_slew_rate(hal::gpio::OutputSlewRate::Fast);
					pin
				},
				nirq_io: hal_pins.gpio20.into_pull_up_input(),
				noutput_en: {
					let mut pin = hal_pins.gpio21.into_push_pull_output();
					pin.set_high().unwrap();
					pin
				},
				i2s_adc_data: hal_pins.gpio22.into_mode(),
				i2s_dac_data: hal_pins.gpio26.into_mode(),
				i2s_bit_clock: hal_pins.gpio27.into_mode(),
				i2s_lr_clock: hal_pins.gpio28.into_mode(),
				pico_led: hal_pins.led.into_mode(),
			},
			// Set SPI up for 2 MHz clock, 8 data bits.
			spi_bus: hal::Spi::new(spi).init(
				resets,
				clocks.peripheral_clock.freq(),
				2_000_000.Hz(),
				&embedded_hal::spi::MODE_0,
			),
			delay,
			debug_leds: 0,
			last_cs: 0,
			keyboard: pc_keyboard::Keyboard::new(pc_keyboard::HandleControl::Ignore),
			event_queue: heapless::Deque::new(),
		}
	}

	/// Perform some SPI operation with the I/O chip selected.
	///
	/// You are required to have called `self.release_cs_lines()` previously,
	/// otherwise the I/O chip and your selected bus device will both see a
	/// chip-select signal.
	fn with_io_cs<F>(&mut self, func: F)
	where
		F: FnOnce(&mut hal::Spi<hal::spi::Enabled, pac::SPI0, 8_u8>),
	{
		// Select MCP23S17
		self.pins.nspi_cs_io.set_low().unwrap();
		// Setup time
		cortex_m::asm::delay(Self::CS_IO_SETUP_CPU_CLOCKS);
		// Do the SPI thing
		func(&mut self.spi_bus);
		// Hold the CS pin a bit longer
		cortex_m::asm::delay(Self::CS_IO_HOLD_CPU_CLOCKS);
		// Release the CS pin
		self.pins.nspi_cs_io.set_high().unwrap();
	}

	/// Write to a register on the MCP23S17 I/O chip.
	///
	/// * `register` - the address of the register to write to
	/// * `data` - the value to write
	fn io_chip_write(&mut self, register: u8, data: u8) {
		// Inter-packet delay
		cortex_m::asm::delay(Self::CS_IO_DISABLE_CPU_CLOCKS);

		// Do the operation with CS pin active
		self.with_io_cs(|spi| {
			spi.write(&[0x40, register, data]).unwrap();
		});
	}

	/// Read from a register on the MCP23S17 I/O chip.
	///
	/// * `register` - the address of the register to read from
	fn io_chip_read(&mut self, register: u8) -> u8 {
		// Inter-packet delay
		cortex_m::asm::delay(Self::CS_IO_DISABLE_CPU_CLOCKS);

		// Starts with outbound, is replaced with inbound
		let mut buffer = [0x41, register, 0x00];

		// Do the operation with CS pin active
		self.with_io_cs(|spi| {
			spi.transfer(&mut buffer).unwrap();
		});

		// Last byte has the value we read
		buffer[2]
	}

	/// Set the four debug LEDs on the PCB.
	///
	/// These are connected to the top 4 bits of GPIOA on the MCP23S17.
	fn set_debug_leds(&mut self, leds: u8) {
		// LEDs are active-low.
		let leds = (leds ^ 0xFF) & 0xF;
		self.debug_leds = leds << 1 | (self.debug_leds & 1);
		self.io_chip_write(0x12, self.debug_leds << 3 | self.last_cs);
	}

	/// Set the HDD LED on the PCB.
	///
	/// These are connected to the bit 4 of GPIOA on the MCP23S17.
	fn set_hdd_led(&mut self, enabled: bool) {
		// LEDs are active-low.
		self.debug_leds = (self.debug_leds & 0x1e) | if enabled { 0 } else { 1 };
		self.io_chip_write(0x12, self.debug_leds << 3 | self.last_cs);
	}

	/// Perform some SPI transaction with a specific bus chip-select pin active.
	///
	/// Activates a specific chip-select line, runs the closure (passing in the
	/// SPI bus object), then de-activates the CS pin.
	fn with_bus_cs<F>(&mut self, cs: u8, func: F)
	where
		F: FnOnce(&mut hal::Spi<hal::spi::Enabled, pac::SPI0, 8_u8>),
	{
		// Only CS0..CS7 is valid
		let cs = cs & 0b111;

		if cs != self.last_cs {
			// Set CS Outputs into decoder/buffer
			self.io_chip_write(0x12, self.debug_leds << 3 | cs);
			self.last_cs = cs;
		}

		// Drive CS lines from decoder/buffer
		self.drive_cs_lines();

		// Setup time
		cortex_m::asm::delay(Self::CS_BUS_SETUP_CPU_CLOCKS);

		// Call function
		func(&mut self.spi_bus);

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

	fn init_io_chip(&mut self) {
		// Undrive CS lines from decoder/buffer
		self.release_cs_lines();

		// Inter-packet delay
		cortex_m::asm::delay(Self::CS_IO_DISABLE_CPU_CLOCKS);

		// Set IODIRA = 0x00 => GPIOA is all outputs
		self.io_chip_write(Self::MCP23S17_DDRA, 0x00);

		// Inter-packet delay
		cortex_m::asm::delay(Self::CS_IO_DISABLE_CPU_CLOCKS);

		// Set GPIOA = 0x00 => GPIOA is all low
		self.io_chip_write(Self::MCP23S17_GPIOA, 0x00);

		// Set GPPUB to = 0xFF => GPIOB is pulled-up
		self.io_chip_write(Self::MCP23S17_GPPUB, 0xFF);
	}

	/// Read the BMC firmware version string.
	///
	/// You get 32 bytes of probably UTF-8 data.
	fn bmc_read_firmware_version(&mut self) -> Result<[u8; 32], ()> {
		let req = neotron_bmc_protocol::Request::new_read(false, 0x01, 32);
		let mut buffer = [0xFF; 64];
		buffer[0..=3].copy_from_slice(&req.as_bytes());
		self.with_bus_cs(0, |spi| {
			spi.transfer(&mut buffer).unwrap();
		});
		defmt::info!("buffer: {=[u8]:x}", buffer);
		let mut result = &buffer[..];
		let mut latency = 0;
		while !result.is_empty() && ((result[0] == 0xFF) || (result[0] == 0x00)) {
			latency += 1;
			result = &result[1..];
		}
		defmt::info!("latency: {}", latency);
		// 32 bytes of data requested, plus one bytes of response code and one byte of CRC
		if result.len() >= 34 {
			match neotron_bmc_protocol::Response::from_bytes(&result[0..34]) {
				Ok(res) => {
					if res.result == neotron_bmc_protocol::ResponseResult::Ok
						&& res.data.len() == 32
					{
						defmt::info!("Got BMC version {=[u8]:a}", res.data);
						let mut string_bytes = [0u8; 32];
						string_bytes.copy_from_slice(res.data);
						return Ok(string_bytes);
					} else {
						defmt::warn!(
							"Error getting BMC version: Error from BMC {:?} {=[u8]:x}",
							res.result,
							res.data
						);
					}
				}
				Err(e) => {
					defmt::warn!(
						"Error getting BMC version: Decoding Error {:?} {=[u8]:x}",
						e,
						result
					);
				}
			}
		}

		Err(())
	}

	/// Read the BMC PS/2 keyboard FIFO.
	///
	/// We ask for 8 bytes of data. We get `1` byte of 'length', then `N` bytes of valid data, and `32 - (N + 1)` bytes of padding.
	fn bmc_read_ps2_keyboard_fifo(&mut self, out_buffer: &mut [u8; 8]) -> Result<usize, ()> {
		static COUNTER: AtomicU8 = AtomicU8::new(0);
		let req = neotron_bmc_protocol::Request::new_read(USE_ALT.get(), 0x40, 8);
		for _retry in 0..4 {
			let mut buffer = [0xFF; 32];
			buffer[0..=3].copy_from_slice(&req.as_bytes());
			buffer[4] = COUNTER.load(Ordering::Relaxed);
			COUNTER.store(buffer[4].wrapping_add(1), Ordering::Relaxed);
			defmt::trace!("out: {=[u8]:02x}", buffer);
			self.with_bus_cs(0, |spi| {
				spi.transfer(&mut buffer).unwrap();
			});
			defmt::trace!("in : {=[u8]:02x}", buffer);
			// Skip the first four bytes at least (that's our command, and also
			// the BMC FIFO length which might have crud in it). Then trip any padding.
			let mut result = &buffer[4..];
			let mut latency = 0;
			while !result.is_empty() && (result[0] == 0xFF || result[0] == 0x00) {
				latency += 1;
				result = &result[1..];
			}
			defmt::trace!("latency: {}", latency);
			// 8 bytes of data requested, plus one bytes of response code and one byte of CRC
			if result.len() >= 10 {
				match neotron_bmc_protocol::Response::from_bytes(&result[0..10]) {
					Ok(res) => {
						if res.result == neotron_bmc_protocol::ResponseResult::Ok
							&& res.data.len() == 8
						{
							if res.data[0] == 0 {
								defmt::trace!("Got no PS/2 bytes");
							} else {
								defmt::debug!("Got PS/2 bytes {=[u8]:x}", res.data);
							}
							for (dest, src) in out_buffer.iter_mut().zip(res.data.iter().skip(1)) {
								*dest = *src;
							}
							return Ok(res.data[0] as usize);
						} else {
							defmt::warn!(
								"Error getting keyboard bytes: Error from BMC {:?} {=[u8]:x}",
								res.result,
								res.data
							);
						}
					}
					Err(e) => {
						defmt::warn!(
							"Error getting BMC keyboard bytes: Decoding Error {:?} {=[u8]:x}",
							e,
							result
						);
					}
				}
			} else {
				defmt::warn!("Short packet!?");
			}

			// Wait a bit before we try again
			cortex_m::asm::delay(Self::SPI_RETRY_CPU_CLOCKS);
		}
		// Ran out of retries
		panic!("KB retry timeout");
	}
}

fn sign_on() {
	static LICENCE_TEXT: &str = "\
        Copyright © Jonathan 'theJPster' Pallant and the Neotron Developers, 2022\n\
        \n\
        This program is free software under GPL v3 (or later)\n";

	// Create a new temporary console for some boot-up messages
	let tc = vga::TextConsole::new();
	tc.set_text_buffer(unsafe { &mut vga::GLYPH_ATTR_ARRAY });

	// A crude way to clear the screen
	for _col in 0..vga::MAX_TEXT_ROWS {
		writeln!(&tc).unwrap();
	}

	tc.move_to(0, 0);

	writeln!(&tc, "Neotron Pico BIOS {}", VERSION.trim_matches('\0')).unwrap();
	write!(&tc, "{}", LICENCE_TEXT).unwrap();

	let bmc_ver = critical_section::with(|cs| {
		let mut lock = HARDWARE.borrow_ref_mut(cs);
		let hw = lock.as_mut().unwrap();
		hw.bmc_read_firmware_version()
	});

	match bmc_ver {
		Ok(string_bytes) => match core::str::from_utf8(&string_bytes) {
			Ok(s) => {
				writeln!(&tc, "BMC Version: {}", s).unwrap();
			}
			Err(_e) => {
				writeln!(&tc, "BMC Version: Unknown").unwrap();
			}
		},
		Err(_e) => {
			writeln!(&tc, "BMC Version: Error reading").unwrap();
		}
	}

	critical_section::with(|cs| {
		let mut lock = HARDWARE.borrow_ref_mut(cs);
		let hw = lock.as_mut().unwrap();
		hw.delay.delay_ms(5000);
	});
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
	common::ApiString::new(VERSION)
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
pub extern "C" fn time_clock_get() -> common::Time {
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
pub extern "C" fn time_clock_set(_time: common::Time) {
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
	if vga::set_video_mode(mode) {
		common::Result::Ok(())
	} else {
		common::Result::Err(common::Error::UnsupportedConfiguration(0))
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
pub extern "C" fn memory_get_region(region: u8) -> common::Option<common::MemoryRegion> {
	match region {
		0 => {
			// Application Region
			common::Option::Some(MemoryRegion {
				start: unsafe { &mut _ram_os_start as *mut u32 } as *mut u8,
				length: unsafe { &mut _ram_os_len as *const u32 } as usize,
				kind: common::MemoryKind::Ram,
			})
		}
		_ => common::Option::None,
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
pub extern "C" fn hid_get_event() -> common::Result<common::Option<common::hid::HidEvent>> {
	let mut buffer = [0u8; 8];

	critical_section::with(|cs| {
		let mut lock = HARDWARE.borrow_ref_mut(cs);
		let hw = lock.as_mut().unwrap();

		if let Some(ev) = hw.event_queue.pop_front() {
			// Queued data available, so short-cut
			return common::Result::Ok(common::Option::Some(ev));
		}

		match hw.bmc_read_ps2_keyboard_fifo(&mut buffer) {
			Ok(n) if n > 0 => {
				let slice = if n >= 8 { &buffer } else { &buffer[0..n] };
				defmt::info!("{} bytes in KB FIFO, got: {=[u8]:x}", n, &slice);
				for b in slice.iter() {
					match hw.keyboard.add_byte(*b) {
						Ok(Some(key_event)) => {
							convert_hid_event(key_event, &mut hw.event_queue);
						}
						Ok(None) => {
							// Need more data
						}
						Err(e) => {
							panic!("Keyboard decode error!");
						}
					}
				}
			}
			Ok(_) => {
				// Say nothing - FIFO is empty
			}
			Err(e) => {
				defmt::warn!("Read KB error: {:?}", e);
			}
		}

		if let Some(ev) = hw.event_queue.pop_front() {
			// Queued data available, so short-cut
			common::Result::Ok(common::Option::Some(ev))
		} else {
			common::Result::Ok(common::Option::None)
		}
	})
}

fn convert_hid_event(
	pc_keyboard_ev: pc_keyboard::KeyEvent,
	ev_queue: &mut heapless::Deque<common::hid::HidEvent, 16>,
) {
	match pc_keyboard_ev.state {
		pc_keyboard::KeyState::Down => {
			ev_queue
				.push_back(common::hid::HidEvent::KeyPress(convert_hid_code(
					pc_keyboard_ev.code,
				)))
				.unwrap();
		}
		pc_keyboard::KeyState::Up => {
			ev_queue
				.push_back(common::hid::HidEvent::KeyRelease(convert_hid_code(
					pc_keyboard_ev.code,
				)))
				.unwrap();
		}
		pc_keyboard::KeyState::SingleShot => {
			ev_queue
				.push_back(common::hid::HidEvent::KeyPress(convert_hid_code(
					pc_keyboard_ev.code,
				)))
				.unwrap();
			ev_queue
				.push_back(common::hid::HidEvent::KeyRelease(convert_hid_code(
					pc_keyboard_ev.code,
				)))
				.unwrap();
		}
	}
}

fn convert_hid_code(pc_code: pc_keyboard::KeyCode) -> common::hid::KeyCode {
	match pc_code {
		pc_keyboard::KeyCode::AltLeft => common::hid::KeyCode::AltLeft,
		pc_keyboard::KeyCode::AltRight => common::hid::KeyCode::AltRight,
		pc_keyboard::KeyCode::ArrowDown => common::hid::KeyCode::ArrowDown,
		pc_keyboard::KeyCode::ArrowLeft => common::hid::KeyCode::ArrowLeft,
		pc_keyboard::KeyCode::ArrowRight => common::hid::KeyCode::ArrowRight,
		pc_keyboard::KeyCode::ArrowUp => common::hid::KeyCode::ArrowUp,
		pc_keyboard::KeyCode::BackSlash => common::hid::KeyCode::BackSlash,
		pc_keyboard::KeyCode::Backspace => common::hid::KeyCode::Backspace,
		pc_keyboard::KeyCode::BackTick => common::hid::KeyCode::BackTick,
		pc_keyboard::KeyCode::BracketSquareLeft => common::hid::KeyCode::BracketSquareLeft,
		pc_keyboard::KeyCode::BracketSquareRight => common::hid::KeyCode::BracketSquareRight,
		pc_keyboard::KeyCode::Break => common::hid::KeyCode::PauseBreak,
		pc_keyboard::KeyCode::CapsLock => common::hid::KeyCode::CapsLock,
		pc_keyboard::KeyCode::Comma => common::hid::KeyCode::Comma,
		pc_keyboard::KeyCode::ControlLeft => common::hid::KeyCode::ControlLeft,
		pc_keyboard::KeyCode::ControlRight => common::hid::KeyCode::ControlRight,
		pc_keyboard::KeyCode::Delete => common::hid::KeyCode::Delete,
		pc_keyboard::KeyCode::End => common::hid::KeyCode::End,
		pc_keyboard::KeyCode::Enter => common::hid::KeyCode::Enter,
		pc_keyboard::KeyCode::Escape => common::hid::KeyCode::Escape,
		pc_keyboard::KeyCode::Equals => common::hid::KeyCode::Equals,
		pc_keyboard::KeyCode::F1 => common::hid::KeyCode::F1,
		pc_keyboard::KeyCode::F2 => common::hid::KeyCode::F2,
		pc_keyboard::KeyCode::F3 => common::hid::KeyCode::F3,
		pc_keyboard::KeyCode::F4 => common::hid::KeyCode::F4,
		pc_keyboard::KeyCode::F5 => common::hid::KeyCode::F5,
		pc_keyboard::KeyCode::F6 => common::hid::KeyCode::F6,
		pc_keyboard::KeyCode::F7 => common::hid::KeyCode::F7,
		pc_keyboard::KeyCode::F8 => common::hid::KeyCode::F8,
		pc_keyboard::KeyCode::F9 => common::hid::KeyCode::F9,
		pc_keyboard::KeyCode::F10 => common::hid::KeyCode::F10,
		pc_keyboard::KeyCode::F11 => common::hid::KeyCode::F11,
		pc_keyboard::KeyCode::F12 => common::hid::KeyCode::F12,
		pc_keyboard::KeyCode::Fullstop => common::hid::KeyCode::Fullstop,
		pc_keyboard::KeyCode::Home => common::hid::KeyCode::Home,
		pc_keyboard::KeyCode::Insert => common::hid::KeyCode::Insert,
		pc_keyboard::KeyCode::Key1 => common::hid::KeyCode::Key1,
		pc_keyboard::KeyCode::Key2 => common::hid::KeyCode::Key2,
		pc_keyboard::KeyCode::Key3 => common::hid::KeyCode::Key3,
		pc_keyboard::KeyCode::Key4 => common::hid::KeyCode::Key4,
		pc_keyboard::KeyCode::Key5 => common::hid::KeyCode::Key5,
		pc_keyboard::KeyCode::Key6 => common::hid::KeyCode::Key6,
		pc_keyboard::KeyCode::Key7 => common::hid::KeyCode::Key7,
		pc_keyboard::KeyCode::Key8 => common::hid::KeyCode::Key8,
		pc_keyboard::KeyCode::Key9 => common::hid::KeyCode::Key9,
		pc_keyboard::KeyCode::Key0 => common::hid::KeyCode::Key0,
		pc_keyboard::KeyCode::Menus => common::hid::KeyCode::Menus,
		pc_keyboard::KeyCode::Minus => common::hid::KeyCode::Minus,
		pc_keyboard::KeyCode::Numpad0 => common::hid::KeyCode::Numpad0,
		pc_keyboard::KeyCode::Numpad1 => common::hid::KeyCode::Numpad1,
		pc_keyboard::KeyCode::Numpad2 => common::hid::KeyCode::Numpad2,
		pc_keyboard::KeyCode::Numpad3 => common::hid::KeyCode::Numpad3,
		pc_keyboard::KeyCode::Numpad4 => common::hid::KeyCode::Numpad4,
		pc_keyboard::KeyCode::Numpad5 => common::hid::KeyCode::Numpad5,
		pc_keyboard::KeyCode::Numpad6 => common::hid::KeyCode::Numpad6,
		pc_keyboard::KeyCode::Numpad7 => common::hid::KeyCode::Numpad7,
		pc_keyboard::KeyCode::Numpad8 => common::hid::KeyCode::Numpad8,
		pc_keyboard::KeyCode::Numpad9 => common::hid::KeyCode::Numpad9,
		pc_keyboard::KeyCode::NumpadEnter => common::hid::KeyCode::NumpadEnter,
		pc_keyboard::KeyCode::NumpadLock => common::hid::KeyCode::NumpadLock,
		pc_keyboard::KeyCode::NumpadSlash => common::hid::KeyCode::NumpadSlash,
		pc_keyboard::KeyCode::NumpadStar => common::hid::KeyCode::NumpadStar,
		pc_keyboard::KeyCode::NumpadMinus => common::hid::KeyCode::NumpadMinus,
		pc_keyboard::KeyCode::NumpadPeriod => common::hid::KeyCode::NumpadPeriod,
		pc_keyboard::KeyCode::NumpadPlus => common::hid::KeyCode::NumpadPlus,
		pc_keyboard::KeyCode::PageDown => common::hid::KeyCode::PageDown,
		pc_keyboard::KeyCode::PageUp => common::hid::KeyCode::PageUp,
		pc_keyboard::KeyCode::PauseBreak => common::hid::KeyCode::PauseBreak,
		pc_keyboard::KeyCode::PrintScreen => common::hid::KeyCode::PrintScreen,
		pc_keyboard::KeyCode::ScrollLock => common::hid::KeyCode::ScrollLock,
		pc_keyboard::KeyCode::SemiColon => common::hid::KeyCode::SemiColon,
		pc_keyboard::KeyCode::ShiftLeft => common::hid::KeyCode::ShiftLeft,
		pc_keyboard::KeyCode::ShiftRight => common::hid::KeyCode::ShiftRight,
		pc_keyboard::KeyCode::Slash => common::hid::KeyCode::Slash,
		pc_keyboard::KeyCode::Spacebar => common::hid::KeyCode::Spacebar,
		pc_keyboard::KeyCode::Tab => common::hid::KeyCode::Tab,
		pc_keyboard::KeyCode::Quote => common::hid::KeyCode::Quote,
		pc_keyboard::KeyCode::WindowsLeft => common::hid::KeyCode::WindowsLeft,
		pc_keyboard::KeyCode::WindowsRight => common::hid::KeyCode::WindowsRight,
		pc_keyboard::KeyCode::A => common::hid::KeyCode::A,
		pc_keyboard::KeyCode::B => common::hid::KeyCode::B,
		pc_keyboard::KeyCode::C => common::hid::KeyCode::C,
		pc_keyboard::KeyCode::D => common::hid::KeyCode::D,
		pc_keyboard::KeyCode::E => common::hid::KeyCode::E,
		pc_keyboard::KeyCode::F => common::hid::KeyCode::F,
		pc_keyboard::KeyCode::G => common::hid::KeyCode::G,
		pc_keyboard::KeyCode::H => common::hid::KeyCode::H,
		pc_keyboard::KeyCode::I => common::hid::KeyCode::I,
		pc_keyboard::KeyCode::J => common::hid::KeyCode::J,
		pc_keyboard::KeyCode::K => common::hid::KeyCode::K,
		pc_keyboard::KeyCode::L => common::hid::KeyCode::L,
		pc_keyboard::KeyCode::M => common::hid::KeyCode::M,
		pc_keyboard::KeyCode::N => common::hid::KeyCode::N,
		pc_keyboard::KeyCode::O => common::hid::KeyCode::O,
		pc_keyboard::KeyCode::P => common::hid::KeyCode::P,
		pc_keyboard::KeyCode::Q => common::hid::KeyCode::Q,
		pc_keyboard::KeyCode::R => common::hid::KeyCode::R,
		pc_keyboard::KeyCode::S => common::hid::KeyCode::S,
		pc_keyboard::KeyCode::T => common::hid::KeyCode::T,
		pc_keyboard::KeyCode::U => common::hid::KeyCode::U,
		pc_keyboard::KeyCode::V => common::hid::KeyCode::V,
		pc_keyboard::KeyCode::W => common::hid::KeyCode::W,
		pc_keyboard::KeyCode::X => common::hid::KeyCode::X,
		pc_keyboard::KeyCode::Y => common::hid::KeyCode::Y,
		pc_keyboard::KeyCode::Z => common::hid::KeyCode::Z,
		pc_keyboard::KeyCode::HashTilde => common::hid::KeyCode::HashTilde,
		pc_keyboard::KeyCode::PrevTrack => common::hid::KeyCode::PrevTrack,
		pc_keyboard::KeyCode::NextTrack => common::hid::KeyCode::NextTrack,
		pc_keyboard::KeyCode::Mute => common::hid::KeyCode::Mute,
		pc_keyboard::KeyCode::Calculator => common::hid::KeyCode::Calculator,
		pc_keyboard::KeyCode::Play => common::hid::KeyCode::Play,
		pc_keyboard::KeyCode::Stop => common::hid::KeyCode::Stop,
		pc_keyboard::KeyCode::VolumeDown => common::hid::KeyCode::VolumeDown,
		pc_keyboard::KeyCode::VolumeUp => common::hid::KeyCode::VolumeUp,
		pc_keyboard::KeyCode::WWWHome => common::hid::KeyCode::WWWHome,
		pc_keyboard::KeyCode::PowerOnTestOk => common::hid::KeyCode::PowerOnTestOk,
		_ => common::hid::KeyCode::X,
	}
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

/// Read the RGB palette. Currently we only have two colours and you can't
/// change them.
extern "C" fn video_get_palette(index: u8) -> common::Option<common::video::RGBColour> {
	match index {
		0 => common::Option::Some(vga::colours::BLUE.into()),
		1 => common::Option::Some(vga::colours::YELLOW.into()),
		_ => common::Option::None,
	}
}

/// Update the RGB palette
extern "C" fn video_set_palette(index: u8, rgb: common::video::RGBColour) {
	// TODO set the palette when we actually have one
}

/// Update all the RGB palette
unsafe extern "C" fn video_set_whole_palette(
	palette: *const common::video::RGBColour,
	length: usize,
) {
	// TODO set the palette when we actually have one
}

extern "C" fn i2c_bus_get_info(_i2c_bus: u8) -> common::Option<common::i2c::BusInfo> {
	unimplemented!();
}

extern "C" fn i2c_write_read(
	_i2c_bus: u8,
	_i2c_device_address: u8,
	_tx: common::ApiByteSlice,
	_tx2: common::ApiByteSlice,
	_rx: common::ApiBuffer,
) -> common::Result<()> {
	unimplemented!();
}

extern "C" fn audio_mixer_channel_get_info(
	_audio_mixer_id: u8,
) -> common::Result<common::audio::MixerChannelInfo> {
	unimplemented!();
}

extern "C" fn audio_mixer_channel_set_level(_audio_mixer_id: u8, _level: u8) -> common::Result<()> {
	unimplemented!();
}

extern "C" fn audio_output_set_config(_config: common::audio::Config) -> common::Result<()> {
	unimplemented!();
}

extern "C" fn audio_output_get_config() -> common::Result<common::audio::Config> {
	unimplemented!();
}

unsafe extern "C" fn audio_output_data(_samples: common::ApiByteSlice) -> common::Result<usize> {
	unimplemented!();
}

extern "C" fn audio_output_get_space() -> common::Result<usize> {
	unimplemented!();
}

extern "C" fn audio_input_set_config(_config: common::audio::Config) -> common::Result<()> {
	unimplemented!();
}

extern "C" fn audio_input_get_config() -> common::Result<common::audio::Config> {
	unimplemented!();
}

extern "C" fn audio_input_data(_samples: common::ApiBuffer) -> common::Result<usize> {
	unimplemented!();
}

extern "C" fn audio_input_get_count() -> common::Result<usize> {
	unimplemented!();
}

extern "C" fn bus_select(_periperal_id: common::Option<u8>) {
	unimplemented!();
}

extern "C" fn bus_get_info(_periperal_id: u8) -> common::Option<common::bus::PeripheralInfo> {
	unimplemented!();
}

extern "C" fn bus_write_read(
	_tx: common::ApiByteSlice,
	_tx2: common::ApiByteSlice,
	_rx: common::ApiBuffer,
) -> common::Result<()> {
	unimplemented!();
}

extern "C" fn bus_exchange(_buffer: common::ApiBuffer) -> common::Result<()> {
	unimplemented!();
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
pub extern "C" fn block_dev_get_info(device: u8) -> common::Option<common::block_dev::DeviceInfo> {
	match device {
		0 => {
			common::Option::Some(common::block_dev::DeviceInfo {
				// This is the built-in SD card slot
				name: common::types::ApiString::new("SdCard0"),
				device_type: common::block_dev::DeviceType::SecureDigitalCard,
				// This is the standard for SD cards
				block_size: 512,
				// TODO: scan the card here
				num_blocks: 0,
				// No motorised eject
				ejectable: false,
				// But you can take the card out
				removable: true,
				// Pretend the card is out
				media_present: true,
				// Don't care about this value when card is out
				read_only: false,
			})
		}
		_ => {
			// Nothing else supported by this BIOS
			common::Option::None
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
	_data: common::ApiByteSlice,
) -> common::Result<()> {
	common::Result::Err(common::Error::Unimplemented)
}

/// Read one or more sectors to a block device.
///
/// The function will block until all data is read. The array pointed
/// to by `data` must be `num_blocks * block_size` in length, where
/// `block_size` is given by `block_dev_get_info`.
///
/// There are no requirements on the alignment of `data` but if it is
/// aligned, the BIOS may be able to use a higher-performance code path.
pub extern "C" fn block_read(
	_device: u8,
	_block: common::block_dev::BlockIdx,
	_num_blocks: u8,
	_data: common::ApiBuffer,
) -> common::Result<()> {
	common::Result::Err(common::Error::Unimplemented)
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
	_data: common::ApiByteSlice,
) -> common::Result<()> {
	common::Result::Err(common::Error::Unimplemented)
}

extern "C" fn block_dev_eject(dev_id: u8) -> common::Result<()> {
	common::Result::Ok(())
}

/// Sleep the CPU until the next interrupt.
extern "C" fn power_idle() {
	// cortex_m::asm::wfe();
	cortex_m::asm::delay(1_000_000);
}

/// TODO: Get the monotonic run-time of the system from SysTick.
extern "C" fn time_ticks_get() -> common::Ticks {
	common::Ticks(0)
}

/// We have a 1 kHz SysTick
extern "C" fn time_ticks_per_second() -> common::Ticks {
	common::Ticks(1000)
}

/// Called when DMA raises IRQ0; i.e. when a DMA transfer to the pixel FIFO or
/// the timing FIFO has completed.
#[interrupt]
fn DMA_IRQ_0() {
	unsafe {
		vga::irq();
	}
}

// -----------------------------------------------------------------------------
// End of file
// -----------------------------------------------------------------------------
