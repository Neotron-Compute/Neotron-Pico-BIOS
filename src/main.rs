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
use core::fmt::Write;

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

/// The BIOS version string
static BIOS_VERSION: &str = concat!("Neotron Pico BIOS version ", env!("BIOS_VERSION"), "\0");

/// Our global hardware object.
///
/// You need to grab this to do anything with the hardware.
static HARDWARE: Mutex<core::cell::RefCell<Option<Hardware>>> =
	Mutex::new(core::cell::RefCell::new(None));

/// This is our Operating System. It must be compiled separately.
///
/// The RP2040 requires an OS linked at `0x1002_0000`, which is the OS binary
/// `flash1002`. Use `objdump` as per the README file to make a `flash1002.bin`.
#[link_section = ".flash_os"]
#[used]
pub static OS_IMAGE: [u8; include_bytes!("flash1002.bin").len()] = *include_bytes!("flash1002.bin");

/// The table of API calls we provide the OS
static API_CALLS: common::Api = common::Api {
	api_version_get,
	bios_version_get,
	serial_configure,
	serial_get_info,
	serial_write,
	serial_read,
	time_get,
	time_set,
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
		xosc.operating_frequency().into(),
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
	// let code: &common::OsStartFn = unsafe { ::core::mem::transmute(&_flash_os_start) };
	// code(&API_CALLS);

	// OS has returned?! Just hang.
	loop {
		cortex_m::asm::wfi();
	}
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

	/// Give the device 2us (2 clocks @ 1 MHz) to get ready.
	const CS_BUS_SETUP_CPU_CLOCKS: u32 = 2000 / Self::NS_PER_CLOCK_CYCLE;

	/// Give the device 2us (2 clocks @ 1 MHz) before we take away CS.
	const CS_BUS_HOLD_CPU_CLOCKS: u32 = 2000 / Self::NS_PER_CLOCK_CYCLE;

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
			// Set SPI up for 1 MHz clock, 8 data bits.
			spi_bus: hal::Spi::new(spi).init(
				resets,
				clocks.peripheral_clock.freq(),
				1_000_000.Hz(),
				&embedded_hal::spi::MODE_0,
			),
			delay,
			debug_leds: 0,
			last_cs: 0,
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

	/// Write to a registers on the MCP23S17 I/O chip.
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

	/// Write to a registers on the MCP23S17 I/O chip.
	///
	/// * `register` - the address of the register to write to
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
		self.debug_leds = (self.debug_leds & 0x17) | if enabled { 0 } else { 1 };
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

	fn dump_io_chip(&mut self) {
		// Inter-packet delay
		cortex_m::asm::delay(Self::CS_IO_DISABLE_CPU_CLOCKS);

		// Dump registers
		for reg in 0x00..=0x15 {
			let data = self.io_chip_read(reg);
			defmt::debug!("Reg 0x{:02x} => 0x{:02x}", reg, data);
		}
	}

	/// Read the BMC firmware version string.
	///
	/// You get 32 bytes of probably UTF-8 data.
	fn bmc_read_firmware_version(&mut self) -> Result<[u8; 32], ()> {
		let req = neotron_bmc_protocol::Request::new_read(false, 0, 32);
		let mut buffer = [0xFF; 64];
		buffer[0..=3].copy_from_slice(&req.as_bytes());
		self.with_bus_cs(0, |spi| {
			spi.transfer(&mut buffer).unwrap();
		});
		let mut result = &buffer[..];
		let mut latency = 0;
		while result.len() > 0 && result[0] == 0xFF {
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

	writeln!(&tc, "{}", &BIOS_VERSION[0..BIOS_VERSION.len() - 1]).unwrap();
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
	_block: u64,
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
	_block: u64,
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
	_block: u64,
	_num_blocks: u8,
	_data: common::ApiByteSlice,
) -> common::Result<()> {
	common::Result::Err(common::Error::Unimplemented)
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
