//! Multi-core support.
//!
//! Functions for launching code onto Core 1.

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

use core::sync::atomic::{AtomicBool, Ordering};

// -----------------------------------------------------------------------------
// Types
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
// Static and Const Data
// -----------------------------------------------------------------------------

/// Used to signal when Core 1 has started
static CORE1_START_FLAG: AtomicBool = AtomicBool::new(false);

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

// -----------------------------------------------------------------------------
// Functions
// -----------------------------------------------------------------------------

/// Starts core 1 running the given function, with the given stack.
pub fn launch_core1_with_stack(
	main_func: unsafe extern "C" fn() -> u32,
	stack: &mut [usize],
	ppb: &mut crate::pac::PPB,
	fifo: &mut rp_pico::hal::sio::SioFifo,
	psm: &mut crate::pac::PSM,
) {
	defmt::debug!("Resetting CPU1...");

	psm.frce_off.modify(|_, w| w.proc1().set_bit());
	while !psm.frce_off.read().proc1().bit_is_set() {
		cortex_m::asm::nop();
	}
	psm.frce_off.modify(|_, w| w.proc1().clear_bit());

	defmt::debug!("Setting up stack...");

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

	defmt::debug!("Stack ptr is 0x{:x}", stack_ptr);
	defmt::debug!("Stack bottom is 0x{:x}", stack.as_ptr());
	defmt::debug!("Stack top is 0x{:x}", &stack[stack.len() - 4..stack.len()]);

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
			defmt::debug!("Sending command {:x}...", *cmd);

			// we drain before sending a 0
			if *cmd == 0 {
				defmt::debug!("Draining FIFO...");
				fifo.drain();
				// core 1 may be waiting for fifo space
				cortex_m::asm::sev();
			}
			defmt::debug!("Pushing to FIFO...");
			fifo.write_blocking(*cmd);

			defmt::debug!("Getting response from FIFO...");
			let response = loop {
				if let Some(x) = fifo.read() {
					break x;
				} else {
					defmt::debug!("ST is {:x}", fifo.status());
				}
			};

			// move to next state on correct response otherwise start over
			defmt::debug!("Got {:x}", response);
			if *cmd != response {
				continue 'outer;
			}
		}
		break;
	}

	if enabled {
		unsafe { crate::pac::NVIC::unmask(crate::pac::Interrupt::SIO_IRQ_PROC0) };
	}

	defmt::debug!("Waiting for Core 1 to start...");
	while !CORE1_START_FLAG.load(Ordering::Relaxed) {
		cortex_m::asm::nop();
	}
	defmt::debug!("Core 1 started!!");
}

/// The bootrom code will call this function on core1 to perform any set-up, before the
/// entry function is called.
extern "C" fn core1_wrapper(entry_func: extern "C" fn() -> u32, _stack_base: *mut u32) -> u32 {
	CORE1_START_FLAG.store(true, Ordering::Relaxed);
	entry_func()
}

// -----------------------------------------------------------------------------
// End of file
// -----------------------------------------------------------------------------
