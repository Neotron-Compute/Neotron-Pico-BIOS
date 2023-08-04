//! Code to set up a PIO to generate I2S Audio

use rp_pico::hal::prelude::*;

/// Initialise PIO1 to produce I2S audio.
///
/// Returns a FIFO you can send samples into.
pub fn init(
	pio: super::pac::PIO1,
	resets: &mut super::pac::RESETS,
) -> rp_pico::hal::pio::Tx<(rp_pico::pac::PIO1, rp_pico::hal::pio::SM0)> {
	// Grab PIO0 and the state machines it contains
	let (mut pio, sm0, _sm1, _sm2, _sm3) = pio.split(resets);
	// This is the I2S program. It transfers two 16-bit words (left and right),
	// packed as a 32-bit value in the FIFO. It clocks out 15 bits in a loop
	// then clocks out bit 16 manually, to avoid the cost of the jump. It takes
	// 2 cycles per sample.
	//
	// You should clock out data Left (i.e. MSB  first) to be I2S compatible.
	let samples_program = pio_proc::pio_asm!(
		".side_set 2"
		"set x, 14         side 0b00"
		".wrap_target"
		"leftloop:"
		"out pins, 1       side 0b10"
		"jmp x-- leftloop  side 0b11"
		"out pins, 1       side 0b00"
		"set x, 14         side 0b01"
		"rightloop: "
		"out pins, 1       side 0b00"
		"jmp x-- rightloop side 0b01"
		"out pins, 1       side 0b10"
		"set x, 14         side 0b11"
		".wrap"
	);

	let samples_installed = pio.install(&samples_program.program).unwrap();
	let (mut samples_sm, _sample_rx_fifo, samples_tx_fifo) =
		rp_pico::hal::pio::PIOBuilder::from_program(samples_installed)
			.buffers(rp_pico::hal::pio::Buffers::OnlyTx)
			.out_pins(26, 1) // Data is GPIO26
			.side_set_pin_base(27) // BCLK is GPIO27, LRCLK is GPIO28
			.autopull(true)
			.out_shift_direction(rp_pico::hal::pio::ShiftDirection::Left)
			.pull_threshold(32)
			.build(sm0);
	samples_sm.set_pindirs((26..=28).map(|x| (x, rp_pico::hal::pio::PinDir::Output)));
	// We are at 151.2 MHz and we want 48 kHz * 16 * 2 samples per second, or
	// 1.536 MHz. We need two PIO clocks per sample.
	let divisor = 151_200_000.0 / (2.0 * 1_536_000.0);
	samples_sm.set_clock_divisor(divisor);
	let _running_sam = samples_sm.start();

	samples_tx_fifo
}

/// Play some samples on a PIO TX FIFO.
///
/// The samples must be 16-bit LE stereo, packed as Left|Right into 32-bit
/// words.
///
/// This function blocks the CPU until all the samples are played.
pub fn play<T>(fifo: &mut rp_pico::hal::pio::Tx<T>, samples: &[u32])
where
	T: rp_pico::hal::pio::ValidStateMachine,
{
	for sample in samples {
		while fifo.is_full() {
			// spin
		}
		fifo.write(*sample);
	}
}

/// JGP saying "test".
///
/// This sample is in 16-bit little-endian PCM stereo at 48 kHz. The click at
/// the end is me pressing the mouse button to stop the recording in audacity.
#[link_section = ".flash_samples"]
pub static TEST_DATA: [u8; 236480] = *include_bytes!("./test-sample.raw");
