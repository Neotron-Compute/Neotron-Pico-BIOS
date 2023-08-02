//! Code to set up a PIO to generate I2S Audio

use rp_pico::hal::{pac::interrupt, prelude::*};

/// Holds the objects we need to read from the RAM fifo and write to the PIO hardware FIFO.
static mut PLAYBACK_TO_PIO: Option<PlaybackToPio> = None;

/// The reader end of the RAM FIFO and the writer end of the PIO hardware FIFO.
struct PlaybackToPio {
	pio_fifo: rp_pico::hal::pio::Tx<(rp_pico::pac::PIO1, rp_pico::hal::pio::SM0)>,
	ram_fifo: heapless::spsc::Consumer<'static, u32, 1024>,
}

/// Used to 'play' samples by writing them to the RAM FIFO.
///
/// Holds the writer end of the RAM FIFO.
pub struct Player {
	fifo: heapless::spsc::Producer<'static, u32, 1024>,
}

impl Player {
	/// Queue some samples for playback.
	///
	/// Takes as many as will fit in the FIFO. Returns how many were taken,
	/// which will always be a multiple of 4 because it always takes them in
	/// Little Endian 16-bit stereo.
	///
	/// The length of `samples` must be a multiple of 4, as they should be
	/// 16-bit stereo pairs.
	pub fn play_samples_16bit_stereo_48khz(&mut self, samples: &[u8]) -> usize {
		let mut count = 0;
		for samples in samples.chunks_exact(4) {
			if self.fifo.ready() {
				let left_sample = (samples[1] as u32) << 8 | (samples[0] as u32);
				let right_sample = (samples[3] as u32) << 8 | (samples[2] as u32);
				let stereo_sample = left_sample << 16 | right_sample;
				self.fifo.enqueue(stereo_sample).unwrap();
				count += 4;
			} else {
				break;
			}
		}
		count
	}

	/// Space, in stereo samples.
	pub fn available_space(&self) -> usize {
		self.fifo.capacity() - self.fifo.len()
	}
}

/// Initialise PIO1 to produce I2S audio.
///
/// Returns a FIFO you can send samples into.
pub fn init(pio: super::pac::PIO1, resets: &mut super::pac::RESETS) -> Player {
	// Grab PIO0 and the state machines it contains
	let (mut pio, sm0, _sm1, _sm2, _sm3) = pio.split(resets);

	// This is the I2S program. It transfers two 16-bit words (left and right),
	// packed as a 32-bit value in the FIFO, according to the clock signals received.
	//
	// You should clock out data Left (i.e. MSB  first) to be I2S compatible.
	let samples_program = pio_proc::pio_asm!(
		".wrap_target"

		// 1. Spin until L word starts (LRCLK goes from low to high).

		"wait 0 pin 3"         // Wait for LRCLK low
		"wait 1 pin 3"         // Wait for LRCLK high

		// 2. Skip dummy bit (which comes after LRCLK transition)

		"set x, 15"            // Set loop count whilst we wait (it's free)
		"wait 0 pin 2"         // Wait for BCLK to finish going low
		"wait 1 pin 2"         // Wait for BCLK rising edge (middle of the dummy bit)

		// 3. Read/Write 16 bits of left channel data

		"left_loop:"
		"  wait 0 pin 2"       // Wait for BCLK falling edge (start of bit)
		"  out pins, 1"        // Write DAC bit
		"  wait 1 pin 2"       // Wait for BCLK rising edge (middle of bit)
		"  in pins, 1"         // Read ADC bit
		"  jmp x-- left_loop"  // Repeat until x is 0 (runs for N + 1 loops)

		// 4. Spin until R word starts (LRCLK goes low)

		"wait 0 pin 3"         // Wait for LRCLK low

		// 5. Skip dummy bit (which comes after LRCLK transition)

		"set x, 15"            // Set loop count whilst we wait (it's free)
		"wait 0 pin 2"         // Wait for BCLK to finish going low
		"wait 1 pin 2"         // Wait for BCLK rising edge (middle of the dummy bit)

		// 6. Read/Write 16 bits of left channel data

		"right_loop:"
		"  wait 0 pin 2"       // Wait for BCLK falling edge (start of bit)
		"  out pins, 1"        // Write DAC bit
		"  wait 1 pin 2"       // Wait for BCLK rising edge (middle of bit)
		"  in pins, 1"         // Read ADC bit
		"  jmp x-- right_loop" // Repeat until x is 0 (runs for N + 1 loops)

		// 7. Load + Store all 32 bits (16 bits left, 16 bits right)

		"push noblock"
		"pull noblock"

		".wrap"
	);

	// R00 is the LSB of the right word, R15 is the MSB of the right word.
	// L00 is the LSB of the left word, L15 is the MSB of the left word.
	//
	// There is one dummy bit after the LRCLK edge. There may be more than 17 bit-clocks in each
	// phase of the LRCLK signal - ignore any extra bits. In fact on the TLV320AIC23B we get
	// a total of 125 bit-clocks for the left and another 125 bit-clocks for the right because
	// the bit clock is 12 MHz and the LRCLK is 48 kHz.
	//
	// DAC   :  ..┬──┬──┬──┬──┬──┬──┬──┬──┬...┬──┬──┬──┬──┬──┬──┬...┬──┬──┬──┬──┬──┬──┬..
	//       :    │ Pad │ L15 │ L14 │ L13 │   │ L02 │ L01 │ L00 │   │ Pad │ R15 │ R14 │
	//       :  ..┴──┴──┴──┴──┴──┴──┴──┴──┴...┴──┴──┴──┴──┴──┴──┴...┴──┴──┴──┴──┴──┴──┴..
	// LRCLK :    ┌─────────────────────────────────────────────────┐
	//       :    │                                                 │
	//       : ..─┘                                                 └─────────────────..
	// BCLK  : ..─┐  ┌──┐  ┌──┐  ┌──┐  ┌──┐ ..┐  ┌──┐  ┌──┐  ┌──┐ ..┐  ┌──┐  ┌──┐  ┌──┐
	//       :    │  │  │  │  │  │  │  │  │   │  │  │  │  │  │  │   │  │  │  │  │  │  │
	//       :    └──┘  └──┘  └──┘  └──┘  └.. └──┘  └──┘  └──┘  └.. └──┘  └──┘  └──┘  └..
	//                  ^  ^
	//                  │  │
	//                  │  └── Read ADC on rising BCLK
	//                  └── Update DAC on falling BCLK
	//
	// ADC (CODEC-to-Pico) is GPIO25
	// DAC (Pico-to-CODEC) is GPIO26
	// BCLK is GPIO27
	// LRCLK is GPIO28

	let samples_installed = pio.install(&samples_program.program).unwrap();
	let (mut samples_sm, _sample_rx_fifo, pio_tx_fifo) =
		rp_pico::hal::pio::PIOBuilder::from_program(samples_installed)
			.buffers(rp_pico::hal::pio::Buffers::RxTx)
			.out_pins(26, 1) // Data is GPIO26
			.in_pin_base(25)
			.autopull(false)
			.autopush(false)
			.out_shift_direction(rp_pico::hal::pio::ShiftDirection::Left)
			.in_shift_direction(rp_pico::hal::pio::ShiftDirection::Left)
			.build(sm0);
	samples_sm.set_pindirs([
		(25, rp_pico::hal::pio::PinDir::Input),
		(26, rp_pico::hal::pio::PinDir::Output),
		(27, rp_pico::hal::pio::PinDir::Input),
		(28, rp_pico::hal::pio::PinDir::Input),
	]);

	let _running_sam = samples_sm.start();

	static mut SAMPLE_QUEUE: heapless::spsc::Queue<u32, 1024> = heapless::spsc::Queue::new();
	let (q_producer, q_consumer) = unsafe { SAMPLE_QUEUE.split() };

	pio_tx_fifo.enable_tx_not_full_interrupt(rp_pico::hal::pio::PioIRQ::Irq0);

	critical_section::with(|_| unsafe {
		PLAYBACK_TO_PIO.replace(PlaybackToPio {
			pio_fifo: pio_tx_fifo,
			ram_fifo: q_consumer,
		});
	});

	unsafe {
		rp_pico::hal::pac::NVIC::unmask(rp_pico::hal::pac::Interrupt::PIO1_IRQ_0);
	}

	Player { fifo: q_producer }
}

/// Called when the PIO1 IRQ fires.
///
/// We mapped this to "TX FIFO is not empty", so this interrupt will be
/// repeatedly called whilst the PIO FIFO is not full.
#[interrupt]
fn PIO1_IRQ_0() {
	// This is the only function (apart from `init`) which accesses this
	// variable, and `init()` is sure to disable interrupts until the global is
	// set up, so this is safe.
	let Some(fifo) = (unsafe { PLAYBACK_TO_PIO.as_mut() }) else {
		return;
	};
	// Read from fifo.ram_fifo
	if let Some(sample) = fifo.ram_fifo.dequeue() {
		// .. and write to fifo.pio_fifo
		fifo.pio_fifo.write(sample);
	} else {
		// Read failed to play null sample to in-fill
		fifo.pio_fifo.write(0);
	}
	// Send an event, just in case this CPU just went to sleep
	cortex_m::asm::sev();
}
