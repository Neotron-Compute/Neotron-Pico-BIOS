# Neotron Pico BIOS

This is the [Neotron](https://github.com/neotron-compute) BIOS for the [Neotron
Pico] board.

[Neotron Pico]: (https://github.com/neotron-compute/neotron-pico)

![Build Status](https://github.com/neotron-compute/neotron-pico-bios/workflows/Build/badge.svg "Github Action Build Status")

![Format Status](https://github.com/neotron-compute/neotron-pico-bios/workflows/Format/badge.svg "Github Action Format Check Status")

## Compatibility

This BIOS will run on the [Neotron Pico] v0.5.0. Other boards based around the
Raspberry Pi Pico could be supported with just a minor change to the pin
configurations.

## Features

The Neotron Pico offers:

* Dual Cortex-M0+ clocked at 133 MHz
* 256 KiB RAM
* 2048 KiB Flash
* SD/MMC Slot
* 4096-colour VGA output
* Stereo 16-bit 48kHz audio
* Four Neotron expansion slots
* A dedicated Board Management Controller, offering:
  * Dual PS/2 ports
  * 5-wire TTL UART
  * Power/reset control

Currently the BIOS only uses a single core, and starts the OS on the same core.
Later versions may move some functionality onto the second core (e.g. screen
updates) for performance reasons.

## Programming

The Neotron BIOS uses the [defmt](https://crates.io/crates/defmt) crate to provide structured logging over the SWD interface. The easiest way to flash and debug your Neotron Pico BIOS is with a second Raspberry Pi Pico.

1. Connect your *Debugger* Pico to the *Neotron* Pico:
    * connect Pin 3 on the *Debugger* Pico to GND on the *Neotron* Pico
    * connect Pin 4 on the *Debugger* Pico to SWCLK on the *Neotron* Pico
    * connect Pin 5 on the *Debugger* Pico to SWDIO on the *Neotron* Pico
    * connect USB on the *Debugger* Pico to your PC

2. Flash your *Debugger* Pico with https://github.com/majbthrd/DapperMime firmware (e.g. by copying the UF2 file to the USB Mass Storage device)

3. On your PC, install *probe-rs-rp* from the RP2040-specific [probe-run](https://github.com/knurling-rs/probe-run) fork at https://github.com/rp-rs/probe-run.

```console
user@host ~ $ cargo install probe-rs-rp
```

4. Power on your Neotron Pico.

5. Build and load the Neotron BIOS, and view the debug output stream, with `cargo run`:

```console
user@host ~/neotron-pico-bios $ cargo run --release
   Compiling neotron-pico-bios v0.1.0 (/home/jonathan/Documents/neotron/neotron-pico-bios)
    Finished release [optimized + debuginfo] target(s) in 0.76s
     Running `probe-run-rp --chip RP2040 target/thumbv6m-none-eabi/release/neotron-pico-bios`
(HOST) INFO  flashing program (7.30 KiB)
(HOST) INFO  success!
────────────────────────────────────────────────────────────────────────────────
 INFO  Neotron BIOS starting...
└─ neotron_pico_bios::__cortex_m_rt_main @ src/main.rs:79
 INFO  Clocks OK
└─ neotron_pico_bios::__cortex_m_rt_main @ src/main.rs:102
 INFO  Pins OK
└─ neotron_pico_bios::__cortex_m_rt_main @ src/main.rs:121
 DEBUG Loop...
└─ neotron_pico_bios::__cortex_m_rt_main @ src/main.rs:128
``` 

## Pinout

The pinout assumed by this BIOS matches that of the Neotron Pico at v0.5.0.

| Pin  | Name | Signal        | Function                                           |
| :--- | :--- | :------------ | :------------------------------------------------- |
| 01   | GP0  | VGA_HSYNC     | VGA Horizontal Sync (31.5 kHz)                     |
| 02   | GP1  | VGA_VSYNC     | VGA Vertical Sync (60 Hz)                          |
| 04   | GP2  | VGA_RED0      | Digital VGA signal, Red channel LSB                |
| 05   | GP3  | VGA_RED1      | Digital VGA signal, Red channel                    |
| 06   | GP4  | VGA_RED2      | Digital VGA signal, Red channel                    |
| 07   | GP5  | VGA_RED3      | Digital VGA signal, Red channel MSB                |
| 09   | GP6  | VGA_GREEN0    | Digital VGA signal, Green channel LSB              |
| 10   | GP7  | VGA_GREEN1    | Digital VGA signal, Green channel                  |
| 11   | GP8  | VGA_GREEN2    | Digital VGA signal, Green channel                  |
| 12   | GP9  | VGA_GREEN3    | Digital VGA signal, Green channel MSB              |
| 14   | GP10 | VGA_BLUE0     | Digital VGA signal, Blue channel LSB               |
| 15   | GP11 | VGA_BLUE1     | Digital VGA signal, Blue channel                   |
| 16   | GP12 | VGA_BLUE2     | Digital VGA signal, Blue channel                   |
| 17   | GP13 | VGA_BLUE3     | Digital VGA signal, Blue channel MSB               |
| 19   | GP14 | I2C_SDA       | I²C Data                                           |
| 20   | GP15 | I2C_SCL       | I²C Clock                                          |
| 21   | GP16 | SPI_CIPO      | SPI Data In                                        |
| 22   | GP17 | SPI_CS_nIOCS  | Low selects MCP23S17, High selects Expansion Slots |
| 24   | GP18 | SPI_CLK       | SPI Clock                                          |
| 25   | GP19 | SPI_COPI      | SPI Data Out                                       |
| 26   | GP20 | nIRQn         | Interrupt Request Input from MCP23S17              |
| 27   | GP21 | nOUTPUT_EN    | Enable buffered CS outputs from MCP23S17           |
| 29   | GP22 | I2S_DAC_DATA  | Digital Audio Output                               |
| 31   | GP26 | I2S_ADC_DATA  | Digital Audio Input                                |
| 32   | GP27 | I2S_LR_CLOCK  | Digital Audio Sync (96kHz)                         |
| 34   | GP28 | I2S_BIT_CLOCK | Digital Audio Bit Clock (1.536MHz)                 |

## Clocks

The RP2040 has a two Phase-Locked-Loop (PLLs) - one for the system clock and
one for the 48 MHz USB clock. Each can convert the incoming clock signal,
multiply it up to a very high frequency signal using a Voltage Controlled
Oscillator (VCO), and then divide it down to the desired value using two
divider circuits. We have control over the system clock PLL, which is used
for the Cortex-M0 core clock rate, as well as the input to the PIO, SPI, I²C
and UART peripherals.

The important clock signal we need is for the PIO, which needs to generate
pixels at a rate of 25.175 MHz ± 0.5%. The PIO will need at least four clock
cycles per pixel, so we need a clock speed of at least 100.7 MHz. Also note
that a value of 25.0 MHz is out of rate (it is -0.7% of nominal, which is
greater than -0.5% allowed). Many systems will just use this frequency
anyway, but we strive for perfection!

The Raspberry Pi Pico has a 12 MHz Crystal. We set the multiplier to x126 to
give a VCO frequency of 1512 MHz. This is very near the VCO maximum of 1600
MHz and we deliberately do this to try and reduce jitter on the divided clock
signal. We then divide by 4 * 3 (= 12) to give a clock frequency of 126.0
MHz. This is 25.2 MHz x 5, where 25.2 MHz is +0.1% from nominal for the video
clock.

We can also divide the system clock clock of 126 MHz by 82, and then by 32
bits per stereo sample, to give an audio sample rate of 48.018 kHz, which
is +0.03% of a nominal 48.0 kHz.

There is a second video frequency of 28.321875 MHz (9/8 of the nominal
frequency), which is used in VGA modes which have 720 pixels across
(80 characters of 9 pixels each) rather than the usual 640 pixel across(or 80
characters of 8 pixels each). Unfortunately this frequency doesn't divide out
from the frequencies above, so there will be no 720 or 360 pixel wide modes.

## Changelog

See [CHANGELOG.md](./CHANGELOG.md)

## Licence

    Neotron-Pico-BIOS Copyright (c) Jonathan 'theJPster' Pallant and the Neotron Developers, 2021

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.

See the full text in [LICENSE.txt](./LICENSE.txt). Broadly, we (the developers)
interpret this to mean (and note that we are not lawyers and this is not
legal advice) that if you give someone a Neotron Pico, you must also give them
one of:

* Complete and corresponding source code (e.g. as a link to your own on-line
  Git repo) for any GPL components (e.g. the BIOS and the OS), as supplied on
  the Neotron Pico.
* A written offer to provide complete and corresponding source code on
  request.

If you are not offering Neotron Pico commercially (i.e. you are not selling
the board for commercial gain), and you are using an unmodified upstream
version of the source code, then the third option is to give them:

* A link to the tag/commit-hash on the relevant official Neotron Github
  repositories - https://github.com/Neotron-Compute/Neotron-Pico-BIOS and
  https://github.com/Neotron-Compute/Neotron-OS.

This is to ensure everyone always has the freedom to access the source code in
their Neotron Pico.

## Contribution

Unless you explicitly state otherwise, any contribution intentionally
submitted for inclusion in the work by you shall be licensed as above,
without any additional terms or conditions.

