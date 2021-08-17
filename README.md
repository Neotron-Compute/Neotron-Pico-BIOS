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

