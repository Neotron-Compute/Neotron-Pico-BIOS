# Neotron Pico BIOS

This is the [Neotron](https://github.com/neotron-compute) BIOS for the [Neotron
Pico] board.

[Neotron Pico]: (https://github.com/neotron-compute/neotron-pico)

![Build Status](https://github.com/neotron-compute/neotron-pico-bios/actions/workflows/build.yml/badge.svg?branch=develop)

![Format Status](https://github.com/neotron-compute/neotron-pico-bios/actions/workflows/format.yml/badge.svg?branch=develop)

## Compatibility

This BIOS will run on the [Neotron Pico] v1.0.0 and v1.1.0. Other boards based
around the Raspberry Pi Pico could be supported with just a minor change to the
pin configurations.

## Hardware Features

The Neotron Pico PCB offers:

* Dual Cortex-M0+ clocked at ~~133 MHz~~ 151.2 MHz
* 256 KiB RAM
* 2048 KiB Flash
* SD/MMC Slot
* 640x480 VGA output
* Stereo 16-bit 48 kHz audio
* Four Neotron expansion slots
* A dedicated Board Management Controller, offering:
  * Dual PS/2 ports
  * 5-wire 3.3V TTL UART
  * Power/reset control

## Software Features

Currently the BIOS uses Core 0 for running the Neotron OS (and application code), and Core 1 for updating the video display and performing 80-column colour text rendering in real-time.

* [x] 640x480 @ 60 Hz text mode
  * [x] 80 columns, 60 rows, 8x8 font
  * [x] 80 columns, 30 rows, 8x16 font
  * [x] 16 foreground and 8 background colours
  * [x] Fixed font
  * [ ] Loadable soft-font
* [x] 640x400 @ 70 Hz text mode
  * [x] 80 columns, 50 rows, 8x8 font
  * [x] 80 columns, 25 rows, 8x16 font
  * [x] 16 foreground and 8 background colours
  * [x] Fixed font
  * [ ] Loadable soft-font
* [ ] 640x480 @ 60 Hz graphics mode
  * [x] 1-bpp mode (2 colours) in 38 KiB
  * [ ] 2-bpp mode (4 colours) in 75 KiB
  * [ ] 4-bpp mode (4 colours) in 150 KiB
* [ ] 320x240 @ 60 Hz graphics mode
  * [ ] 1-bpp mode (2 colours) in 10 KiB
  * [ ] 2-bpp mode (4 colours) in 19 KiB
  * [ ] 4-bpp mode (4 colours) in 38 KiB
  * [ ] 8-bpp mode (8 colours) in 75 KiB
* [x] SPI Bus
* [x] I2C Bus
* [x] SPI I/O Expander for Chip Selects and LEDs
* [x] External Interrupts
* [x] SD Card support
  * [x] Read/write 512 byte blocks
  * [x] Insert/remove detection
* [x] Communications with the BMC
  * [x] Reading from a PS/2 keyboard
  * [x] Sending beeps/boops to the PC Speaker
  * [ ] Writing to the PS/2 keyboard
  * [ ] Reading/writing to/from the PS/2 mouse
  * [ ] UART support
  * [ ] Second I2C Bus support
  * [ ] Voltage monitoring
  * [ ] Soft-power
  * [x] Soft-reset
* [x] Dallas or Microchip RTC support
* [x] Audio CODEC mixer programming
* [x] I2S Audio Output
* [ ] I2S Audio Input

## Programming

The Neotron BIOS uses the [defmt](https://crates.io/crates/defmt) crate to provide structured logging over the SWD interface. The easiest way to flash and debug your Neotron Pico BIOS is with a second Raspberry Pi Pico, or the official Raspberry Pi Debug Probe.

1. If your BMC has not been programmed, [do that first](https://github.com/Neotron-Compute/Neotron-BMC/blob/v0.5.2/neotron-bmc-pico/README.md).

1. Connect your *Debug Probe* to the three *DEBUG* pins on the *Raspberry Pi Pico* fitted to your Neotron Pico, following the instructions for your particular debugging device or firmware.

1. On your PC, install the [*probe-rs*](https://github.com/probe-rs), a microcontroller flashing tool. See website for instructions.

1. Power on your Neotron Pico into bootloader mode by applying 12V, holding down
   the "BOOTSEL" button on the Raspberry Pi Pico and then and tapping the On/Off
   button on the Neotron.

   If you don't put the Raspberry Pi Pico into bootloader mode then when
   `probe-rs` resets the chip after programming, the firmware will detect it has
   had an incomplete reset and cause a full reset. This makes the video output
   more reliable, but the full reset will immediately disconnect `probe-rs` so
   you won't see any log messages. Booting the RP2040 in USB bootloader mode
   avoids this issue by making the `probe-rs` triggered reset look more like a
   full reset.

1. Flash the BIOS.

    1. You can compile from source and flash all-in-one:

        ```bash
        cargo run --release
        ```

    1. You can download the `neotron-pico-bios.elf` file from [Github
    Releases](https://github.com/Neotron-Compute/Neotron-Pico-BIOS/releases) and
    flash with `probe-rs`:

        ```bash
        probe-rs run --chip RP2040 ~/Downloads/neotron-pico-bios
        ```

    1. You can download the `neotron-pico-bios.uf2` file from [Github
    Releases](https://github.com/Neotron-Compute/Neotron-Pico-BIOS/releases) and
    flash by copying to the RP2040 bootloader's fake USB Mass Storage Device. You will need to connect a USB cable to the Raspberry Pi Pico's external USB micro-AB port, and ensure that the USB Host Power jumper near the 12V input is NOT fitted.

1. Now when you reset your board, you will see the BIOS splash screen on any
   connected VGA display and some start-up tones on any connected PC Speaker. No
   OS will be booted, as we haven't flashed the OS yet. Grab a compatible OS
   release from <https://github.com/Neotron-Compute/Neotron-OS/releases>. You
   need the `thumbv6m-none-eabi-flash1002` variant for the Neotron Pico, because
   we have an ARMv6-M CPU that executes Thumb-2 instructions, and the BIOS
   expects the OS to live in flash at address `0x1002_0000`.

## Multiple Probes

If you have multiple `probe-rs` compatible probes attached to your computer,
you will receive an error message.

You will need to check what probes are available and edit `.cargo/config.toml` to add the appropriate `--probe VID:PID` argument.

```console
$ probe-rs list
the following probes were found:
[0]: Picoprobe CMSIS-DAP (VID: 2e8a, PID: 000c, Serial: Exxxxxxxxxxxxxx6, CmsisDap)
[1]: STLink V2 (VID: 0483, PID: 3748, Serial: 0xxxxxxxxxxxxxxxxxxxxxxE, StLink)
user@host ~/neotron-pico-bios $ $EDITOR .cargo/config.toml
user@host ~/neotron-pico-bios $ cat .cargo/config.toml | grep runner
# runner = "elf2uf2-rs -d"
# runner = "probe-rs run --chip RP2040"
runner = "probe-rs run --chip RP2040 --probe 2e8a:000c"
```

You can also just provide the probe Serial number, for example if you have multiple
identical probes.

## Changelog

See [CHANGELOG.md](./CHANGELOG.md)

## Licence

```text
Neotron-Pico-BIOS Copyright (c) Jonathan 'theJPster' Pallant and the Neotron Developers, 2024

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
```

See the full text in [LICENSE.txt](./LICENSE.txt). Broadly, we (the developers)
interpret this to mean (and note that we are not lawyers and this is not
legal advice) that if you give someone a Neotron Pico, you must also give them
one of:

* Complete and corresponding source code (e.g. as a link to your **own** on-line
  Git repo) for any GPL components (e.g. the BIOS and the OS), as supplied on
  the Neotron Pico.
* A written offer to provide complete and corresponding source code on
  request.

If you are not offering Neotron Pico commercially (i.e. you are not selling
the board for commercial gain), and you are using an unmodified upstream
version of the source code, then the third option is to give them:

* A link to the tag/commit-hash on the relevant official Neotron Github
  repositories - <https://github.com/Neotron-Compute/Neotron-Pico-BIOS> and
  <https://github.com/Neotron-Compute/Neotron-OS>.

This is to ensure everyone always has the freedom to access the source code in
their Neotron Pico.

## Contribution

Unless you explicitly state otherwise, any contribution intentionally
submitted for inclusion in the work by you shall be licensed as above,
without any additional terms or conditions.
