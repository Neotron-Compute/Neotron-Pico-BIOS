# Changelog

## Unreleased Changes ([Source](https://github.com/neotron-compute/neotron-pico-bios/tree/develop) | [Changes](https://github.com/neotron-compute/neotron-pico-bios/compare/v0.5.1...develop))

* Updated to neotron-common-bios 0.10.0, and changed the VGA palette to match
* Use published neotron-bmc-protocol and neotron-bmc-commands crates
* Clarify how BMC speaker works
* Add ANSI art boot-up logo
* Speed up boot, and add "ESC to Pause"
* Print message on hardfault
* Update to OS 0.4.0
* Re-arrange BIOS memory and stack for Core 1
* Re-wrote the video render system to be more robust
* Added a cursor
* Added ANSI decoding support (for basic colours)

## v0.5.1 ([Source](https://github.com/neotron-compute/neotron-pico-bios/tree/v0.5.1) | [Release](https://github.com/neotron-compute/neotron-pico-bios/release/tag/v0.5.1))

* Implement SD Card block read/write
* SPI bus clock changes depending on which device is being accessed
* Updated to OS 0.3.3
* Running `git` in `build.rs` is now optional

## v0.5.0 ([Source](https://github.com/neotron-compute/neotron-pico-bios/tree/v0.5.0) | [Release](https://github.com/neotron-compute/neotron-pico-bios/release/tag/v0.5.0))

* Full-colour 80 column text
* Bump to 151.2 MHz clock speed (needed to get 80 column colour text working)
* Reports video timing on Pi Pico LED
* Shiny new colour start-up banner
* Plays start-up tune
* Updated keyboard driver
* Added RTC support
* Implement new kind of Mutex
* Updated to OS 0.3.2
* Supports initialising SD Cards and reporting card size

## v0.4.1 ([Source](https://github.com/neotron-compute/neotron-pico-bios/tree/v0.4.1) | [Release](https://github.com/neotron-compute/neotron-pico-bios/release/tag/v0.4.1))

* Wait for interrupts from the BMC before reading PS/2 key codes
  * Requires Neotron Pico BMC v0.5.0
* Debug LEDs change every time you get an interrupt from the IO controller
* Doubled the speed of the RP2040's QSPI flash interface
* Documentation updates to make programming your RP2040 easier

## v0.4.0 ([Source](https://github.com/neotron-compute/neotron-pico-bios/tree/v0.4.0) | [Release](https://github.com/neotron-compute/neotron-pico-bios/release/tag/v0.4.0))

* Updated dependencies
* Basic keyboard support by talking to BMC
* Revised how version string is generated
* Includes Neotron OS 0.2.0

## v0.3.0 ([Source](https://github.com/neotron-compute/neotron-pico-bios/tree/v0.3.0) | [Release](https://github.com/neotron-compute/neotron-pico-bios/release/tag/v0.3.0))

* Boots OS 0.1.0
* VGA 80x30 text mode (640x480 @ 60Hz)
* Update defmt and other crates

## v0.2.0 ([Source](https://github.com/neotron-compute/neotron-pico-bios/tree/v0.2.0) | [Release](https://github.com/neotron-compute/neotron-pico-bios/release/tag/v0.2.0))

* Add RTT debugging using defmt
* Report git version over defmt
* Describe how to flash/debug with a second Raspberry Pi Pico

## v0.1.0 ([Source](https://github.com/neotron-compute/neotron-pico-bios/tree/v0.1.0) | [Release](https://github.com/neotron-compute/neotron-pico-bios/release/tag/v0.1.0))

* First release - it compiles, and blinks an LED.
