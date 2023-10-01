/**
 * Linker script for the Raspberry Pi Pico.
 * 
 * Part of the Neotron Pico BIOS.
 * 
 * This file licenced as CC0.
 */

MEMORY {
    /*
     * This is bootloader for the RP2040. It must live at the start of the
       external flash chip.
     */
    BOOT2 : ORIGIN = 0x10000000, LENGTH = 0x100
    /*
     * The Pico has 2048 KiB of external Flash Memory. We allow ourselves 128
     * KiB for the BIOS, leaving the rest
     * for the OS and any user applications.
     */
    FLASH : ORIGIN = 0x10000100, LENGTH = 128K - 0x100
    /*
     * This is the remainder of the 2048 KiB flash chip.
     */
    FLASH_OS : ORIGIN = 0x10020000, LENGTH = 2048K - 128K
    /*
     * This is the bottom of the four striped banks of SRAM in the RP2040.
     */
    RAM_OS : ORIGIN = 0x20000000, LENGTH = 0x42000 - 0x9080
    /*
     * This is the top of the four striped banks of SRAM in the RP2040, plus
     * SRAM_BANK4 and SRAM_BANK5.
     *
     * This is carefully calculated to give us 8 KiB of stack space and ensure
     * the defmt buffer doesn't span across SRAM_BANK3 and SRAM_BANK4.
     *
     * 0x9080 should be the (size of .data + size of .bss + size of .uninit +
     * 0x2000 for the stack).
     */
    RAM : ORIGIN = 0x20042000 - 0x9080, LENGTH = 0x9080
}

/*
 * Export some symbols to tell the BIOS where it might find the OS.
 */
_flash_os_start = ORIGIN(FLASH_OS);
_flash_os_len = LENGTH(FLASH_OS);
_ram_os_start = ORIGIN(RAM_OS);
_ram_os_len = LENGTH(RAM_OS);

SECTIONS {
    /* ### RP2040 Boot loader */
    .boot2 ORIGIN(BOOT2) :
    {
        KEEP(*(.boot2));
    } > BOOT2

    /* ### Neotron OS */
    .flash_os ORIGIN(FLASH_OS) :
    {
        KEEP(*(.flash_os));
    } > FLASH_OS
} INSERT BEFORE .text;


