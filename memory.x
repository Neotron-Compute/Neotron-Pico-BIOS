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
     *
     * We can think of RAM as 66 (0x42) blocks of 4 KiB. The OS gets the first
     * 56 (0x38) of them. Of those it'll keep one for itself and give the other
     * 55 (0x37) over as the Transient Program Area (TPA).
     */
    RAM_OS : ORIGIN = 0x20000000, LENGTH = 0x38000
    /*
     * This is the top of the four striped banks of SRAM in the RP2040, plus
     * SRAM_BANK4 and SRAM_BANK5.
     *
     * We give ourselves ten 4K pages [0x38_000..0x41_FFF]. This includes our
     * stack space.
     */
    RAM : ORIGIN = 0x20038000, LENGTH = 40K
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


