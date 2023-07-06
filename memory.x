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
    RAM_OS : ORIGIN = 0x20000000, LENGTH = 0x39000
    /*
     * This is the top of the four striped banks of SRAM in the RP2040.
     *
     * We give ourselves size 4K pages [0x39_000..0x3E_FFF]
     */
    RAM : ORIGIN = 0x20039000, LENGTH = 24K
    /*
     * This 4K from the top of striped RAM, plus the fifth bank - another a 4KB
     * block. We use this for Core 0 Stack. We tried 4K but it wasn't enough.
     */
    RAM_CORE0_STACK : ORIGIN = 0x2003F000, LENGTH = 8K
    /*
     * This is the sixth bank, a 4KB block. We use this for Core 1 Stack.
     * As of 0.5.1 Pico BIOS uses about 316 bytes of this but we give it the
     * full 4K so it can have uncontended access to this SRAM bank.
     */
    RAM_CORE1_STACK : ORIGIN = 0x20041000, LENGTH = 4K
}

/*
 * This is where the call stack for Core 0 will be located. The stack is of
 * the full descending type.
 */
_stack_start = ORIGIN(RAM_CORE0_STACK) + LENGTH(RAM_CORE0_STACK);
_stack_bottom = ORIGIN(RAM_CORE0_STACK);
_stack_len = LENGTH(RAM_CORE0_STACK);

/*
 * This is where the call stack for Core 1 will be located.
 */
_core1_stack_bottom = ORIGIN(RAM_CORE1_STACK);
_core1_stack_len = LENGTH(RAM_CORE1_STACK);

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


