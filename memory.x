/**
 * Linker script for the Raspberry Pi Pico.
 * 
 * Part of the Neotron Pico BIOS.
 * 
 * This file licenced as CC0.
 */

MEMORY {
    /*
     * The Pico 2 has 4096 KiB of external Flash Memory. We allow ourselves 128
     * KiB for the BIOS, leaving the rest for the OS and any user applications.
     */
    FLASH : ORIGIN = 0x10000000, LENGTH = 128K
    /*
     * This is the remainder of the 4096 KiB flash chip.
     */
    FLASH_OS : ORIGIN = 0x10020000, LENGTH = 4096K - 128K
    /*
     * This is the bottom of the four striped banks of SRAM in the RP2040.
     */
    RAM_OS : ORIGIN = 0x20000000, LENGTH = 0x82000 - 0x9690
    /*
     * This is the top of the four striped banks of SRAM in the RP2040, plus
     * SRAM_BANK4 and SRAM_BANK5.
     *
     * This is carefully calculated to give us 8 KiB of stack space and ensure
     * the defmt buffer doesn't span across SRAM_BANK3 and SRAM_BANK4.
     *
     * 0x9690 should be the (size of .data + size of .bss + size of .uninit +
     * 0x2000 for the stack).
     */
    RAM : ORIGIN = 0x20082000 - 0x9690, LENGTH = 0x9690
}

/*
 * Export some symbols to tell the BIOS where it might find the OS.
 */
_flash_os_start = ORIGIN(FLASH_OS);
_flash_os_len = LENGTH(FLASH_OS);
_ram_os_start = ORIGIN(RAM_OS);
_ram_os_len = LENGTH(RAM_OS);

SECTIONS {
    /* ### Boot ROM info
     *
     * Goes after .vector_table, to keep it in the first 4K of flash
     * where the Boot ROM (and picotool) can find it
     */
    .start_block : ALIGN(4)
    {
        __start_block_addr = .;
        KEEP(*(.start_block));
    } > FLASH

} INSERT AFTER .vector_table;

/* move .text to start /after/ the boot info */
_stext = ADDR(.start_block) + SIZEOF(.start_block);

SECTIONS {
    /* ### Picotool 'Binary Info' Entries
     *
     * Picotool looks through this block (as we have pointers to it in our
     * header) to find interesting information.
     */
    .bi_entries : ALIGN(4)
    {
        /* We put this in the header */
        __bi_entries_start = .;
        /* Here are the entries */
        KEEP(*(.bi_entries));
        /* Keep this block a nice round size */
        . = ALIGN(4);
        /* We put this in the header */
        __bi_entries_end = .;
    } > FLASH
} INSERT AFTER .text;

SECTIONS {
    /* ### Boot ROM extra info
     *
     * Goes after everything in our program, so it can contain a signature.
     */
    .end_block : ALIGN(4)
    {
        __end_block_addr = .;
        KEEP(*(.end_block));
    } > FLASH

} INSERT AFTER .uninit;

PROVIDE(start_to_end = __end_block_addr - __start_block_addr);
PROVIDE(end_to_start = __start_block_addr - __end_block_addr);


