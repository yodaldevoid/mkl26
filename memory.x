MEMORY
{
    FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 64K
    RAM  (rwx) : ORIGIN = 0x1FFFF800, LENGTH = 8K
}

/* This is where the call stack will be allocated. */
/* The stack is of the full descending type. */
/* You may want to use this variable to locate the call stack and static
   variables in different memory regions. Below is shown the default value */
/* _stack_start = ORIGIN(RAM) + LENGTH(RAM); */

/* Example of putting non-initialized variables into custom RAM locations. */
/* This assumes you have defined a region RAM2 above, and in the Rust
   sources added the attribute `#[link_section = ".ram2bss"]` to the data
   you want to place there. */
/* Note that the section will not be zero-initialized by the runtime! */
/*  SECTIONS {
        .ram2bss (NOLOAD) : ALIGN(4) {
            *(.ram2bss);
            . = ALIGN(4);
        } > RAM2
    } INSERT AFTER .bss;
*/

EXTERN(_FLASHCONFIG);

SECTIONS
{
    .flashconfig ORIGIN(FLASH) + 0x400 :
    {
        KEEP(*(.flashconfig))
        _stext = .;
    } > FLASH
}
