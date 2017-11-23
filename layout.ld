MEMORY
{
    FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 256K
    RAM  (rwx) : ORIGIN = 0x1FFF8000, LENGTH = 64K
}

PROVIDE(_stack_start = ORIGIN(RAM) + LENGTH(RAM));

SECTIONS
{
    .text : {
        . = 0;
        LONG(_stack_start);
        KEEP(*(.vectors.reset));
        KEEP(*(.vectors.exceptions))
        KEEP(*(.vectors.interrupts))
        . = 0x400;
        KEEP(*(.flashconfig))
        . = ALIGN(4);
        *(.text*)
    } > FLASH = 0xFF

    .rodata : {
        *(.rodata*)
    } > FLASH

    .bss : {
         . = ORIGIN(RAM);
         _bss_start = .;
         *(.bss*)
         _bss_end = .;
    } > RAM

    /DISCARD/ : {
        *(.ARM.*)
    }
}