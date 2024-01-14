MEMORY
{
    FLASH (rx) : org = 0x08000000, len = 128K
    RAM (rwx) : org = 0x20000000, len = 20K
}
__reset_stack_pointer = ORIGIN(RAM) + LENGTH(RAM);

SECTIONS
{
    .text : {
        /* Set the initial address to 0 */
        . = 0;
        LONG(__reset_stack_pointer);
        LONG(__My_Start | 1);
        LONG(NMIHandler | 1);
        LONG(HardFaultHandler | 1);
        LONG(MemoryManagementFaultHandler | 1);
        LONG(BusFaultHandler | 1);
        LONG(UsageFaultHandler | 1);
        /* The whole interrupt table is 332 bytes long. Advance to that position. */
        . = 332;
        /* And here comes the rest of the code */
        *(.text*);
    } > FLASH /* Put this in the flash memory region */
    .bss : {
    __bss_start__ = .;
     *(.dynbss)
     *(.bss .bss.* .gnu.linkonce.b.*)
     *(COMMON)
     /* Align here to ensure that the .bss section occupies space up to
        _end.  Align after .bss to ensure correct alignment even if the
        .bss section disappears because there are no input sections.
        FIXME: Why do we need it? When there is no .bss section, we do not
        pad the .data section.  */
     . = ALIGN(. != 0 ? 32 / 8 : 1);
     __bss_end__ = .;
    }
    __end__ = .;
    _end = .; PROVIDE (end = .);
    __exidx_start = .;
    __exidx_end = .;
}