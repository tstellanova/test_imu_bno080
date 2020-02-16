MEMORY
{
  /* FLASH and RAM are mandatory memory regions */
  FLASH  : ORIGIN = 0x08000000, LENGTH = 64K
  /* .bss, .data and the heap go in this region */
  RAM    : ORIGIN = 0x20000000, LENGTH = 12K
  /* Core coupled (faster) RAM dedicated to hold the stack */
  CCRAM : ORIGIN = 0x10000000, LENGTH = 4K
}

/* The location of the stack can be overridden using the
   `_stack_start` symbol.  Place the stack at the end of RAM */
_stack_start = ORIGIN(RAM) + LENGTH(RAM);

/* The location of the .text section can be overridden using the
   `_stext` symbol.  By default it will place after .vector_table */
/* _stext = ORIGIN(FLASH) + 0x40c; */


