/* Label for the program's entry point */
ENTRY(Reset_Handler)

/* End of RAM/Start of stack */
/* (4KB SRAM) */
_estack = 0x2001ffff;
/*_estack = ORIGIN(RAM) + LENGTH(RAM);end of RAM */

/* Set minimum size for stack and dynamic memory. */
/* (The linker will generate an error if there is
 * less than this much RAM leftover.) */
/* (1KB) */
_Min_Leftover_RAM = 0x400;

MEMORY
{
/*FLASH*/flash ( rx )   : ORIGIN = 0x08000000, LENGTH = 1024K
/*RAM*/  ram   ( rxw )  : ORIGIN = 0x20000000, LENGTH = 128K
}
SECTIONS
{
  /* The vector table goes to the start of flash. 
  .vector_table :
  {
    . = ALIGN(4);
    KEEP (*(.vector_table))
    . = ALIGN(4);
  } >FLASH
*/
.vectors  : { KEEP(*(.isr_vector)) } > flash

  /* The 'text' section contains the main program code.
  .text :
  {
    . = ALIGN(4);
    *(.text)
    *(.text*)
    . = ALIGN(4);
  } >FLASH */
.text     : { *(.text*) }           > flash

/* The 'rodata' section contains read-only data,
 constants, strings, information that won't change. 
  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)
    *(.rodata*)
    . = ALIGN(4);
  } >FLASH */
.rodata   : { *(.rodata*) }         > flash

  /* The 'data' section is space set aside in RAM for
   * things like variables, which can change. */
  _sidata = .;
  .data : AT(_sidata)
  {
    . = ALIGN(4);
    /* Mark start/end locations for the 'data' section. */
    _sdata = .;
    *(.data)
    *(.data*)
    _edata = .;
    . = ALIGN(4);
  } > ram

  /* The 'bss' section is similar to the 'data' section,
   * but its space is initialized to all 0s at the
   * start of the program. */
  .bss :
  {
    . = ALIGN(4);
    /* Also mark the start/end of the BSS section. */
    _sbss = .;
    *(.bss)
    *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;
  } > ram

  /* Space set aside for the application's heap/stack. */
  .dynamic_allocations :
  {
    . = ALIGN(4);
    _ssystem_ram = .;
    . = . + _Min_Leftover_RAM;
    . = ALIGN(4);
    _esystem_ram = .;
  } > ram
}
