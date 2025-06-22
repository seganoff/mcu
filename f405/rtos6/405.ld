ENTRY(Reset_Handler);
MEMORY 
{ 
 /*FLASH*/flash ( rx )   : ORIGIN = 0x08000000, LENGTH = 1024K
 /*RAM*/  sram  ( rxw )  : ORIGIN = 0x20000000, LENGTH = 128K
} 
_estack     = ORIGIN(sram) + LENGTH(sram);    /* stack points to end of SRAM */

SECTIONS {
  .vectors  : { KEEP(*(.isr_vector)) }  > flash
  .text     : { *(.text* .text.*) }     > flash
  .rodata   : { *(.rodata*) }           > flash

  .data : {
    _sdata = .;
    *(.first_data)
    *(.data SORT(.data.*))
    *(.iram .iram* .iram.*)
    _edata = .;
  } > sram AT > flash
  _sidata = LOADADDR(.data);

  .bss : { _sbss = .; *(.bss SORT(.bss.*) COMMON) _ebss = .; } > sram

  . = ALIGN(8);
  _end = .;
}
