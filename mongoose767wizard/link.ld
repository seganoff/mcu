ENTRY(Reset_Handler);
MEMORY {
  flash(rx) : ORIGIN = 0x08000000, LENGTH = 2048k
      /*sram1 368kb+sram2 16kb*/
  /*sram(rwx) : ORIGIN = 0x20020000, LENGTH = 384k*/
    sram(rwx) : ORIGIN = 0x20000000, LENGTH = 512k
/*dtcm(rwx) : ORIGIN=  0x20000000, LENGTH = 128k*/
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
