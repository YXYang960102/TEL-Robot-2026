ENTRY(Reset_Handler)
MEMORY {
  FLASH (rx) : ORIGIN = 0x08004000, LENGTH = 240K
  RAM (rwx) : ORIGIN = 0x20000000, LENGTH = 64K
}
_stack_top = ORIGIN(RAM) + LENGTH(RAM);
SECTIONS {
  .isr_vector : { KEEP(*(.isr_vector)) } > FLASH
  .text : { *(.text*) *(.rodata*) . = ALIGN(4); } > FLASH
  .ARM.exidx : { *(.ARM.exidx*) } > FLASH
  .data : {
    . = ALIGN(4); _sdata = .; *(.data*) . = ALIGN(4); _edata = .;
  } > RAM AT> FLASH
  _sidata = LOADADDR(.data);
  .bss (NOLOAD) : {
    . = ALIGN(4); _sbss = .; *(.bss*) *(COMMON) . = ALIGN(4); _ebss = .;
  } > RAM
  /DISCARD/ : { *(.comment) *(.note*) }
  ASSERT(ADDR(.isr_vector) == 0x08004000, "Wrong application base")
  ASSERT(_ebss <= _stack_top - 4096, "Insufficient stack reserve")
}
