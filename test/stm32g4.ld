/* STM32G474RE: 512KB FLASH, 128KB RAM */
MEMORY
{
  FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 512K
  RAM   (rwx) : ORIGIN = 0x20000000, LENGTH = 128K
}

_estack = ORIGIN(RAM) + LENGTH(RAM);

ENTRY(Reset_Handler)

SECTIONS
{
  .isr_vector : {
    KEEP(*(.isr_vector))
  } > FLASH

  .text : {
    *(.text*) *(.rodata*) *(.glue_7) *(.glue_7t) *(.eh_frame*)
  } > FLASH

  _sidata = LOADADDR(.data);
  .data : AT(ADDR(.text) + SIZEOF(.text)) {
    _sdata = .;
    *(.data*)
    _edata = .;
  } > RAM

  .bss : {
    _sbss = .;
    *(.bss*) *(COMMON)
    _ebss = .;
  } > RAM

  /DISCARD/ : { *(.comment) *(.note*) }
}

