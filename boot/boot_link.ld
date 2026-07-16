/*
 * Bootloader linker script for the motor MCU (STM32G474RETX).
 *
 * The CPU always boots at 0x08000000 — this region. The bootloader reads
 * the persistent boot flag (page 16, 0x08008000); if CLEAR it jumps to the
 * app at 0x08008800, if STAY it serves a firmware update over the inter-MCU
 * SPI2 slave link (segmented SDO).
 *
 * Flash map (512 KB, dual-bank, 2 KB pages — see boot/README.md + ADR-063*):
 *   0x08000000..0x08007FFF  (32 K)   bootloader        — this region (bank1 pg0..15)
 *   0x08008000..0x080087FF  (2 K)    boot-flag page    — STAY/CLEAR blob (bank1 pg16)
 *   0x08008800..0x0807EFFF  (~474 K) app               — STM32G474RETX_FLASH.ld (bank1 pg17..127 + bank2 pg0..125)
 *   0x0807F000..0x0807FFFF  (4 K)    config persist A/B — untouched by updates (bank2 pg126/127)
 *
 * Shares all 128 KB SRAM with the app — the two never run at once.
 */

ENTRY(Reset_Handler)

_estack = ORIGIN(RAM) + LENGTH(RAM);

_Min_Heap_Size  = 0x0;      /* bootloader has no heap use */
_Min_Stack_Size = 0x400;

MEMORY
{
  RAM   (xrw) : ORIGIN = 0x20000000, LENGTH = 128K
  FLASH (rx)  : ORIGIN = 0x8000000,  LENGTH = 32K
}

SECTIONS
{
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector))
    . = ALIGN(4);
  } >FLASH

  .text :
  {
    . = ALIGN(4);
    *(.text)
    *(.text*)
    *(.glue_7)
    *(.glue_7t)
    *(.eh_frame)
    KEEP (*(.init))
    KEEP (*(.fini))
    . = ALIGN(4);
    _etext = .;
  } >FLASH

  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)
    *(.rodata*)
    . = ALIGN(4);
  } >FLASH

  .ARM.extab (READONLY) : { . = ALIGN(4); *(.ARM.extab* .gnu.linkonce.armextab.*) . = ALIGN(4); } >FLASH
  .ARM (READONLY)       : { . = ALIGN(4); __exidx_start = .; *(.ARM.exidx*) __exidx_end = .; . = ALIGN(4); } >FLASH

  .preinit_array (READONLY) : { . = ALIGN(4); PROVIDE_HIDDEN (__preinit_array_start = .); KEEP (*(.preinit_array*)) PROVIDE_HIDDEN (__preinit_array_end = .); . = ALIGN(4); } >FLASH
  .init_array (READONLY)    : { . = ALIGN(4); PROVIDE_HIDDEN (__init_array_start = .);    KEEP (*(SORT(.init_array.*))) KEEP (*(.init_array*)) PROVIDE_HIDDEN (__init_array_end = .); . = ALIGN(4); } >FLASH
  .fini_array (READONLY)    : { . = ALIGN(4); PROVIDE_HIDDEN (__fini_array_start = .);    KEEP (*(SORT(.fini_array.*))) KEEP (*(.fini_array*)) PROVIDE_HIDDEN (__fini_array_end = .); . = ALIGN(4); } >FLASH

  _sidata = LOADADDR(.data);
  .data :
  {
    . = ALIGN(4);
    _sdata = .;
    *(.data)
    *(.data*)
    *(.RamFunc)
    *(.RamFunc*)
    . = ALIGN(4);
    _edata = .;
  } >RAM AT> FLASH

  . = ALIGN(4);
  .bss :
  {
    _sbss = .;
    __bss_start__ = _sbss;
    *(.bss)
    *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;
    __bss_end__ = _ebss;
  } >RAM

  ._user_heap_stack :
  {
    . = ALIGN(8);
    PROVIDE ( end = . );
    PROVIDE ( _end = . );
    . = . + _Min_Heap_Size;
    . = . + _Min_Stack_Size;
    . = ALIGN(8);
  } >RAM

  /DISCARD/ :
  {
    libc.a ( * )
    libm.a ( * )
    libgcc.a ( * )
  }

  .ARM.attributes 0 : { *(.ARM.attributes) }
}
