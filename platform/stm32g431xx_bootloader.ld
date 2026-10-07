/* Bootloader Linker Script for STM32G431xx */
/* Bootloader: 0x08000000 - 0x08003FFF (16KB) */

ENTRY(Reset_Handler)

_estack = ORIGIN(RAM) + LENGTH(RAM);
_Min_Heap_Size = 0;
_Min_Stack_Size = 0x200;

MEMORY
{
  RAM (xrw)      : ORIGIN = 0x20000000, LENGTH = 0x57F0
  BOOTFLAG (rw)  : ORIGIN = 0x200057F0, LENGTH = 0x10
  CCMRAM (xrw)   : ORIGIN = 0x10000000, LENGTH = 10K
  FLASH (rx)     : ORIGIN = 0x08000000, LENGTH = 16K
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

  .ARM.extab   : { *(.ARM.extab* .gnu.linkonce.armextab.*) } >FLASH
  .ARM : {
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
  } >FLASH

  .preinit_array (READONLY) :
  {
    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP (*(.preinit_array*))
    PROVIDE_HIDDEN (__preinit_array_end = .);
  } >FLASH

  .init_array (READONLY) :
  {
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array*))
    PROVIDE_HIDDEN (__init_array_end = .);
  } >FLASH

  .fini_array (READONLY) :
  {
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP (*(SORT(.fini_array.*)))
    KEEP (*(.fini_array*))
    PROVIDE_HIDDEN (__fini_array_end = .);
  } >FLASH

  _sidata = LOADADDR(.data);

  .data :
  {
    . = ALIGN(4);
    _sdata = .;
    *(.data)
    *(.data*)
    . = ALIGN(4);
    _edata = .;
  } >RAM AT> FLASH

  /* Boot flag 区域 (不初始化，复位保持) */
  .noinit (NOLOAD) :
  {
    . = ALIGN(4);
    *(.noinit)
    *(.noinit*)
    . = ALIGN(4);
  } >RAM

  .ccm_bss (NOLOAD) :
  {
    . = ALIGN(8);
    _sccm_bss = .;
    *(.ccm_bss)
    *(.ccm_bss*)
    . = ALIGN(8);
    _eccm_bss = .;
  } >CCMRAM

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

  .boot_flag (NOLOAD) :
  {
    . = ALIGN(4);
    KEEP(*(.boot_flag))
    . = ALIGN(4);
  } >BOOTFLAG

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

  ASSERT(SIZEOF(.boot_flag) == 16, "boot flag must occupy exactly 16 bytes")
  ASSERT(ADDR(.boot_flag) == ORIGIN(BOOTFLAG), "boot flag address mismatch")
  ASSERT(SIZEOF(.ccm_bss) <= LENGTH(CCMRAM), "CCM RAM overflow")
}
