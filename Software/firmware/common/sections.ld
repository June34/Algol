OUTPUT_ARCH( "riscv" )

SECTIONS
{
    /* text: test code section*/
  . = 0x100;
  .text :
  {
    crt.o(.text)
    *(.text)
  }

  /* data segment */
  .data : { *(.data) }

  .sdata :
  {
    _gp = . + 0x800;
    *(.*sdata*)
  }

  /* bss segment */
  .sbss :
  {
    *(.sbss .sbss.*)
    *(.scommon)
  }
  .bss : { *(.bss) }

  /* End of uninitalized data segement */
  _end = .;
}
