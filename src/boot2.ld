ENTRY(boot2_entry)
SECTIONS
{
  . = 0;
  .text : { *(.text*) }
  /DISCARD/ : { *(.comment*) *(.note*) }
}
