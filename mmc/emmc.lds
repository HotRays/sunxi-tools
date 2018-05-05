SECTIONS
{
. = 0x0000;
.start    : { *(.start) }
.text     : { *(.text) }
/DISCARD/ : { *(.dynstr*) }
/DISCARD/ : { *(.dynamic*) }
/DISCARD/ : { *(.plt*) }
/DISCARD/ : { *(.interp*) }
/DISCARD/ : { *(.gnu*) }
/DISCARD/ : { *(.note*) }
}

