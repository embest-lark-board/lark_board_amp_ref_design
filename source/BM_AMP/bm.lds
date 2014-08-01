OUTPUT_ARCH(arm)
ENTRY(start)

SECTIONS
{
	. = 0x1E000000;
	.text	:
	{
	        AMP_BM_REF_DESIGN/start.o (.text)
		*(.text)
	}
	.rodata : { *(.rodata) }
	.data :   { *(.data) }
	.bss : { 
		bss_start = .;
		*(.bss) 
		. = ALIGN(4);
		bss_end = .;
	}
}
