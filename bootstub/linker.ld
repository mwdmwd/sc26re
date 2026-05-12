/* SPDX-License-Identifier: AGPL-3.0-or-later */
MEMORY
{
	FLASH (rx)  : ORIGIN = 0x00000000, LENGTH = 512K
	RAM   (rwx) : ORIGIN = 0x20000000, LENGTH = 128K
}

ENTRY(reset_handler)

SECTIONS
{
	.vectors ORIGIN(FLASH) :
	{
		KEEP(*(.vectors))
	} > FLASH

	.text :
	{
		*(.text.reset_handler)
		*(.text*)
		*(.rodata*)
		. = ALIGN(4);
	} > FLASH

	.ARM.exidx :
	{
		*(.ARM.exidx*)
	} > FLASH

	ASSERT(. <= APP_BASE, "boot stub overlaps application base")

	.ibex_app APP_BASE :
	{
		KEEP(*(.ibex_app))
	} > FLASH

	/DISCARD/ :
	{
		*(.comment*)
		*(.note*)
		*(.ARM.attributes)
	}
}
