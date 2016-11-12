/****************************************************************************/
/*  Link Command File														*/
/*  Copyright (c) 2003  Texas Instruments Incorporated						*/
/****************************************************************************/
-heap 0x5000
-c
-x

/* Memory Map 0 - the default */
MEMORY
{
	FLASHRAM:		o = 000008010h	l = 00027ff0h
	BSS:			o = 000030000h	l = 00010000h
	CE0:			o = 080000040h	l = 007fffc0h
	CE1:			o = 090000000h	l = 10000000h
	CE2:			o = 0a0000000h	l = 00400000h
	CE3:			o = 0b0000000h	l = 00400000h
}

SECTIONS
{
	.cinit		>		FLASHRAM
	.const		>		FLASHRAM
	.switch		>		FLASHRAM
	.text		>		FLASHRAM
	.stack		>		BSS
	.bss		>		BSS
/*	.data		>		IRAM	*/
	.far		>		BSS
	.sysmem		>		FLASHRAM
/*	.tables		>		IRAM	*/
/*	.cio		>		IRAM	*/
}
