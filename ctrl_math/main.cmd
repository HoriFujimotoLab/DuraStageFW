/****************************************************************************/
/*  Linker Command File														*/
/* 	Description: memory map of MyWay PE-Expert3 C6713A						*/
/*	Author: Thomas Beauduin, University of Tokyo, 2015						*/
/*  Copyright (c) 2010  Texas Instruments Incorporated						*/
/****************************************************************************/
-heap 0x5000
-c
-x

MEMORY
{
/*	A. INTERNAL RAM 256kb: */
/*	KERNEL: 	o = 000000000h  l = 0000800fh *//* 28kb: myway memory (reserved)*/
	FLASHRAM:	o = 000008010h	l = 00027ff0h	/* 163kb:user program 			*/
	BSS:		o = 000030000h	l = 00010000h	/* 65kb: parameters, stack 		*/
/*	B. EXTERNAL RAM 8mb: */
/*	TRACE:		o = 0x80000000	l = 0x000003F *//* 63b:	 trace control memory	*/
	CE0:		o = 080000040h	l = 007fffc0h	/* 8mb:	 user free memory		*/
}

SECTIONS
{
	.cinit		>		FLASHRAM		/* tables which init global var 		*/
	.const		>		FLASHRAM		/* initialized global var 				*/
	.switch		>		FLASHRAM		/* jump table of switch statements 		*/
	.text		>		FLASHRAM		/* executable code (.out) 				*/
	.data		>		FLASHRAM		/* LUT necessary with fastmath67x.lib 	*/
	.stack		>		BSS				/* system var stack 					*/
	.bss		>		BSS				/* global var 							*/
	.far		>		CE0				/* far pointer tables 					*/
	.sysmem		>		CE0				/* malloc heap (trace) 					*/
}
