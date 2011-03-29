/*
 * Copyright 2011 Team Embedded VOF
 *     Ithamar R. Adema <ihamar.adema@team-embedded.nl>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */


#include <mach/hardware.h>

	.macro	addruart, rp, rv
	ldr	\rp, =APB_UART0_BASE
	ldr	\rv, =APB_UART0_BASE
	.endm

#define UART_SHIFT	2
#include <asm/hardware/debug-8250.S>
