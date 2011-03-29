/*
 * Copyright 2011 Team Embedded VOF
 *     Ithamar R. Adema <ihamar.adema@team-embedded.nl>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef MACH_LPC2K_HARDWARE_H
#define MACH_LPC2K_HARDWARE_H

#include <asm/sizes.h>

/* Default memory size if no ATAGS found */
#define MEM_SIZE	(SZ_32M)

#define FAST_GPIO_BASE		0x3fffc000
#define APB_WDT_BASE		0xe0000000
#define APB_TIMER0_BASE		0xe0004000
#define APB_TIMER1_BASE		0xe0008000
#define APB_UART0_BASE		0xe000c000
#define APB_GPIO_BASE		0xe0028000
#define APB_SCB_BASE		0xe01fc000
#define APH_VIC_BASE		0xfffff000

#endif /* MACH_LPC2K_HARDWARE_H */
