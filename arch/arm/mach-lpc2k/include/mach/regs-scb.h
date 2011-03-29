/*
 * System Control Block register definitions
 *
 * Copyright 2011 Team Embedded VOF
 *     Ithamar R. Adema <ihamar.adema@team-embedded.nl>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef MACH_LPC2K_REGS_SCB_H
#define MACH_LPC2K_REGS_SCB_H

#define CLKSRCSEL	0x10c
#define CLKSRC(x)	((x) & 3)
#define PLLCON		0x080
#define PLLCFG		0x084
#define PLLSTAT		0x088
#define M(x)		(((x) & 0x7fff) + 1)
#define N(x)		((((x) >> 16) & 0xff) + 1)
#define PLLE		(1 << 24)
#define PLLC		(1 << 25)
#define PLLFEED		0x08c

#define CCLKCFG		0x104
#define CCLKSEL(x)	((x) & 0xff)
#define USBCLKCFG	0x108
#define USBSEL(x)	((x) & 0xf)
#define IRCTRIM		0x1a4
#define PCLKSEL0	0x1a8
#define PCLKSEL1	0x1ac

#define PCON		0x0c0
#define PCON_PM0	(1 << 0)
#define PCON_PM1	(1 << 1)
#define PCON_PM2	(1 << 7)
#define PCON_PMMASK	(PCON_PM0 | PCON_PM1 | PCON_PM2)
#define INTWAKE		0x144
#define PCONP		0x0c4

#endif /* MACH_LPC2K_REGS_SCB_H */
