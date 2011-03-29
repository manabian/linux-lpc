/*
 * Watchdog Timer register definitions
 *
 * Copyright 2011 Team Embedded VOF
 *     Ithamar R. Adema <ihamar.adema@team-embedded.nl>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef MACH_LPC2K_REGS_WDT_H
#define MACH_LPC2K_REGS_WDT_H

#define WDMOD		0x0000
#define WDTC		0x0004
#define WDFEED		0x0008
#define WDTV		0x000a
#define WDCLKSEL	0x0010

#define WDMOD_WDEN	(1 << 0)
#define WDMOD_WDRESET	(1 << 1)
#define WDMOD_WDTOF	(1 << 2)
#define WDMOD_WDINT	(1 << 3)

#endif /* MACH_LPC2K_REGS_WDT_H */
