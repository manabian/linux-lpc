/*
 * Copyright 2011 Team Embedded VOF
 *     Ithamar R. Adema <ihamar.adema@team-embedded.nl>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef MACH_LPC2K_SYSTEM_H
#define MACH_LPC2K_SYSTEM_H

#include <mach/regs-scb.h>
#include <mach/regs-wdt.h>
#include <mach/hardware.h>

static void arch_idle(void)
{
	/* Set PM0, IDL: idle mode */
	iowrite32((ioread32(APB_SCB_BASE + PCON) & ~PCON_PMMASK) | PCON_PM0,
		APB_SCB_BASE + PCON);
}

static inline void arch_reset(char mode, const char *cmd)
{
	switch (mode) {
	case 's':
	case 'h':
		printk(KERN_CRIT "RESET: Rebooting system\n");

		/* Disable interrupts */
		local_irq_disable();

		/* Setup WDT to fire on shortest allowed time */
		iowrite32(0xff, APB_WDT_BASE + WDTC);
		iowrite32(WDMOD_WDEN | WDMOD_WDRESET, APB_WDT_BASE + WDMOD);
		iowrite32(0xaa, APB_WDT_BASE + WDFEED);
		iowrite32(0x55, APB_WDT_BASE + WDFEED);
		break;

	default:
		/* Do nothing */
		break;
	}

	/* Wait for watchdog to reset system */
	while (1)
		;
}

#endif /* MACH_LPC2K_SYSTEM_H */
