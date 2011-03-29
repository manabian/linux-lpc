/*
 * Copyright (C) 2011 Team Embeded VOF
 *     Ithamar R. Adema <ihamar.adema@team-embedded.nl>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <asm/hardware/vic.h>
#include <mach/hardware.h>

void __init lpc2k_init_irq(void)
{
	vic_init((void __iomem *)APH_VIC_BASE, 0, ~0, 0);
}
