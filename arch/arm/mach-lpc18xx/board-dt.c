/*
 * Device Tree board file for NXP LPC18xx/43xx
 *
 * Copyright (C) 2015 Joachim Eastwood <manabian@gmail.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/kernel.h>

#include <asm/v7m.h>
#include <asm/mach/arch.h>

static const char *const lpc18xx_43xx_compat[] __initconst = {
	"ea,lpc4357-developers-kit",
	NULL
};

DT_MACHINE_START(LPC18XXDT, "NXP LPC18xx/43xx (Device Tree Support)")
	.dt_compat = lpc18xx_43xx_compat,
MACHINE_END
