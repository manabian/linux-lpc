/*
 * Copyright (C) 2011 Team Embeded VOF
 *     Ithamar R. Adema <ihamar.adema@team-embedded.nl>
 *
 * Based on MFP code from mach-pxa.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/io.h>

#include <mach/hardware.h>
#include <mach/mfp.h>

#define PINSEL(bank)	(0x00 + (bank)*4)
#define PINMODE(bank)	(0x40 + (bank)*4)

struct gpio_desc {
	unsigned valid:1;
	unsigned long config;
};

static struct gpio_desc gpio_desc[LPC2K_GPIO(4,31) + 1];

static int __mfp_config_gpio(unsigned gpio, unsigned long c)
{
	unsigned bank = gpio >> 4;
	unsigned shift = (gpio & 0xf) * 2;
	unsigned val;

	/* Configure alternate function */
	val = __raw_readl(APB_PINSEL_BASE + PINSEL(bank));
	if ((val & (3 << shift)) &&
	    (val & (3 << shift)) != (MFP_AF(c) << shift))
		pr_warning
		    ("GPIO%d is already configured (%x), not reconfigured!\n",
		     gpio, (val & (3 << shift)) >> shift);
	else {
		val &= ~(0x3 << shift);
		val |= MFP_AF(c) << shift;
		__raw_writel(val, APB_PINSEL_BASE + PINSEL(bank));
	}

	/* Configuration pullup/dn */
	val = __raw_readl(APB_PINSEL_BASE + PINMODE(bank));
	val &= ~(0x3 << shift);
	val |= MFP_PULL(c) << shift;
	__raw_writel(val, APB_PINSEL_BASE + PINMODE(bank));

	return 0;
}

static inline int __mfp_validate(int mfp)
{
	int gpio = mfp_to_gpio(mfp);

	if (!gpio_desc[gpio].valid) {
		pr_warning("%s: GPIO%d is invalid pin\n", __func__, gpio);
		return -1;
	}

	return gpio;
}

void lpc2k_mfp_config(unsigned long *mfp_cfgs, int num)
{
	unsigned long flags;
	unsigned long *c;
	int i, gpio;

	for (i = 0, c = mfp_cfgs; i < num; i++, c++) {

		gpio = __mfp_validate(MFP_PIN(*c));
		if (gpio < 0)
			continue;

		local_irq_save(flags);

		gpio_desc[gpio].config = *c;
		__mfp_config_gpio(gpio, *c);

		local_irq_restore(flags);
	}
}

static int __init lpc2k_mfp_init(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(gpio_desc); i++)
		gpio_desc[i].valid = 1;

	return 0;
}

postcore_initcall(lpc2k_mfp_init);
