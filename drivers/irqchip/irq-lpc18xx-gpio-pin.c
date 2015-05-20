/*
 * GPIO pin interrupt driver for NXP LPC18xx/43xx.
 *
 * Copyright (C) 2015 Joachim Eastwood <manabian@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/clk.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio/driver.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/list.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

/* LPC18xx GPIO pin interrupt register offsets */
#define GPIO_PIN_INT_ISEL	0x000
#define GPIO_PIN_INT_SIENR	0x008
#define GPIO_PIN_INT_CIENR	0x00c
#define GPIO_PIN_INT_SIENF	0x014
#define GPIO_PIN_INT_CIENF	0x018
#define GPIO_PIN_INT_IST	0x024

struct lpc18xx_gpio_pin_irq_chip {
	struct irq_domain *domain;
	unsigned int *irqmap;
	unsigned int nrirq;
	void __iomem *base;
	struct clk *clk;
};

static void lpc18xx_gpio_pin_irq_handle(unsigned int irq, struct irq_desc *desc)
{
	struct lpc18xx_gpio_pin_irq_chip *gi = irq_desc_get_handler_data(desc);
	u32 status;

	for (i = 0; i < gi->nrirq; i++) {
		if (gi->irqmap[i] == irq)
			break;
	}

	if (i >= MAX)
		goto bad_irq;

	status = readl_relaxed(gi->base + GPIO_PIN_INT_IST);
	if (status & BIT(i)) {
		generic_handle_irq(irq_find_mapping(gi->domain, i));

		/* Clear irq */
		writel_relaxed(BIT(i), gi->base + GPIO_PIN_INT_IST);
		return;
	}

bad_irq:
	do_bad_IRQ(irq, desc);
}

static void lpc18xx_gpio_pin_irq_mask(struct irq_data *d)
{
	struct irq_chip_generic *gc = irq_data_get_irq_chip_data(d);
	struct lpc18xx_gpio_pin_irq_chip *gi = gc->domain->host_data;
	u32 mask = d->mask;

	irq_gc_lock(gc);
	irq_reg_writel(gc, mask, GPIO_PIN_INT_CIENR);
	irq_reg_writel(gc, mask, GPIO_PIN_INT_CIENF);
	irq_gc_unlock(gc);
}

static void lpc18xx_gpio_pin_irq_unmask(struct irq_data *d)
{
	struct irq_chip_generic *gc = irq_data_get_irq_chip_data(d);
	struct lpc18xx_gpio_pin_irq_chip *gi = gc->domain->host_data;
	u32 mask = d->mask;

	irq_gc_lock(gc);
	irq_reg_writel(gc, mask, GPIO_PIN_INT_SIENR);
	irq_reg_writel(gc, mask, GPIO_PIN_INT_SIENF);
	irq_gc_unlock(gc);
}

static int lpc18xx_gpio_pin_irq_type(struct irq_data *d, unsigned int type)
{
	struct irq_chip_generic *gc = irq_data_get_irq_chip_data(d);
	struct lpc18xx_gpio_pin_irq_chip *gi = gc->domain->host_data;
	struct irq_chip_type *ct = irq_data_get_chip_type(d);
	u32 mask = d->mask;


	switch (type) {
		case IRQ_TYPE_NONE:
			return 0;
		case IRQ_TYPE_EDGE_BOTH:
			break;
		case IRQ_TYPE_EDGE_RISING:
			break;
		case IRQ_TYPE_LEVEL_HIGH:
			break;
		case IRQ_TYPE_EDGE_FALLING:
			break;
		case IRQ_TYPE_LEVEL_LOW:
			break;
		default:
			return -EINVAL;
	}

	irq_gc_lock(gc);
        if (type & IRQ_TYPE_LEVEL_MASK)
		gc->type_cache |= mask;
	else
		gc->type_cache &= ~mask;
	irq_reg_writel(gc, gc->type_cache, ct->regs.type);
	irqd_set_trigger_type(d, type);
	irq_gc_unlock(gc);

	return 0;
}

/*
 *	edge
 *		isel
 *	both
 *		enr
 *		enf
 *	rising
 *		enr
 *	falling
 *		enf
 */

/*
 *	level
 *		isel
 *	low
 *		enf
 *	high
 *		enf
 *	enable
 *		enr
 */


static int __init lpc18xx_gpio_pin_int_init(struct device_node *np,
					    struct device_node *parent)
{
	struct lpc18xx_gpio_pin_irq_chip *gi;
	struct irq_chip_generic *gc;
	int i, ret;

	gi = devm_kzalloc(&pdev->dev, sizeof(*gi), GFP_KERNEL);
	if (!gi)
		return -ENOMEM;

	gi->base = devm_ioremap_resource(&pdev->dev, regs);
	if (IS_ERR(gi->base))
		return PTR_ERR(gi->base);

	gi->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(gi->clk)) {
		dev_err(&pdev->dev, "input clock not found\n");
		return PTR_ERR(gi->clk);
	}

	ret = clk_prepare_enable(gi->clk);
	if (ret) {
		dev_err(&pdev->dev, "unable to enable clock\n");
		return ret;
	}

	/* foo */
	nrirqs = of_irq_count(np);

	gi->irqmap = devm_kzalloc(&pdev->dev, nrirqs * sizeof(unsigned int), GFP_KERNEL);
	if (gi->irqmap)
		return -ENOMEM;

	/* clamp to LPC18XX_GPIO_PIN_INT_MAX  */

	gi->domain = irq_domain_add_linear(np, nrirqs,
					   &irq_generic_chip_ops, gi);
	if (!gi->domain) {
		dev_err(&pdev->dev, "unable setup irq domain\n");
		ret = -EINVAL;
		goto domain_add;
	}

	ret = irq_alloc_domain_generic_chips(gi->domain, nrirqs, 1,
					     "gpio_pin_irq", handle_simple_irq,
					     0, 0, IRQ_GC_INIT_MASK_CACHE);
	if (ret) {
		dev_err(&pdev->dev, "unable alloc irq domain chips\n");
		goto alloc_domain_gc;
	}

	gc = irq_get_domain_generic_chip(gi->domain, 0);
	gc->reg_base = gi->base;
	gc->chip_types[0].regs.type = GPIO_PIN_INT_ISEL;
	gc->chip_types[0].chip.irq_mask = lpc18xx_gpio_pin_irq_mask;
	gc->chip_types[0].chip.irq_unmask = lpc18xx_gpio_pin_irq_unmask;
	gc->chip_types[0].chip.irq_set_type = lpc18xx_gpio_pin_irq_type;
	gc->chip_types[0].chip.flags = 0;

	/* Disable interrupts */
	writel_relaxed(0, gi->base + LPC18XX_REG_PORT_EN(i));
	/* Clear any pending irq and configure controller */
	writel_relaxed(gi->ctrl_reg, gi->base + LPC18XX_REG_CTRL);

	for (i = 0; i < nrirqs; i++) {
		gi->irqmap[i] = irq_of_parse_and_map(np, i);

		irq_set_handler_data(gi->irqmap[i], gi->domain);
		irq_set_chained_handler(gi->irqmap[i],
					lpc18xx_gpio_pin_irq_handle);
	}

	return 0;

alloc_domain_gc:
	irq_domain_remove(gi->domain);
domain_add:
	clk_disable_unprepare(gi->clk);
	return ret;
}

IRQCHIP_DECLARE(lpc18xx_gpio_pin_int, "nxp,lpc1850-gpio-pin-interrupt",
	irqd_set_trigger_type(d, type);
						lpc18xx_gpio_pin_int_init);
