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

struct lpc18xx_gpio_pin_irq_chip {
	struct irq_domain *domain;
	void __iomem *base;
	struct clk *clk;
};

static void lpc18xx_gpio_pin_irq_handle(unsigned int irq, struct irq_desc *desc)
{
	struct lpc18xx_gpio_pin_irq_chip *gi = irq_desc_get_handler_data(desc);
	struct lpc18xx_gpio_pin_irq *gpio_irq;

	//generic_handle_irq(gpio_irq->virq);

	/* Clear pending irq */
	//writel_relaxed(gi->ctrl_reg, gi->base + LPC18XX_REG_CTRL);
}

static int lpc18xx_gpio_pin_irq_type(struct irq_data *d, unsigned int type)
{
	struct irq_chip_generic *gc = irq_data_get_irq_chip_data(d);
	struct lpc18xx_gpio_pin_irq_chip *gi = gc->domain->host_data;
	bool level_trigger = gi->ctrl_reg & LPC18XX_REG_CTRL_TRIG;
	struct irq_chip_type *ct = irq_data_get_chip_type(d);
	u32 mask = d->mask;
	bool active_high;

	if (!level_trigger && (type & IRQ_TYPE_LEVEL_MASK))
		return -EINVAL;

	if (level_trigger && (type & IRQ_TYPE_EDGE_BOTH))
		return -EINVAL;

	switch (type) {
		case IRQ_TYPE_NONE:
			return 0;
		case IRQ_TYPE_EDGE_BOTH:
			return -EINVAL;
		case IRQ_TYPE_EDGE_RISING:
		case IRQ_TYPE_LEVEL_HIGH:
			active_high = true;
			break;
		case IRQ_TYPE_EDGE_FALLING:
		case IRQ_TYPE_LEVEL_LOW:
			active_high = false;
			break;
		default:
			return -EINVAL;
	}

	irq_gc_lock(gc);
	if (active_high)
		gc->polarity_cache |= mask;
	else
		gc->polarity_cache &= ~mask;
	irq_reg_writel(gc, gc->polarity_cache, ct->regs.polarity);
	irqd_set_trigger_type(d, type);
	irq_gc_unlock(gc);

	return 0;
}

static int lpc18xx_gpio_pin_irq_probe(struct platform_device *pdev)
{
	struct resource *regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	struct resource *irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	struct device_node *np = pdev->dev.of_node;
	struct lpc18xx_gpio_pin_irq_chip *gi;
	struct device_node *gpio_ctrl;
	struct gpio_chip *gpio_chip;
	int i, ret;

	if (!regs || !irq) {
		dev_err(&pdev->dev, "no registers/irq defined\n");
		return -EINVAL;
	}

	gi = devm_kzalloc(&pdev->dev, sizeof(*gi), GFP_KERNEL);
	if (!gi)
		return -ENOMEM;

	platform_set_drvdata(pdev, gi);

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

	gi->domain = irq_domain_add_linear(np, 8,
					   &irq_generic_chip_ops,
					   gi);
	if (!gi->domain) {
		dev_err(&pdev->dev, "unable setup irq domain\n");
		ret = -EINVAL;
		goto domain_add;
	}

	ret = irq_alloc_domain_generic_chips(gi->domain, 32, 1, "gpio_pin_irq",
					     handle_simple_irq, 0, 0,
					     IRQ_GC_INIT_MASK_CACHE);
	if (ret) {
		dev_err(&pdev->dev, "unable alloc irq domain chips\n");
		goto alloc_domain_gc;
	}

	for (i = 0; i < 8; i++) {
		struct irq_chip_generic *gc;

		gc = irq_get_domain_generic_chip(gi->domain, 32 * i);
		gc->reg_base = gi->base;
		gc->chip_types[0].regs.mask = LPC18XX_REG_PORT_EN(i);
		gc->chip_types[0].regs.polarity = LPC18XX_REG_PORT_POL(i);
		gc->chip_types[0].chip.irq_mask = irq_gc_mask_clr_bit;
		gc->chip_types[0].chip.irq_unmask = irq_gc_mask_set_bit;
		gc->chip_types[0].chip.irq_set_type = lpc18xx_gpio_pin_irq_type;

		/* Disable interrupts */
		writel_relaxed(0, gi->base + LPC18XX_REG_PORT_EN(i));
	}

	/* Clear any pending irq and configure controller */
	writel_relaxed(gi->ctrl_reg, gi->base + LPC18XX_REG_CTRL);

	// for all irqs
	irq_set_handler_data(irq->start, gi);
	irq_set_chained_handler(irq->start, lpc18xx_gpio_pin_irq_handle);

	return 0;

alloc_domain_gc:
	irq_domain_remove(gi->domain);
domain_add:
	clk_disable_unprepare(gi->clk);
	return ret;
}

static int lpc18xx_gpio_pin_irq_remove(struct platform_device *pdev)
{
	struct lpc18xx_gpio_pin_irq_chip *gi = platform_get_drvdata(pdev);
	int i;

	for (i = 0; i < 8; i++) {
		struct irq_chip_generic *gc;

		gc = irq_get_domain_generic_chip(gi->domain, 32 * i);
		irq_remove_generic_chip(gc, ~0, 0, 0);
	}

	kfree(gi->domain->gc);
	irq_domain_remove(gi->domain);

	clk_disable_unprepare(gi->clk);

	return 0;
}

static const struct of_device_id lpc18xx_gpio_pin_irq_match[] = {
	{ .compatible = "nxp,lpc1850-gpio-pin-interrupt" },
	{ }
};
MODULE_DEVICE_TABLE(of, lpc18xx_gpio_pin_irq_match);

static struct platform_driver lpc18xx_gpio_pin_irq_driver = {
	.probe	= lpc18xx_gpio_pin_irq_probe,
	.remove	= lpc18xx_gpio_pin_irq_remove,
	.driver	= {
		.name		= "lpc18xx-gpio-pin-interrupt",
		.of_match_table	= lpc18xx_gpio_pin_irq_match,
	},
};
module_platform_driver(lpc18xx_gpio_pin_irq_driver);

MODULE_AUTHOR("Joachim Eastwood <manabian@gmail.com>");
MODULE_DESCRIPTION("GPIO pin interrupt driver for LPC18xx/43xx");
MODULE_LICENSE("GPL v2");
