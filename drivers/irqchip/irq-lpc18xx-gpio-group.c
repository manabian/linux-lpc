/*
 * GPIO group interrupt driver for NXP LPC18xx/43xx.
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

/* LPC18xx GPIO group interrupt register offsets */
#define LPC18XX_REG_CTRL		0x000
#define  LPC18XX_REG_CTRL_INT		BIT(0)
#define  LPC18XX_REG_CTRL_COMB		BIT(1)
#define  LPC18XX_REG_CTRL_TRIG		BIT(2)
#define LPC18XX_REG_PORT_POL(n)		(0x020 + n * sizeof(u32))
#define LPC18XX_REG_PORT_EN(n)		(0x040 + n * sizeof(u32))

#define LPC18XX_GPIO_GRP_IRQ_PORTS	8
#define LPC18XX_GPIO_GRP_IRQ_PER_PORT	32

#define GPIO_POL_HIGH	(IRQ_TYPE_EDGE_RISING  | IRQ_TYPE_LEVEL_HIGH)
#define GPIO_POL_LOW	(IRQ_TYPE_EDGE_FALLING | IRQ_TYPE_LEVEL_LOW)

struct lpc18xx_gpio_grp_irq_chip {
	struct hlist_head gpio_irqs;
	struct gpio_chip *gpio_chip;
	struct irq_domain *domain;
	void __iomem *base;
	struct clk *clk;
	u32 ctrl_reg;
};

struct lpc18xx_gpio_grp_irq {
	struct hlist_node entry;
	struct gpio_desc *gpio_desc;
	unsigned int virq;
};

static void lpc18xx_gpio_grp_irq_handle(unsigned int irq, struct irq_desc *desc)
{
	struct lpc18xx_gpio_grp_irq_chip *gi = irq_desc_get_handler_data(desc);
	struct lpc18xx_gpio_grp_irq *gpio_irq;

	hlist_for_each_entry(gpio_irq, &gi->gpio_irqs, entry) {
		int value = gpiod_get_raw_value(gpio_irq->gpio_desc);
		u32 trigger = irq_get_trigger_type(gpio_irq->virq);

		if (value && (trigger & GPIO_POL_HIGH))
			generic_handle_irq(gpio_irq->virq);
		else if (!value && (trigger & GPIO_POL_LOW))
			generic_handle_irq(gpio_irq->virq);
	}

	/* Clear pending irq */
	writel_relaxed(gi->ctrl_reg, gi->base + LPC18XX_REG_CTRL);
}

static int lpc18xx_gpio_grp_irq_type(struct irq_data *d, unsigned int type)
{
	struct irq_chip_generic *gc = irq_data_get_irq_chip_data(d);
	struct lpc18xx_gpio_grp_irq_chip *gi = gc->domain->host_data;
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

static int lpc18xx_gpio_grp_irq_map(struct irq_domain *d, unsigned int virq,
				    irq_hw_number_t hw_irq)
{
	struct lpc18xx_gpio_grp_irq_chip *gi = d->host_data;
	struct lpc18xx_gpio_grp_irq *gpio_irq;
	struct gpio_desc *gpio_desc;
	int ret;

	gpio_desc = gpiochip_get_desc(gi->gpio_chip, hw_irq);
	if (IS_ERR(gpio_desc)) {
		pr_info("%s: gpiochip_get_desc failed\n", __func__);
		return -EINVAL;
	}

	ret = gpiod_request(gpio_desc, NULL);
	if (ret) {
		pr_info("%s: gpiod_request failed\n", __func__);
		return ret;
	}

	gpio_irq = kzalloc(sizeof(*gpio_irq), GFP_KERNEL);
	if (!gpio_irq) {
		ret = -ENOMEM;
		goto kzalloc_fail;
	}

	ret = irq_map_generic_chip(d, virq, hw_irq);
	if (ret)
		goto map_generic_fail;

	INIT_HLIST_NODE(&gpio_irq->entry);
	gpio_irq->gpio_desc = gpio_desc;
	gpio_irq->virq = virq;

	/* add to hlist head under lock */
	hlist_add_head(&gpio_irq->entry, &gi->gpio_irqs);

	return 0;

map_generic_fail:
	kfree(gpio_irq);
kzalloc_fail:
	gpiod_put(gpio_desc);
	return ret;
}

static void lpc18xx_gpio_grp_irq_unmap(struct irq_domain *d, unsigned int virq)
{
	struct lpc18xx_gpio_grp_irq_chip *gi = d->host_data;
	struct lpc18xx_gpio_grp_irq *gpio_irq;

	hlist_for_each_entry(gpio_irq, &gi->gpio_irqs, entry) {
		if (gpio_irq->virq == virq) {
			hlist_del(&gpio_irq->entry);
			gpiod_put(gpio_irq->gpio_desc);
			kfree(gpio_irq);
			return;
		}
	}

	pr_warn("%s: virq %u was not mapped\n", __func__, virq);
}

static const struct irq_domain_ops lpc18xx_gpio_grp_irq_domain_ops = {
	.map    = lpc18xx_gpio_grp_irq_map,
	.unmap  = lpc18xx_gpio_grp_irq_unmap,
	.xlate  = irq_domain_xlate_twocell,
};

static int of_node_gpiochip_match(struct gpio_chip *chip, void *data)
{
	struct device_node *gpio_ctrl = data;
	return chip->of_node == gpio_ctrl;
}

static int lpc18xx_gpio_grp_irq_probe(struct platform_device *pdev)
{
	struct resource *regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	struct resource *irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	struct device_node *np = pdev->dev.of_node;
	struct lpc18xx_gpio_grp_irq_chip *gi;
	struct device_node *gpio_ctrl;
	struct gpio_chip *gpio_chip;
	int i, ret;

	if (!regs || !irq) {
		dev_err(&pdev->dev, "no registers/irq defined\n");
		return -EINVAL;
	}

	gpio_ctrl = of_parse_phandle(pdev->dev.of_node, "gpio-controller", 0);
	if (!gpio_ctrl) {
		dev_err(&pdev->dev, "can't parse gpio-controller node\n");
		return -EINVAL;
	}

	gpio_chip = gpiochip_find(gpio_ctrl, of_node_gpiochip_match);
	if (!gpio_chip)
		return -EPROBE_DEFER;

	gi = devm_kzalloc(&pdev->dev, sizeof(*gi), GFP_KERNEL);
	if (!gi)
		return -ENOMEM;

	gi->ctrl_reg = LPC18XX_REG_CTRL_INT;

	if (of_property_read_bool(np, "level-triggered-interrupts"))
		gi->ctrl_reg |= LPC18XX_REG_CTRL_TRIG;

	if (of_property_read_bool(np, "and-combined-interrupts"))
		gi->ctrl_reg |= LPC18XX_REG_CTRL_COMB;

	INIT_HLIST_HEAD(&gi->gpio_irqs);
	gi->gpio_chip = gpio_chip;
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

	gi->domain = irq_domain_add_linear(np, 32 * 8,
					   &lpc18xx_gpio_grp_irq_domain_ops,
					   gi);
	if (!gi->domain) {
		dev_err(&pdev->dev, "unable setup irq domain\n");
		ret = -EINVAL;
		goto domain_add;
	}

	ret = irq_alloc_domain_generic_chips(gi->domain, 32, 1, "gpio_grp_irq",
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
		gc->chip_types[0].chip.irq_set_type = lpc18xx_gpio_grp_irq_type;

		/* Disable interrupts */
		writel_relaxed(0, gi->base + LPC18XX_REG_PORT_EN(i));
	}

	/* Clear any pending irq and configure controller */
	writel_relaxed(gi->ctrl_reg, gi->base + LPC18XX_REG_CTRL);

	irq_set_handler_data(irq->start, gi);
	irq_set_chained_handler(irq->start, lpc18xx_gpio_grp_irq_handle);

	return 0;

alloc_domain_gc:
	irq_domain_remove(gi->domain);
domain_add:
	clk_disable_unprepare(gi->clk);
	return ret;
}

static int lpc18xx_gpio_grp_irq_remove(struct platform_device *pdev)
{
	struct lpc18xx_gpio_grp_irq_chip *gi = platform_get_drvdata(pdev);
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

static const struct of_device_id lpc18xx_gpio_grp_irq_match[] = {
	{ .compatible = "nxp,lpc1850-gpio-group-interrupt" },
	{ }
};
MODULE_DEVICE_TABLE(of, lpc18xx_gpio_grp_irq_match);

static struct platform_driver lpc18xx_gpio_grp_irq_driver = {
	.probe	= lpc18xx_gpio_grp_irq_probe,
	.remove	= lpc18xx_gpio_grp_irq_remove,
	.driver	= {
		.name		= "lpc18xx-gpio-group-interrupt",
		.of_match_table	= lpc18xx_gpio_grp_irq_match,
	},
};
module_platform_driver(lpc18xx_gpio_grp_irq_driver);

MODULE_AUTHOR("Joachim Eastwood <manabian@gmail.com>");
MODULE_DESCRIPTION("GPIO group interrupt driver for LPC18xx/43xx");
MODULE_LICENSE("GPL v2");
