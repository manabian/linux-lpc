/*
 * Irqchip driver for GPIO Pin Interrupt (PINT) on NXP LPC18xx/43xx.
 *
 * Copyright (C) 2016 Joachim Eastwood <manabian@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#define pr_fmt(fmt) "%s: " fmt, __func__

#include <linux/clk.h>
#include <linux/init.h>
#include <linux/irq.h>
#include <linux/irqchip.h>
#include <linux/irqdomain.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

/* LPC18xx GPIO pin interrupt register offsets */
#define LPC18XX_GPIO_PINT_ISEL		0x000
#define LPC18XX_GPIO_PINT_SIENR		0x008
#define LPC18XX_GPIO_PINT_CIENR		0x00c
#define LPC18XX_GPIO_PINT_SIENF		0x014
#define LPC18XX_GPIO_PINT_CIENF		0x018
#define LPC18XX_GPIO_PINT_IST		0x024

#define LPC18XX_GPIO_PINT_IRQS		8
#define LPC18XX_GPIO_PINT_IRQ_OFFSET	32

struct lpc18xx_gpio_pint_chip {
	void __iomem	  *base;
	spinlock_t	  *lock;
	unsigned	  irq_offset;
	u32		  type_cache;
};

static void lpc18xx_gpio_pint_edge_ack(struct irq_data *data)
{
	struct lpc18xx_gpio_pint_chip *pint = data->chip_data;
	u32 mask = BIT(data->hwirq - pint->irq_offset);

	writel(mask, pint->base + LPC18XX_GPIO_PINT_IST);
}

static void lpc18xx_gpio_pint_mask(struct irq_data *data)
{
	struct lpc18xx_gpio_pint_chip *pint = data->chip_data;
	u32 mask = BIT(data->hwirq - pint->irq_offset);
	u32 type = irqd_get_trigger_type(data);

	if (type & IRQ_TYPE_LEVEL_MASK) {
		writel(mask, pint->base + LPC18XX_GPIO_PINT_SIENR);
	} else if (type & IRQ_TYPE_EDGE_BOTH) {
		writel(mask, pint->base + LPC18XX_GPIO_PINT_CIENR);
		writel(mask, pint->base + LPC18XX_GPIO_PINT_CIENF);
	}

	irq_chip_mask_parent(data);
}

static void lpc18xx_gpio_pint_unmask(struct irq_data *data)
{
	struct lpc18xx_gpio_pint_chip *pint = data->chip_data;
	u32 mask = BIT(data->hwirq - pint->irq_offset);
	u32 type = irqd_get_trigger_type(data);

	if (type & IRQ_TYPE_LEVEL_MASK) {
		writel(mask, pint->base + LPC18XX_GPIO_PINT_CIENR);
	} else {
		if (type & IRQ_TYPE_EDGE_RISING)
			writel(mask, pint->base + LPC18XX_GPIO_PINT_SIENR);
		if (type & IRQ_TYPE_EDGE_FALLING)
			writel(mask, pint->base + LPC18XX_GPIO_PINT_SIENF);
	}

	irq_chip_unmask_parent(data);
}

static int lpc18xx_gpio_pint_type(struct irq_data *data, unsigned int type)
{
	struct lpc18xx_gpio_pint_chip *pint = data->chip_data;
	u32 mask = BIT(data->hwirq - pint->irq_offset);

	spin_lock(pint->lock);
	if (type & IRQ_TYPE_LEVEL_MASK)
		pint->type_cache |= mask;
	else
		pint->type_cache &= ~mask;
	writel(pint->type_cache, pint->base + LPC18XX_GPIO_PINT_ISEL);

	switch (type) {
	case IRQ_TYPE_LEVEL_HIGH:
		writel(mask, pint->base + LPC18XX_GPIO_PINT_SIENF);
		break;

	case IRQ_TYPE_LEVEL_LOW:
		writel(mask, pint->base + LPC18XX_GPIO_PINT_CIENF);
		break;

	/* IRQ_TYPE_EDGE_* is set in lpc18xx_gpio_pint_unmask */
	}

	irqd_set_trigger_type(data, type);
	spin_unlock(pint->lock);

	return 0;
}

static struct irq_chip lpc18xx_gpio_pint_irq_chip = {
	.name			= "gpio-pint",
	.irq_ack		= lpc18xx_gpio_pint_edge_ack,
	.irq_mask		= lpc18xx_gpio_pint_mask,
	.irq_unmask		= lpc18xx_gpio_pint_unmask,
	.irq_eoi		= irq_chip_eoi_parent,
	.irq_set_type		= lpc18xx_gpio_pint_type,
};

static int lpc18xx_gpio_pint_domain_alloc(struct irq_domain *domain,
					  unsigned int virq,
					  unsigned int nr_irqs, void *arg)
{
	struct irq_fwspec *fwspec = arg;
	struct irq_fwspec parent_fwspec;
	irq_hw_number_t hwirq;
	int i;

	pr_err("%s\n", __func__);

	if (!irq_domain_get_of_node(domain->parent))
		return -EINVAL;

	if (fwspec->param_count != 2)
		return -EINVAL;

	pr_err("%s for loop\n", __func__);
	hwirq = fwspec->param[0];
	for (i = 0; i < nr_irqs; i++) {
		pr_err("%s: hwirq %lu\n", __func__, hwirq);
		irq_domain_set_hwirq_and_chip(domain, virq + i, hwirq + i,
					      &lpc18xx_gpio_pint_irq_chip,
					      domain->host_data);
	}

	parent_fwspec.fwnode = domain->parent->fwnode;

	parent_fwspec.param_count = 1;
	parent_fwspec.param[0] = fwspec->param[0];

	return irq_domain_alloc_irqs_parent(domain, virq, nr_irqs,
					    &parent_fwspec);
}

static int lpc18xx_gpio_pint_domain_translate(struct irq_domain *d,
					      struct irq_fwspec *fwspec,
					      unsigned long *hwirq,
					      unsigned int *type)
{
	pr_err("%s\n", __func__);

	if (WARN_ON(fwspec->param_count < 2))
		return -EINVAL;

	pr_err("%s set hwirq/type\n", __func__);

	*hwirq = fwspec->param[0];
	*type = fwspec->param[1] & IRQ_TYPE_SENSE_MASK;

	return 0;
}

static const struct irq_domain_ops lpc18xx_gpio_pint_irq_domain_ops = {
	.translate = lpc18xx_gpio_pint_domain_translate,
	.alloc = lpc18xx_gpio_pint_domain_alloc,
	.free = irq_domain_free_irqs_common,
};

static int __init lpc18xx_gpio_pint_init(struct device_node *node,
					 struct device_node *parent)
{
	struct irq_domain *domain, *domain_parent;
	struct lpc18xx_gpio_pint_chip *pint;
	struct clk *clk;
	int ret;

	domain_parent = irq_find_host(parent);
	if (!domain_parent) {
		pr_err("interrupt-parent not found\n");
		return -EINVAL;
	}

	clk = of_clk_get_by_name(node, NULL);
	if (IS_ERR(clk)) {
		pr_err("clock get failed (%ld)\n", PTR_ERR(clk));
		return PTR_ERR(clk);
	}

	ret = clk_prepare_enable(clk);
	if (ret) {
		pr_err("clock enable failed (%d)\n", ret);
		goto err_clk_enable;
	}

	pint = kzalloc(sizeof(*pint), GFP_KERNEL);
	if (!pint) {
		ret = -ENOMEM;
		goto err_alloc;
	}

	pint->base = of_io_request_and_map(node, 0, NULL);
	if (IS_ERR(pint->base)) {
		pr_err("unable to map register\n");
		ret = PTR_ERR(pint->base);
		goto err_map;
	}

	spin_lock_init(pint->lock);
	pint->irq_offset = LPC18XX_GPIO_PINT_IRQ_OFFSET;

	/* Disable and clear all interrupts */
	writel(~0, pint->base + LPC18XX_GPIO_PINT_CIENR);
	writel(~0, pint->base + LPC18XX_GPIO_PINT_CIENF);
	writel(0,  pint->base + LPC18XX_GPIO_PINT_ISEL);
	writel(~0, pint->base + LPC18XX_GPIO_PINT_IST);

	domain = irq_domain_add_hierarchy(domain_parent, 0,
					  LPC18XX_GPIO_PINT_IRQS, node,
					  &lpc18xx_gpio_pint_irq_domain_ops,
					  pint);
	if (!domain) {
		ret = -ENOMEM;
		goto err_irq_domain;
	}

	return 0;

err_irq_domain:
	iounmap(pint->base);
err_map:
	kfree(pint);
err_alloc:
	clk_disable_unprepare(clk);
err_clk_enable:
	clk_put(clk);
	return ret;
}
IRQCHIP_DECLARE(lpc18xx_gpio_pint, "nxp,lpc1850-gpio-pint", lpc18xx_gpio_pint_init);
