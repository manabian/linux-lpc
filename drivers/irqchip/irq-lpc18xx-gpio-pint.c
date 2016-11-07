/*
 * Irqchip driver for GPIO Pin Interrupt (PINT) on NXP LPC18xx/43xx.
 *
 * Copyright (C) 2016 Joachim Eastwood <manabian@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/init.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
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
	struct irq_domain *domain;
	void __iomem	  *base;
	struct clk	  *clk;
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

static int lpc18xx_gpio_pint_probe(struct platform_device *pdev)
{
	struct device_node *parent, *np = pdev->dev.of_node;
	struct lpc18xx_gpio_pint_chip *pint;
	struct irq_domain *domain_parent;
	struct resource *regs;
	int ret;

	parent = of_irq_find_parent(np);
	if (!parent) {
		dev_err(&pdev->dev, "irq find parent failed\n");
		return -ENXIO;
	}

	domain_parent = irq_find_host(parent);
	if (!domain_parent) {
		dev_err(&pdev->dev, "interrupt-parent not found\n");
		return -EINVAL;
	}

	pint = devm_kzalloc(&pdev->dev, sizeof(*pint), GFP_KERNEL);
	if (!pint)
		return -ENOMEM;

	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	pint->base = devm_ioremap_resource(&pdev->dev, regs);
	if (IS_ERR(pint->base))
		return PTR_ERR(pint->base);

	pint->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(pint->clk)) {
		dev_err(&pdev->dev, "input clock not found\n");
		return PTR_ERR(pint->clk);
	}

	ret = clk_prepare_enable(pint->clk);
	if (ret) {
		dev_err(&pdev->dev, "unable to enable clock\n");
		return ret;
	}

	spin_lock_init(pint->lock);
	pint->irq_offset = LPC18XX_GPIO_PINT_IRQ_OFFSET;

	/* Disable and clear all interrupts */
	writel(~0, pint->base + LPC18XX_GPIO_PINT_CIENR);
	writel(~0, pint->base + LPC18XX_GPIO_PINT_CIENF);
	writel(0,  pint->base + LPC18XX_GPIO_PINT_ISEL);
	writel(~0, pint->base + LPC18XX_GPIO_PINT_IST);

	pint->domain = irq_domain_add_hierarchy(domain_parent, 0,
					  LPC18XX_GPIO_PINT_IRQS, np,
					  &lpc18xx_gpio_pint_irq_domain_ops,
					  pint);
	if (!pint->domain) {
		clk_disable_unprepare(pint->clk);
		return -ENOMEM;
	}

	pr_err("%s: WTF\n", __func__);

	return 0;
}

static const struct of_device_id lpc18xx_gpio_pint_match[] = {
	{ .compatible = "nxp,lpc1850-gpio-pint" },
	{ }
};

static struct platform_driver lpc18xx_gpio_pint_driver = {
	.probe	= lpc18xx_gpio_pint_probe,
	.driver	= {
		.name = "lpc18xx-gpio-pint",
		.of_match_table = lpc18xx_gpio_pint_match,
	},
};
builtin_platform_driver(lpc18xx_gpio_pint_driver);
