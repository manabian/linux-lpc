/*
 * Reset driver for NXP LPC18xx/43xx Reset Generation Unit (RGU).
 *
 * Copyright (C) 2014 Joachim Eastwood <manabian@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>
#include <linux/reset-controller.h>
#include <linux/slab.h>
#include <linux/types.h>

#include <asm/system_misc.h>

#define LPC18XX_RGU_RESETS_PER_REG	32

#define LPC18XX_RGU_CTRL0		0x100
#define  LPC18XX_RGU_CTRL0_CORE		BIT(0)
#define LPC18XX_RGU_CTRL1		0x104

struct lpc18xx_reset_data {
	struct reset_controller_dev rcdev;
	struct clk *clk_delay;
	struct clk *clk_reg;
	void __iomem *base;
	u32 delay_us;
};

static void __iomem *rgu_base;

static void lpc18xx_rgu_restart(enum reboot_mode mode, const char *cmd)
{
	while (1) {
		writel(LPC18XX_RGU_CTRL0_CORE, rgu_base + LPC18XX_RGU_CTRL0);
		mdelay(100);
	}
}

static int lpc18xx_reset_assert(struct reset_controller_dev *rcdev,
				unsigned long id)
{
	struct lpc18xx_reset_data *rc;
	u32 offset = LPC18XX_RGU_CTRL0;
	u8 bit;

	rc = container_of(rcdev, struct lpc18xx_reset_data, rcdev);

	offset += (id / LPC18XX_RGU_RESETS_PER_REG) * sizeof(u32);
	bit = id % LPC18XX_RGU_RESETS_PER_REG;

	writel(bit, rgu_base + offset);

	udelay(rc->delay_us);

	return 0;
}

static struct reset_control_ops lpc18xx_reset_ops = {
	.assert		= lpc18xx_reset_assert,
};

static int lpc18xx_reset_probe(struct platform_device *pdev)
{
	struct lpc18xx_reset_data *rc;
	struct resource *res;
	u32 fcclk, firc;
	int ret;

	rc = devm_kzalloc(&pdev->dev, sizeof(*rc), GFP_KERNEL);
	if (!rc)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	rc->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(rc->base))
		return PTR_ERR(rc->base);

	rc->clk_reg = devm_clk_get(&pdev->dev, "reg");
	if (IS_ERR(rc->clk_reg)) {
		dev_err(&pdev->dev, "Register clock not found.\n");
		return PTR_ERR(rc->clk_reg);
	}

	rc->clk_delay = devm_clk_get(&pdev->dev, "delay");
	if (IS_ERR(rc->clk_delay)) {
		dev_err(&pdev->dev, "Delay clock not found.\n");
		return PTR_ERR(rc->clk_delay);
	}

	ret = clk_prepare_enable(rc->clk_reg);
	if (ret) {
		dev_err(&pdev->dev, "Unable to enable reg clock.\n");
		return ret;
	}

	ret = clk_prepare_enable(rc->clk_delay);
	if (ret) {
		dev_err(&pdev->dev, "Unable to enable delay clock.\n");
		goto dis_clk_reg;
	}

	fcclk = clk_get_rate(rc->clk_reg) / USEC_PER_SEC;
	firc = clk_get_rate(rc->clk_delay) / USEC_PER_SEC;
	if (fcclk == 0 || firc == 0)
		rc->delay_us = 2;
	else
		rc->delay_us = DIV_ROUND_UP(fcclk, firc * firc);

	rc->rcdev.owner = THIS_MODULE;
	rc->rcdev.nr_resets = 64;
	rc->rcdev.ops = &lpc18xx_reset_ops;
	rc->rcdev.of_node = pdev->dev.of_node;

	platform_set_drvdata(pdev, rc);

	ret = reset_controller_register(&rc->rcdev);
	if (ret) {
		dev_err(&pdev->dev, "Unable to register device.\n");
		goto dis_clks;
	}

	rgu_base = rc->base;
	arm_pm_restart = lpc18xx_rgu_restart;

	return 0;

dis_clks:
	clk_disable_unprepare(rc->clk_delay);
dis_clk_reg:
	clk_disable_unprepare(rc->clk_reg);

	return ret;
}

static const struct of_device_id lpc18xx_reset_match[] = {
	{ .compatible = "nxp,lpc1850-reset" },
	{ }
};
MODULE_DEVICE_TABLE(of, lpc18xx_reset_match);

static struct platform_driver lpc18xx_reset_driver = {
	.probe	= lpc18xx_reset_probe,
	.driver	= {
		.name		= "lpc18xx-reset",
		.of_match_table	= lpc18xx_reset_match,
	},
};
module_platform_driver(lpc18xx_reset_driver);

MODULE_AUTHOR("Joachim Eastwood <manabian@gmail.com>");
MODULE_DESCRIPTION("Reset driver for LPC18xx/43xx RGU");
MODULE_LICENSE("GPL v2");
