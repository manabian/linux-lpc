/*
 * USB PHY driver for NXP LPC18xx/43xx internal USB PHY.
 *
 * Based on "usb-nop-xceiv"
 * Copyright (C) 2009 Texas Instruments Inc
 * Author: Ajay Kumar Gupta <ajay.gupta@ti.com>
 *
 * Copyright (C) 2014 Joachim Eastwood <manabian@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/clk.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/usb/otg.h>

#include "phy-generic.h"

#define LPC18XX_CREG_CREG0			0x004
# define LPC18XX_CREG_CREG0_USB0PHY		BIT(5)

struct lpc18xx_phy {
	struct usb_phy phy;
	struct regmap *reg;
	struct clk *clk;
};

static int nop_set_suspend(struct usb_phy *x, int suspend)
{
	return 0;
}

static int lpc18xx_usb_phy_init(struct usb_phy *phy)
{
	struct lpc18xx_phy *lpc = dev_get_drvdata(phy->dev);

	clk_prepare_enable(lpc->clk);
	regmap_update_bits(lpc->reg, LPC18XX_CREG_CREG0,
			   LPC18XX_CREG_CREG0_USB0PHY, 0);

	return 0;
}

static void lpc18xx_usb_phy_shutdown(struct usb_phy *phy)
{
	struct lpc18xx_phy *lpc = dev_get_drvdata(phy->dev);

	regmap_update_bits(lpc->reg, LPC18XX_CREG_CREG0,
			   LPC18XX_CREG_CREG0_USB0PHY,
			   LPC18XX_CREG_CREG0_USB0PHY);
	clk_disable_unprepare(lpc->clk);
}

static int lpc18xx_phy_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct lpc18xx_phy *lpc;
	struct clk *parent_clk;
	u32 clk_rate = 0;
	int ret;

	lpc = devm_kzalloc(dev, sizeof(*lpc), GFP_KERNEL);
	if (!lpc)
		return -ENOMEM;

	lpc->phy.otg = devm_kzalloc(dev, sizeof(*lpc->phy.otg), GFP_KERNEL);
	if (!lpc->phy.otg)
		return -ENOMEM;

	ret = of_property_read_u32(dev->of_node, "clock-frequency", &clk_rate);
	if (ret) {
		dev_err(dev, "Can't get clock-frequency property\n");
		return ret;
	}

	lpc->reg = syscon_regmap_lookup_by_compatible("nxp,lpc1850-creg");
	if (IS_ERR(lpc->reg)) {
		dev_err(dev, "Syscon lookup failed\n");
		return PTR_ERR(lpc->reg);
	}

	lpc->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(lpc->clk)) {
		dev_err(dev, "Can't get phy clock\n");
		return PTR_ERR(lpc->clk);
	}

	parent_clk = clk_get_parent(clk_get_parent(lpc->clk));
	if (IS_ERR(parent_clk)) {
		dev_err(dev, "Can't get phy parent clock\n");
		return PTR_ERR(parent_clk);
	}

	ret = clk_set_rate(parent_clk, clk_rate);
	if (ret) {
		dev_err(dev, "Error setting clock rate\n");
		return ret;
	}

	lpc->phy.dev		= dev;
	lpc->phy.label		= "lpc1850 usb phy";
	lpc->phy.set_suspend	= nop_set_suspend;
	lpc->phy.state		= OTG_STATE_UNDEFINED;
	lpc->phy.type		= USB_PHY_TYPE_USB2;
	lpc->phy.init		= lpc18xx_usb_phy_init;
	lpc->phy.shutdown	= lpc18xx_usb_phy_shutdown;

	lpc->phy.otg->phy		= &lpc->phy;
	lpc->phy.otg->set_host		= usb_gen_phy_otg_set_host;
	lpc->phy.otg->set_peripheral	= usb_gen_phy_otg_set_peripheral;

	platform_set_drvdata(pdev, lpc);

	ret = usb_add_phy_dev(&lpc->phy);
	if (ret) {
		dev_err(&pdev->dev, "can't register transceiver, err: %d\n", ret);
		return ret;
	}

	return 0;
}

static int lpc18xx_phy_remove(struct platform_device *pdev)
{
	struct lpc18xx_phy *lpc = platform_get_drvdata(pdev);

	usb_remove_phy(&lpc->phy);

	return 0;
}

static const struct of_device_id lpc18xx_phy_match[] = {
	{ .compatible = "nxp,lpc1850-usb-otg-phy" },
	{ }
};
MODULE_DEVICE_TABLE(of, lpc18xx_phy_match);

static struct platform_driver lpc18xx_phy_driver = {
	.probe		= lpc18xx_phy_probe,
	.remove		= lpc18xx_phy_remove,
	.driver		= {
		.name	= "lpc18xx_phy",
		.of_match_table = lpc18xx_phy_match,
	},
};
module_platform_driver(lpc18xx_phy_driver);

MODULE_AUTHOR("Joachim Eastwood <manabian@gmail.com>");
MODULE_DESCRIPTION("LPC18xx/43xx USB transceiver driver");
MODULE_LICENSE("GPL v2");
