/*
 * EHCI USB driver for NXP LPC18xx/43xx.
 *
 * Copyright (C) 2014 Joachim Eastwood <manabian@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/usb/ehci_def.h>
#include <linux/usb.h>
#include <linux/usb/hcd.h>
#include <linux/usb/otg.h>

#include "ehci.h"

#define LPC18XX_EHCI_CAPS_OFFSET	0x100

struct lpc18xx_ehci_hcd {
	struct clk *clk_usb;
	struct clk *clk_reg;
};

static struct hc_driver __read_mostly lpc18xx_hc_driver;

static int lpc18xx_ehci_reset(struct usb_hcd *hcd)
{
	/*struct ehci_hcd *ehci = hcd_to_ehci(hcd);*/

	pr_info("%s\n", __func__);

	return 0;
}

static const struct ehci_driver_overrides lpc18xx_echi_overrides __initconst = {
	.extra_priv_size = sizeof(struct lpc18xx_ehci_hcd),
	.reset = lpc18xx_ehci_reset,
};

static int lpc18xx_ehci_probe(struct platform_device *pdev)
{
	struct lpc18xx_ehci_hcd *lpc;
	struct ehci_hcd *ehci;
	struct resource *res;
	struct usb_hcd *hcd;
	int irq, ret;

	/* Right now device-tree probed devices don't get dma_mask set.
	 * Since shared usb code relies on it, set it here for now.
	 * Once we have dma capability bindings this can go away.
	 */
	ret = dma_coerce_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "Unable to get IRQ resource\n");
		return ret;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "Unable to get memory resource\n");
		return -ENODEV;
	}

	hcd = usb_create_hcd(&lpc18xx_hc_driver, &pdev->dev, dev_name(&pdev->dev));
	if (!hcd) {
		dev_err(&pdev->dev, "Unable to create HCD\n");
		return -ENOMEM;
	}

	platform_set_drvdata(pdev, hcd);
	ehci = hcd_to_ehci(hcd);
	lpc = (struct lpc18xx_ehci_hcd *)ehci->priv;

	lpc->clk_usb = devm_clk_get(&pdev->dev, "usb");
	if (IS_ERR(lpc->clk_usb)) {
		dev_err(&pdev->dev, "Can't get EHCI clock\n");
		ret = PTR_ERR(lpc->clk_usb);
		goto put_hcd;
	}

	lpc->clk_reg = devm_clk_get(&pdev->dev, "reg");
	if (IS_ERR(lpc->clk_reg)) {
		dev_err(&pdev->dev, "Can't get EHCI reg clock\n");
		ret = PTR_ERR(lpc->clk_reg);
		goto put_hcd;
	}

	hcd->rsrc_start = res->start;
	hcd->rsrc_len = resource_size(res);
	hcd->regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(hcd->regs)) {
		ret = PTR_ERR(hcd->regs);
		goto put_hcd;
	}

	ehci->caps = hcd->regs + LPC18XX_EHCI_CAPS_OFFSET;

	hcd->phy = devm_usb_get_phy_by_phandle(&pdev->dev, "usb-phy", 0);
	if (IS_ERR(hcd->phy)) {
		dev_err(&pdev->dev, "Unable to find transceiver\n");
		ret = -EPROBE_DEFER;
		goto put_hcd;
	}

	ret = clk_prepare_enable(lpc->clk_usb);
	if (ret)
		goto put_hcd;

	ret = clk_prepare_enable(lpc->clk_reg);
	if (ret)
		goto put_clk_usb;


	/* hmm, why call otg_set_x  ? */
	ret = otg_set_host(hcd->phy->otg, &hcd->self);
	if (ret < 0) {
		dev_err(&pdev->dev, "Unable to register with transceiver\n");
		goto put_clk;
	}

	ret = usb_add_hcd(hcd, irq, 0);
	if (ret) {
		dev_err(&pdev->dev, "Failed to add USB HCD\n");
		goto put_clk;
	}

	return 0;

put_clk:
	clk_disable_unprepare(lpc->clk_reg);
put_clk_usb:
	clk_disable_unprepare(lpc->clk_usb);
put_hcd:
	usb_put_hcd(hcd);

	return ret;
}

static int lpc18xx_ehci_remove(struct platform_device *pdev)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);
	struct lpc18xx_ehci_hcd *lpc = (struct lpc18xx_ehci_hcd *)hcd_to_ehci(hcd)->priv;

	otg_set_host(hcd->phy->otg, NULL);
	usb_remove_hcd(hcd);

	usb_put_hcd(hcd);

	clk_disable_unprepare(lpc->clk_usb);
	clk_disable_unprepare(lpc->clk_reg);

	return 0;
}

static const struct of_device_id lpc18xx_ehci_match[] = {
	{ .compatible = "nxp,lpc1850-ehci" },
	{ }
};
MODULE_DEVICE_TABLE(of, lpc18xx_ehci_match);

static struct platform_driver lpc18xx_ehci_driver = {
	.probe	= lpc18xx_ehci_probe,
	.remove	= lpc18xx_ehci_remove,
	.driver	= {
		.name		= "lpc18xx-ehci",
		.of_match_table	= lpc18xx_ehci_match,
	},
};

static int __init lpc18xx_ehci_init(void)
{
	if (usb_disabled())
		return -ENODEV;

	pr_info("lpc18xx-ehci: NXP On-Chip EHCI Controller\n");
	ehci_init_driver(&lpc18xx_hc_driver, &lpc18xx_echi_overrides);

	return platform_driver_register(&lpc18xx_ehci_driver);
}
module_init(lpc18xx_ehci_init);

static void __exit lpc18xx_ehci_cleanup(void)
{
	platform_driver_unregister(&lpc18xx_ehci_driver);
}
module_exit(lpc18xx_ehci_cleanup);

MODULE_AUTHOR("Joachim Eastwood <manabian@gmail.com>");
MODULE_DESCRIPTION("EHCI USB driver for LPC18xx/43xx");
MODULE_LICENSE("GPL v2");
