/*
 * NXP LPC43xx Cortex-M0 remoteproc driver
 *
 * Copyright (C) 2016 Joachim Eastwood <manabian@gmail.com>
 * Copyright (C) 2015 Ariel D'Alessandro <ariel.dalessandro@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/regmap.h>
#include <linux/remoteproc.h>
#include <linux/reset.h>

#include "remoteproc_internal.h"

/* CREG syscon registers */
#define LPC43XX_CREG_M0SUBMEMMAP	0x308
#define LPC43XX_CREG_M0SUBTXEVENT	0x314
#define LPC43XX_CREG_M0APPTXEVENT	0x400
#define LPC43XX_CREG_M0APPMEMMAP	0x404

struct lpc43xx_rproc_data {
	unsigned int txevent;
	unsigned int memmap;
	const char *fw_name;
};

struct lpc43xx_rproc_priv {
	struct reset_control *reset;
	unsigned int txevent;
	unsigned int memmap;
	struct regmap *reg;
	struct clk *clk;
};

static irqreturn_t lpc43xx_rproc_irq(int irq, void *data)
{
	struct rproc *rproc = data;
	struct lpc43xx_rproc_priv *priv = rproc->priv;

	/* Clear TXEV from remote cpu */
	regmap_write(priv->reg, priv->txevent, 0);

        /* Process incoming buffers on all our vrings */
	/*
        rproc_vq_interrupt(rproc, 0);
        rproc_vq_interrupt(rproc, 1);
	*/

	return IRQ_HANDLED;
}

static int lpc43xx_rproc_start(struct rproc *rproc)
{
	struct lpc43xx_rproc_priv *priv = rproc->priv;
	int ret;

	regmap_write(priv->reg, priv->memmap, rproc->bootaddr);

	ret = clk_prepare_enable(priv->clk);
	if (ret) {
		dev_err(&rproc->dev, "unable to enable clock\n");
		return ret;
	}

	return reset_control_deassert(priv->reset);
}

static int lpc43xx_rproc_stop(struct rproc *rproc)
{
	struct lpc43xx_rproc_priv *priv = rproc->priv;

	reset_control_assert(priv->reset);
	clk_disable_unprepare(priv->clk);

	return 0;
}

static void lpc43xx_rproc_kick(struct rproc *rproc, int vqid)
{
	asm volatile("sev");
}

static struct rproc_ops lpc43xx_rproc_ops = {
	.start = lpc43xx_rproc_start,
	.stop  = lpc43xx_rproc_stop,
	.kick  = lpc43xx_rproc_kick,
};

static int lpc43xx_rproc_probe(struct platform_device *pdev)
{
	const struct lpc43xx_rproc_data *data;
	struct lpc43xx_rproc_priv *priv;
	struct rproc *rproc;
	int irq, ret;

	data = of_device_get_match_data(&pdev->dev);
	if (!data) {
		dev_err(&pdev->dev, "error getting match data\n");
		return -EINVAL;
	}

	rproc = rproc_alloc(&pdev->dev, pdev->name, &lpc43xx_rproc_ops,
			    data->fw_name, sizeof(*priv));
	if (!rproc)
		return -ENOMEM;

	platform_set_drvdata(pdev, rproc);
	rproc->has_iommu = false;
	priv = rproc->priv;
	priv->memmap = data->memmap;
	priv->txevent = data->txevent;

	priv->reset = reset_control_get(&pdev->dev, NULL);
	if (IS_ERR(priv->reset)) {
		dev_err(&pdev->dev, "error getting reset\n");
		ret = PTR_ERR(priv->reset);
		goto put_rproc;
	}

	/* Ensure remote cpu is held in reset */
	ret = reset_control_assert(priv->reset);
	if (ret) {
		dev_err(&pdev->dev, "unable to reset cpu\n");
		goto put_rproc;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "error getting interrupt\n");
		goto put_rproc;
	}

	priv->reg = syscon_regmap_lookup_by_compatible("nxp,lpc1850-creg");
	if (IS_ERR(priv->reg)) {
		dev_err(&pdev->dev, "error getting syscon\n");
		ret = PTR_ERR(priv->reg);
		goto put_rproc;
	}

	priv->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(priv->clk)) {
		dev_err(&pdev->dev, "error getting clock\n");
		ret = PTR_ERR(priv->clk);
		goto put_rproc;
	}

        ret = devm_request_threaded_irq(&pdev->dev, irq, NULL, lpc43xx_rproc_irq,
                                        0, "lpc43xx-remoteproc", rproc);
        if (ret) {
                dev_err(&pdev->dev, "unable to setup irq handler\n");
                goto put_rproc;
        }

	ret = rproc_add(rproc);
	if (ret) {
		dev_err(&pdev->dev, "unable to add rproc\n");
		goto put_rproc;
	}

	/* Boot core */
	wait_for_completion(&rproc->firmware_loading_complete);
	if (list_empty(&rproc->rvdevs)) {
		dev_info(&pdev->dev, "booting remote cpu manually\n");
		ret = rproc_boot(rproc);
		if (ret) {
			dev_err(&pdev->dev, "failed boot remote cpu\n");
			goto put_rproc;
		}
	}

	return 0;

put_rproc:
	rproc_put(rproc);
	return ret;
}

static int lpc43xx_rproc_remove(struct platform_device *pdev)
{
	struct rproc *rproc = platform_get_drvdata(pdev);

	rproc_del(rproc);
	rproc_put(rproc);

	return 0;
}

static const struct lpc43xx_rproc_data lpc4350_data = {
	.txevent = LPC43XX_CREG_M0APPTXEVENT,
	.memmap  = LPC43XX_CREG_M0APPMEMMAP,
	.fw_name = "m0app-firmware.elf",
};

static const struct lpc43xx_rproc_data lpc4370_data = {
	.txevent = LPC43XX_CREG_M0SUBTXEVENT,
	.memmap  = LPC43XX_CREG_M0SUBMEMMAP,
	.fw_name = "m0sub-firmware.elf",
};

static const struct of_device_id lpc43xx_rproc_match[] = {
	{ .compatible = "nxp,lpc4350-rproc", .data = &lpc4350_data },
	{ .compatible = "nxp,lpc4370-rproc", .data = &lpc4370_data },
	{ }
};
MODULE_DEVICE_TABLE(of, lpc43xx_rproc_match);

static struct platform_driver lpc43xx_rproc_driver = {
	.probe = lpc43xx_rproc_probe,
	.remove = lpc43xx_rproc_remove,
	.driver = {
		.name = "lpc43xx-rproc",
		.of_match_table	= lpc43xx_rproc_match,
	},
};
module_platform_driver(lpc43xx_rproc_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("LPC43xx remote processor control driver");
