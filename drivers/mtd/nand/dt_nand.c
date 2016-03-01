/*
 * Generic NAND driver for DT
 *
 * Copyright (C) 2016 Joachim Eastwood <manabian@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

struct nand_dt_data {
	struct nand_chip chip;
	unsigned long ale;
	unsigned long cle;
};

static void nand_dt_cmd_ctrl(struct mtd_info *mtd, int cmd, unsigned int ctrl)
{
	struct nand_chip *chip = mtd_to_nand(mtd);
	struct nand_dt_data *nand = nand_get_controller_data(chip);
	unsigned long addr = 0;

	if (cmd == NAND_CMD_NONE)
		return;

	if (ctrl & NAND_ALE)
		addr |= nand->ale;

	if (ctrl & NAND_CLE)
		addr |= nand->cle;

	writeb(cmd, chip->IO_ADDR_W + addr);
}

static int nand_dt_probe(struct platform_device *pdev)
{
	struct nand_dt_data *nand;
	void __iomem *io_base;
	struct mtd_info *mtd;
	struct resource *res;
	u32 val;
	int ret;

	nand = devm_kzalloc(&pdev->dev, sizeof(*nand), GFP_KERNEL);
	if (!nand)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	io_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(io_base))
		return PTR_ERR(io_base);

	nand_set_flash_node(&nand->chip, pdev->dev.of_node);
	mtd = nand_to_mtd(&nand->chip);
	mtd->dev.parent = &pdev->dev;

	nand->chip.IO_ADDR_R = io_base;
	nand->chip.IO_ADDR_W = io_base;
	nand->chip.cmd_ctrl = nand_dt_cmd_ctrl;

//	nand->chip.dev_ready = pdata->ctrl.dev_ready; // get from dt? gpio

	if (!of_property_read_u32(pdev->dev.of_node, "nand-chip-delay", &val))
		nand->chip.chip_delay = val;

	if (!of_property_read_u32(pdev->dev.of_node, "nand-ale-mask", &val))
		nand->ale = val;

	if (!of_property_read_u32(pdev->dev.of_node, "nand-cle-mask", &val))
		nand->cle = val;

	platform_set_drvdata(pdev, nand);
	nand_set_controller_data(&nand->chip, nand);

	ret = nand_scan(mtd, 1);
	if (ret) {
		return ret;
	}

	ret = mtd_device_register(mtd, NULL, 0);
	if (ret) {
		nand_release(mtd);
		return ret;
	}

	return 0;
}

static int nand_dt_remove(struct platform_device *pdev)
{
	struct nand_dt_data *nand = platform_get_drvdata(pdev);

	nand_release(nand_to_mtd(&nand->chip));

	return 0;
}

static const struct of_device_id nand_dt_match[] = {
	{ .compatible = "onfi,nand" },
	{},
};
MODULE_DEVICE_TABLE(of, nand_dt_match);

static struct platform_driver nand_dt_driver = {
	.probe	= nand_dt_probe,
	.remove = nand_dt_remove,
	.driver = {
		.name		= "nand_dt",
		.of_match_table = nand_dt_match,
	},
};
module_platform_driver(nand_dt_driver);

MODULE_AUTHOR("Joachim Eastwood <manabian@gmail.com>");
MODULE_DESCRIPTION("Generic NAND driver for DT");
MODULE_LICENSE("GPL v2");
