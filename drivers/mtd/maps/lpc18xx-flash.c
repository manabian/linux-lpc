/*
 * MTD driver for LPC18xx/43xx internal Flash
 *
 * Copyright (C) 2015 Joachim Eastwood <manabian@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/of.h>
#include <linux/platform_device.h>

struct lpc18xx_flash {
	struct clk *clk;
	struct device *dev;
	void __iomem *io_base;
	void __iomem *flash_base;
	struct mtd_info mtd;
};

static int lpc18xx_flash_read(struct mtd_info *mtd, loff_t from, size_t len,
			      size_t *retlen, u_char *buf)
{
	struct lpc18xx_flash *data = mtd->priv;

	dev_dbg(data->dev, "%s: from 0x%p + 0x%08x, len %zd\n", __func__, data->flash_base, (u32)from, len);

	memcpy_fromio(buf, data->flash_base + from, len);
	*retlen += len;

	return 0;
}

static int lpc18xx_flash_write(struct mtd_info *mtd, loff_t to, size_t len,
			       size_t *retlen, const u_char *buf)
{
	struct lpc18xx_flash *data = mtd->priv;

	dev_info(data->dev, "%s: to 0x%p + 0x%08x, len %zd\n", __func__, data->flash_base, (u32)to, len);

	return 0;
}

static int lpc18xx_flash_erase(struct mtd_info *mtd, struct erase_info *instr)
{
	struct lpc18xx_flash *data = mtd->priv;

	dev_info(data->dev, "%s: at 0x%llx, len %lld\n", __func__,
		(long long)instr->addr, (long long)instr->len);

	return 0;
}

static int lpc18xx_flash_setup_flash(struct lpc18xx_flash *data,
				     struct device_node *np)
{
	struct mtd_part_parser_data ppdata;
	int ret;

	data->mtd.priv = data;
	data->mtd.name = dev_name(data->dev);
        data->mtd.dev.parent = data->dev;

        data->mtd.flags = MTD_CAP_NORFLASH;
        data->mtd.flags = MTD_CAP_ROM;

        data->mtd.type = MTD_NORFLASH;
        data->mtd.type = MTD_ROM;

	data->mtd.numeraseregions = 0;
        data->mtd.writesize = 1;
        data->mtd.size = 0x80000; /* 512 kB */

        data->mtd._read  = lpc18xx_flash_read;
	data->mtd._write = lpc18xx_flash_write;
        data->mtd._erase = lpc18xx_flash_erase;

        data->mtd.writebufsize = 512;
	data->mtd.erasesize = 512;

	ppdata.of_node = np;
	ret = mtd_device_parse_register(&data->mtd, NULL, &ppdata, NULL, 0);
	if (ret) {
		dev_err(data->dev, "mtd device parse failed\n");
		return ret;
	}

	return 0;
}

static int lpc18xx_flash_probe(struct platform_device *pdev)
{
	struct lpc18xx_flash *data;
	struct resource *res;
	int ret;

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "controller");
	data->io_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(data->io_base))
		return PTR_ERR(data->io_base);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "flash");
	data->flash_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(data->flash_base))
		return PTR_ERR(data->flash_base);

	data->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(data->clk)) {
		dev_err(&pdev->dev, "flash clock not found\n");
		return PTR_ERR(data->clk);
	}

	ret = clk_prepare_enable(data->clk);
	if (ret) {
		dev_err(&pdev->dev, "unable to enable flash clock\n");
		goto dis_clk;
	}

	data->dev = &pdev->dev;
	platform_set_drvdata(pdev, data);

	ret = lpc18xx_flash_setup_flash(data, pdev->dev.of_node);
	if (ret) {
		dev_err(&pdev->dev, "unable to setup internal flash\n");
		goto dis_clk;
	}

	return 0;

dis_clk:
	clk_disable_unprepare(data->clk);
	return ret;
}

static int lpc18xx_flash_remove(struct platform_device *pdev)
{
	struct lpc18xx_flash *data = platform_get_drvdata(pdev);

	mtd_device_unregister(&data->mtd);
	clk_disable_unprepare(data->clk);

	return 0;
}

static struct of_device_id lpc18xx_flash_match[] = {
	{.compatible = "nxp,lpc1857-flash"},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, lpc18xx_flash_match);

static struct platform_driver lpc18xx_flash_driver = {
	.probe	= lpc18xx_flash_probe,
	.remove	= lpc18xx_flash_remove,
	.driver	= {
		.name = "lpc18xx-flash",
		.of_match_table = lpc18xx_flash_match,
	},
};
module_platform_driver(lpc18xx_flash_driver);

MODULE_DESCRIPTION("MTD driver for LPC18xx/43xx Flash");
MODULE_AUTHOR("Joachim Eastwood <manabian@gmail.com>");
MODULE_LICENSE("GPL v2");
