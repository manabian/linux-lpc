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
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/sizes.h>
#include <soc/nxp/nxp_boot_rom.h>

struct lpc_flash_data {
	struct mtd_erase_region_info *eraseregions;
	unsigned int numeraseregions;
	const unsigned int *writesizes;
	unsigned int numwritesizes;
};

struct lpc18xx_flash {
	struct clk *clk;
	struct device *dev;
	void __iomem *io_base;
	void __iomem *flash_base;
	struct mtd_info mtd;
	struct iap_rom *iap;

	u32 flash_bank;
	u32 cclk_khz;

	const struct lpc_flash_data *flash;
};

static const unsigned int lpc1857_flash_write_sizes[] = {512, 1024, 4096};

static struct mtd_erase_region_info lpc1857_flash_eraseregions[] = {
	{
		.offset = 0x00000,
		.erasesize = SZ_8K,
		.numblocks = 8,
	},
	{
		.offset = 0x10000,
		.erasesize = SZ_64K,
		.numblocks = 7,
	},
};

static const struct lpc_flash_data lpc1857_flash_data = {
	.eraseregions = lpc1857_flash_eraseregions,
	.numeraseregions = ARRAY_SIZE(lpc1857_flash_eraseregions),
	.writesizes = lpc1857_flash_write_sizes,
	.numwritesizes = ARRAY_SIZE(lpc1857_flash_write_sizes),
};

static int lpc18xx_flash_offset_to_sector(struct lpc18xx_flash *data,
					  unsigned int offset)
{
	unsigned int region_size, sector_offset = 0;
	struct mtd_erase_region_info *r;
	int i;

	for (i = 0; i < data->mtd.numeraseregions; i++) {
		r = &data->mtd.eraseregions[i];
		region_size = r->offset + r->erasesize * r->numblocks;

		if (offset < region_size)
			return sector_offset + (offset / r->erasesize);

		offset -= region_size;
		sector_offset += r->numblocks;
	}

	return -EINVAL;
}

static unsigned lpc18xx_flash_writelen(struct lpc18xx_flash *data, size_t len)
{
	int i;

	for (i = data->flash->numwritesizes; i-- > 0;) {
		if (len >= data->flash->writesizes[i])
			return data->flash->writesizes[i];
	}

	return data->flash->writesizes[0];
}

static int lpc18xx_flash_read(struct mtd_info *mtd, loff_t from, size_t len,
			      size_t *retlen, u_char *buf)
{
	struct lpc18xx_flash *data = mtd->priv;

	memcpy_fromio(buf, data->flash_base + from, len);
	*retlen += len;

	return 0;
}

static int lpc18xx_flash_write_one(struct lpc18xx_flash *data, loff_t to,
				   size_t len, const u_char *buf)
{
	int start_sector, end_sector;

	start_sector = lpc18xx_flash_offset_to_sector(data, to);
	if (start_sector < 0) {
		dev_err(data->dev, "%s: invalid start sector\n", __func__);
		return -EINVAL;
	}

	end_sector = lpc18xx_flash_offset_to_sector(data, to + len - 1);
	if (end_sector < 0) {
		dev_err(data->dev, "%s: invalid end sector\n", __func__);
		return -EINVAL;
	}

	dev_info(data->dev, "%s: 0x%08x, %zu\n", __func__, (u32)to, len);

	return iap_copy_to_flash(data->iap, start_sector, end_sector,
				 data->flash_base + to, buf, len,
				 data->cclk_khz, data->flash_bank);
}

static int lpc18xx_flash_write(struct mtd_info *mtd, loff_t to, size_t len,
			       size_t *retlen, const u_char *buf)
{
	struct lpc18xx_flash *data = mtd->priv;
	unsigned writelen;
	u32 rem;
	int ret;

	if (len == 0)
		return 0;

	dev_info(data->dev, "%s: to 0x%p + 0x%08x, len %zd\n", __func__, data->flash_base, (u32)to, len);

	/* Offset and length must be aligned to write size */
	div_u64_rem(to, mtd->writesize, &rem);
	if (rem)
		return -EINVAL;

	div_u64_rem(len, mtd->writesize, &rem);
	if (rem)
		return -EINVAL;

	while (len) {
		writelen = lpc18xx_flash_writelen(data, len);
		dev_info(data->dev, "%s: writelen %u\n", __func__, writelen);

		dev_info(data->dev, "%s: to 0x%p + 0x%08x, len %zd\n", __func__, data->flash_base, (u32)to, len);
		ret = lpc18xx_flash_write_one(data, to, writelen, buf);
		if (ret)
			return ret;

		to += writelen;
		buf += writelen;
		len -= writelen;
		*retlen += writelen;
	}

	return 0;
}

static int lpc18xx_flash_erase(struct mtd_info *mtd, struct erase_info *instr)
{
	struct lpc18xx_flash *data = mtd->priv;
	int start_sector, end_sector;
	unsigned int addr_end;
	int ret;

	start_sector = lpc18xx_flash_offset_to_sector(data, instr->addr);
	if (start_sector < 0) {
		dev_err(data->dev, "%s: invalid start sector\n", __func__);
		return -EINVAL;
	}

	addr_end = instr->addr + instr->len - 1;
	end_sector = lpc18xx_flash_offset_to_sector(data, addr_end);
	if (end_sector < 0) {
		dev_err(data->dev, "%s: invalid end sector\n", __func__);
		return -EINVAL;
	}

	ret = iap_erase_sectors(data->iap, start_sector, end_sector,
				data->cclk_khz, data->flash_bank);
	if (ret)
		return ret;

	instr->state = MTD_ERASE_DONE;
	mtd_erase_callback(instr);

	return 0;
}

static int lpc18xx_flash_setup(struct lpc18xx_flash *data,
			       struct device_node *np)
{
	struct mtd_part_parser_data ppdata;
	const struct nxp_lpc_part *part;
	int ret;

	data->mtd.priv = data;
	data->mtd.name = dev_name(data->dev);
	data->mtd.dev.parent = data->dev;

	data->mtd.flags = MTD_CAP_NORFLASH;
	data->mtd.type = MTD_NORFLASH;

	data->mtd.writesize = data->flash->writesizes[0];
	data->mtd.writebufsize = data->flash->writesizes[data->flash->numwritesizes - 1];

	part = iap_get_part_info(data->iap);
	data->mtd.size = part->flash_size[data->flash_bank];

	data->mtd._read  = lpc18xx_flash_read;
	data->mtd._write = lpc18xx_flash_write;
	data->mtd._erase = lpc18xx_flash_erase;

	data->mtd.eraseregions = data->flash->eraseregions;
	data->mtd.numeraseregions = data->flash->numeraseregions;

	mtd_set_of_node(&data->mtd, np);
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

	data->flash = of_device_get_match_data(&pdev->dev);
	if (!data->flash) {
		dev_err(&pdev->dev, "no match data provided\n");
		return -ENODEV;
	}

	data->iap = nxp_rom_iap_lookup(pdev->dev.of_node, "nxp,iap");
	if (IS_ERR(data->iap)) {
		if (PTR_ERR(data->iap) != -EPROBE_DEFER)
			dev_err(&pdev->dev, "iap unavailable\n");

		return PTR_ERR(data->iap);
	}

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

	data->cclk_khz = clk_get_rate(data->clk) / 1000;
	if (data->cclk_khz == 0) {
		dev_err(&pdev->dev, "unable to get clock rate\n");
		ret = -EINVAL;
		goto dis_clk;
	}

	if (of_property_read_u32(pdev->dev.of_node, "nxp,flash_bank",
				 &data->flash_bank))
		data->flash_bank = 0;

	data->dev = &pdev->dev;
	platform_set_drvdata(pdev, data);

	ret = lpc18xx_flash_setup(data, pdev->dev.of_node);
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

static const struct of_device_id lpc18xx_flash_match[] = {
	{ .compatible = "nxp,lpc1857-flash", .data = &lpc1857_flash_data },
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
