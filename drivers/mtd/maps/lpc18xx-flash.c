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
#include <linux/sizes.h>

struct lpc18xx_flash {
	struct clk *clk;
	struct device *dev;
	void __iomem *io_base;
	void __iomem *flash_base;
	struct mtd_info mtd;

	/* For boot-rom */
	void __iomem *boot_rom;
	void (*iap_entry)(u32 *, u32 *);
};

/* Temp boot-rom stuff */
static struct resource lpc18xx_boot_rom = {
	.start	= 0x10400000,
	.end	= 0x1040ffff,
	.flags	= IORESOURCE_MEM,
};

#define LPC18XX_ROM_TABLE	0x100
#define LPC18XX_IAP_TABLE	(LPC18XX_ROM_TABLE + 0x0)

#define IAP_CMD_MAX_LEN		6
#define IAP_RES_MAX_LEN		5

/* IAP commands */
#define IAP_INIT		49
#define IAP_PREPARE		50
#define IAP_COPY_TO_FLASH	51
#define IAP_ERASE_SECTOR	52
#define IAP_BLANK_CHECK		53
#define IAP_READ_PART_ID	54
#define IAP_BOOT_CODE_VER	55
#define IAP_READ_SERIAL_NO	58
#define IAP_ERASE_PAGE		59

/* IAP return codes */
#define IAP_CMD_SUCCESS			0x00
#define IAP_SRC_ADDR_ERROR		0x02
#define IAP_DST_ADDR_ERROR		0x03
#define IAP_SRC_ADDR_NOT_MAPPED		0x04
#define IAP_DST_ADDR_NOT_MAPPED		0x05
#define IAP_COUNT_ERROR			0x06
#define IAP_INVALID_SECTOR		0x07
#define IAP_SECTOR_NOT_PREPARED		0x09
#define IAP_BUSY			0x0b
#define IAP_INVALID_FLASH_UNIT		0x14


static void lpc18xx_play_with_rom(struct lpc18xx_flash *data)
{
	u32 command[IAP_CMD_MAX_LEN];
	u32 result[IAP_RES_MAX_LEN];
	u32 ptr;

	ptr = readl(data->boot_rom + LPC18XX_IAP_TABLE);
	pr_info("%s: ptr = 0x%08x\n", __func__, ptr);

	data->iap_entry = (void *)ptr;

	command[0] = IAP_READ_PART_ID;
	data->iap_entry(command, result);

	pr_info("%s: ret: 0x%02x, res: 0x%08x, 0x%08x\n", __func__,
		result[0], result[1], result[2]);
}

static int nxp_rom_iap_read_id(struct lpc18xx_flash *data, u32 *id)
{
	u32 command[IAP_CMD_MAX_LEN];
	u32 result[IAP_RES_MAX_LEN];

	command[0] = IAP_READ_PART_ID;
	data->iap_entry(command, result);

	id[0] = result[1];
	id[1] = result[2];

	return 0;
}

static int nxp_rom_iap_init(struct lpc18xx_flash *data)
{
	u32 command[IAP_CMD_MAX_LEN];
	u32 result[IAP_RES_MAX_LEN];

	command[0] = IAP_INIT;
	data->iap_entry(command, result);

	return 0;
}

static int nxp_rom_iap_prepare_sectors(struct lpc18xx_flash *data,
				       u32 start_sector, u32 stop_sector,
				       u32 flash_bank)
{
	u32 command[IAP_CMD_MAX_LEN];
	u32 result[IAP_RES_MAX_LEN];

	command[0] = IAP_PREPARE;
	command[1] = start_sector;
	command[2] = stop_sector;
	command[3] = flash_bank;
	data->iap_entry(command, result);

	switch (result[0]) {
	case IAP_CMD_SUCCESS:
		return 0;
	case IAP_BUSY:
		return -EBUSY;
	case IAP_INVALID_SECTOR:
	default:
		break;
	}

	return -EINVAL;
}

static int nxp_rom_iap_erase_sectors(struct lpc18xx_flash *data,
				     u32 start_sector, u32 stop_sector,
				     u32 freq_khz, u32 flash_bank)
{
	u32 command[IAP_CMD_MAX_LEN];
	u32 result[IAP_RES_MAX_LEN];
	int ret;

	ret = nxp_rom_iap_prepare_sectors(data, start_sector, stop_sector,
					  flash_bank);
	if (ret)
		return ret;

	command[0] = IAP_ERASE_SECTOR;
	command[1] = start_sector;
	command[2] = stop_sector;
	command[3] = freq_khz;
	command[4] = flash_bank;
	data->iap_entry(command, result);

	switch (result[0]) {
	case IAP_CMD_SUCCESS:
		ret = 0;
		break;
	case IAP_BUSY:
		ret = -EBUSY;
		break;
	case IAP_INVALID_SECTOR:
	default:
		ret = -EINVAL;
	}

	return ret;
}

static int nxp_rom_iap_copy_to_flash(struct lpc18xx_flash *data,
				     u32 start_sector, u32 stop_sector,
				     void __iomem *dst, void *src,
				     u32 freq_khz, u32 flash_bank)
{
	u32 command[IAP_CMD_MAX_LEN];
	u32 result[IAP_RES_MAX_LEN];
	int ret;

	ret = nxp_rom_iap_prepare_sectors(data, start_sector, stop_sector,
					  flash_bank);
	if (ret)
		return ret;

	command[0] = IAP_COPY_TO_FLASH;
	command[1] = (u32)dst;
	command[2] = (u32)src;
	command[3] = freq_khz;
	data->iap_entry(command, result);
}

static struct mtd_erase_region_info lpc18xx_flash_erase_regions[] = {
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

static const unsigned int lpc18xx_flash_write_sizes[] = {512, 1024, 4096};

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

        data->mtd.writesize = 512;
        data->mtd.size = 0x80000; /* 512 kB */

        data->mtd._read  = lpc18xx_flash_read;
	data->mtd._write = lpc18xx_flash_write;
        data->mtd._erase = lpc18xx_flash_erase;

        data->mtd.writebufsize = 512;
	data->mtd.erasesize = 512;

	data->mtd.numeraseregions = ARRAY_SIZE(lpc18xx_flash_erase_regions);
	data->mtd.eraseregions = lpc18xx_flash_erase_regions;

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

	/* boot-rom */
	data->boot_rom = devm_ioremap_resource(&pdev->dev, &lpc18xx_boot_rom);
	if (IS_ERR(data->boot_rom)) {
		dev_info(&pdev->dev, "ioremap failed on boot rom\n");
		return PTR_ERR(data->boot_rom);
	}


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

	lpc18xx_play_with_rom(data);

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
