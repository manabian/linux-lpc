/*
 * Firmware driver for NXP LPC18xx/43xx boot ROM
 *
 * Copyright (C) 2016 Joachim Eastwood <manabian@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/err.h>
#include <linux/export.h>
#include <linux/io.h>
#include <linux/genalloc.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/nvmem-consumer.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/random.h>
#include <linux/regmap.h>
#include <linux/spinlock.h>
#include <linux/sys_soc.h>

#include <soc/nxp/nxp_lpc_boot_rom.h>

/*
 * The boot ROM on LPC18xx/43xx contain APIs to program
 * the internal Flash and OTP memory.
 *
 * IAP (In-Application Programming) is used to program
 * the internal Flash and to retrieve device specific
 * information like part ID, ROM version and serial
 * number. IAP is only avaliable on Flash devices.
 */

/* IAP commands */
#define IAP_INIT		49
#define IAP_PREPARE		50
#define IAP_COPY_TO_FLASH	51
#define IAP_ERASE_SECTOR	52
#define IAP_BLANK_CHECK		53
#define IAP_READ_PART_ID	54
#define IAP_BOOT_CODE_VER	55
#define IAP_READ_SERIAL_NUMBER	58
#define IAP_ERASE_PAGE		59

#define IAP_CMD_MAX_LEN		6
#define IAP_RES_MAX_LEN		5

#define IAP_BUFFER_SIZE		4096

/* IAP return codes */
#define IAP_CMD_SUCCESS		0x00
#define IAP_BUSY		0x0b

/* IAP device information */
#define IAP_BOOT_CODE_VER_MAJOR(v)	((v >> 16) & 0xff)
#define IAP_BOOT_CODE_VER_MINOR(v)	(v & 0xff)
#define IAP_BOOT_CODE_VER_SIZE		sizeof(u32)
#define IAP_PART_ID_SIZE		(sizeof(u32) * 2)
#define IAP_SERIAL_NUMBER_SIZE		(sizeof(u32) * 4)

#define LPC18XX_OTP_PART_ID_SIZE	(sizeof(u32) * 4)

/* LPC18xx/43xx ROM addresses */
#define LPC18XX_ROM_TABLE		0x100
#define LPC18XX_IAP_TABLE		(LPC18XX_ROM_TABLE + 0x0)
#define LPC18XX_ROM_VERSION		0x7ffc

/* LPC18xx/43xx CREG (syscon) defines */
#define LPC18XX_CREG_CHIPID		0x200
#define LPC18XX_FLASH_CHIPID0		0x4284e02b
#define LPC18XX_FLASH_CHIPID1		0x7284e02b
#define LPC18XX_FLASHLESS_CHIPID0	0x5284e02b
#define LPC18XX_FLASHLESS_CHIPID1	0x6284e02b
#define LPC43XX_FLASH_CHIPID0		0x4906002b
#define LPC43XX_FLASH_CHIPID1		0x7906002b
#define LPC43XX_FLASHLESS_CHIPID0	0x5906002b
#define LPC43XX_FLASHLESS_CHIPID1	0x6906002b

#define NXP_PART_LPC(_num, _id0, _id1, _sz0, _sz1)	\
	{						\
		.name = "LPC"#_num,			\
		.id[0] = _id0, .id[1] = _id1,		\
		.flash_size[0] = _sz0 * 1024,		\
		.flash_size[1] = _sz1 * 1024,		\
	}

static const struct nxp_lpc_part nxp_lpc_parts[] = {
	/* LPC18xx Flashless parts */
	NXP_PART_LPC(1850,  0xf000d830, 0x00,   0,   0),
	NXP_PART_LPC(18S50, 0xf000d860, 0x00,   0,   0),
	NXP_PART_LPC(1830,  0xf000da30, 0x00,   0,   0),
	NXP_PART_LPC(18S30, 0xf000da60, 0x00,   0,   0),
	NXP_PART_LPC(1820,  0xf00adb3c, 0x00,   0,   0),
	NXP_PART_LPC(18S20, 0xf00adb6c, 0x00,   0,   0),
	NXP_PART_LPC(1810,  0xf00b5b3f, 0x00,   0,   0),
	NXP_PART_LPC(18S10, 0xf00b5b6f, 0x00,   0,   0),
	/* LPC18xx Flash parts */
	NXP_PART_LPC(1857,  0xf001d830, 0x00, 512, 512),
	NXP_PART_LPC(18S57, 0xf001d860, 0x00, 512, 512),
	NXP_PART_LPC(1853,  0xf001d830, 0x44, 256, 256),
	NXP_PART_LPC(1837,  0xf001da30, 0x00, 512, 512),
	NXP_PART_LPC(18S37, 0xf001d860, 0x00, 512, 512),
	NXP_PART_LPC(1833,  0xf001da30, 0x44, 256, 256),
	NXP_PART_LPC(1827,  0xf001db3c, 0x00, 512, 512),
	NXP_PART_LPC(1825,  0xf001db3c, 0x22, 384, 384),
	NXP_PART_LPC(1823,  0xf00bdb3c, 0x44, 256, 256),
	NXP_PART_LPC(1822,  0xf00bdb3c, 0x80, 512,   0),
	NXP_PART_LPC(1817,  0xf001db3f, 0x00, 512, 512),
	NXP_PART_LPC(1815,  0xf001db3f, 0x22, 384, 384),
	NXP_PART_LPC(1813,  0xf00bdb3f, 0x44, 256, 256),
	NXP_PART_LPC(1812,  0xf00bdb3f, 0x80, 512,   0),
	/* LPC43xx Flashless parts */
	NXP_PART_LPC(4370,  0x00000030, 0x00,   0,   0), /* LBGA256 */
	NXP_PART_LPC(4370,  0x00000230, 0x00,   0,   0), /* TFBGA100 */
	NXP_PART_LPC(43S70, 0x00000060, 0x00,   0,   0),
	NXP_PART_LPC(4350,  0xa0000830, 0x00,   0,   0),
	NXP_PART_LPC(43S50, 0xa0000860, 0x00,   0,   0),
	NXP_PART_LPC(4330,  0xa0000a30, 0x00,   0,   0),
	NXP_PART_LPC(43S30, 0xa0000a60, 0x00,   0,   0),
	NXP_PART_LPC(4320,  0xa000cb3c, 0x00,   0,   0),
	NXP_PART_LPC(43S20, 0xa000cb6c, 0x00,   0,   0),
	NXP_PART_LPC(4310,  0xa00acb3f, 0x00,   0,   0),
	/* LPC43xx parts with Flash */
	NXP_PART_LPC(4367,  0x8001c030, 0x00, 512, 512),
	NXP_PART_LPC(43S67, 0x8001c060, 0x00, 512, 512),
	NXP_PART_LPC(4357,  0xa001c830, 0x00, 512, 512),
	NXP_PART_LPC(43S57, 0xa001c860, 0x00, 512, 512), /* LBGA256 */
	NXP_PART_LPC(43S57, 0xa001ca60, 0x00, 512, 512), /* LQFP208 */
	NXP_PART_LPC(4353,  0xa001c830, 0x44, 256, 256),
	NXP_PART_LPC(4337,  0xa001ca30, 0x00, 512, 512),
	NXP_PART_LPC(43S37, 0xa001ca60, 0x00, 512, 512),
	NXP_PART_LPC(4333,  0xa001ca30, 0x44, 256, 256),
	NXP_PART_LPC(4327,  0xa001cb3c, 0x00, 512, 512),
	NXP_PART_LPC(4325,  0xa001cb3c, 0x22, 384, 384),
	NXP_PART_LPC(4323,  0xa00bcb3c, 0x44, 256, 256),
	NXP_PART_LPC(4322,  0xa00bcb3c, 0x80, 512,   0),
	NXP_PART_LPC(4317,  0xa001cb3f, 0x00, 512, 512),
	NXP_PART_LPC(4315,  0xa001cb3f, 0x22, 384, 384),
	NXP_PART_LPC(4313,  0xa00bcb3f, 0x44, 256, 256),
	NXP_PART_LPC(4312,  0xa00bcb3f, 0x80, 512,   0),
};

struct iap_rom {
	void (*entry)(u32 *, u32 *);
};

struct otp_rom {
	/* place holder */
};

struct nxp_rom_api {
	struct device *dev;
	void __iomem *rom;

	bool has_iap;
	struct iap_rom iap;
	spinlock_t lock;

	const struct nxp_lpc_part *part;
	const char *partname;
	u32 boot_version;

	struct soc_device *soc_dev;
	struct soc_device_attribute soc_dev_attr;

	struct gen_pool *sram;
	unsigned long buf_addr;
};

static const struct nxp_lpc_part *nxp_lpc_boot_rom_find_part(u32 *id, bool flash)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(nxp_lpc_parts); i++) {
		if (nxp_lpc_parts[i].id[0] == id[0]) {
			if (!flash || ((nxp_lpc_parts[i].id[1] & 0xff) == id[1]))
				return &nxp_lpc_parts[i];
		}
	}

	return NULL;
}

static inline void iap_entry(struct nxp_rom_api *data, u32 *cmd, u32 *res)
{
	unsigned long flags;

	spin_lock_irqsave(&data->lock, flags);
	data->iap.entry(cmd, res);
	spin_unlock_irqrestore(&data->lock, flags);
}

static int iap_init(struct nxp_rom_api *data)
{
	u32 command[IAP_CMD_MAX_LEN];
	u32 result[IAP_RES_MAX_LEN];

	command[0] = IAP_INIT;
	iap_entry(data, command, result);

	return 0;
}

static int iap_read_id(struct nxp_rom_api *data, u32 *id)
{
	u32 command[IAP_CMD_MAX_LEN];
	u32 result[IAP_RES_MAX_LEN];

	command[0] = IAP_READ_PART_ID;
	iap_entry(data, command, result);

	id[0] = result[1];
	id[1] = result[2];

	return 0;
}

static int iap_read_boot_version(struct nxp_rom_api *data, u32 *version)
{
	u32 command[IAP_CMD_MAX_LEN];
	u32 result[IAP_RES_MAX_LEN];

	command[0] = IAP_BOOT_CODE_VER;
	iap_entry(data, command, result);

	*version = result[1];

	return 0;
}

static int iap_read_serial_number(struct nxp_rom_api *data, u32 *serial)
{
	u32 command[IAP_CMD_MAX_LEN];
	u32 result[IAP_RES_MAX_LEN];

	command[0] = IAP_READ_SERIAL_NUMBER;
	iap_entry(data, command, result);

	serial[0] = result[1];
	serial[1] = result[2];
	serial[2] = result[3];
	serial[3] = result[4];

	return 0;
}

static int iap_prepare_sectors(struct nxp_rom_api *data, u32 start_sector,
			       u32 end_sector, u32 flash_bank)
{
	u32 command[IAP_CMD_MAX_LEN];
	u32 result[IAP_RES_MAX_LEN];

	if (!data->part->flash_size)
		return -ENODEV;

	command[0] = IAP_PREPARE;
	command[1] = start_sector;
	command[2] = end_sector;
	command[3] = flash_bank;
	data->iap.entry(command, result);

	if (result[0] == IAP_CMD_SUCCESS)
		return 0;

	dev_warn(data->dev, "%s: IAP command failed with 0x%08x\n", __func__,
		 result[0]);

	if (result[0] == IAP_BUSY)
		return -EBUSY;

	return -EINVAL;
}

int iap_erase_sectors(struct iap_rom *iap, u32 start_sector, u32 end_sector,
		      u32 freq_khz, u32 flash_bank)
{
	struct nxp_rom_api *data = container_of(iap, struct nxp_rom_api, iap);
	u32 command[IAP_CMD_MAX_LEN];
	u32 result[IAP_RES_MAX_LEN];
	unsigned long flags;
	int ret;


	spin_lock_irqsave(&data->lock, flags);
	ret = iap_prepare_sectors(data, start_sector, end_sector, flash_bank);
	if (ret) {
		spin_unlock_irqrestore(&data->lock, flags);
		return ret;
	}

	command[0] = IAP_ERASE_SECTOR;
	command[1] = start_sector;
	command[2] = end_sector;
	command[3] = freq_khz;
	command[4] = flash_bank;
	iap->entry(command, result);
	spin_unlock_irqrestore(&data->lock, flags);

	if (result[0] == IAP_CMD_SUCCESS)
		return 0;

	dev_warn(data->dev, "%s: IAP command failed with 0x%08x\n", __func__,
		 result[0]);

	if (result[0] == IAP_BUSY)
		return -EBUSY;

	return -EINVAL;
}
EXPORT_SYMBOL_GPL(iap_erase_sectors);

int iap_copy_to_flash(struct iap_rom *iap, u32 start_sector, u32 end_sector,
		      void __iomem *dst, const void *src, u32 size,
		      u32 freq_khz, u32 flash_bank)
{
	struct nxp_rom_api *data = container_of(iap, struct nxp_rom_api, iap);
	u32 command[IAP_CMD_MAX_LEN];
	u32 result[IAP_RES_MAX_LEN];
	unsigned long flags;
	int ret;

	if (size > IAP_BUFFER_SIZE)
		return -EINVAL;

	spin_lock_irqsave(&data->lock, flags);
	ret = iap_prepare_sectors(data, start_sector, end_sector, flash_bank);
	if (ret) {
		spin_unlock_irqrestore(&data->lock, flags);
		return ret;
	}

	memcpy((void *)data->buf_addr, src, size);

	command[0] = IAP_COPY_TO_FLASH;
	command[1] = (unsigned long)dst;
	command[2] = data->buf_addr;
	command[3] = size;
	command[4] = freq_khz;
	dev_info(data->dev, "%s: cmd[0] = 0x%08x, cmd[1] = 0x%08x, cmd[2] = 0x%08x, cmd[3] = 0x%08x, cmd[4] = 0x%08x\n", __func__, command[0], command[1], command[2], command[3], command[4]);
	iap->entry(command, result);
	spin_unlock_irqrestore(&data->lock, flags);

	if (result[0] == IAP_CMD_SUCCESS)
		return 0;

	dev_warn(data->dev, "%s: IAP command failed with 0x%08x\n", __func__,
		 result[0]);

	if (result[0] == IAP_BUSY)
		return -EBUSY;

	return -EINVAL;
}
EXPORT_SYMBOL_GPL(iap_copy_to_flash);

const struct nxp_lpc_part *iap_get_part_info(struct iap_rom *iap)
{
	struct nxp_rom_api *data = container_of(iap, struct nxp_rom_api, iap);
	return data->part;
}
EXPORT_SYMBOL_GPL(iap_get_part_info);

struct iap_rom *nxp_rom_iap_lookup(struct device_node *np, const char *prop)
{
	struct nxp_rom_api *data = NULL;
	struct platform_device *pdev;
	struct device_node *fw_node;

	if (prop)
		fw_node = of_parse_phandle(np, prop, 0);
	else
		fw_node = np;

	if (!fw_node)
		return ERR_PTR(-ENODEV);

	pdev = of_find_device_by_node(fw_node);
	of_node_put(fw_node);
	if (!pdev)
		return ERR_PTR(-ENODEV);

	data = platform_get_drvdata(pdev);
	if (data && data->has_iap)
		return &data->iap;

	return ERR_PTR(-EPROBE_DEFER);
}
EXPORT_SYMBOL_GPL(nxp_rom_iap_lookup);

static int nxp_get_flashless_part_id(struct nxp_rom_api *data)
{
	size_t id_len = LPC18XX_OTP_PART_ID_SIZE;
	struct nvmem_cell *cell;
	u32 *id;

	cell = of_nvmem_cell_get(data->dev->of_node, "PartID");
	if (IS_ERR(cell))
		return PTR_ERR(cell);

	id = nvmem_cell_read(cell, &id_len);
	nvmem_cell_put(cell);

	if (IS_ERR(id)) {
		dev_err(data->dev, "unable to read part id from nvmem");
		return PTR_ERR(id);
	}

	data->part = nxp_lpc_boot_rom_find_part(id, false);
	if (!data->part)
		dev_warn(data->dev, "unknown LPC part\n");
	else
		data->partname = data->part->name;

	return 0;
}

static int nxp_lpc_boot_rom_iap_setup(struct nxp_rom_api *data)
{
	u32 ser[IAP_SERIAL_NUMBER_SIZE];
	unsigned long rom_pointer;
	u32 id[IAP_PART_ID_SIZE];

	rom_pointer = readl(data->rom + LPC18XX_IAP_TABLE);
	data->iap.entry = (void *)rom_pointer;

	iap_read_id(data, id);
	data->part = nxp_lpc_boot_rom_find_part(id, true);
	if (!data->part) {
		dev_err(data->dev, "unknown LPC part\n");
		return -ENODEV;
	}

	data->partname = data->part->name;

	iap_read_boot_version(data, &data->boot_version);

	iap_read_serial_number(data, ser);
	add_device_randomness(ser, sizeof(ser));
	data->soc_dev_attr.soc_id = devm_kasprintf(data->dev, GFP_KERNEL,
						   "%08x%08x%08x%08x", ser[0],
						   ser[1], ser[2], ser[3]);

	data->sram = of_gen_pool_get(data->dev->of_node, "sram", 0);
	if (!data->sram) {
		dev_err(data->dev, "sram unavailable\n");
		return -ENODEV;
	}

	data->buf_addr = gen_pool_alloc(data->sram, IAP_BUFFER_SIZE);
	if (!data->buf_addr) {
		dev_err(data->dev, "sram memory full\n");
		return -ENOMEM;
	}

	iap_init(data);

	return 0;
}

/*
 * To determin if IAP is available on a LPC18xx/43xx device the
 * chip ID in the CREG block must be checked. IAP is only
 * available on devices with internal Flash memory.
 *
 * The part ID can be retrieved using IAP on Flash devices while
 * Flashless devices has the part ID in OTP. Note that all
 * LPC18xx/43xx parts have OTP API for writing in boot ROM.
 */
static int lpc18xx_boot_rom_setup(struct nxp_rom_api *data)
{
	struct regmap *syscon;
	u32 reg;

	syscon = syscon_regmap_lookup_by_phandle(data->dev->of_node, "syscon");
	if (IS_ERR(syscon)) {
		dev_err(data->dev, "unable to get syscon\n");
		return PTR_ERR(syscon);
	}

	regmap_read(syscon, LPC18XX_CREG_CHIPID, &reg);
	switch (reg) {
	case LPC18XX_FLASHLESS_CHIPID0:
	case LPC18XX_FLASHLESS_CHIPID1:
		data->partname = "LPC18x0";
		break;

	case LPC43XX_FLASHLESS_CHIPID0:
	case LPC43XX_FLASHLESS_CHIPID1:
		data->partname = "LPC43x0";
		break;

	case LPC18XX_FLASH_CHIPID0:
	case LPC18XX_FLASH_CHIPID1:
	case LPC43XX_FLASH_CHIPID0:
	case LPC43XX_FLASH_CHIPID1:
		data->has_iap = true;
		return 0;

	default:
		dev_err(data->dev, "unknown chip id\n");
		return -ENODEV;
	}

	data->boot_version = readl(data->rom + LPC18XX_ROM_VERSION);

	return 0;
}

static int nxp_lpc_boot_rom_probe(struct platform_device *pdev)
{
	struct nxp_rom_api *data;
	u8 ver_major, ver_minor;
	struct resource *res;
	int ret;

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	data->rom = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(data->rom))
		return PTR_ERR(data->rom);

	spin_lock_init(&data->lock);
	data->dev = &pdev->dev;

	ret = lpc18xx_boot_rom_setup(data);
	if (ret)
		return ret;

	if (data->has_iap) {
		ret = nxp_lpc_boot_rom_iap_setup(data);
		if (ret)
			return ret;
	} else {
		ret = nxp_get_flashless_part_id(data);
		if (ret)
			return ret;
	}

	data->soc_dev_attr.family = "NXP LPC";
	data->soc_dev_attr.machine = devm_kasprintf(&pdev->dev, GFP_KERNEL,
						    "NXP %s", data->partname);

	ver_major = IAP_BOOT_CODE_VER_MAJOR(data->boot_version);
	ver_minor = IAP_BOOT_CODE_VER_MINOR(data->boot_version);
	data->soc_dev_attr.revision = devm_kasprintf(&pdev->dev, GFP_KERNEL,
						     "%u.%u", ver_major,
						     ver_minor);

	dev_info(&pdev->dev, "%s boot code version %s\n",
		 data->soc_dev_attr.machine, data->soc_dev_attr.revision);

	data->soc_dev = soc_device_register(&data->soc_dev_attr);
	if (IS_ERR(data->soc_dev))
		return -ENODEV;

	platform_set_drvdata(pdev, data);

	return 0;
}

static int nxp_lpc_boot_rom_remove(struct platform_device *pdev)
{
	struct nxp_rom_api *data = platform_get_drvdata(pdev);

	soc_device_unregister(data->soc_dev);

	if (data->has_iap)
		gen_pool_free(data->sram, data->buf_addr, IAP_BUFFER_SIZE);

	return 0;
}

static const struct of_device_id nxp_lpc_boot_rom_match[] = {
	{.compatible = "nxp,lpc1850-boot-rom"},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, nxp_lpc_boot_rom_match);

static struct platform_driver nxp_lpc_boot_rom_driver = {
	.probe	= nxp_lpc_boot_rom_probe,
	.remove	= nxp_lpc_boot_rom_remove,
	.driver	= {
		.name = "nxp-boot-rom",
		.of_match_table = nxp_lpc_boot_rom_match,
	},
};
module_platform_driver(nxp_lpc_boot_rom_driver);

MODULE_DESCRIPTION("NXP LPC boot ROM firmware driver");
MODULE_AUTHOR("Joachim Eastwood <manabian@gmail.com>");
MODULE_LICENSE("GPL v2");
