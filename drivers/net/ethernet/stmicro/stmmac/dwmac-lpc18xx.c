/*
 * STMAC glue for NXP LPC18xx/LPC43xx Ethernet
 *
 * Copyright (C) 2015 Joachim Eastwood <manabian@gmail.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_net.h>
#include <linux/phy.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/stmmac.h>

#include "stmmac_platform.h"

/* Register defines for CREG syscon */
#define LPC18XX_CREG_CREG6			0x12c
# define LPC18XX_CREG_CREG6_ETHMODE_MASK	0x7
# define LPC18XX_CREG_CREG6_ETHMODE_MII		0x0
# define LPC18XX_CREG_CREG6_ETHMODE_RMII	0x4

static int lpc18xx_dwmac_probe(struct platform_device *pdev)
{
	struct plat_stmmacenet_data *plat_dat = NULL;
	struct stmmac_resources stmmac_res;
	struct regmap *reg;
	int phy_mode;
	u8 ethmode;
	int ret;

	memset(&stmmac_res, 0, sizeof(stmmac_res));
	ret = stmmac_pltfr_get_resources(pdev, &stmmac_res);
	if (ret)
		return ret;

	phy_mode = of_get_phy_mode(pdev->dev.of_node);
	if (phy_mode < 0)
		return phy_mode;

	reg = syscon_regmap_lookup_by_compatible("nxp,lpc1850-creg");
	if (IS_ERR(reg)) {
		dev_err(&pdev->dev, "syscon lookup failed\n");
		return PTR_ERR(reg);
	}

	if (phy_mode == PHY_INTERFACE_MODE_MII) {
		ethmode = LPC18XX_CREG_CREG6_ETHMODE_MII;
	} else if (phy_mode == PHY_INTERFACE_MODE_RMII) {
		ethmode = LPC18XX_CREG_CREG6_ETHMODE_RMII;
	} else {
		dev_err(&pdev->dev, "only MII and RMII mode supported\n");
		return -EINVAL;
	}

	regmap_update_bits(reg, LPC18XX_CREG_CREG6,
			   LPC18XX_CREG_CREG6_ETHMODE_MASK, ethmode);

	ret = stmmac_probe_config_dt(pdev, plat_dat, &stmmac_res.mac);
	if (ret)
		return ret;

	return stmmac_dvr_probe(&pdev->dev, plat_dat, &stmmac_res);
}

static const struct stmmac_of_data lpc18xx_dwmac_data = {
	.has_gmac = 1,
};

static const struct of_device_id lpc18xx_dwmac_match[] = {
	{ .compatible = "nxp,lpc1850-dwmac", .data = &lpc18xx_dwmac_data },
	{ }
};
MODULE_DEVICE_TABLE(of, lpc18xx_dwmac_match);

static struct platform_driver lpc18xx_dwmac_driver = {
	.probe  = lpc18xx_dwmac_probe,
	.remove = stmmac_pltfr_remove,
	.driver = {
		.name           = "lpc18xx-dwmac",
		.pm		= &stmmac_pltfr_pm_ops,
		.of_match_table = lpc18xx_dwmac_match,
	},
};
module_platform_driver(lpc18xx_dwmac_driver);

MODULE_AUTHOR("Joachim Eastwood <manabian@gmail.com>");
MODULE_DESCRIPTION("STMMAC glue for LPC18xx/43xx");
MODULE_LICENSE("GPL v2");
