/*
 * STMAC glue for NXP LPC18xx/LPC43xx Ethernet
 *
 * Copyright (C) 2014 Joachim Eastwood <manabian@gmail.com>
 * Based on dwmac-sunxi.c, Copyright (C) 2013 Chen-Yu Tsai
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/mfd/syscon.h>
#include <linux/of.h>
#include <linux/of_net.h>
#include <linux/phy.h>
#include <linux/regmap.h>
#include <linux/stmmac.h>

#define LPC18XX_CREG_CREG6			0x12c
# define LPC18XX_CREG_CREG6_ETHMODE_MASK	0x7
# define LPC18XX_CREG_CREG6_ETHMODE_MII		0x0
# define LPC18XX_CREG_CREG6_ETHMODE_RMII	0x4

struct lpc18xx_gmac_priv_data {
	struct regmap *reg;
	int interface;
};

static void *lpc18xx_gmac_setup(struct platform_device *pdev)
{
	struct lpc18xx_gmac_priv_data *gmac;

	gmac = devm_kzalloc(&pdev->dev, sizeof(*gmac), GFP_KERNEL);
	if (!gmac)
		return ERR_PTR(-ENOMEM);

	gmac->interface = of_get_phy_mode(pdev->dev.of_node);
	if (gmac->interface < 0)
		return ERR_PTR(gmac->interface);

	gmac->reg = syscon_regmap_lookup_by_compatible("nxp,lpc1850-creg");
	if (IS_ERR(gmac->reg)) {
		dev_err(&pdev->dev, "Syscon lookup failed\n");
		return gmac->reg;
	}

	return gmac;
}

static int lpc18xx_gmac_init(struct platform_device *pdev, void *priv)
{
	struct lpc18xx_gmac_priv_data *gmac = priv;
	u8 ethmode;

	if (gmac->interface == PHY_INTERFACE_MODE_MII) {
		ethmode = LPC18XX_CREG_CREG6_ETHMODE_MII;
	} else if (gmac->interface == PHY_INTERFACE_MODE_RMII) {
		ethmode = LPC18XX_CREG_CREG6_ETHMODE_RMII;
	} else {
		dev_err(&pdev->dev, "Only MII or RMII mode supported\n");
		return -EINVAL;
	}

	regmap_update_bits(gmac->reg, LPC18XX_CREG_CREG6,
			   LPC18XX_CREG_CREG6_ETHMODE_MASK, ethmode);

	return 0;
}

const struct stmmac_of_data lpc18xx_gmac_data = {
	.has_gmac = 1,
	.setup = lpc18xx_gmac_setup,
	.init = lpc18xx_gmac_init,
};
