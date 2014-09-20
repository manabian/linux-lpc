/*
 * Clk driver for NXP LPC18xx/LPC43xx Configuration Registers (CREG)
 *
 * Copyright (C) 2014 Joachim Eastwood <manabian@gmail.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/mfd/syscon.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/slab.h>

#define LPC18XX_CREG_CREG0			0x004
# define LPC18XX_CREG_CREG0_EN1KHZ		BIT(0)
# define LPC18XX_CREG_CREG0_EN32KHZ		BIT(1)
# define LPC18XX_CREG_CREG0_RESET32KHZ		BIT(2)
# define LPC18XX_CREG_CREG0_PD32KHZ		BIT(3)

enum {
	CREG_CLK_1KHZ,
	CREG_CLK_32KHZ,
	CREG_CLK_MAX,
};

static const char *clk_creg_names[CREG_CLK_MAX];

#define to_clk_creg(_hw) container_of(_hw, struct clk_creg_data, hw)
struct clk_creg_data {
	struct		clk_hw hw;
	struct regmap	*reg;
};

static int clk_creg_32k_prepare(struct clk_hw *hw)
{
	struct clk_creg_data *creg = to_clk_creg(hw);
	int ret;

	ret = regmap_update_bits(creg->reg, LPC18XX_CREG_CREG0,
				 LPC18XX_CREG_CREG0_PD32KHZ |
				 LPC18XX_CREG_CREG0_RESET32KHZ, 0);

	/* Powering up the 32k oscillator takes a while */
	usleep_range(2 * USEC_PER_SEC, 3 * USEC_PER_SEC);

	return ret;
}

static void clk_creg_32k_unprepare(struct clk_hw *hw)
{
	struct clk_creg_data *creg = to_clk_creg(hw);

	regmap_update_bits(creg->reg, LPC18XX_CREG_CREG0,
			   LPC18XX_CREG_CREG0_PD32KHZ,
			   LPC18XX_CREG_CREG0_PD32KHZ);
}

static int clk_creg_32k_is_prepared(struct clk_hw *hw)
{
	struct clk_creg_data *creg = to_clk_creg(hw);
	u32 reg;

	regmap_read(creg->reg, LPC18XX_CREG_CREG0, &reg);

	return !(reg & LPC18XX_CREG_CREG0_PD32KHZ) &&
	       !(reg & LPC18XX_CREG_CREG0_RESET32KHZ);
}

static int clk_creg_32k_enable(struct clk_hw *hw)
{
	struct clk_creg_data *creg = to_clk_creg(hw);

	return regmap_update_bits(creg->reg, LPC18XX_CREG_CREG0,
				  LPC18XX_CREG_CREG0_EN32KHZ,
				  LPC18XX_CREG_CREG0_EN32KHZ);
}

static void clk_creg_32k_disable(struct clk_hw *hw)
{
	struct clk_creg_data *creg = to_clk_creg(hw);

	regmap_update_bits(creg->reg, LPC18XX_CREG_CREG0,
			   LPC18XX_CREG_CREG0_EN32KHZ, 0);
}

static int clk_creg_1k_enable(struct clk_hw *hw)
{
	struct clk_creg_data *creg = to_clk_creg(hw);

	return regmap_update_bits(creg->reg, LPC18XX_CREG_CREG0,
				  LPC18XX_CREG_CREG0_EN1KHZ,
				  LPC18XX_CREG_CREG0_EN1KHZ);
}

static void clk_creg_1k_disable(struct clk_hw *hw)
{
	struct clk_creg_data *creg = to_clk_creg(hw);

	regmap_update_bits(creg->reg, LPC18XX_CREG_CREG0,
			   LPC18XX_CREG_CREG0_EN1KHZ, 0);
}

static unsigned long clk_creg_1k_recalc_rate(struct clk_hw *hw,
					     unsigned long parent_rate)
{
	return parent_rate / 32;
}

static struct clk_ops clk_creg_32k = {
	.prepare	= clk_creg_32k_prepare,
	.unprepare	= clk_creg_32k_unprepare,
	.is_prepared	= clk_creg_32k_is_prepared,
	.enable		= clk_creg_32k_enable,
	.disable	= clk_creg_32k_disable,
};

static struct clk_ops clk_creg_1k = {
	.enable		= clk_creg_1k_enable,
	.disable	= clk_creg_1k_disable,
	.recalc_rate	= clk_creg_1k_recalc_rate,
};

static struct clk *clk_register_creg_clk(int creg_num, struct regmap *syscon,
					 const char **parent_names, int num_parents,
					 const struct clk_ops *creg_ops)
{
	struct clk_creg_data *creg_clk;
	struct clk_init_data init;
	struct clk *clk;

	creg_clk = kzalloc(sizeof(*creg_clk), GFP_KERNEL);
	if (!creg_clk)
		return ERR_PTR(-ENOMEM);

	init.name = clk_creg_names[creg_num];
	init.flags = CLK_IS_BASIC;
	init.parent_names = parent_names;
	init.num_parents = num_parents;
	init.ops = creg_ops;

	creg_clk->reg = syscon;
	creg_clk->hw.init = &init;

	clk = clk_register(NULL, &creg_clk->hw);
	if (IS_ERR(clk))
		goto err;

	return clk;

err:
	kfree(creg_clk);
	return clk;
}

static struct clk *clk_creg[CREG_CLK_MAX];
static struct clk_onecell_data clk_base_data = {
	.clks = clk_creg,
	.clk_num = CREG_CLK_MAX,
};

static void __init lpc18xx_creg_clk_init(struct device_node *np)
{
	const char *clk_32khz_parent;
	struct regmap *syscon;
	const char *name;
	int ret, i;

	syscon = syscon_regmap_lookup_by_compatible("nxp,lpc1850-creg");
	if (IS_ERR(syscon)) {
		pr_err("%s: syscon lookup failed\n", __func__);
		return;
	}

	/* Get creg clk names from DT */
	for (i = 0; i < CREG_CLK_MAX; i++) {
		ret = of_property_read_string_index(np, "clock-output-names",
						    i, &name);
		if (ret)
			break;

		clk_creg_names[i] = name;
	}

	clk_32khz_parent = of_clk_get_parent_name(np, 0);

	clk_creg[CREG_CLK_32KHZ] = clk_register_creg_clk(CREG_CLK_32KHZ, syscon,
							 &clk_32khz_parent,
							 1, &clk_creg_32k);

	clk_creg[CREG_CLK_1KHZ] = clk_register_creg_clk(CREG_CLK_1KHZ, syscon,
							&clk_creg_names[CREG_CLK_32KHZ],
							1, &clk_creg_1k);

	of_clk_add_provider(np, of_clk_src_onecell_get, &clk_base_data);
}
CLK_OF_DECLARE(lpc18xx_creg_clk, "nxp,lpc1850-creg-clks", lpc18xx_creg_clk_init);
