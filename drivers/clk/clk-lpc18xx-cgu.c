/*
 * Clk driver for NXP LPC18xx/LPC43xx Clock Generation Unit (CGU)
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
#include <linux/of.h>
#include <linux/of_address.h>

#include <dt-bindings/clock/lpc18xx-cgu.h>

/* Clock Generation Unit (CGU) registers */
#define LPC18XX_CGU_XTAL_OSC_CTRL	0x018
#define LPC18XX_CGU_PLL0USB_STAT	0x01c
#define LPC18XX_CGU_PLL0USB_CTRL	0x020
#define LPC18XX_CGU_PLL0USB_MDIV	0x024
#define LPC18XX_CGU_PLL0USB_NP_DIV	0x028
#define LPC18XX_CGU_PLL0AUDIO_STAT	0x02c
#define LPC18XX_CGU_PLL0AUDIO_CTRL	0x030
#define LPC18XX_CGU_PLL0AUDIO_MDIV	0x034
#define LPC18XX_CGU_PLL0AUDIO_NP_DIV	0x038
#define LPC18XX_CGU_PLL0AUDIO_FRAC	0x03c
#define LPC18XX_CGU_PLL1_STAT		0x040
#define LPC18XX_CGU_PLL1_CTRL		0x044
#define  LPC18XX_PLL1_CTRL_FBSEL	BIT(6)
#define  LPC18XX_PLL1_CTRL_DIRECT	BIT(7)
#define LPC18XX_CGU_IDIV_CTRL(n)	(0x048 + (n) * sizeof(u32))
#define LPC18XX_CGU_BASE_CLK(id)	(0x05c + (id) * sizeof(u32))

/* PLL0 bits common to both audio and USB0 PLL */
#define LPC18XX_PLL0_STAT_LOCK		BIT(0)
#define LPC18XX_PLL0_CTRL_PD		BIT(0)
#define LPC18XX_PLL0_CTRL_BYPASS	BIT(1)
#define LPC18XX_PLL0_CTRL_DIRECTI	BIT(2)
#define LPC18XX_PLL0_CTRL_DIRECTO	BIT(3)
#define LPC18XX_PLL0_CTRL_CLKEN		BIT(4)
#define LPC18XX_PLL0_MDIV_MDEC_MASK	0x1ffff
#define LPC18XX_PLL0_MDIV_SELP_SHIFT	17
#define LPC18XX_PLL0_MDIV_SELI_SHIFT	22
#define LPC18XX_PLL0_MSEL_MAX		BIT(15)

/* Register value that gives PLL0 post/pre dividers equal to 1 */
#define LPC18XX_PLL0_NP_DIVS_1		0x00302062

#define LPC18XX_CGU_PLLS_NUM		3
#define LPC18XX_CGU_DIVIDERS_NUM	5

enum {
	CLK_SRC_OSC32,
	CLK_SRC_IRC,
	CLK_SRC_ENET_RX_CLK,
	CLK_SRC_ENET_TX_CLK,
	CLK_SRC_GP_CLKIN,
	CLK_SRC_RESERVED1,
	CLK_SRC_OSC,
	CLK_SRC_PLL0USB,
	CLK_SRC_PLL0AUDIO,
	CLK_SRC_PLL1,
	CLK_SRC_RESERVED2,
	CLK_SRC_RESERVED3,
	CLK_SRC_IDIVA,
	CLK_SRC_IDIVB,
	CLK_SRC_IDIVC,
	CLK_SRC_IDIVD,
	CLK_SRC_IDIVE,
	CLK_SRC_MAX
};

static const char *clk_src_names[CLK_SRC_MAX] = {
	[CLK_SRC_OSC32]		= "osc32",
	[CLK_SRC_IRC]		= "irc",
	[CLK_SRC_ENET_RX_CLK]	= "enet_rx_clk",
	[CLK_SRC_ENET_TX_CLK]	= "enet_tx_clk",
	[CLK_SRC_GP_CLKIN]	= "gp_clkin",
	[CLK_SRC_RESERVED1]	= "",
	[CLK_SRC_OSC]		= "osc",
	[CLK_SRC_PLL0USB]	= "pll0usb",
	[CLK_SRC_PLL0AUDIO]	= "pll0audio",
	[CLK_SRC_PLL1]		= "pll1",
	[CLK_SRC_RESERVED2]	= "",
	[CLK_SRC_RESERVED3]	= "",
	[CLK_SRC_IDIVA]		= "idiva",
	[CLK_SRC_IDIVB]		= "idivb",
	[CLK_SRC_IDIVC]		= "idivc",
	[CLK_SRC_IDIVD]		= "idivd",
	[CLK_SRC_IDIVE]		= "idive",
};

static const char *base_clk_names[BASE_CLK_MAX];

static u32 pll0_srcs_ids[] = {
	CLK_SRC_OSC32, CLK_SRC_IRC, CLK_SRC_ENET_RX_CLK,
	CLK_SRC_ENET_TX_CLK, CLK_SRC_GP_CLKIN, CLK_SRC_OSC,
	CLK_SRC_PLL1, CLK_SRC_IDIVA, CLK_SRC_IDIVB, CLK_SRC_IDIVC,
	CLK_SRC_IDIVD, CLK_SRC_IDIVE,
};

static u32 pll1_srcs_ids[] = {
	CLK_SRC_OSC32, CLK_SRC_IRC, CLK_SRC_ENET_RX_CLK,
	CLK_SRC_ENET_TX_CLK, CLK_SRC_GP_CLKIN, CLK_SRC_OSC,
	CLK_SRC_PLL0USB, CLK_SRC_PLL0AUDIO, CLK_SRC_IDIVA,
	CLK_SRC_IDIVB, CLK_SRC_IDIVC, CLK_SRC_IDIVD, CLK_SRC_IDIVE,
};

static u32 idiva_src_ids[] = {
	CLK_SRC_OSC32, CLK_SRC_IRC, CLK_SRC_ENET_RX_CLK,
	CLK_SRC_ENET_TX_CLK, CLK_SRC_GP_CLKIN, CLK_SRC_OSC,
	CLK_SRC_PLL0USB, CLK_SRC_PLL0AUDIO, CLK_SRC_PLL1
};

static u32 idivbcde_src_ids[] = {
	CLK_SRC_OSC32, CLK_SRC_IRC, CLK_SRC_ENET_RX_CLK,
	CLK_SRC_ENET_TX_CLK, CLK_SRC_GP_CLKIN, CLK_SRC_OSC,
	CLK_SRC_PLL0AUDIO, CLK_SRC_PLL1, CLK_SRC_IDIVA,
};

static u32 base_all_srcs_ids[] = {
	CLK_SRC_OSC32, CLK_SRC_IRC, CLK_SRC_ENET_RX_CLK,
	CLK_SRC_ENET_TX_CLK, CLK_SRC_GP_CLKIN, CLK_SRC_OSC,
	CLK_SRC_PLL0USB, CLK_SRC_PLL0AUDIO, CLK_SRC_PLL1,
	CLK_SRC_IDIVA, CLK_SRC_IDIVB, CLK_SRC_IDIVC,
	CLK_SRC_IDIVD, CLK_SRC_IDIVE,
};

static u32 base_common_srcs_ids[] = {
	CLK_SRC_OSC32, CLK_SRC_IRC, CLK_SRC_ENET_RX_CLK,
	CLK_SRC_ENET_TX_CLK, CLK_SRC_GP_CLKIN, CLK_SRC_OSC,
	CLK_SRC_PLL0AUDIO, CLK_SRC_PLL1, CLK_SRC_IDIVA,
	CLK_SRC_IDIVB, CLK_SRC_IDIVC, CLK_SRC_IDIVD, CLK_SRC_IDIVE,
};

#define CLK_MUX_TABLE(t) .table = t, .table_size = ARRAY_SIZE(t)

static struct clk_gate clk_idiv_gates[LPC18XX_CGU_DIVIDERS_NUM];
static struct clk_mux  clk_idiv_muxes[LPC18XX_CGU_DIVIDERS_NUM] = {
	{CLK_MUX_TABLE(idiva_src_ids)},
	{CLK_MUX_TABLE(idivbcde_src_ids)},
	{CLK_MUX_TABLE(idivbcde_src_ids)},
	{CLK_MUX_TABLE(idivbcde_src_ids)},
	{CLK_MUX_TABLE(idivbcde_src_ids)},
};

static struct clk_divider clk_idiv_divs[LPC18XX_CGU_DIVIDERS_NUM] = {
	{.shift = 2, .width = 2},
	{.shift = 2, .width = 4},
	{.shift = 2, .width = 4},
	{.shift = 2, .width = 4},
	{.shift = 2, .width = 8},
};

static struct clk_gate clk_base_gates[BASE_CLK_MAX];
static struct clk_mux  clk_base_muxes[BASE_CLK_MAX] = {
	[BASE_SAFE_CLK]		= { /* Source can only be IRC */ },
	[BASE_USB0_CLK]		= { /* Source can only be USB0PLL */ },
	[BASE_PERIPH_CLK]	= {CLK_MUX_TABLE(base_common_srcs_ids)},
	[BASE_USB1_CLK]		= {CLK_MUX_TABLE(base_all_srcs_ids)},
	[BASE_CPU_CLK]		= {CLK_MUX_TABLE(base_common_srcs_ids)},
	[BASE_SPIFI_CLK]	= {CLK_MUX_TABLE(base_common_srcs_ids)},
	[BASE_SPI_CLK]		= {CLK_MUX_TABLE(base_common_srcs_ids)},
	[BASE_PHY_RX_CLK]	= {CLK_MUX_TABLE(base_common_srcs_ids)},
	[BASE_PHY_TX_CLK]	= {CLK_MUX_TABLE(base_common_srcs_ids)},
	[BASE_APB1_CLK]		= {CLK_MUX_TABLE(base_common_srcs_ids)},
	[BASE_APB3_CLK]		= {CLK_MUX_TABLE(base_common_srcs_ids)},
	[BASE_LCD_CLK]		= {CLK_MUX_TABLE(base_common_srcs_ids)},
	[BASE_ADCHS_CLK]	= {CLK_MUX_TABLE(base_common_srcs_ids)},
	[BASE_SDIO_CLK]		= {CLK_MUX_TABLE(base_common_srcs_ids)},
	[BASE_SSP0_CLK]		= {CLK_MUX_TABLE(base_common_srcs_ids)},
	[BASE_SSP1_CLK]		= {CLK_MUX_TABLE(base_common_srcs_ids)},
	[BASE_UART0_CLK]	= {CLK_MUX_TABLE(base_common_srcs_ids)},
	[BASE_UART1_CLK]	= {CLK_MUX_TABLE(base_common_srcs_ids)},
	[BASE_UART2_CLK]	= {CLK_MUX_TABLE(base_common_srcs_ids)},
	[BASE_UART3_CLK]	= {CLK_MUX_TABLE(base_common_srcs_ids)},
	[BASE_OUT_CLK]		= {CLK_MUX_TABLE(base_all_srcs_ids)},
	[BASE_RES1_CLK]		= {},
	[BASE_RES2_CLK]		= {},
	[BASE_RES3_CLK]		= {},
	[BASE_RES4_CLK]		= {},
	[BASE_AUDIO_CLK]	= {CLK_MUX_TABLE(base_common_srcs_ids)},
	[BASE_CGU_OUT0_CLK]	= {CLK_MUX_TABLE(base_all_srcs_ids)},
	[BASE_CGU_OUT1_CLK]	= {CLK_MUX_TABLE(base_all_srcs_ids)},
};

struct clk_lpc_pll {
	struct		clk_hw hw;
	void __iomem	*reg;
	spinlock_t	*lock;
	u8		flags;
};

#define to_lpc_pll(hw) container_of(hw, struct clk_lpc_pll, hw)
static struct clk_lpc_pll clk_lpc_plls[LPC18XX_CGU_PLLS_NUM];

static struct clk_gate clk_pll_gates[LPC18XX_CGU_PLLS_NUM];
static struct clk_mux  clk_pll_muxes[LPC18XX_CGU_PLLS_NUM] = {
	{CLK_MUX_TABLE(pll0_srcs_ids), .mask = 0x1f, .shift = 24},
	{CLK_MUX_TABLE(pll0_srcs_ids), .mask = 0x1f, .shift = 24},
	{CLK_MUX_TABLE(pll1_srcs_ids), .mask = 0x1f, .shift = 24},
};

static unsigned long clk_lpc_pll1_recalc_rate(struct clk_hw *hw,
					      unsigned long parent_rate)
{
	struct clk_lpc_pll *pll = to_lpc_pll(hw);
	u16 msel, nsel, psel;
	bool direct, fbsel;
	u32 stat, ctrl;

	stat = clk_readl(pll->reg + LPC18XX_CGU_PLL1_STAT);
	ctrl = clk_readl(pll->reg + LPC18XX_CGU_PLL1_CTRL);

	direct = (ctrl & LPC18XX_PLL1_CTRL_DIRECT) ? true : false;
	fbsel = (ctrl & LPC18XX_PLL1_CTRL_FBSEL) ? true : false;

	msel = ((ctrl >> 16) & 0xff) + 1;
	nsel = ((ctrl >> 12) & 0x3) + 1;

	if (direct || fbsel)
		return msel * (parent_rate / nsel);

	psel = (ctrl >>  8) & 0x3;
	psel = 1 << psel;

	return (msel / (2 * psel)) * (parent_rate / nsel);
}

const struct clk_ops clk_lpc_pll1_ops = {
	.recalc_rate = clk_lpc_pll1_recalc_rate,
};

/* Compute PLL0 multiplier from decoded version */
static u32 lpc18xx_pll0_mdec2msel(u32 x)
{
	int i;

	switch (x) {
	case 0x18003: return 1;
	case 0x10003: return 2;
	default:
		for (i = LPC18XX_PLL0_MSEL_MAX + 1; x != 0x4000 && i > 0; i--)
			x = ((x ^ x>>14) & 1) | (x<<1 & 0x7fff);
		return i;
	}
}
/* Compute PLL0 decoded multiplier from binary version */
static u32 lpc18xx_pll0_msel2mdec(u32 msel)
{
	u32 i, x = 0x4000;

	switch (msel) {
	case 0: return 0;
	case 1: return 0x18003;
	case 2: return 0x10003;
	default:
		for (i = msel; i <= LPC18XX_PLL0_MSEL_MAX; i++)
			x = ((x ^ x>>1) & 1) << 14 | (x>>1 & 0xffff);
		return x;
	}
}

/* Compute PLL0 bandwidth SELI reg from multiplier */
static u32 lpc18xx_pll0_msel2seli(u32 msel)
{
	u32 tmp;

	if (msel > 16384) return 1;
	if (msel >  8192) return 2;
	if (msel >  2048) return 4;
	if (msel >=  501) return 8;
	if (msel >=   60) {
		tmp = 1024 / (msel + 9);
		return ((1024 == (tmp*(msel+9))) == 0) ? tmp*4 : (tmp+1)*4;
	}

	return (msel & 0x3c) + 4;
}

/* Compute PLL0 bandwidth SELP reg from multiplier */
static u32 lpc18xx_pll0_msel2selp(u32 msel)
{
	if (msel < 60)
		return (msel >> 1) + 1;

	return 31;
}

static unsigned long clk_lpc_pll0_usb_recalc_rate(struct clk_hw *hw,
						  unsigned long parent_rate)
{
	struct clk_lpc_pll *pll = to_lpc_pll(hw);
	u32 ctrl, mdiv, msel, npdiv;

	ctrl = clk_readl(pll->reg + LPC18XX_CGU_PLL0USB_CTRL);
	mdiv = clk_readl(pll->reg + LPC18XX_CGU_PLL0USB_MDIV);
	npdiv = clk_readl(pll->reg + LPC18XX_CGU_PLL0USB_NP_DIV);

	if (ctrl & LPC18XX_PLL0_CTRL_BYPASS)
		return parent_rate;

	if (npdiv != LPC18XX_PLL0_NP_DIVS_1) {
		pr_warn("%s: pre/post dividers unsupported\n", __func__);
		return 0;
	}

	msel = lpc18xx_pll0_mdec2msel(mdiv & LPC18XX_PLL0_MDIV_MDEC_MASK);
	if (msel)
		return 2 * msel * parent_rate;

	pr_warn("%s: unable to calculate rate\n", __func__);

	return 0;
}

static long clk_lpc_pll0_usb_round_rate(struct clk_hw *hw, unsigned long rate,
					unsigned long *prate)
{
	unsigned long m;

	if (*prate < rate) {
		pr_warn("%s: pll dividers not supported\n", __func__);
		return -EINVAL;
	}

	m = DIV_ROUND_UP_ULL(*prate, rate * 2);
	if (m <= 0 && m > LPC18XX_PLL0_MSEL_MAX) {
		pr_warn("%s: unable to support rate %lu\n", __func__, rate);
		return -EINVAL;
	}

	return 2 * *prate * m;
}

static int clk_lpc_pll0_usb_set_rate(struct clk_hw *hw, unsigned long rate,
				     unsigned long parent_rate)
{
	struct clk_lpc_pll *pll = to_lpc_pll(hw);
	u32 ctrl, stat, m;
	int retry = 3;

	if (parent_rate < rate) {
		pr_warn("%s: pll dividers not supported\n", __func__);
		return -EINVAL;
	}

	m = DIV_ROUND_UP_ULL(parent_rate, rate * 2);
	if (m <= 0 && m > LPC18XX_PLL0_MSEL_MAX) {
		pr_warn("%s: unable to support rate %lu\n", __func__, rate);
		return -EINVAL;
	}

	m  = lpc18xx_pll0_msel2mdec(m);
	m |= lpc18xx_pll0_msel2selp(m) << LPC18XX_PLL0_MDIV_SELP_SHIFT;
	m |= lpc18xx_pll0_msel2seli(m) << LPC18XX_PLL0_MDIV_SELI_SHIFT;

	/* Power down PLL, disable clk output and dividers */
	ctrl = clk_readl(pll->reg + LPC18XX_CGU_PLL0USB_CTRL);
	ctrl |= LPC18XX_PLL0_CTRL_PD;
	ctrl &= ~(LPC18XX_PLL0_CTRL_BYPASS | LPC18XX_PLL0_CTRL_DIRECTI |
		  LPC18XX_PLL0_CTRL_DIRECTO | LPC18XX_PLL0_CTRL_CLKEN);
	clk_writel(ctrl, pll->reg + LPC18XX_CGU_PLL0USB_CTRL);

	/* Configure new PLL settings */
	clk_writel(m, pll->reg + LPC18XX_CGU_PLL0USB_MDIV);
	clk_writel(LPC18XX_PLL0_NP_DIVS_1, pll->reg + LPC18XX_CGU_PLL0USB_NP_DIV);

	/* Power up PLL and wait for lock */
	ctrl &= ~LPC18XX_PLL0_CTRL_PD;
	clk_writel(ctrl, pll->reg + LPC18XX_CGU_PLL0USB_CTRL);
	do {
		udelay(10);
		stat = clk_readl(pll->reg + LPC18XX_CGU_PLL0USB_STAT);
		if (stat & LPC18XX_PLL0_STAT_LOCK) {
			ctrl |= LPC18XX_PLL0_CTRL_CLKEN;
			clk_writel(ctrl, pll->reg + LPC18XX_CGU_PLL0USB_CTRL);

			return 0;
		}
	} while (retry--);

	pr_warn("%s: unable to lock pll\n", __func__);

	return -EINVAL;
}

static const struct clk_ops clk_lpc_pll0_usb_ops = {
	.recalc_rate	= clk_lpc_pll0_usb_recalc_rate,
	.round_rate	= clk_lpc_pll0_usb_round_rate,
	.set_rate	= clk_lpc_pll0_usb_set_rate,
};

static void __init lpc18xx_fill_parent_names(const char **parent, u32 *id,
					     int size)
{
	int i;

	for (i = 0; i < size; i++)
		parent[i] = clk_src_names[id[i]];
}

static void __init lpc18xx_cgu_register_clk_sources(void __iomem *base,
						    struct device_node *np)
{
	const char *parent_name[CLK_SRC_MAX];
	struct clk *clk;
	int i;

	/* Register the internal 12 MHz RC oscillator (IRC) */
	clk = clk_register_fixed_rate(NULL, clk_src_names[CLK_SRC_IRC],
				      NULL, CLK_IS_ROOT, 12000000);

	/* Register crystal oscillator controlller */
	parent_name[0] = of_clk_get_parent_name(np, 0);
	clk = clk_register_gate(NULL, clk_src_names[CLK_SRC_OSC], parent_name[0],
				0, base + LPC18XX_CGU_XTAL_OSC_CTRL,
				0, CLK_GATE_SET_TO_DISABLE, NULL);

	/* Register PLL0 for USB */
	lpc18xx_fill_parent_names(parent_name, clk_pll_muxes[0].table,
				  clk_pll_muxes[0].table_size);
	clk_pll_muxes[0].reg = LPC18XX_CGU_PLL0USB_CTRL + base;
	clk_pll_gates[0].reg = LPC18XX_CGU_PLL0USB_CTRL + base;
	clk_pll_gates[0].flags = CLK_GATE_SET_TO_DISABLE;
	clk_lpc_plls[0].reg = base;
	clk = clk_register_composite(NULL, clk_src_names[CLK_SRC_PLL0USB],
				     parent_name, clk_pll_muxes[0].table_size,
				     &clk_pll_muxes[0].hw, &clk_mux_ops,
				     &clk_lpc_plls[0].hw,  &clk_lpc_pll0_usb_ops,
				     &clk_pll_gates[0].hw, &clk_gate_ops, 0);

	/* Register PLL0 for audio */
	lpc18xx_fill_parent_names(parent_name, clk_pll_muxes[1].table,
				  clk_pll_muxes[1].table_size);
	clk_pll_muxes[1].reg = LPC18XX_CGU_PLL0AUDIO_CTRL + base;
	clk_pll_gates[1].reg = LPC18XX_CGU_PLL0AUDIO_CTRL + base;
	clk_pll_gates[1].flags = CLK_GATE_SET_TO_DISABLE;
	clk_lpc_plls[1].reg = base;
	clk = clk_register_composite(NULL, clk_src_names[CLK_SRC_PLL0AUDIO],
				     parent_name, clk_pll_muxes[1].table_size,
				     &clk_pll_muxes[1].hw, &clk_mux_ops,
				     NULL,  NULL,
				     &clk_pll_gates[1].hw, &clk_gate_ops, 0);

	/* Register main PLL1 */
	lpc18xx_fill_parent_names(parent_name, clk_pll_muxes[2].table,
				  clk_pll_muxes[2].table_size);
	clk_pll_muxes[2].reg = LPC18XX_CGU_PLL1_CTRL + base;
	clk_pll_gates[2].reg = LPC18XX_CGU_PLL1_CTRL + base;
	clk_pll_gates[2].flags = CLK_GATE_SET_TO_DISABLE;
	clk_lpc_plls[2].reg = base;
	clk = clk_register_composite(NULL, clk_src_names[CLK_SRC_PLL1],
				     parent_name, clk_pll_muxes[2].table_size,
				     &clk_pll_muxes[2].hw, &clk_mux_ops,
				     &clk_lpc_plls[2].hw,  &clk_lpc_pll1_ops,
				     &clk_pll_gates[2].hw, &clk_gate_ops, 0);

	/* Register dividers A-E */
	for (i = 0; i < LPC18XX_CGU_DIVIDERS_NUM; i++) {
		lpc18xx_fill_parent_names(parent_name, clk_idiv_muxes[i].table,
					  clk_idiv_muxes[i].table_size);

		clk_idiv_divs[i].reg = LPC18XX_CGU_IDIV_CTRL(i) + base;
		clk_idiv_gates[i].reg = LPC18XX_CGU_IDIV_CTRL(i) + base;
		clk_idiv_gates[i].flags = CLK_GATE_SET_TO_DISABLE;
		clk_idiv_muxes[i].reg = LPC18XX_CGU_IDIV_CTRL(i) + base;
		clk_idiv_muxes[i].mask = 0x1f;
		clk_idiv_muxes[i].shift = 24;

		clk = clk_register_composite(NULL, clk_src_names[CLK_SRC_IDIVA + i],
					     parent_name, clk_idiv_muxes[i].table_size,
					     &clk_idiv_muxes[i].hw, &clk_mux_ops,
					     &clk_idiv_divs[i].hw,  &clk_divider_ops,
					     &clk_idiv_gates[i].hw, &clk_gate_ops, 0);
	}
}

static struct clk *clk_base[BASE_CLK_MAX];
static struct clk_onecell_data clk_base_data = {
	.clks = clk_base,
	.clk_num = BASE_CLK_MAX,
};

static void __init lpc18xx_cgu_register_base_clks(void __iomem *base)
{
	const char *parent_name[CLK_SRC_MAX];
	int i;

	/* Register base safe clk as a gate since parent is always IRC */
	clk_base[BASE_SAFE_CLK] =
		clk_register_fixed_rate(NULL, base_clk_names[BASE_SAFE_CLK],
					clk_src_names[CLK_SRC_IRC], 0, 12000000);

	/* Register base USB0 clk as a gate since parent is always PLL0 USB */
	clk_base[BASE_USB0_CLK] =
		clk_register_gate(NULL, base_clk_names[BASE_USB0_CLK],
				  clk_src_names[CLK_SRC_PLL0USB], 0,
				  LPC18XX_CGU_BASE_CLK(BASE_USB0_CLK) + base,
				  0, CLK_GATE_SET_TO_DISABLE, NULL);

	/* Register all other base clks with gate and mux */
	for (i = BASE_PERIPH_CLK; i < BASE_CLK_MAX; i++) {

		if (base_clk_names[i] == NULL) {
			clk_base[i] = ERR_PTR(-ENOENT);
			continue;
		}

		lpc18xx_fill_parent_names(parent_name, clk_base_muxes[i].table,
					  clk_base_muxes[i].table_size);

		clk_base_gates[i].reg = LPC18XX_CGU_BASE_CLK(i) + base;
		clk_base_gates[i].flags = CLK_GATE_SET_TO_DISABLE;
		clk_base_muxes[i].reg = LPC18XX_CGU_BASE_CLK(i) + base;
		clk_base_muxes[i].mask = 0x1f;
		clk_base_muxes[i].shift = 24;

		clk_base[i] =
			clk_register_composite(NULL, base_clk_names[i],
					       parent_name, clk_base_muxes[i].table_size,
					       &clk_base_muxes[i].hw, &clk_mux_ops,
					       NULL,  NULL,
					       &clk_base_gates[i].hw, &clk_gate_ops, 0);
	}
}

static void __init lpc18xx_cgu_init(struct device_node *np)
{
	void __iomem *base;
	const char *name;
	int ret, i;
	u32 idx;

	base = of_iomap(np, 0);
	if (!base) {
		pr_warn("Failed to map address range for LPC CGU node\n");
		return;
	}

	/* Get base clk names and ids from DT */
	for (i = 0; i < BASE_CLK_MAX; i++) {
		ret = of_property_read_u32_index(np, "clock-indices", i, &idx);
		if (ret)
			break;

		ret = of_property_read_string_index(np, "clock-output-names",
						    i, &name);
		if (ret)
			break;

		if (idx < BASE_CLK_MAX)
			base_clk_names[idx] = name;
	}

	lpc18xx_cgu_register_clk_sources(base, np);
	lpc18xx_cgu_register_base_clks(base);

	of_clk_add_provider(np, of_clk_src_onecell_get, &clk_base_data);
}
CLK_OF_DECLARE(lpc18xx_cgu, "nxp,lpc1850-cgu", lpc18xx_cgu_init);
