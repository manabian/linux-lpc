/*
 * Clk driver for NXP LPC18xx/LPC43xx Clock Control Unit (CCU)
 *
 * Copyright (C) 2014 Joachim Eastwood <manabian@gmail.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/string.h>

#include <dt-bindings/clock/lpc18xx-cgu.h>
#include <dt-bindings/clock/lpc18xx-ccu.h>

/* Bit defines for CCU branch configuration register */
#define LPC18XX_CCU_RUN		BIT(0)
#define LPC18XX_CCU_AUTO	BIT(1)
#define LPC18XX_CCU_DIV		BIT(5)
#define LPC18XX_CCU_DIVSTAT	BIT(27)

/* CCU branch feature bits */
#define CCU_BRANCH_IS_BUS	BIT(0)
#define CCU_BRANCH_HAVE_DIV2	BIT(1)

#define to_clk_gate(_hw) container_of(_hw, struct clk_gate, hw)

struct lpc18xx_branch_clk_data {
	int *base_ids;
	int num_base_ids;
	spinlock_t lock;
};

struct lpc18xx_clk_branch {
	int base_id;
	const char *name;
	u16 offset;
	u16 flags;
	struct clk *clk;
	struct clk_gate gate;
};

static struct lpc18xx_clk_branch clk_branches[] = {
	{BASE_APB3_CLK, "apb3_bus",		CLK_APB3_BUS,		CCU_BRANCH_IS_BUS},
	{BASE_APB3_CLK, "apb3_i2c1",		CLK_APB3_I2C1,		0},
	{BASE_APB3_CLK, "apb3_dac",		CLK_APB3_DAC,		0},
	{BASE_APB3_CLK, "apb3_adc0",		CLK_APB3_ADC0,		0},
	{BASE_APB3_CLK, "apb3_adc1",		CLK_APB3_ADC1,		0},
	{BASE_APB3_CLK, "apb3_can0",		CLK_APB3_CAN0,		0},

	{BASE_APB1_CLK, "apb1_bus",		CLK_APB1_BUS,		CCU_BRANCH_IS_BUS},
	{BASE_APB1_CLK, "apb1_motorcon_pwm",	CLK_APB1_MOTOCON_PWM,	0},
	{BASE_APB1_CLK, "apb1_i2c0",		CLK_APB1_I2C0,		0},
	{BASE_APB1_CLK, "apb1_i2s",		CLK_APB1_I2S,		0},
	{BASE_APB1_CLK, "apb1_can1",		CLK_APB1_CAN1,		0},

	{BASE_SPIFI_CLK, "spifi",		CLK_SPIFI,		0},

	{BASE_CPU_CLK, "cpu_bus",		CLK_CPU_BUS,		CCU_BRANCH_IS_BUS},
	{BASE_CPU_CLK, "cpu_spifi",		CLK_CPU_SPIFI,		0},
	{BASE_CPU_CLK, "cpu_gpio",		CLK_CPU_GPIO,		0},
	{BASE_CPU_CLK, "cpu_lcd",		CLK_CPU_LCD,		0},
	{BASE_CPU_CLK, "cpu_ethernet",		CLK_CPU_ETHERNET,	0},
	{BASE_CPU_CLK, "cpu_usb0",		CLK_CPU_USB0,		0},
	{BASE_CPU_CLK, "cpu_emc",		CLK_CPU_EMC,		0},
	{BASE_CPU_CLK, "cpu_sdio",		CLK_CPU_SDIO,		0},
	{BASE_CPU_CLK, "cpu_dma",		CLK_CPU_DMA,		0},
	{BASE_CPU_CLK, "cpu_core",		CLK_CPU_CORE,		0},
	{BASE_CPU_CLK, "cpu_sct",		CLK_CPU_SCT,		0},
	{BASE_CPU_CLK, "cpu_usb1",		CLK_CPU_USB1,		0},
	{BASE_CPU_CLK, "cpu_emcdiv",		CLK_CPU_EMCDIV,		CCU_BRANCH_HAVE_DIV2},
	{BASE_CPU_CLK, "cpu_flasha",		CLK_CPU_FLASHA,		CCU_BRANCH_HAVE_DIV2},
	{BASE_CPU_CLK, "cpu_flashb",		CLK_CPU_FLASHB,		CCU_BRANCH_HAVE_DIV2},
	{BASE_CPU_CLK, "cpu_m0app",		CLK_CPU_M0APP,		CCU_BRANCH_HAVE_DIV2},
	{BASE_CPU_CLK, "cpu_adchs",		CLK_CPU_ADCHS,		CCU_BRANCH_HAVE_DIV2},
	{BASE_CPU_CLK, "cpu_eeprom",		CLK_CPU_EEPROM,		CCU_BRANCH_HAVE_DIV2},
	{BASE_CPU_CLK, "cpu_wwdt",		CLK_CPU_WWDT,		0},
	{BASE_CPU_CLK, "cpu_uart0",		CLK_CPU_UART0,		0},
	{BASE_CPU_CLK, "cpu_uart1",		CLK_CPU_UART1,		0},
	{BASE_CPU_CLK, "cpu_ssp0",		CLK_CPU_SSP0,		0},
	{BASE_CPU_CLK, "cpu_timer0",		CLK_CPU_TIMER0,		0},
	{BASE_CPU_CLK, "cpu_timer1",		CLK_CPU_TIMER1,		0},
	{BASE_CPU_CLK, "cpu_scu",		CLK_CPU_SCU,		0},
	{BASE_CPU_CLK, "cpu_creg",		CLK_CPU_CREG,		0},
	{BASE_CPU_CLK, "cpu_ritimer",		CLK_CPU_RITIMER,	0},
	{BASE_CPU_CLK, "cpu_uart2",		CLK_CPU_UART2,		0},
	{BASE_CPU_CLK, "cpu_uart3",		CLK_CPU_UART3,		0},
	{BASE_CPU_CLK, "cpu_timer2",		CLK_CPU_TIMER2,		0},
	{BASE_CPU_CLK, "cpu_timer3",		CLK_CPU_TIMER3,		0},
	{BASE_CPU_CLK, "cpu_ssp1",		CLK_CPU_SSP1,		0},
	{BASE_CPU_CLK, "cpu_qei",		CLK_CPU_QEI,		0},

	{BASE_PERIPH_CLK, "periph_bus",		CLK_PERIPH_BUS,		CCU_BRANCH_IS_BUS},
	{BASE_PERIPH_CLK, "periph_core",	CLK_PERIPH_CORE,	0},
	{BASE_PERIPH_CLK, "periph_sgpio",	CLK_PERIPH_SGPIO,	0},

	{BASE_USB0_CLK,  "usb0",		CLK_USB0,		0},
	{BASE_USB1_CLK,  "usb1",		CLK_USB1,		0},
	{BASE_SPI_CLK,   "spi",			CLK_SPI,		0},
	{BASE_ADCHS_CLK, "adchs",		CLK_ADCHS,		0},

	{BASE_AUDIO_CLK, "audio",		CLK_AUDIO,		0},
	{BASE_UART3_CLK, "apb2_uart3",		CLK_APB2_UART3,		0},
	{BASE_UART2_CLK, "apb2_uart2",		CLK_APB2_UART2,		0},
	{BASE_UART1_CLK, "apb0_uart1",		CLK_APB0_UART1,		0},
	{BASE_UART0_CLK, "apb0_uart0",		CLK_APB0_UART0,		0},
	{BASE_SSP1_CLK,  "apb2_ssp1",		CLK_APB2_SSP1,		0},
	{BASE_SSP0_CLK,  "apb0_ssp0",		CLK_APB0_SSP0,		0},
	{BASE_SDIO_CLK,  "sdio",		CLK_SDIO,		0},
};

static int of_clk_get_parent_arg(struct device_node *np, int index)
{
	struct of_phandle_args clkspec;
	int rc;

	if (index < 0)
		return -EINVAL;

	rc = of_parse_phandle_with_args(np, "clocks", "#clock-cells", index,
					&clkspec);
	if (rc)
		return -EINVAL;

	return clkspec.args_count ? clkspec.args[0] : -EINVAL;
}

static struct clk *lpc18xx_ccu_branch_clk_get(struct of_phandle_args *clkspec,
					      void *data)
{
	struct lpc18xx_branch_clk_data *clk_data = data;
	unsigned int offset = clkspec->args[0];
	int i, j;

	for (i = 0; i < ARRAY_SIZE(clk_branches); i++) {
		if (clk_branches[i].offset != offset)
			continue;

		for (j = 0; j < clk_data->num_base_ids; j++) {
			if (clk_data->base_ids[j] == clk_branches[i].base_id)
				return clk_branches[i].clk;
		}
	}

	pr_err("%s: invalid clock offset %d\n", __func__, offset);

	return ERR_PTR(-EINVAL);
}

static int lpc18xx_ccu_gate_endisable(struct clk_hw *hw, bool enable)
{
	struct clk_gate *gate = to_clk_gate(hw);
	unsigned long flags = 0;
	u32 val;

	if (gate->lock)
		spin_lock_irqsave(gate->lock, flags);

	/*
	 * Divider field is write only so divider stat field must
	 * be read and divider field set accordingly.
	 */
	val = clk_readl(gate->reg);
	if (val & LPC18XX_CCU_DIVSTAT)
		val |= LPC18XX_CCU_DIV;

	if (enable) {
		val |= LPC18XX_CCU_RUN;
	} else {
		/*
		 * To safely disable a branch clock a squence of two separate
		 * writes must be used. First write should set the AUTO bit
		 * and the next write should clear the RUN bit.
		 */
		val |= LPC18XX_CCU_AUTO;
		clk_writel(val, gate->reg);

		val &= ~LPC18XX_CCU_RUN;
	}

	clk_writel(val, gate->reg);

	if (gate->lock)
		spin_unlock_irqrestore(gate->lock, flags);

	return 0;
}

static int lpc18xx_ccu_gate_enable(struct clk_hw *hw)
{
	return lpc18xx_ccu_gate_endisable(hw, true);
}

static void lpc18xx_ccu_gate_disable(struct clk_hw *hw)
{
	lpc18xx_ccu_gate_endisable(hw, false);
}

static int lpc18xx_ccu_gate_is_enabled(struct clk_hw *hw)
{
	struct clk_gate *gate = to_clk_gate(hw);

	return clk_readl(gate->reg) & LPC18XX_CCU_RUN;
}

static const struct clk_ops lpc18xx_ccu_gate_ops = {
	.enable		= lpc18xx_ccu_gate_enable,
	.disable	= lpc18xx_ccu_gate_disable,
	.is_enabled	= lpc18xx_ccu_gate_is_enabled,
};

static void lpc18xx_ccu_register_branch_gate_div(struct lpc18xx_clk_branch *branch,
						 void __iomem *base,
						 const char *parent,
						 spinlock_t *lock)
{
	const struct clk_ops *div_ops = NULL;
	struct clk_divider *div = NULL;
	struct clk_hw *div_hw = NULL;

	if (branch->flags & CCU_BRANCH_HAVE_DIV2) {
		div = kzalloc(sizeof(*div), GFP_KERNEL);
		if (!div)
			return;

		div->reg = branch->offset + base;
		div->shift = 27;
		div->width = 1;
		div->lock = lock;

		div_hw = &div->hw;
		div_ops = &clk_divider_ro_ops;
	}

	branch->gate.reg = branch->offset + base;
	branch->gate.bit_idx = 0;
	branch->gate.lock = lock;

	branch->clk = clk_register_composite(NULL, branch->name, &parent, 1,
					     NULL, NULL,
					     div_hw, div_ops,
					     &branch->gate.hw, &lpc18xx_ccu_gate_ops, 0);
	if (IS_ERR(branch->clk)) {
		kfree(div);
		pr_warn("%s: failed to register %s\n", __func__, branch->name);
		return;
	}

	/* Grab essential branch clocks */
	switch (branch->offset) {
	case CLK_CPU_EMC:
	case CLK_CPU_CORE:
	case CLK_CPU_CREG:
	case CLK_CPU_EMCDIV:
		clk_prepare_enable(branch->clk);
	}
}

static void lpc18xx_ccu_register_branch_clks(void __iomem *base, int base_clk_id,
					     const char *parent,
					     spinlock_t *lock)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(clk_branches); i++) {
		if (clk_branches[i].base_id != base_clk_id)
			continue;

		lpc18xx_ccu_register_branch_gate_div(&clk_branches[i], base,
						     parent, lock);

		if (clk_branches[i].flags & CCU_BRANCH_IS_BUS)
			parent = clk_branches[i].name;
	}
}

static void __init lpc18xx_ccu_init(struct device_node *np)
{
	struct lpc18xx_branch_clk_data *clk_data;
	int num_base_ids, *base_ids;
	void __iomem *base;
	const char *parent;
	int base_clk_id;
	int i;

	base = of_iomap(np, 0);
	if (!base) {
		pr_warn("%s: Failed to map address range\n", __func__);
		return;
	}

	clk_data = kzalloc(sizeof(*clk_data), GFP_KERNEL);
	if (!clk_data)
		return;

	num_base_ids = of_clk_get_parent_count(np);

	base_ids = kcalloc(num_base_ids, sizeof(int), GFP_KERNEL);
	if (!base_ids) {
		kfree(clk_data);
		return;
	}

	clk_data->base_ids = base_ids;
	clk_data->num_base_ids = num_base_ids;
	spin_lock_init(&clk_data->lock);

	for (i = 0; i < num_base_ids; i++) {
		parent = of_clk_get_parent_name(np, i);
		base_clk_id = of_clk_get_parent_arg(np, i);
		if (base_clk_id < 0 && base_clk_id >= BASE_CLK_MAX) {
			pr_warn("%s: invalid base clk at idx %d\n", __func__, i);
			base_ids[i] = -EINVAL;
			continue;
		}

		clk_data->base_ids[i] = base_clk_id;

		lpc18xx_ccu_register_branch_clks(base, base_clk_id, parent,
						 &clk_data->lock);
	}


	of_clk_add_provider(np, lpc18xx_ccu_branch_clk_get, clk_data);
}
CLK_OF_DECLARE(lpc18xx_ccu, "nxp,lpc1850-ccu", lpc18xx_ccu_init);
