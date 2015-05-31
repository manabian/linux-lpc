/*
 * ASoC driver for NXP LPC32xx I2S controller
 *
 * Copyright (C) 2015 Joachim Eastwood <manabian@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/clk.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <sound/dmaengine_pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc-dai.h>

/* LPC32xx register offsets and bit definitions */
#define LPC32XX_I2S_DA(id)		(0x000 + id * sizeof(u32))
#define  LPC32XX_I2S_WORDWIDTH_8BIT	0x0
#define  LPC32XX_I2S_WORDWIDTH_16BIT	0x1
#define  LPC32XX_I2S_WORDWIDTH_32BIT	0x3
#define  LPC32XX_I2S_WORDWIDTH_MASK	0x3
#define  LPC32XX_I2S_MONO		BIT(2)
#define  LPC32XX_I2S_STOP		BIT(3)
#define  LPC32XX_I2S_RESET		BIT(4)
#define  LPC32XX_I2S_WS_SEL		BIT(5)
#define LPC32XX_I2S_FIFO(id)		(0x008 + id * sizeof(u32))
#define LPC32XX_I2S_DMA(id)		(0x014 + id * sizeof(u32))
#define  LPC32XX_I2S_DMA_RX_ENABLE	BIT(0)
#define  LPC32XX_I2S_DMA_TX_ENABLE	BIT(1)
#define  LPC32XX_I2S_DMA_RX_DEPTH(lvl)	((lvl) << 8)
#define  LPC32XX_I2S_DMA_TX_DEPTH(lvl)	((lvl) << 16)
#define LPC32XX_I2S_RATE(id)		(0x020 + id * sizeof(u32))
#define  LPC32XX_I2S_RATE_Y_DIV(div)	((div) << 0)
#define  LPC32XX_I2S_RATE_X_DIV(div)	((div) << 8)
/* Registers only on newer IP blocks: */
#define LPC32XX_I2S_BITRATE(id)		(0x028 + id * sizeof(u32))
#define LPC32XX_I2S_RMODE(id)		(0x030 + id * sizeof(u32))

enum {
	I2S_TX = 0,
	I2S_RX,
};

struct lpc32xx_i2s_data {
	struct clk *clk;
	struct clk *audio_clk;
	struct regmap *regmap;
	struct regmap *syscon;
	struct snd_dmaengine_dai_dma_data capture_dma_data;
	struct snd_dmaengine_dai_dma_data playback_dma_data;
};

static int lpc32xx_i2s_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct lpc32xx_i2s_data *i2s = snd_soc_dai_get_drvdata(dai);
	u32 reg_offset = LPC32XX_I2S_DA(dai->driver->id);
	u32 val = 0;

	if ((fmt & SND_SOC_DAIFMT_INV_MASK) != SND_SOC_DAIFMT_NB_NF)
		return -EINVAL;

	if ((fmt & SND_SOC_DAIFMT_FORMAT_MASK) != SND_SOC_DAIFMT_I2S)
		return -EINVAL;

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		val |= LPC32XX_I2S_WS_SEL;
		break;
	default:
		return -EINVAL;
	}

	regmap_update_bits(i2s->regmap, reg_offset, LPC32XX_I2S_WS_SEL, val);

	return 0;
}

static int lpc32xx_i2s_trigger(struct snd_pcm_substream *substream, int cmd,
			       struct snd_soc_dai *dai)
{
	//struct lpc32xx_i2s_data *i2s = snd_soc_dai_get_drvdata(dai);
	//u32 reg_offset = LPC32XX_I2S_DA(dai->driver->id);
	//u32 val = 0;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
	default:
		return -EINVAL;
	}

	return 0;
}

static int lpc32xx_i2s_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params,
				 struct snd_soc_dai *dai)
{
	struct lpc32xx_i2s_data *i2s = snd_soc_dai_get_drvdata(dai);
	u32 reg_offset = LPC32XX_I2S_DA(dai->driver->id);
	u32 val = 0;

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S8:
		val |= LPC32XX_I2S_WORDWIDTH_8BIT;
		break;
	case SNDRV_PCM_FMTBIT_S16:
		val |= LPC32XX_I2S_WORDWIDTH_16BIT;
		break;
	case SNDRV_PCM_FMTBIT_S32:
		val |= LPC32XX_I2S_WORDWIDTH_32BIT;
		break;
	default:
		return -EINVAL;
	}

	if (params_channels(params) == 1)
		val |= LPC32XX_I2S_MONO;

	regmap_update_bits(i2s->regmap, reg_offset, LPC32XX_I2S_WORDWIDTH_MASK |
			   LPC32XX_I2S_MONO, val);

	return 0;
}

#define LPC32XX_I2S_MCLK		0
#define LPC32XX_I2S_AUDIO_CLK		1

static int lpc32xx_i2s_set_sysclk(struct snd_soc_dai *dai, int clk_id,
				  unsigned int freq, int dir)
{
	return 0;
}

static const struct snd_soc_dai_ops lpc32xx_i2s_dai_ops = {
	.set_fmt	= lpc32xx_i2s_set_fmt,
	.trigger	= lpc32xx_i2s_trigger,
	.hw_params	= lpc32xx_i2s_hw_params,
	.set_sysclk	= lpc32xx_i2s_set_sysclk,
	//.set_clkdiv
	//.set_bclk_ratio
};

static struct snd_soc_dai_driver lpc32xx_i2s_dai[] = {
	{
		.id = I2S_TX,
		.name = "lpc32xx_i2s_tx",
		.playback = {
			.channels_min = 1,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = SNDRV_PCM_FMTBIT_S8 |
				   SNDRV_PCM_FMTBIT_S16 |
				   SNDRV_PCM_FMTBIT_S32,
		},
		.ops = &lpc32xx_i2s_dai_ops,
	},
	{
		.id = I2S_RX,
		.name = "lpc32xx_i2s_rx",
		.capture = {
			.channels_min = 1,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = SNDRV_PCM_FMTBIT_S8 |
				   SNDRV_PCM_FMTBIT_S16 |
				   SNDRV_PCM_FMTBIT_S32,
		},
		.ops = &lpc32xx_i2s_dai_ops,
	},
};

static const struct snd_soc_component_driver lpc32xx_i2s_comp = {
	.name = "lpc32xx-i2s",
};

static const struct regmap_config lpc32xx_i2s_reg_config = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
	.max_register = LPC32XX_I2S_RMODE(I2S_RX),
};

static int lpc32xx_i2s_probe(struct platform_device *pdev)
{
	struct lpc32xx_i2s_data *i2s;
	struct resource *res;
	void __iomem *regs;
	int ret;

	i2s = devm_kzalloc(&pdev->dev, sizeof(*i2s), GFP_KERNEL);
	if (!i2s)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(regs))
		return PTR_ERR(regs);

	i2s->regmap = devm_regmap_init_mmio(&pdev->dev, regs,
					    &lpc32xx_i2s_reg_config);
	if (IS_ERR(i2s->regmap)) {
		dev_err(&pdev->dev, "unable to init regmap\n");
		return PTR_ERR(i2s->regmap);
	}

	i2s->clk = devm_clk_get(&pdev->dev, "i2sclk");
	if (IS_ERR(i2s->clk)) {
		dev_err(&pdev->dev, "clock not found\n");
		return PTR_ERR(i2s->clk);
	}

	i2s->audio_clk = devm_clk_get(&pdev->dev, "audioclk");
	if (IS_ERR(i2s->clk)) {
		if (PTR_ERR(i2s->clk) == -EPROBE_DEFER)
			return -EPROBE_DEFER;

		i2s->audio_clk = NULL;
	}

	ret = clk_prepare_enable(i2s->clk);
	if (ret) {
		dev_err(&pdev->dev, "unable to enable i2s clock\n");
		return ret;
	}

	/* syscon needed for mclk control  */

	/* Ensure that DMA is disabled */
	regmap_write(i2s->regmap, LPC32XX_I2S_DMA(I2S_TX), 0);
	regmap_write(i2s->regmap, LPC32XX_I2S_DMA(I2S_RX), 0);

	i2s->playback_dma_data.addr = res->start + LPC32XX_I2S_FIFO(I2S_TX);
	i2s->playback_dma_data.addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
	i2s->playback_dma_data.maxburst = 32;

	i2s->capture_dma_data.addr = res->start + LPC32XX_I2S_FIFO(I2S_RX);
	i2s->capture_dma_data.addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
	i2s->capture_dma_data.maxburst = 32;

	platform_set_drvdata(pdev, i2s);

	ret = devm_snd_soc_register_component(&pdev->dev, &lpc32xx_i2s_comp,
					      lpc32xx_i2s_dai,
					      ARRAY_SIZE(lpc32xx_i2s_dai));
	if (ret) {
		dev_err(&pdev->dev, "could not register DAI\n");
		return ret;
	}

	ret = devm_snd_dmaengine_pcm_register(&pdev->dev, NULL, 0);
	if (ret) {
		dev_err(&pdev->dev, "could not register DMA PCM\n");
		return ret;
	}

	return 0;
}

static int lpc32xx_i2s_remove(struct platform_device *pdev)
{
	struct lpc32xx_i2s_data *i2s = platform_get_drvdata(pdev);

	clk_disable_unprepare(i2s->clk);
	//clk_disable_unprepare(i2s->audio_clk);

	return 0;
}

static const struct of_device_id lpc32xx_i2s_match[] = {
	{.compatible = "nxp,lpc3220-i2s"},
	{.compatible = "nxp,lpc1850-i2s"},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, lpc32xx_i2s_match);

static struct platform_driver lpc32xx_i2s_driver = {
	.probe  = lpc32xx_i2s_probe,
	.remove = lpc32xx_i2s_remove,
	.driver = {
		.name = "lpc32xx-i2s",
		.of_match_table = lpc32xx_i2s_match,
	},
};
module_platform_driver(lpc32xx_i2s_driver);

MODULE_DESCRIPTION("NXP LPC32xx I2S driver");
MODULE_AUTHOR("Joachim Eastwood <manabian@gmail.com>");
MODULE_LICENSE("GPL v2");
