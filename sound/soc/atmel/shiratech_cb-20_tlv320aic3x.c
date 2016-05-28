/*
 * ASoC machine driver for ShiraTech CB-20 audio
 *
 * Copyright (C) 2016 Joachim Eastwood <manabian@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/of.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <sound/soc.h>

#include "../codecs/tlv320aic3x.h"
#include "atmel_ssc_dai.h"

#define MCLK_RATE       12000000

struct shiratech_cb20_audio_data {
	struct snd_soc_dai_link dai;
	struct snd_soc_card card;
	struct clk *codec_clk;
	int ssc_id;
};

static const struct snd_soc_dapm_widget shiratech_cb20_dapm_widgets[] = {
	SND_SOC_DAPM_HP("Headphone Jack", NULL),
	SND_SOC_DAPM_LINE("Line In Jack", NULL),
};

static int shiratech_cb20_audio_dai_init(struct snd_soc_pcm_runtime *rtd)
{
	int ret;

	ret = snd_soc_dai_set_sysclk(rtd->codec_dai, CLKIN_MCLK, MCLK_RATE,
				     SND_SOC_CLOCK_IN);
	if (ret) {
		dev_err(rtd->card->dev, "unable to set codec system clock\n");
		return ret;
	}

	return 0;
}

static int shiratech_cb20_audio_suspend_pre(struct snd_soc_card *card)
{
	struct shiratech_cb20_audio_data *data = snd_soc_card_get_drvdata(card);
	clk_disable(data->codec_clk);
	return 0;
}

static int shiratech_cb20_audio_resume_pre(struct snd_soc_card *card)
{
	struct shiratech_cb20_audio_data *data = snd_soc_card_get_drvdata(card);
	return clk_enable(data->codec_clk);
}

static int shiratech_cb20_audio_dt_init(struct platform_device *pdev,
					struct shiratech_cb20_audio_data *data)
{
	struct device_node *np = pdev->dev.of_node;
	struct device_node *codec_np, *cpu_np;
	int ret;

	ret = snd_soc_of_parse_card_name(&data->card, "atmel,model");
	if (ret) {
		dev_err(&pdev->dev, "failed to parse card name\n");
		return ret;
	}

	ret = snd_soc_of_parse_audio_routing(&data->card, "atmel,audio-routing");
	if (ret) {
		dev_err(&pdev->dev, "failed to parse audio routing\n");
		return ret;
	}

	cpu_np = of_parse_phandle(np, "atmel,ssc-controller", 0);
	if (!cpu_np) {
		dev_err(&pdev->dev, "failed to get dai and pcm info\n");
		ret = -EINVAL;
		return ret;
	}

	data->dai.cpu_of_node = cpu_np;
	data->dai.platform_of_node = cpu_np;
	of_node_put(cpu_np);

	codec_np = of_parse_phandle(np, "atmel,audio-codec", 0);
	if (!codec_np) {
		dev_err(&pdev->dev, "failed to get codec info\n");
		ret = -EINVAL;
		return ret;
	}

	data->dai.codec_of_node = codec_np;
	of_node_put(codec_np);

	return 0;
}

static int shiratech_cb20_audio_probe(struct platform_device *pdev)
{
	struct shiratech_cb20_audio_data *data;
	int ret;

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->codec_clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(data->codec_clk)) {
		dev_err(&pdev->dev, "unable to get clock\n");
		return PTR_ERR(data->codec_clk);
	}

	data->dai.name = "TLV320AIC3X";
	data->dai.stream_name = "TLV320AIC3X PCM";
	data->dai.codec_dai_name = "tlv320aic3x-hifi";
	data->dai.dai_fmt = SND_SOC_DAIFMT_DSP_A | SND_SOC_DAIFMT_IB_NF |
			    SND_SOC_DAIFMT_CBM_CFM;
	data->dai.init = shiratech_cb20_audio_dai_init;

	data->card.dev = &pdev->dev;
	data->card.name = "TLV320AIC3X @ SAMA5D3";
	data->card.dai_link = &data->dai;
	data->card.num_links = 1;
	data->card.suspend_pre = shiratech_cb20_audio_suspend_pre;
	data->card.resume_pre = shiratech_cb20_audio_resume_pre;
	data->card.dapm_widgets = shiratech_cb20_dapm_widgets;
	data->card.num_dapm_widgets = ARRAY_SIZE(shiratech_cb20_dapm_widgets);
	data->card.fully_routed = true;

	ret = shiratech_cb20_audio_dt_init(pdev, data);
	if (ret)
		return ret;

	data->ssc_id = of_alias_get_id(data->dai.cpu_of_node, "ssc");
	ret = atmel_ssc_set_audio(data->ssc_id);
	if (ret) {
		dev_err(&pdev->dev, "unable to set SSC for audio\n");
		return ret;
	}

	ret = clk_set_rate(data->codec_clk, MCLK_RATE);
	if (ret) {
		dev_err(&pdev->dev, "unable to set clock rate\n");
		goto put_ssc;
	}

	ret = clk_prepare_enable(data->codec_clk);
	if (ret) {
		dev_err(&pdev->dev, "unable to enable clock\n");
		goto put_ssc;
	}

	platform_set_drvdata(pdev, &data->card);
	snd_soc_card_set_drvdata(&data->card, data);

	ret = devm_snd_soc_register_card(&pdev->dev, &data->card);
	if (ret) {
		dev_err(&pdev->dev, "unable to register card\n");
		goto dis_clock;
	}

	return 0;

dis_clock:
	clk_disable_unprepare(data->codec_clk);
put_ssc:
	atmel_ssc_put_audio(data->ssc_id);
	return ret;
}

static int shiratech_cb20_audio_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);
	struct shiratech_cb20_audio_data *data = snd_soc_card_get_drvdata(card);

	clk_disable_unprepare(data->codec_clk);
	atmel_ssc_put_audio(data->ssc_id);

	return 0;
}

static const struct of_device_id shiratech_cb20_audio_dt_ids[] = {
	{ .compatible = "shiratech,cb-20-tlv320aic3x", },
	{ }
};
MODULE_DEVICE_TABLE(of, shiratech_cb20_audio_dt_ids);

static struct platform_driver shiratech_cb20_audio_driver = {
	.probe = shiratech_cb20_audio_probe,
	.remove = shiratech_cb20_audio_remove,
	.driver = {
		.name = "shiratech-cb-20-audio",
		.of_match_table = shiratech_cb20_audio_dt_ids,
	},
};
module_platform_driver(shiratech_cb20_audio_driver);

MODULE_AUTHOR("Joachim Eastwood <manabian@gmail.com>");
MODULE_DESCRIPTION("ASoC machine driver for ShiraTech CB-20 with TLV320AIC3x");
MODULE_LICENSE("GPL v2");
