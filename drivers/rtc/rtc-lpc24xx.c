/*
 * Driver for NXP LPC24xx/178x/18xx/43xx Real-Time Clock (RTC)
 *
 * Copyright (C) 2011 NXP Semiconductors
 * Copyright (C) 2015 Joachim Eastwood <manabian@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#include <linux/clk.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/rtc.h>

/* LPC24xx RTC register offsets */
#define LPC24XX_ILR		0x00
#define LPC24XX_CTC		0x04
#define LPC24XX_CCR		0x08
#define LPC24XX_CIIR		0x0c
#define LPC24XX_AMR		0x10
#define LPC24XX_CTIME0		0x14
#define LPC24XX_CTIME1		0x18
#define LPC24XX_CTIME2		0x1c
#define LPC24XX_SEC		0x20
#define LPC24XX_MIN		0x24
#define LPC24XX_HOUR		0x28
#define LPC24XX_DOM		0x2c
#define LPC24XX_DOW		0x30
#define LPC24XX_DOY		0x34
#define LPC24XX_MONTH		0x38
#define LPC24XX_YEAR		0x3c
#define LPC24XX_CISS		0x40 /* LPC24xx only */
#define LPC178X_CALIBRATION	0x40 /* LPC178x and up */
#define LPC24XX_ALSEC		0x60
#define LPC24XX_ALMIN		0x64
#define LPC24XX_ALHOUR		0x68
#define LPC24XX_ALDOM		0x6c
#define LPC24XX_ALDOW		0x70
#define LPC24XX_ALDOY		0x74
#define LPC24XX_ALMON		0x78
#define LPC24XX_ALYEAR		0x7c
#define LPC24XX_PREINT		0x80 /* LPC24xx only */
#define LPC24XX_PREFRAC		0x84 /* LPC24xx only */

#define LPC24XX_RTCCIF		BIT(0)
#define LPC24XX_RTCALF		BIT(1)
#define LPC24XX_RTSSF		BIT(2) /* LPC24xx only */

#define LPC24XX_CLKEN		BIT(0)
#define LPC24XX_CTCRST		BIT(1)
#define LPC24XX_CLKSRC		BIT(4) /* LPC24xx only */
#define LPC178X_CCALEN		BIT(4) /* LPC178x and up */

#define LPC24XX_IMSEC		BIT(0)
#define LPC24XX_IMMIN		BIT(1)
#define LPC24XX_IMHOUR		BIT(2)
#define LPC24XX_IMDOM		BIT(3)
#define LPC24XX_IMDOW		BIT(4)
#define LPC24XX_IMDOY		BIT(5)
#define LPC24XX_IMMON		BIT(6)
#define LPC24XX_IMYEAR		BIT(7)

#define LPC24XX_ALARM_DISABLE	0xff

#define CT0_SECS(x)		(((x) >> 0)  & 0x3f)
#define CT0_MINS(x)		(((x) >> 8)  & 0x3f)
#define CT0_HOURS(x)		(((x) >> 16) & 0x1f)
#define CT0_DOW(x)		(((x) >> 24) & 0x07)
#define CT1_DOM(x)		(((x) >> 0)  & 0x1f)
#define CT1_MONTH(x)		(((x) >> 8)  & 0x0f)
#define CT1_YEAR(x)		(((x) >> 16) & 0xfff)
#define CT2_DOY(x)		(((x) >> 0)  & 0xfff)

#define rtc_readl(dev, reg)		readl((dev)->rtc_base + (reg))
#define rtc_writel(dev, reg, val)	writel((val), (dev)->rtc_base + (reg))

/* lpc2k-rtc driver feature flags */
#define HAVE_SUBSECOND		BIT(0)
#define HAVE_CALIBRATION	BIT(1)
#define HAVE_PRESCALER		BIT(2)

struct lpc24xx_rtc {
	void __iomem *rtc_base;
	struct rtc_device *rtc;
	struct clk *clk_rtc;
	struct clk *clk_reg;
	u32 features;
};

static int lpc24xx_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	struct lpc24xx_rtc *rtc = dev_get_drvdata(dev);
	u32 rtc_ctrl;

	/* Disable RTC during update */
	rtc_ctrl = rtc_readl(rtc, LPC24XX_CCR);
	rtc_writel(rtc, LPC24XX_CCR, rtc_ctrl & ~LPC24XX_CLKEN);

	rtc_writel(rtc, LPC24XX_SEC,	tm->tm_sec);
	rtc_writel(rtc, LPC24XX_MIN,	tm->tm_min);
	rtc_writel(rtc, LPC24XX_HOUR,	tm->tm_hour);
	rtc_writel(rtc, LPC24XX_DOW,	tm->tm_wday);
	rtc_writel(rtc, LPC24XX_DOM,	tm->tm_mday);
	rtc_writel(rtc, LPC24XX_DOY,	tm->tm_yday);
	rtc_writel(rtc, LPC24XX_MONTH,	tm->tm_mon);
	rtc_writel(rtc, LPC24XX_YEAR,	tm->tm_year);

	rtc_writel(rtc, LPC24XX_CCR, rtc_ctrl | LPC24XX_CLKEN);

	return 0;
}

static int lpc24xx_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct lpc24xx_rtc *rtc = dev_get_drvdata(dev);
	u32 ct0, ct1, ct2;

	ct0 = rtc_readl(rtc, LPC24XX_CTIME0);
	ct1 = rtc_readl(rtc, LPC24XX_CTIME1);
	ct2 = rtc_readl(rtc, LPC24XX_CTIME2);

	tm->tm_sec  = CT0_SECS(ct0);
	tm->tm_min  = CT0_MINS(ct0);
	tm->tm_hour = CT0_HOURS(ct0);
	tm->tm_wday = CT0_DOW(ct0);
	tm->tm_mon  = CT1_MONTH(ct1);
	tm->tm_mday = CT1_DOM(ct1);
	tm->tm_year = CT1_YEAR(ct1);
	tm->tm_yday = CT2_DOY(ct2);

	if (rtc_valid_tm(tm) < 0) {
		dev_warn(dev, "retrieved date and time is invalid\n");
		rtc_time64_to_tm(0, tm);
		lpc24xx_rtc_set_time(dev, tm);
	}

	return 0;
}

static int lpc24xx_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *wkalrm)
{
	struct lpc24xx_rtc *rtc = dev_get_drvdata(dev);
	struct rtc_time *tm = &wkalrm->time;

	tm->tm_sec  = rtc_readl(rtc, LPC24XX_ALSEC);
	tm->tm_min  = rtc_readl(rtc, LPC24XX_ALMIN);
	tm->tm_hour = rtc_readl(rtc, LPC24XX_ALHOUR);
	tm->tm_mday = rtc_readl(rtc, LPC24XX_ALDOM);
	tm->tm_wday = rtc_readl(rtc, LPC24XX_ALDOW);
	tm->tm_yday = rtc_readl(rtc, LPC24XX_ALDOY);
	tm->tm_mon  = rtc_readl(rtc, LPC24XX_ALMON);
	tm->tm_year = rtc_readl(rtc, LPC24XX_ALYEAR);

	wkalrm->enabled = rtc_readl(rtc, LPC24XX_AMR) == 0;
	wkalrm->pending = !!(rtc_readl(rtc, LPC24XX_ILR) & LPC24XX_RTCCIF);

	return rtc_valid_tm(&wkalrm->time);
}

static int lpc24xx_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *wkalrm)
{
	struct lpc24xx_rtc *rtc = dev_get_drvdata(dev);
	struct rtc_time *tm = &wkalrm->time;

	/* Disable alarm IRQs during update */
	rtc_writel(rtc, LPC24XX_AMR, LPC24XX_ALARM_DISABLE);

	rtc_writel(rtc, LPC24XX_ALSEC,  tm->tm_sec);
	rtc_writel(rtc, LPC24XX_ALMIN,  tm->tm_min);
	rtc_writel(rtc, LPC24XX_ALHOUR, tm->tm_hour);
	rtc_writel(rtc, LPC24XX_ALDOM,  tm->tm_mday);
	rtc_writel(rtc, LPC24XX_ALDOW,  tm->tm_wday);
	rtc_writel(rtc, LPC24XX_ALDOY,  tm->tm_yday);
	rtc_writel(rtc, LPC24XX_ALMON,  tm->tm_mon);
	rtc_writel(rtc, LPC24XX_ALYEAR, tm->tm_year);

	if (wkalrm->enabled)
		rtc_writel(rtc, LPC24XX_AMR, 0);

	return 0;
}

static int lpc24xx_rtc_alarm_irq_enable(struct device *dev,
					unsigned int enabled)
{
	struct lpc24xx_rtc *rtc = dev_get_drvdata(dev);

	if (enabled)
		rtc_writel(rtc, LPC24XX_AMR, 0);
	else
		rtc_writel(rtc, LPC24XX_AMR, LPC24XX_ALARM_DISABLE);

	return 0;
}

static irqreturn_t lpc24xx_rtc_interrupt(int irq, void *dev)
{
	unsigned long events = RTC_IRQF;
	struct lpc24xx_rtc *rtc = dev;
	u32 rtc_iir;

	/* Check interrupt cause */
	rtc_iir = rtc_readl(rtc, LPC24XX_ILR);

	/* 1s tick */
	if (rtc_iir & LPC24XX_RTCCIF)
		events |= RTC_UF;

	/* Alarm */
	if (rtc_iir & LPC24XX_RTCALF) {
		events |= RTC_AF;
		rtc_writel(rtc, LPC24XX_AMR, LPC24XX_ALARM_DISABLE);
	}

	/* Clear interrupt status and report event */
	rtc_writel(rtc, LPC24XX_ILR, rtc_iir);
	rtc_update_irq(rtc->rtc, 1, events);

	return IRQ_HANDLED;
}

static const struct rtc_class_ops lpc24xx_rtc_ops = {
	.read_time		= lpc24xx_rtc_read_time,
	.set_time		= lpc24xx_rtc_set_time,
	.read_alarm		= lpc24xx_rtc_read_alarm,
	.set_alarm		= lpc24xx_rtc_set_alarm,
	.alarm_irq_enable	= lpc24xx_rtc_alarm_irq_enable,
};

static const struct of_device_id lpc24xx_rtc_match[] = {
	{ .compatible = "nxp,lpc1788-rtc", .data = (void *)HAVE_CALIBRATION},
	{ }
};
MODULE_DEVICE_TABLE(of, lpc24xx_rtc_match);

static int lpc24xx_rtc_probe(struct platform_device *pdev)
{
	const struct of_device_id *match;
	struct lpc24xx_rtc *rtc;
	struct resource *res;
	u32 rtc_ccr = 0;
	int irq, ret;

	match = of_match_device(lpc24xx_rtc_match, &pdev->dev);
	if (!match) {
		dev_err(&pdev->dev, "no device match found\n");
		return -ENODEV;
	}

	rtc = devm_kzalloc(&pdev->dev, sizeof(*rtc), GFP_KERNEL);
	if (!rtc)
		return -ENOMEM;

	rtc->features = (u32)match->data;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	rtc->rtc_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(rtc->rtc_base))
		return PTR_ERR(rtc->rtc_base);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_warn(&pdev->dev, "can't get interrupt resource\n");
		return irq;
	}

	rtc->clk_rtc = devm_clk_get(&pdev->dev, "rtc");
	if (IS_ERR(rtc->clk_rtc)) {
		dev_err(&pdev->dev, "error getting rtc clock\n");
		return PTR_ERR(rtc->clk_rtc);
	}

	rtc->clk_reg = devm_clk_get(&pdev->dev, "reg");
	if (IS_ERR(rtc->clk_reg)) {
		dev_err(&pdev->dev, "error getting reg clock\n");
		return PTR_ERR(rtc->clk_reg);
	}

	ret = clk_prepare_enable(rtc->clk_rtc);
	if (ret) {
		dev_err(&pdev->dev, "unable to enable rtc clock\n");
		return ret;
	}

	ret = clk_prepare_enable(rtc->clk_reg);
	if (ret) {
		dev_err(&pdev->dev, "unable to enable reg clock\n");
		goto disable_rtc_clk;
	}

	platform_set_drvdata(pdev, rtc);

	/* Clear the counter increment state */
	rtc_writel(rtc, LPC24XX_ILR, LPC24XX_RTCCIF);

	if (rtc->features & HAVE_PRESCALER) {
		/* Clock source is 32k oscillator */
		rtc_ccr = LPC24XX_CLKSRC;
		rtc_writel(rtc, LPC24XX_CCR, rtc_ccr);

		/* Set prescaler to divide by 1 */
		rtc_writel(rtc, LPC24XX_PREINT, 0);
		rtc_writel(rtc, LPC24XX_PREFRAC, 0);
	}

	if (rtc->features & HAVE_SUBSECOND) {
		/* Disable sub-second interrupt */
		rtc_writel(rtc, LPC24XX_CISS, 0);
	}

	/* Only 1-second interrupt will generate interrupt */
	rtc_writel(rtc, LPC24XX_CIIR, LPC24XX_IMSEC);

	/* Enable RTC count */
	rtc_writel(rtc, LPC24XX_CCR, rtc_ccr | LPC24XX_CLKEN);

	ret = devm_request_irq(&pdev->dev, irq, lpc24xx_rtc_interrupt, 0,
			       pdev->name, rtc);
	if (ret < 0) {
		dev_warn(&pdev->dev, "can't request interrupt\n");
		goto disable_clks;
	}

	rtc->rtc = devm_rtc_device_register(&pdev->dev, "lpc2k-rtc",
					    &lpc24xx_rtc_ops, THIS_MODULE);
	if (IS_ERR(rtc->rtc)) {
		dev_err(&pdev->dev, "can't register rtc device\n");
		ret = PTR_ERR(rtc->rtc);
		goto disable_clks;
	}

	return 0;

disable_clks:
	clk_disable_unprepare(rtc->clk_reg);
disable_rtc_clk:
	clk_disable_unprepare(rtc->clk_rtc);
	return ret;
}

static int lpc24xx_rtc_remove(struct platform_device *pdev)
{
	struct lpc24xx_rtc *rtc = platform_get_drvdata(pdev);

	/*
	 * Disable alarm, but leave RTC enabled so it
	 * remains valid across reset cycles.
	 */
	rtc_writel(rtc, LPC24XX_AMR, LPC24XX_ALARM_DISABLE);
	rtc_writel(rtc, LPC24XX_CCR, 0);

	clk_disable_unprepare(rtc->clk_rtc);
	clk_disable_unprepare(rtc->clk_reg);

	return 0;
}

static struct platform_driver lpc24xx_rtc_driver = {
	.probe	= lpc24xx_rtc_probe,
	.remove	= lpc24xx_rtc_remove,
	.driver	= {
		.name = "lpc2k-rtc",
		.of_match_table	= lpc24xx_rtc_match,
	},
};
module_platform_driver(lpc24xx_rtc_driver);

MODULE_AUTHOR("Kevin Wells <wellsk40@gmail.com");
MODULE_DESCRIPTION("RTC driver for the LPC24xx SoC");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:lpc2k-rtc");
