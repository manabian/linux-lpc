/*
 * Copyright (C) 2011 NXP Semiconductors
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/rtc.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/clk.h>

/*
 * Clock and Power control register offsets
 */
#define LPC24XX_ILR			0x00
#define LPC24XX_CTC			0x04
#define LPC24XX_CCR			0x08
#define LPC24XX_CIIR			0x0C
#define LPC24XX_AMR			0x10
#define LPC24XX_CTIME0			0x14
#define LPC24XX_CTIME1			0x18
#define LPC24XX_CTIME2			0x1C
#define LPC24XX_SEC			0x20
#define LPC24XX_MIN			0x24
#define LPC24XX_HOUR			0x28
#define LPC24XX_DOM			0x2C
#define LPC24XX_DOW			0x30
#define LPC24XX_DOY			0x34
#define LPC24XX_MONTH			0x38
#define LPC24XX_YEAR			0x3C
#define LPC24XX_CISS			0x40
#define LPC24XX_ALSEC			0x60
#define LPC24XX_ALMIN			0x64
#define LPC24XX_ALHOUR			0x68
#define LPC24XX_ALDOM			0x6C
#define LPC24XX_ALDOW			0x70
#define LPC24XX_ALDOY			0x74
#define LPC24XX_ALMON			0x78
#define LPC24XX_ALYEAR			0x7C
#define LPC24XX_PREINT			0x80
#define LPC24XX_PREFRAC			0x84

#define LPC24XX_RTCCIF			(1 << 0)
#define LPC24XX_RTCALF			(1 << 1)
#define LPC24XX_RTSSF			(1 << 2)

#define LPC24XX_CLKEN			(1 << 0)
#define LPC24XX_CTCRST			(1 << 1)
#define LPC24XX_CLKSRC			(1 << 4)

#define LPC24XX_IMSEC			(1 << 0)
#define LPC24XX_IMMIN			(1 << 1)
#define LPC24XX_IMHOUR			(1 << 2)
#define LPC24XX_IMDOM			(1 << 3)
#define LPC24XX_IMDOW			(1 << 4)
#define LPC24XX_IMDOY			(1 << 5)
#define LPC24XX_IMMON			(1 << 6)
#define LPC24XX_IMYEAR			(1 << 7)

#define CT0_SECS(x)			(((x) >> 0)  & 0x3F)
#define CT0_MINS(x)			(((x) >> 8)  & 0x3F)
#define CT0_HOURS(x)			(((x) >> 16) & 0x1F)
#define CT0_DOW(x)			(((x) >> 24) & 0x7)
#define CT1_DOM(x)			(((x) >> 0)  & 0x1F)
#define CT1_MONTH(x)			(((x) >> 8)  & 0xF)
#define CT1_YEAR(x)			(((x) >> 16) & 0xFFF)
#define CT2_DOY(x)			(((x) >> 0)  & 0xFFF)

#define RTC_NAME "lpc2k-rtc"

#define rtc_readl(dev, reg) \
	__raw_readl((dev)->rtc_base + (reg))
#define rtc_writel(dev, reg, val) \
	__raw_writel((val), (dev)->rtc_base + (reg))

struct lpc24xx_rtc {
	void __iomem *rtc_base;
	int irq;
	unsigned char alarm_enabled;
	struct rtc_device *rtc;
	struct clk *clk;
};

static int lpc24xx_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	unsigned long ct0, ct1, ct2;
	struct lpc24xx_rtc *rtc = dev_get_drvdata(dev);

	ct0 = rtc_readl(rtc, LPC24XX_CTIME0);
	ct1 = rtc_readl(rtc, LPC24XX_CTIME1);
	ct2 = rtc_readl(rtc, LPC24XX_CTIME2);

	tm->tm_sec = CT0_SECS(ct0);
	tm->tm_min = CT0_MINS(ct0);
	tm->tm_hour = CT0_HOURS(ct0);
	tm->tm_wday = CT0_DOW(ct0);
	tm->tm_mon = CT1_MONTH(ct1);
	tm->tm_mday = CT1_DOM(ct1);
	tm->tm_year = CT1_YEAR(ct1);
	tm->tm_yday = CT2_DOY(ct2);

	return rtc_valid_tm(tm);
}

static int lpc24xx_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	unsigned long rtc_ctrl;
	struct lpc24xx_rtc *rtc = dev_get_drvdata(dev);

	/* Disable RTC during update */
	rtc_ctrl = rtc_readl(rtc, LPC24XX_CCR);
	rtc_writel(rtc, LPC24XX_CCR, rtc_ctrl & ~LPC24XX_CLKEN);

	rtc_writel(rtc, LPC24XX_SEC, tm->tm_sec);
	rtc_writel(rtc, LPC24XX_MIN, tm->tm_min);
	rtc_writel(rtc, LPC24XX_HOUR, tm->tm_hour);
	rtc_writel(rtc, LPC24XX_DOW, tm->tm_wday);
	rtc_writel(rtc, LPC24XX_DOM, tm->tm_mday);
	rtc_writel(rtc, LPC24XX_DOY, tm->tm_yday);
	rtc_writel(rtc, LPC24XX_MONTH, tm->tm_mon);
	rtc_writel(rtc, LPC24XX_YEAR, tm->tm_year);

	rtc_writel(rtc, LPC24XX_CCR, rtc_ctrl | LPC24XX_CLKEN);

	return 0;
}

static int lpc24xx_rtc_read_alarm(struct device *dev,
	struct rtc_wkalrm *wkalrm)
{
	struct lpc24xx_rtc *rtc = dev_get_drvdata(dev);
	struct rtc_time *tm = &wkalrm->time;

	tm->tm_sec = rtc_readl(rtc, LPC24XX_ALSEC);
	tm->tm_min = rtc_readl(rtc, LPC24XX_ALMIN);
	tm->tm_hour = rtc_readl(rtc, LPC24XX_ALHOUR);
	tm->tm_mday = rtc_readl(rtc, LPC24XX_ALDOM);
	tm->tm_wday = rtc_readl(rtc, LPC24XX_ALDOW);
	tm->tm_yday = rtc_readl(rtc, LPC24XX_ALDOY);
	tm->tm_mon = rtc_readl(rtc, LPC24XX_ALMON);
	tm->tm_year = rtc_readl(rtc, LPC24XX_ALYEAR);

	wkalrm->enabled = rtc_readl(rtc, LPC24XX_AMR) == 0;
	wkalrm->pending = !!(rtc_readl(rtc, LPC24XX_ILR) & LPC24XX_RTCCIF);

	return rtc_valid_tm(&wkalrm->time);
}

static int lpc24xx_rtc_set_alarm(struct device *dev,
	struct rtc_wkalrm *wkalrm)
{
	struct lpc24xx_rtc *rtc = dev_get_drvdata(dev);
	struct rtc_time *tm = &wkalrm->time;

	/* Disable alarm IRQs during update */
	rtc_writel(rtc, LPC24XX_AMR, ~0);

	rtc_writel(rtc, LPC24XX_ALSEC, tm->tm_sec);
	rtc_writel(rtc, LPC24XX_ALMIN, tm->tm_min);
	rtc_writel(rtc, LPC24XX_ALHOUR, tm->tm_hour);
	rtc_writel(rtc, LPC24XX_ALDOM, tm->tm_mday);
	rtc_writel(rtc, LPC24XX_ALDOW, tm->tm_wday);
	rtc_writel(rtc, LPC24XX_ALDOY, tm->tm_yday);
	rtc_writel(rtc, LPC24XX_ALMON, tm->tm_mon);
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
		rtc_writel(rtc, LPC24XX_AMR, 0xFF);

	return 0;
}

static irqreturn_t lpc24xx_rtc_interrupt(int irq, void *dev)
{
	unsigned long rtciir, events = RTC_IRQF;
	struct lpc24xx_rtc *rtc = dev;

	/* 1-s or alarm interrupt, or both */
	rtciir = rtc_readl(rtc, LPC24XX_ILR);

	if (rtciir & LPC24XX_RTCCIF) {
		/* 1-s tick */
		events |= RTC_UF;
	}

	if (rtciir & LPC24XX_RTCALF) {
		/* Alarm */
		events |= RTC_AF;

		/* Disable alarm interrupt */
		rtc_writel(rtc, LPC24XX_AMR, 0xFF);
	}

	rtc_writel(rtc, LPC24XX_ILR, rtciir);

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

static int __devinit lpc24xx_rtc_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct lpc24xx_rtc *rtc;
	resource_size_t size;
	int rtcirq;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "Can't get memory resource\n");
		return -ENOENT;
	}

	rtcirq = platform_get_irq(pdev, 0);
	if (rtcirq < 0 || rtcirq >= NR_IRQS) {
		dev_warn(&pdev->dev, "Can't get interrupt resource\n");
		rtcirq = -1;
	}

	rtc = devm_kzalloc(&pdev->dev, sizeof(*rtc), GFP_KERNEL);
	if (unlikely(!rtc)) {
		dev_err(&pdev->dev, "Can't allocate memory\n");
		return -ENOMEM;
	}
	rtc->irq = rtcirq;

	size = resource_size(res);

	if (!devm_request_mem_region(&pdev->dev, res->start, size,
				     pdev->name)) {
		dev_err(&pdev->dev, "RTC registers are not free\n");
		return -EBUSY;
	}

	rtc->rtc_base = devm_ioremap(&pdev->dev, res->start, size);
	if (!rtc->rtc_base) {
		dev_err(&pdev->dev, "Can't map memory\n");
		return -ENOMEM;
	}

	rtc->clk = clk_get(&pdev->dev, NULL);
	if (IS_ERR(rtc->clk)) {
		dev_err(&pdev->dev, "error getting clock\n");
		return IS_ERR(rtc->clk);
	}
	clk_enable(rtc->clk);

	platform_set_drvdata(pdev, rtc);

	rtc->rtc = rtc_device_register(RTC_NAME, &pdev->dev, &lpc24xx_rtc_ops,
		THIS_MODULE);
	if (IS_ERR(rtc->rtc)) {
		dev_err(&pdev->dev, "Can't get RTC\n");
		platform_set_drvdata(pdev, NULL);
		rtc_writel(rtc, LPC24XX_CCR, 0);
		clk_disable(rtc->clk);
		clk_put(rtc->clk);

		return PTR_ERR(rtc->rtc);
	}

	/* Clear the counter increment state */
	rtc_writel(rtc, LPC24XX_ILR, LPC24XX_RTCCIF);

	if (rtc->irq >= 0) {
		if (devm_request_irq(&pdev->dev, rtc->irq,
				     lpc24xx_rtc_interrupt,
				     IRQF_DISABLED, pdev->name, rtc) < 0) {
			dev_warn(&pdev->dev, "Can't request interrupt.\n");
			rtc->irq = -1;
		} else {
			device_init_wakeup(&pdev->dev, 1);
		}
	}

	/* Clock source is 32K oscillator */
	rtc_writel(rtc, LPC24XX_CCR, LPC24XX_CLKSRC);

	/* Set pre-scaler to divider by 1 */
	rtc_writel(rtc, LPC24XX_PREINT, 0);
	rtc_writel(rtc, LPC24XX_PREFRAC, 0);

	/* Disable sub-second interrupt */
	rtc_writel(rtc, LPC24XX_CISS, 0);

	/* Only 1-second interrupt will generate interrupt */
	rtc_writel(rtc, LPC24XX_CIIR, LPC24XX_IMSEC);

	/* Enable RTC count */
	rtc_writel(rtc, LPC24XX_CCR, LPC24XX_CLKSRC | LPC24XX_CLKEN);

	return 0;
}

static int __devexit lpc24xx_rtc_remove(struct platform_device *pdev)
{
	struct lpc24xx_rtc *rtc = platform_get_drvdata(pdev);

	/*
	 * Disable alarm, but leave RTC enabled so it remains valid across
	 * reset cycles
	 */
	rtc_writel(rtc, LPC24XX_AMR, 0xFF);
	rtc_writel(rtc, LPC24XX_CCR, 0);

	if (rtc->irq >= 0)
		device_init_wakeup(&pdev->dev, 0);

	clk_disable(rtc->clk);
	clk_put(rtc->clk);

	platform_set_drvdata(pdev, NULL);
	rtc_device_unregister(rtc->rtc);

	return 0;
}

#ifdef CONFIG_PM
static int lpc24xx_rtc_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct lpc24xx_rtc *rtc = platform_get_drvdata(pdev);

	if (rtc->irq >= 0) {
		if (device_may_wakeup(&pdev->dev))
			enable_irq_wake(rtc->irq);
		else
			disable_irq_wake(rtc->irq);
	}

	clk_disable(rtc->clk);

	return 0;
}

static int lpc24xx_rtc_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct lpc24xx_rtc *rtc = platform_get_drvdata(pdev);

	clk_enable(rtc->clk);

	if (rtc->irq >= 0 && device_may_wakeup(&pdev->dev))
		disable_irq_wake(rtc->irq);

	return 0;
}

static const struct dev_pm_ops lpc24xx_rtc_pm_ops = {
	.suspend = lpc24xx_rtc_suspend,
	.resume = lpc24xx_rtc_resume,
};

#define LPC24XX_RTC_PM_OPS (&lpc24xx_rtc_pm_ops)
#else
#define LPC24XX_RTC_PM_OPS NULL
#endif

static struct platform_driver lpc24xx_rtc_driver = {
	.probe		= lpc24xx_rtc_probe,
	.remove		= __devexit_p(lpc24xx_rtc_remove),
	.driver = {
		.name	= RTC_NAME,
		.owner	= THIS_MODULE,
		.pm	= LPC24XX_RTC_PM_OPS
	},
};

static int __init lpc24xx_rtc_init(void)
{
	return platform_driver_register(&lpc24xx_rtc_driver);
}
module_init(lpc24xx_rtc_init);

static void __exit lpc24xx_rtc_exit(void)
{
	platform_driver_unregister(&lpc24xx_rtc_driver);
}
module_exit(lpc24xx_rtc_exit);

MODULE_AUTHOR("Kevin Wells <wellsk40@gmail.com");
MODULE_DESCRIPTION("RTC driver for the LPC24xx SoC");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:lpc2k-rtc");
