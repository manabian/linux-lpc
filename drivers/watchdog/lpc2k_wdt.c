/*
 * drivers/char/watchdog/lpc2k_wdt.c
 *
 * Watchdog driver for NXP LPC23xx/LPC24xx devices
 *
 * Author: Kevin Wells <kevin.wells@nxp.com>,
 *
 * Copyright (C) 2011 NXP Semiconductors
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/watchdog.h>

#define MODULE_NAME "lpc2k-wdt"

/* Watchdog timer register set definition */
#define LPC2K_WDMOD	0x00
#define LPC2K_WDTC	0x04
#define LPC2K_WDFEED	0x08
#define LPC2K_WDTV	0x0C
#define LPC2K_WDCLKSEL	0x10

/* WDMOD bit definitions */
#define LPC2K_WDEN	0x01
#define LPC2K_WDRESET	0x02
#define LPC2K_WDTOF	0x04
#define LPC2K_WDINT	0x08

/* WDFEED definitions */
#define LPC2K_FEED_LTCH	0xAA
#define LPC2K_FEED_CLR	0x55

/*
 * The LPC2K watchdog clock cannot be disabled, although the clock base can
 * be changed. This driver will always uses PCLK for the watchdog clock base
 */
#define LPC2K_USEPCLK	0x1

#define MAX_HEARTBEAT	20

static struct platform_device *lpc2k_wdt_dev;

struct lpc2k_wdt_dev {
	void __iomem		*base;
	unsigned long		num_users;
	struct miscdevice	miscdev;
	struct clk		*clk;
	long			heartbeat;
};

static void lpc2k_wdt_feed(void __iomem *base)
{
	unsigned long flags;

	/* WDT feeds must not be interrupted */
	local_irq_save(flags);
	__raw_writel(LPC2K_FEED_LTCH, base + LPC2K_WDFEED);
	__raw_writel(LPC2K_FEED_CLR, base + LPC2K_WDFEED);
	local_irq_restore(flags);
}

static void lpc2k_wdt_show_warning(void)
{
	printk(KERN_CRIT MODULE_NAME
		": this driver, once enabled, cannot be stopped. The\n"
		"watchdog must be continuously serviced or the system will\n"
		"reset once the watchdog expires.\n");
}

static void lpc2k_wdt_set_rate(struct lpc2k_wdt_dev *wdtdev)
{
	long clkrate, clkrate2;

	/*
	 * Assume PCLK is 12Mhz if the clock rate fetch fails.
	 * It's not a great assumption, but it will do.
	 */
	clkrate2 = clk_get_rate(wdtdev->clk);
	if (!clkrate2)
		clkrate2 = 12000000;

	/* Input rate into watchdog */
	clkrate = clkrate2 / 4;

	/* Determine divider and set timeout */
	clkrate = clkrate * wdtdev->heartbeat;
	__raw_writel(clkrate, wdtdev->base + LPC2K_WDTC);
	lpc2k_wdt_feed(wdtdev->base);
}

static void lpc2k_wdt_enable(struct lpc2k_wdt_dev *wdtdev)
{
	/* Set clock rate */
	lpc2k_wdt_set_rate(wdtdev);

	/* Enable (1-way) watchdog */
	__raw_writel(LPC2K_WDEN + LPC2K_WDRESET, wdtdev->base + LPC2K_WDMOD);

	/* Initial ping */
	lpc2k_wdt_feed(wdtdev->base);
}

static ssize_t lpc2k_wdt_write(struct file *file, const char *data, size_t len, loff_t *ppos)
{
	struct lpc2k_wdt_dev *wdtdev = platform_get_drvdata(lpc2k_wdt_dev);

	if (len)
		lpc2k_wdt_feed(wdtdev->base);

	return len;
}

static const struct watchdog_info lpc2k_ident = {
	.options = WDIOF_CARDRESET | WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING,
	.identity = "LPC2K Watchdog",
};

static long lpc2k_wdt_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int ret = -ENOTTY;
	int boot_status, time;
	struct lpc2k_wdt_dev *wdtdev = platform_get_drvdata(lpc2k_wdt_dev);

	switch (cmd) {
	case WDIOC_GETSUPPORT:
		ret = copy_to_user((struct watchdog_info *) arg,
			&lpc2k_ident,  sizeof(lpc2k_ident)) ? -EFAULT : 0;
		break;

	case WDIOC_GETSTATUS:
		ret = put_user(0, (int *) arg);
		break;

	case WDIOC_GETBOOTSTATUS:
		boot_status = (int) __raw_readl(wdtdev->base + LPC2K_WDMOD);
		ret = put_user((boot_status >> 2) & 1, (int *) arg);
		break;

	case WDIOC_KEEPALIVE:
		lpc2k_wdt_feed(wdtdev->base);
		ret = 0;
		break;

	case WDIOC_SETTIMEOUT:
		ret = get_user(time, (int *) arg);
		if (ret)
			break;

		if (time <= 0 || time > MAX_HEARTBEAT) {
			ret = -EINVAL;
			break;
		}

		wdtdev->heartbeat = time;
		lpc2k_wdt_enable(wdtdev);
		/* Fall through */

	case WDIOC_GETTIMEOUT:
		ret = put_user(wdtdev->heartbeat, (int *) arg);
		break;
	}
	return ret;
}

static int lpc2k_wdt_open(struct inode *inode, struct file *file)
{
	int ret;
	struct lpc2k_wdt_dev *wdtdev = platform_get_drvdata(lpc2k_wdt_dev);

	/* Bit will never clear once set */
	if (test_and_set_bit(1, &wdtdev->num_users))
		return -EBUSY;

	ret = clk_prepare_enable(wdtdev->clk);
	if (ret) {
		clear_bit(1, &wdtdev->num_users);
		return ret;
	}

	lpc2k_wdt_enable(wdtdev);

	return nonseekable_open(inode, file);
}

static int lpc2k_wdt_release(struct inode *inode, struct file *file)
{
	lpc2k_wdt_show_warning();

	return 0;
}

static const struct file_operations lpc2k_wdt_fops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.write = lpc2k_wdt_write,
	.unlocked_ioctl = lpc2k_wdt_ioctl,
	.open = lpc2k_wdt_open,
	.release = lpc2k_wdt_release,
};

static int lpc2k_wdt_probe(struct platform_device *pdev)
{
	struct lpc2k_wdt_dev *wdtdev;
	struct resource *res;
	int ret;

	wdtdev = devm_kzalloc(&pdev->dev, sizeof(*wdtdev), GFP_KERNEL);
	if (!wdtdev)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	wdtdev->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(wdtdev->base))
		return PTR_ERR(wdtdev->base);

	platform_set_drvdata(pdev, wdtdev);

	/*
	 * Since this peripheral is always clocked, we'll get the clock
	 * here and get the rate on open
	 */
	wdtdev->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(wdtdev->clk)) {
		dev_err(&pdev->dev, "Error getting clock\n");
		return PTR_ERR(wdtdev->clk);
	}

	wdtdev->miscdev.parent = &pdev->dev;
	wdtdev->miscdev.minor = WATCHDOG_MINOR;
	wdtdev->miscdev.name = "watchdog";
	wdtdev->miscdev.fops = &lpc2k_wdt_fops;

	/* Default reset timeout */
	wdtdev->heartbeat = MAX_HEARTBEAT;

	lpc2k_wdt_dev = pdev;

	ret = misc_register(&wdtdev->miscdev);
	if (ret)
		return ret;

	/*
	 * Leave WDT disabled until opened. Once opened, the WDT must
	 * be continually serviced or it will reset, as the WDT cannot be
	 * disabled.
	 */
	__raw_writel(LPC2K_USEPCLK, wdtdev->base + LPC2K_WDCLKSEL);

	lpc2k_wdt_show_warning();

	return 0;
}

static int lpc2k_wdt_remove(struct platform_device *pdev)
{
	struct lpc2k_wdt_dev *wdtdev = platform_get_drvdata(pdev);

	misc_deregister(&wdtdev->miscdev);

	if (test_bit(1, &wdtdev->num_users)) {
		clk_disable_unprepare(wdtdev->clk);
		lpc2k_wdt_show_warning();
	}

	return 0;
}

static struct platform_driver lpc2k_wdt_driver = {
	.probe	= lpc2k_wdt_probe,
	.remove	= lpc2k_wdt_remove,
	.driver = {
		.name	= MODULE_NAME,
	},

};
module_platform_driver(lpc2k_wdt_driver);

MODULE_AUTHOR("Kevin Wells <kevin.wells@nxp.com>");
MODULE_DESCRIPTION("LPC2K Watchdog Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS_MISCDEV(WATCHDOG_MINOR);
MODULE_ALIAS("platform:lpc2k-wdt");
