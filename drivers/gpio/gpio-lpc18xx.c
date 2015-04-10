/*
 * GPIO driver for NXP LPC18xx/43xx.
 *
 * Copyright (C) 2015 Joachim Eastwood <manabian@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/basic_mmio_gpio.h>
#include <linux/clk.h>
#include <linux/gpio/driver.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>

/* LPC18xx GPIO register offsets */
#define LPC18XX_REG_DIR(n)	(0x2000 + n * sizeof(u32))
#define LPC18XX_REG_PIN(n)	(0x2100 + n * sizeof(u32))
#define LPC18XX_REG_SET(n)	(0x2200 + n * sizeof(u32))
#define LPC18XX_REG_CLR(n)	(0x2280 + n * sizeof(u32))

#define LPC18XX_GPIO_PORTS	8

struct lpc18xx_gpio_chip {
	struct clk *clk;
	void __iomem *base;
	struct bgpio_chip bgc[LPC18XX_GPIO_PORTS];
};

static int lpc18xx_gpio_xlate(struct gpio_chip *gc,
			      const struct of_phandle_args *gspec, u32 *flags)
{
	u32 gpio;

	if (WARN_ON(gspec->args_count < gc->of_gpio_n_cells))
		return -EINVAL;

	gpio = gspec->args[0];
	if (gc->base > gpio || gpio >= (gc->base + gc->ngpio))
		return -EINVAL;

	if (flags)
		*flags = gspec->args[1];

	return gpio % gc->ngpio;
}

static int lpc18xx_gpio_request(struct gpio_chip *chip, unsigned offset)
{
	return pinctrl_request_gpio(chip->base + offset);
}

static void lpc18xx_gpio_free(struct gpio_chip *chip, unsigned offset)
{
	pinctrl_free_gpio(chip->base + offset);
}

static int lpc18xx_gpio_add_port(struct lpc18xx_gpio_chip *gc,
				 struct device *dev, int port)
{
	void __iomem *dat = gc->base + LPC18XX_REG_PIN(port);
	void __iomem *dir = gc->base + LPC18XX_REG_DIR(port);
	void __iomem *set = gc->base + LPC18XX_REG_SET(port);
	void __iomem *clr = gc->base + LPC18XX_REG_CLR(port);
	struct bgpio_chip *bgc = &gc->bgc[port];
	int ret;

	ret = bgpio_init(bgc, dev, 4, dat, set, clr, dir, NULL, 0);
	if (ret)
		return ret;

	bgc->gc.base	= port * bgc->gc.ngpio;
	bgc->gc.request	= lpc18xx_gpio_request;
	bgc->gc.free	= lpc18xx_gpio_free;

	bgc->gc.of_xlate = lpc18xx_gpio_xlate;
	bgc->gc.of_gpio_n_cells	= 2;

	return gpiochip_add(&bgc->gc);
}

static int lpc18xx_gpio_probe(struct platform_device *pdev)
{
	struct lpc18xx_gpio_chip *gc;
	struct resource *res;
	int i, ret;

	gc = devm_kzalloc(&pdev->dev, sizeof(*gc), GFP_KERNEL);
	if (!gc)
		return -ENOMEM;

	platform_set_drvdata(pdev, gc);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	gc->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(gc->base))
		return PTR_ERR(gc->base);

	gc->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(gc->clk)) {
		dev_err(&pdev->dev, "input clock not found\n");
		return PTR_ERR(gc->clk);
	}

	ret = clk_prepare_enable(gc->clk);
	if (ret) {
		dev_err(&pdev->dev, "unable to enable clock\n");
		return ret;
	}

	for (i = 0; i < LPC18XX_GPIO_PORTS; i++) {
		ret = lpc18xx_gpio_add_port(gc, &pdev->dev, i);
		if (ret)
			dev_warn(&pdev->dev, "unable to add port %d\n", i);
	}

	return 0;
}

static int lpc18xx_gpio_remove(struct platform_device *pdev)
{
	struct lpc18xx_gpio_chip *gc = platform_get_drvdata(pdev);
	int i;

	for (i = 0; i < LPC18XX_GPIO_PORTS; i++)
		bgpio_remove(&gc->bgc[i]);

	clk_disable_unprepare(gc->clk);

	return 0;
}

static const struct of_device_id lpc18xx_gpio_match[] = {
	{ .compatible = "nxp,lpc1850-gpio" },
	{ }
};
MODULE_DEVICE_TABLE(of, lpc18xx_gpio_match);

static struct platform_driver lpc18xx_gpio_driver = {
	.probe	= lpc18xx_gpio_probe,
	.remove	= lpc18xx_gpio_remove,
	.driver	= {
		.name		= "lpc18xx-gpio",
		.of_match_table	= lpc18xx_gpio_match,
	},
};
module_platform_driver(lpc18xx_gpio_driver);

MODULE_AUTHOR("Joachim Eastwood <manabian@gmail.com>");
MODULE_DESCRIPTION("GPIO driver for LPC18xx/43xx");
MODULE_LICENSE("GPL v2");
