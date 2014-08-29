/*
 * GPIO driver for NXP LPC18xx/43xx.
 *
 * Copyright (C) 2014 Joachim Eastwood <manabian@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/clk.h>
#include <linux/gpio/driver.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>

#define LPC18XX_MAX_PORTS	8
#define LPC18XX_PINS_PER_PORT	32

#define LPC18XX_REG_PWORD(_n)	(0x1000 + _n * sizeof(u32))
#define LPC18XX_REG_DIR(_n)	(0x2000 + _n * sizeof(u32))
#define LPC18XX_REG_SET(_n)	(0x2200 + _n * sizeof(u32))
#define LPC18XX_REG_CLR(_n)	(0x2280 + _n * sizeof(u32))

struct lpc18xx_gpio_chip {
	struct gpio_chip gpio;
	void __iomem *base;
	struct clk *clk;
};

static inline struct lpc18xx_gpio_chip *to_lpc18xx_gpio(struct gpio_chip *chip)
{
	return container_of(chip, struct lpc18xx_gpio_chip, gpio);
}

static int lpc18xx_gpio_request(struct gpio_chip *chip, unsigned offset)
{
	return pinctrl_request_gpio(offset);
}

static void lpc18xx_gpio_free(struct gpio_chip *chip, unsigned offset)
{
	pinctrl_free_gpio(offset);
}

static void lpc18xx_gpio_set(struct gpio_chip *chip, unsigned offset, int value)
{
	struct lpc18xx_gpio_chip *gc = to_lpc18xx_gpio(chip);
	u32 port, pin, reg_offset;

	port = offset / LPC18XX_PINS_PER_PORT;
	pin = offset % LPC18XX_PINS_PER_PORT;

	if (value)
		reg_offset = LPC18XX_REG_SET(port);
	else
		reg_offset = LPC18XX_REG_CLR(port);

	writel(1 << pin, gc->base + reg_offset);
}

static int lpc18xx_gpio_get(struct gpio_chip *chip, unsigned offset)
{
	struct lpc18xx_gpio_chip *gc = to_lpc18xx_gpio(chip);
	u32 reg_offset = LPC18XX_REG_PWORD(offset);

	return !!readl(gc->base + reg_offset);
}

static int lpc18xx_gpio_direction_input(struct gpio_chip *chip, unsigned offset)
{
	struct lpc18xx_gpio_chip *gc = to_lpc18xx_gpio(chip);
	u32 port, pin, dir, reg_offset;

	port = offset / LPC18XX_PINS_PER_PORT;
	pin = offset % LPC18XX_PINS_PER_PORT;
	reg_offset = LPC18XX_REG_DIR(port);

	dir = readl(gc->base + reg_offset) & ~BIT(pin);
	writel(dir, gc->base + reg_offset);

	return 0;
}

static int lpc18xx_gpio_direction_output(struct gpio_chip *chip,
					unsigned offset, int value)
{
	struct lpc18xx_gpio_chip *gc = to_lpc18xx_gpio(chip);
	u32 port, pin, dir, reg_offset;

	lpc18xx_gpio_set(chip, offset, value);

	port = offset / LPC18XX_PINS_PER_PORT;
	pin = offset % LPC18XX_PINS_PER_PORT;
	reg_offset = LPC18XX_REG_DIR(port);

	dir = readl(gc->base + reg_offset) | BIT(pin);
	writel(dir, gc->base + reg_offset);

	return 0;
}

static struct gpio_chip lpc18xx_chip = {
	.label			= "lpc18xx/43xx-gpio",
	.request		= lpc18xx_gpio_request,
	.free			= lpc18xx_gpio_free,
	.direction_input	= lpc18xx_gpio_direction_input,
	.direction_output	= lpc18xx_gpio_direction_output,
	.set			= lpc18xx_gpio_set,
	.get			= lpc18xx_gpio_get,
	.ngpio			= LPC18XX_MAX_PORTS * LPC18XX_PINS_PER_PORT,
	.owner			= THIS_MODULE,
};

static int lpc18xx_gpio_probe(struct platform_device *pdev)
{
	struct lpc18xx_gpio_chip *gc;
	struct resource *res;
	int ret;

	gc = devm_kzalloc(&pdev->dev, sizeof(*gc), GFP_KERNEL);
	if (!gc)
		return -ENOMEM;

	gc->gpio = lpc18xx_chip;
	platform_set_drvdata(pdev, gc);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	gc->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(gc->base))
		return PTR_ERR(gc->base);

	gc->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(gc->clk)) {
		dev_err(&pdev->dev, "Input clock not found.\n");
		return PTR_ERR(gc->clk);
	}

	ret = clk_prepare_enable(gc->clk);
	if (ret) {
		dev_err(&pdev->dev, "Unable to enable clock.\n");
		return ret;
	}

	gc->gpio.dev = &pdev->dev;
	gc->gpio.of_node = pdev->dev.of_node;

	ret = gpiochip_add(&gc->gpio);
	if (ret) {
		dev_err(&pdev->dev, "Failed to add gpio chip\n");
		clk_disable_unprepare(gc->clk);
		return ret;
	}

	return 0;
}

static int lpc18xx_gpio_remove(struct platform_device *pdev)
{
	struct lpc18xx_gpio_chip *gc = platform_get_drvdata(pdev);

	gpiochip_remove(&gc->gpio);
	clk_disable_unprepare(gc->clk);

	return 0;
}

static const struct of_device_id lpc18xx_gpio_match[] = {
	{ .compatible = "nxp,lpc1850-gpio" },
	{ }
};

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
