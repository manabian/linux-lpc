/*
 * Serial port driver for NXP LPC18xx/43xx UART
 *
 * Copyright (C) 2014 Joachim Eastwood <manabian@gmail.com>
 *
 * Based on 8250_mtk.c:
 * Copyright (c) 2014 MundoReader S.L.
 * Matthias Brugger <matthias.bgg@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/clk.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/serial_core.h>
#include <linux/serial_reg.h>

#include "8250.h"

/* Additional LPC18xx/43xx 8250 registers and bits */
#define LPC18XX_UART_RS485CTRL		(0x04c / sizeof(u32))
#define  LPC18XX_UART_RS485CTRL_NMMEN	BIT(0)
#define  LPC18XX_UART_RS485CTRL_DCTRL	BIT(4)
#define  LPC18XX_UART_RS485CTRL_OINV	BIT(5)
#define LPC18XX_UART_RS485DLY		(0x054 / sizeof(u32))
#define  LPC18XX_UART_RS485DLY_MAX	255

struct lpc18xx_uart_data {
	struct clk *clk_uart;
	struct clk *clk_reg;
	int line;
};

static int lpc18xx_rs485_config(struct uart_port *port,
				struct serial_rs485 *rs485)
{
	struct uart_8250_port *up = up_to_u8250p(port);
	u32 rs485_ctrl_reg = 0;
	u32 rs485_dly_reg = 0;

	if (rs485->flags & SER_RS485_ENABLED)
		memset(rs485->padding, 0, sizeof(rs485->padding));
	else
		memset(rs485, 0, sizeof(*rs485));

	rs485->flags &= SER_RS485_ENABLED | SER_RS485_RTS_ON_SEND |
			SER_RS485_RTS_AFTER_SEND;

	if (rs485->flags & SER_RS485_ENABLED) {
		rs485_ctrl_reg |= LPC18XX_UART_RS485CTRL_NMMEN |
				  LPC18XX_UART_RS485CTRL_DCTRL;

		if (rs485->flags & SER_RS485_RTS_ON_SEND) {
			rs485_ctrl_reg |= LPC18XX_UART_RS485CTRL_OINV;
			rs485->flags &= ~SER_RS485_RTS_AFTER_SEND;
		} else {
			rs485->flags |= SER_RS485_RTS_AFTER_SEND;
		}
	}

	if (rs485->delay_rts_after_send) {
		u32 baud_clk = port->uartclk / up->dl_read(up);
		rs485_dly_reg = DIV_ROUND_UP(rs485->delay_rts_after_send
						* baud_clk, MSEC_PER_SEC);

		if (rs485_dly_reg > LPC18XX_UART_RS485DLY_MAX)
			rs485_dly_reg = LPC18XX_UART_RS485DLY_MAX;

		/* Calculate the resulting delay in ms */
		rs485->delay_rts_after_send = (rs485_dly_reg * MSEC_PER_SEC)
						/ baud_clk;
	}

	/* Delay RTS before send not supported */
	rs485->delay_rts_before_send = 0;

	serial_out(up, LPC18XX_UART_RS485CTRL, rs485_ctrl_reg);
	serial_out(up, LPC18XX_UART_RS485DLY, rs485_dly_reg);

	port->rs485 = *rs485;

	return 0;
}

static int lpc18xx_serial_probe(struct platform_device *pdev)
{
	struct resource *regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	struct resource *irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	struct lpc18xx_uart_data *data;
	struct uart_8250_port uart;
	int ret;

	if (!regs || !irq) {
		dev_err(&pdev->dev, "no registers/irq defined\n");
		return -EINVAL;
	}

	memset(&uart, 0, sizeof(uart));

	uart.port.membase = devm_ioremap(&pdev->dev, regs->start,
					 resource_size(regs));
	if (!uart.port.membase)
		return -ENOMEM;

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->clk_uart = devm_clk_get(&pdev->dev, "uartclk");
	if (IS_ERR(data->clk_uart)) {
		dev_err(&pdev->dev, "uart clock not found\n");
		return PTR_ERR(data->clk_uart);
	}

	data->clk_reg = devm_clk_get(&pdev->dev, "reg");
	if (IS_ERR(data->clk_reg)) {
		dev_err(&pdev->dev, "reg clock not found\n");
		return PTR_ERR(data->clk_reg);
	}

	ret = clk_prepare_enable(data->clk_reg);
	if (ret) {
		dev_err(&pdev->dev, "unable to enable reg clock\n");
		return ret;
	}

	ret = clk_prepare_enable(data->clk_uart);
	if (ret) {
		dev_err(&pdev->dev, "unable to enable uart clock\n");
		goto dis_clk_reg;
	}

	ret = of_alias_get_id(pdev->dev.of_node, "serial");
	if (ret >= 0)
		uart.port.line = ret;

	spin_lock_init(&uart.port.lock);
	uart.port.dev = &pdev->dev;
	uart.port.irq = irq->start;
	uart.port.mapbase = regs->start;
	uart.port.iotype = UPIO_MEM32;
	uart.port.regshift = 2;
	uart.port.type = PORT_16550A;
	uart.port.flags = UPF_FIXED_PORT | UPF_FIXED_TYPE | UPF_SKIP_TEST;
	uart.port.uartclk = clk_get_rate(data->clk_uart);
	uart.port.private_data = data;
	uart.port.rs485_config = lpc18xx_rs485_config;

	ret = serial8250_register_8250_port(&uart);
	if (ret < 0) {
		dev_err(&pdev->dev, "unable to register 8250 port\n");
		goto dis_uart_clk;
	}

	data->line = ret;
	platform_set_drvdata(pdev, data);

	return 0;

dis_uart_clk:
	clk_disable_unprepare(data->clk_uart);
dis_clk_reg:
	clk_disable_unprepare(data->clk_reg);
	return ret;
}

static int lpc18xx_serial_remove(struct platform_device *pdev)
{
	struct lpc18xx_uart_data *data = platform_get_drvdata(pdev);

	serial8250_unregister_port(data->line);
	clk_disable_unprepare(data->clk_uart);
	clk_disable_unprepare(data->clk_reg);

	return 0;
}

static struct of_device_id lpc18xx_serial_match[] = {
	{ .compatible = "nxp,lpc1850-uart" },
	{ },
};
MODULE_DEVICE_TABLE(of, lpc18xx_serial_match);

static struct platform_driver lpc18xx_serial_driver = {
	.probe  = lpc18xx_serial_probe,
	.remove = lpc18xx_serial_remove,
	.driver = {
		.name = "lpc18xx-uart",
		.of_match_table = lpc18xx_serial_match,
	},
};
module_platform_driver(lpc18xx_serial_driver);

MODULE_AUTHOR("Joachim Eastwood <manabian@gmail.com>");
MODULE_DESCRIPTION("Serial port driver NXP LPC18xx/43xx devices");
MODULE_LICENSE("GPL v2");
