/*
 * Copyright (C) 2011 Team Embeded VOF
 *     Ithamar R. Adema <ihamar.adema@team-embedded.nl>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/serial_8250.h>
#include <linux/clk.h>
#include <linux/io.h>

#include <linux/amba/bus.h>
#include <linux/amba/mmci.h>
#include <linux/amba/pl022.h>

#include <mach/hardware.h>

/* System Control Block - System Controls and Status register */
#define SCS		0x1a0

#define __IORESOURCE_MEM_16K(x)	{	\
	.start = x,			\
	.end   = x + SZ_16K - 1,	\
	.flags = IORESOURCE_MEM,	\
	}

#define __IORESOURCE_IRQ(x)	{	\
	.start = x,			\
	. end  = x,			\
	}

/* UARTs
 *   0/1 are available on all LPC2Ks
 *   2/3 are available on LPC23XX and up.
 *
 * NOTE: the dev.init_name assignment is there so we can use the clk API to
 *       retrieve and enable the clock, since dev_name() is called from
 *       the clk_get implementation before the platform device is registered.
 */

#define LPC2K_UART(uartnum) \
	static struct plat_serial8250_port lpc2k_pdata_uart##uartnum[] = { \
		{ \
			.mapbase	= APB_UART##uartnum##_BASE, \
			.membase	= (void __iomem *) \
						APB_UART##uartnum##_BASE, \
			.irq		= IRQ_LPC2K_UART##uartnum, \
			.flags		= UPF_SKIP_TEST | UPF_BOOT_AUTOCONF, \
			.iotype		= UPIO_MEM, \
			.regshift	= 2, \
			.uartclk	= 0, \
		}, \
		{ }, \
	}; \
	static struct resource lpc2k_resources_uart##uartnum[] = { \
		[0] = __IORESOURCE_MEM_16K(APB_UART##uartnum##_BASE), \
		[1] = __IORESOURCE_IRQ(IRQ_LPC2K_UART##uartnum), \
	}; \
	static struct platform_device lpc2k_device_uart##uartnum = { \
		.dev = { \
			.init_name = "serial8250." __stringify(uartnum), \
			.platform_data = lpc2k_pdata_uart##uartnum, \
		}, \
		.name		= "serial8250", \
		.id		= uartnum, \
		.resource	= lpc2k_resources_uart##uartnum, \
		.num_resources	= ARRAY_SIZE(lpc2k_resources_uart##uartnum), \
	}


LPC2K_UART(0);
LPC2K_UART(1);
LPC2K_UART(2);
LPC2K_UART(3);

static struct platform_device *uarts[] __initdata = {
	&lpc2k_device_uart0,
	&lpc2k_device_uart1,
	&lpc2k_device_uart2,
	&lpc2k_device_uart3,
};

void __init lpc2k_add_uart(int nr)
{
	struct platform_device *pdev;
	struct clk *clk;
	int res;

	BUG_ON(nr < 0 || nr > ARRAY_SIZE(uarts));
	pdev = uarts[nr];

	clk = clk_get(&pdev->dev, NULL);
	BUG_ON(IS_ERR(clk));
	clk_enable(clk);
	((struct plat_serial8250_port *)pdev->dev.platform_data)->uartclk =
		clk_get_rate(clk);

	res = platform_device_register(pdev);
	if (res)
		dev_err(&pdev->dev, "Unable to register: %d\n", res);
}

/* SSP, aka pl022
 *
 * 1-only available on enhanced LPC21XX and LPC22XX.
 * 0/1 available on LPC23XX and up.
 */

static struct amba_device lpc2k_device_ssp0 = {
	.dev = {
		.coherent_dma_mask = ~0,
		.init_name = "ssp0",
	},
	.res = __IORESOURCE_MEM_16K(APB_SSP0_BASE),
	.irq = { IRQ_LPC2K_SPI0, NO_IRQ },
};

static struct amba_device lpc2k_device_ssp1 = {
	.dev = {
		.coherent_dma_mask = ~0,
		.init_name = "ssp1",
	},
	.res = __IORESOURCE_MEM_16K(APB_SSP1_BASE),
	.irq = { IRQ_LPC2K_SPI1, NO_IRQ },
};

void __init lpc2k_add_ssp(int nr, void *platform_data)
{
	struct amba_device *adev;
	int res;

	BUG_ON(nr < 0 || nr > 1);
	adev = nr ? &lpc2k_device_ssp1 : &lpc2k_device_ssp0;
	adev->dev.platform_data = platform_data;

	res = amba_device_register(adev, &iomem_resource);
	if (res)
		dev_err(&adev->dev, "Unable to register: %d\n", res);
}

/* SD/MMC, aka pl180/1
 *
 * Available on LPC23XX and up.
 */

static struct amba_device lpc2k_device_mci = {
	.dev = {
		.coherent_dma_mask = ~0,
		.init_name = "mci",
	},
	.res = __IORESOURCE_MEM_16K(APB_MCI_BASE),
	.irq = { IRQ_LPC2K_MCI, NO_IRQ },
};

void __init lpc2k_add_mci(void *platform_data, int pwr_high)
{
	int res;
	u32 scs;

	/* Set MCIPWR pin to be active high or low */
	scs = ioread32(APB_SCB_BASE + SCS);
	scs = pwr_high ? scs | (1 << 3) : scs & ~(1 << 3);
	iowrite32(scs, APB_SCB_BASE + SCS);

	lpc2k_device_mci.dev.platform_data = platform_data;
	res = amba_device_register(&lpc2k_device_mci, &iomem_resource);
	if (res)
		dev_err(&lpc2k_device_mci.dev, "Unable to register: %d\n", res);
}
