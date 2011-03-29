/*
 * Copyright (C) 2011 Team Embeded VOF
 *     Ithamar R. Adema <ihamar.adema@team-embedded.nl>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/amba/pl022.h>
#include <linux/amba/mmci.h>
#include <linux/gpio.h>
#include <linux/io.h>

#include <asm/mach/arch.h>
#include <asm/mach-types.h>

#include <mach/mfp-lpc24xx.h>
#include <mach/hardware.h>

#include "common.h"

#define EXTPOLAR	0x14c

static unsigned long mfp_cfgs[] __initdata = {
	/* UART0 */
	P0_2_TXD0,
	P0_3_RXD0,
	/* USB Host */
	P0_12_USB_PPWR2,
	P0_13_USB_UP_LED2,
	P0_31_USB_Dp2,
	P1_30_VBUS,
	P1_31_USB_OVRCR2,
	/* USB Gadget */
	P0_29_USB_Dp1,
	P0_30_USB_Dn1,
	/* Ethernet */
	P1_0_ENET_TXD0,
	P1_1_ENET_TXD1,
	P1_4_ENET_TX_EN,
	P1_8_ENET_CRS,
	P1_9_ENET_RXD0,
	P1_10_ENET_RXD1,
	P1_14_ENET_RX_ERR,
	P1_15_ENET_REF_CLK,
	P1_16_ENET_MDC,
	P1_17_ENET_MDIO,
	/* I2C0 */
	P0_27_SDA0,
	P0_28_SCL0,
	/* SPI */
	P0_15_SCK,
	P0_17_MISO,
	P0_18_MOSI,
	/* LCD */
	P0_4_LCDVD0,
	P0_5_LCDVD1,
	P0_6_LCDVD8,
	P0_7_LCDVD9,
	P0_8_LCDVD16,
	P0_9_LCDVD17,
	P1_20_LCDVD10,
	P1_21_LCDVD11,
	P1_22_LCDVD12,
	P1_23_LCDVD13,
	P1_24_LCDVD14,
	P1_25_LCDVD15,
	P1_26_LCDVD20,
	P1_27_LCDVD21,
	P1_28_LCDVD22,
	P1_29_LCDVD23,
	P2_0_LCDPWR,
	P2_1_LCDLE,
	P2_2_LCDDCLK,
	P2_3_LCDFP,
	P2_4_LCDM,
	P2_5_LCDLP,
	P2_6_LCDVP4,
	P2_7_LCDVP5,
	P2_8_LCDVP6,
	P2_9_LCDVP7,
	P2_11_LCDCLKIN,
	P2_12_LCDVP18,
	P2_13_LCDVP19,
	P4_28_LCDVP2,
	P4_29_LCDVP3,
	/* Backlight */
	P1_18_PWM1,
	/* MMC/SD */
	P1_2_MCICLK,
	P1_3_MCICMD,
	P1_5_MCIPWR,
	P1_6_MCIDAT0,
	P1_7_MCIDAT1,
	P1_11_MCIDAT2,
	P1_12_MCIDAT3,
};

static struct pl022_ssp_controller ssp0_plat_data = {
	.bus_id = 0,
	.enable_dma = 0,
	.num_chipselect = 1,
};

static struct mmci_platform_data mci_plat_data = {
	.ocr_mask = MMC_VDD_32_33 | MMC_VDD_33_34,
	.capabilities = MMC_CAP_4_BIT_DATA,
	.f_max = 200000,
	.gpio_cd = -1,
	.gpio_wp = -1,
};

void __init ea2478devkit_init_machine(void)
{
	lpc2k_mfp_config(mfp_cfgs, ARRAY_SIZE(mfp_cfgs));

	lpc2k_add_uart(0);
	lpc2k_add_ssp(0, &ssp0_plat_data);
	lpc2k_add_mci(&mci_plat_data, 0 /* active low */);
}

static void __init ea2478devkit_init_irq(void)
{
	/* XXX workaround for the fact that EINT3 is connected to high-active
	 * signal, but can't be disabled. As the EINT3 interrupt is also used
	 * for GPIO interrupts, this will cause an interrupt storm without
	 * this setting.
	 */
	iowrite32(ioread32(APB_SCB_BASE + EXTPOLAR) | (1 << 3),
		APB_SCB_BASE + EXTPOLAR);

	lpc2k_init_irq();
	lpc2k_init_clocks(12000000UL, 32768UL);
}

MACHINE_START(EA2478DEVKIT, "Embedded Artists LPC2478 Developer's Kit")
	.init_irq	= ea2478devkit_init_irq,
	.timer		= &lpc2k_timer,
	.init_machine	= ea2478devkit_init_machine,
MACHINE_END
