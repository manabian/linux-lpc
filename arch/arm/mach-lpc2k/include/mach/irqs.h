/*
 * Copyright 2011 Team Embedded VOF
 *     Ithamar R. Adema <ihamar.adema@team-embedded.nl>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef MACH_LPC2K_IRQS_H
#define MACH_LPC2K_IRQS_H

/* VIC */
#define IRQ_LPC2K_WDINT		0
#define IRQ_LPC2K_RSV0		1
#define IRQ_LPC2K_DBGRX		2
#define IRQ_LPC2K_DBGTX		3
#define IRQ_LPC2K_TIMER0	4
#define IRQ_LPC2K_TIMER1	5
#define IRQ_LPC2K_UART0		6
#define IRQ_LPC2K_UART1		7
#define IRQ_LPC2K_PWM0		8
#define IRQ_LPC2K_I2C0		9
#define IRQ_LPC2K_SPI0		10
#define IRQ_LPC2K_SPI1		11
#define IRQ_LPC2K_PLL		12
#define IRQ_LPC2K_RTC		13
#define IRQ_LPC2K_EINT0		14
#define IRQ_LPC2K_EINT1		15
#define IRQ_LPC2K_EINT2		16
#define IRQ_LPC2K_EINT3		17
#define IRQ_LPC2K_ADC		18
#define IRQ_LPC2K_I2C1		19
#define IRQ_LPC2K_BOD		20
#define IRQ_LPC2K_ETH		21
#define IRQ_LPC2K_USB		22
#define IRQ_LPC2K_CAN		23
#define IRQ_LPC2K_MCI		24
#define IRQ_LPC2K_DMA		25
#define IRQ_LPC2K_TIMER2	26
#define IRQ_LPC2K_TIMER3	27
#define IRQ_LPC2K_UART2		28
#define IRQ_LPC2K_UART3		29
#define IRQ_LPC2K_I2C2		30
#define IRQ_LPC2K_I2S		31

#define NR_IRQS			32

#endif /* MACH_LPC2K_IRQS_H */
