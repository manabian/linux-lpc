/*
 * Copyright (C) 2011 Team Embeded VOF
 *                    Ithamar R. Adema <ihamar.adema@team-embedded.nl>
 * Copyright (C) 2009 - 2010 NXP Semiconductors
 * Copyright (C) 2009 Fontys University of Applied Sciences, Eindhoven
 *                    Ed Schouten <e.schouten@fontys.nl>
 *                    Laurens Timmermans <l.timmermans@fontys.nl>
 *
 * Based on timer code from plat-lpc32xx.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/time.h>
#include <linux/err.h>
#include <linux/clockchips.h>
#include <linux/clk.h>

#include <asm/mach/time.h>

#include <mach/hardware.h>

#define IR		0x00
#define TCR		0x04
#define TC		0x08
#define PR		0x0c
#define PC		0x10
#define MCR		0x14
#define MR0		0x18
#define MR1		0x1c
#define MR2		0x20
#define CCR		0x28
#define CR0		0x2c
#define CR1		0x30
#define CR2		0x34
#define CR3		0x38
#define EMR		0x3c
#define CTCR		0x70

#define IR_MR_SHIFT	0
#define IR_CR_SHIFT	4

#define TCR_EN		(1 << 0)
#define TCR_RESET	(1 << 1)


static void __iomem *timer0 = (void __iomem *)APB_TIMER0_BASE;
static void __iomem *timer1 = (void __iomem *)APB_TIMER1_BASE;

static cycle_t lpc2k_clksrc_read(struct clocksource *cs)
{
	return __raw_readl(timer1 + TC);
}

static struct clocksource lpc2k_clksrc = {
	.name	= "lpc2k_clksrc",
	.rating	= 300,
	.read	= lpc2k_clksrc_read,
	.mask	= CLOCKSOURCE_MASK(32),
	.flags	= CLOCK_SOURCE_IS_CONTINUOUS,
};

static int lpc2k_clkevt_next_event(unsigned long delta,
	struct clock_event_device *dev)
{
	__raw_writel(TCR_RESET, timer0 + TCR);
	__raw_writel(delta, timer0 + PR);
	__raw_writel(TCR_EN, timer0 + TCR);

	return 0;
}

static void lpc2k_clkevt_mode(enum clock_event_mode mode,
	struct clock_event_device *dev)
{
	switch (mode) {
	case CLOCK_EVT_MODE_PERIODIC:
		BUG();
		break;

	case CLOCK_EVT_MODE_ONESHOT:
	case CLOCK_EVT_MODE_SHUTDOWN:
		/*
		 * Disable the timer. When using oneshot, we must also
		 * disable the timer to wait for the first call to
		 * set_next_event().
		 */
		__raw_writel(0, timer0 + TCR);
		break;

	case CLOCK_EVT_MODE_UNUSED:
	case CLOCK_EVT_MODE_RESUME:
		break;
	}
}

static struct clock_event_device lpc2k_clkevt = {
	.name		= "lpc2k_clkevt",
	.features	= CLOCK_EVT_FEAT_ONESHOT,
	.rating		= 300,
	.set_next_event	= lpc2k_clkevt_next_event,
	.set_mode	= lpc2k_clkevt_mode,
};

static irqreturn_t lpc2k_timer_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *evt = &lpc2k_clkevt;

	/* Clear match */
	__raw_writel((1 << IR_MR_SHIFT), timer0 + IR);

	evt->event_handler(evt);

	return IRQ_HANDLED;
}

static struct irqaction lpc2k_timer_irq = {
	.name		= "LPC2K Timer Tick",
	.flags		= IRQF_DISABLED | IRQF_TIMER | IRQF_IRQPOLL,
	.handler	= lpc2k_timer_interrupt,
};

/*
 * The clock management driver isn't initialized at this point, so the
 * clocks need to be enabled here manually and then tagged as used in
 * the clock driver initialization
 */
static void __init lpc2k_timer_init(void)
{
	unsigned long rate;
	struct clk *clk;

	clk = clk_get(NULL, "timer1");
	BUG_ON(IS_ERR(clk));
	clk_enable(clk);

	rate = clk_get_rate(clk);

	clk = clk_get(NULL, "timer0");
	BUG_ON(IS_ERR(clk));
	clk_enable(clk);

	/* Initial timer setup */
	__raw_writel(0, timer0 + TCR);
	__raw_writel(1 << IR_MR_SHIFT, timer0 + IR);
	__raw_writel(1, timer0 + MR0);
	__raw_writel(3 << 0, timer0 + MCR);

	/* Setup tick interrupt */
	setup_irq(IRQ_LPC2K_TIMER0, &lpc2k_timer_irq);

	/* Setup the clockevent structure. */
	clockevents_calc_mult_shift(&lpc2k_clkevt, rate, 5);
	lpc2k_clkevt.max_delta_ns = clockevent_delta2ns(-1,
		&lpc2k_clkevt);
	lpc2k_clkevt.min_delta_ns = clockevent_delta2ns(1,
		&lpc2k_clkevt) + 1;
	lpc2k_clkevt.cpumask = cpumask_of(0);
	clockevents_register_device(&lpc2k_clkevt);

	/* Use timer1 as clock source. */
	__raw_writel(TCR_RESET, timer1 + TCR);
	__raw_writel(0, timer1 + PR);
	__raw_writel(0, timer1 + MCR);
	__raw_writel(TCR_EN, timer1 + TCR);
	clocksource_register_hz(&lpc2k_clksrc, rate);
}

struct sys_timer lpc2k_timer = {
	.init		= &lpc2k_timer_init,
};
