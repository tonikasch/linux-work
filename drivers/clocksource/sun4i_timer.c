/*
 * Allwinner A1X SoCs timer handling.
 *
 * Copyright (C) 2012 Maxime Ripard
 *
 * Maxime Ripard <maxime.ripard@free-electrons.com>
 *
 * Based on code from
 * Allwinner Technology Co., Ltd. <www.allwinnertech.com>
 * Benn Huang <benn@allwinnertech.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/clk.h>
#include <linux/clockchips.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqreturn.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>

#include <asm/sched_clock.h>

#define TIMER_IRQ_EN_REG	0x00
#define TIMER_IRQ_EN(val)		BIT(val)
#define TIMER_IRQ_ST_REG	0x04
#define TIMER_CTL_REG(val)	(0x10 * val + 0x10)
#define TIMER_CTL_ENABLE		BIT(0)
#define TIMER_CTL_AUTORELOAD		BIT(1)
#define TIMER_CTL_ONESHOT		BIT(7)
#define TIMER_INTVAL_REG(val)	(0x10 * val + 0x14)
#define TIMER_CNTVAL_REG(val)	(0x10 * val + 0x18)

#define TIMER_CNT64_CTL_REG	0xa0
#define TIMER_CNT64_CTL_CLR		BIT(0)
#define TIMER_CNT64_CTL_RL		BIT(1)
#define TIMER_CNT64_LOW_REG	0xa4
#define TIMER_CNT64_HIGH_REG	0xa8

static void __iomem *timer_base;

static void sun4i_clkevt_time_stop(void)
{
	u32 val = readl(timer_base + TIMER_CTL_REG(0));
	writel(val & ~TIMER_CTL_ENABLE, timer_base + TIMER_CTL_REG(0));
	udelay(1);
}

static void sun4i_clkevt_time_setup(unsigned long delay)
{
	writel(delay, timer_base + TIMER_INTVAL_REG(0));
}

static void sun4i_clkevt_time_start(bool periodic)
{
	u32 val = readl(timer_base + TIMER_CTL_REG(0));

	if (periodic)
		val &= ~TIMER_CTL_ONESHOT;
	else
		val |= TIMER_CTL_ONESHOT;

	writel(val | TIMER_CTL_ENABLE, timer_base + TIMER_CTL_REG(0));
}

static void sun4i_clkevt_mode(enum clock_event_mode mode,
			      struct clock_event_device *clk)
{
	switch (mode) {
	case CLOCK_EVT_MODE_PERIODIC:
		sun4i_clkevt_time_stop();
		sun4i_clkevt_time_start(true);
		break;
	case CLOCK_EVT_MODE_ONESHOT:
		sun4i_clkevt_time_stop();
		sun4i_clkevt_time_start(false);
		break;
	case CLOCK_EVT_MODE_UNUSED:
	case CLOCK_EVT_MODE_SHUTDOWN:
	default:
		sun4i_clkevt_time_stop();
		break;
	}
}

static int sun4i_clkevt_next_event(unsigned long evt,
				   struct clock_event_device *unused)
{
	sun4i_clkevt_time_stop();
	sun4i_clkevt_time_setup(evt);
	sun4i_clkevt_time_start(false);

	return 0;
}

static struct clock_event_device sun4i_clockevent = {
	.name = "sun4i_tick",
	.rating = 300,
	.features = CLOCK_EVT_FEAT_PERIODIC | CLOCK_EVT_FEAT_ONESHOT,
	.set_mode = sun4i_clkevt_mode,
	.set_next_event = sun4i_clkevt_next_event,
};


static irqreturn_t sun4i_timer_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *evt = (struct clock_event_device *)dev_id;

	writel(0x1, timer_base + TIMER_IRQ_ST_REG);
	evt->event_handler(evt);

	return IRQ_HANDLED;
}

static struct irqaction sun4i_timer_irq = {
	.name = "sun4i_timer0",
	.flags = IRQF_DISABLED | IRQF_TIMER | IRQF_IRQPOLL,
	.handler = sun4i_timer_interrupt,
	.dev_id = &sun4i_clockevent,
};

static u32 sun4i_timer_sched_read(void)
{
	u32 reg = readl(timer_base + TIMER_CNT64_CTL_REG);
	writel(reg | TIMER_CNT64_CTL_RL, timer_base + TIMER_CNT64_CTL_REG);
	while (readl(timer_base + TIMER_CNT64_CTL_REG) & TIMER_CNT64_CTL_REG);

	return readl(timer_base + TIMER_CNT64_LOW_REG);
}

static cycle_t sun4i_timer_clksrc_read(struct clocksource *c)
{
	return sun4i_timer_sched_read();
}

static void __init sun4i_timer_init(struct device_node *node)
{
	struct clk *clk;
	int ret, irq;
	u32 val;

	timer_base = of_iomap(node, 0);
	if (!timer_base)
		panic("Can't map registers");

	irq = irq_of_parse_and_map(node, 0);
	if (irq <= 0)
		panic("Can't parse IRQ");

	clk = of_clk_get(node, 0);
	if (IS_ERR(clk))
		panic("Can't get timer clock");
	clk_prepare_enable(clk);

	writel(TIMER_CNT64_CTL_CLR, timer_base + TIMER_CNT64_CTL_REG);
	setup_sched_clock(sun4i_timer_sched_read, 32, clk_get_rate(clk));
	clocksource_mmio_init(timer_base + TIMER_CNT64_LOW_REG, node->name,
			      clk_get_rate(clk), 300, 32,
			      sun4i_timer_clksrc_read);

	writel(clk_get_rate(clk) / HZ,
	       timer_base + TIMER_INTVAL_REG(0));

	/* set clock source to HOSC, 16 pre-division */
	val = readl(timer_base + TIMER_CTL_REG(0));
	val &= ~(0x07 << 4);
	val &= ~(0x03 << 2);
	val |= (4 << 4) | (1 << 2);
	writel(val, timer_base + TIMER_CTL_REG(0));

	/* set mode to auto reload */
	val = readl(timer_base + TIMER_CTL_REG(0));
	writel(val | TIMER_CTL_AUTORELOAD, timer_base + TIMER_CTL_REG(0));

	ret = setup_irq(irq, &sun4i_timer_irq);
	if (ret)
		pr_warn("failed to setup irq %d\n", irq);

	/* Enable timer0 interrupt */
	val = readl(timer_base + TIMER_IRQ_EN_REG);
	writel(val | TIMER_IRQ_EN(0), timer_base + TIMER_IRQ_EN_REG);

	sun4i_clockevent.cpumask = cpumask_of(0);

	clockevents_config_and_register(&sun4i_clockevent, clk_get_rate(clk),
					0x1, 0xffffffff);
}
CLOCKSOURCE_OF_DECLARE(sun4i, "allwinner,sun4i-timer",
		       sun4i_timer_init);
