/*
 *      sunxi Watchdog Driver
 *
 *      Copyright (c) 2013 Carlo Caione
 *                    2012 Henrik Nordstrom
 *
 *      This program is free software; you can redistribute it and/or
 *      modify it under the terms of the GNU General Public License
 *      as published by the Free Software Foundation; either version
 *      2 of the License, or (at your option) any later version.
 *
 *      Based on xen_wdt.c
 *      (c) Copyright 2010 Novell, Inc.
 */

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/watchdog.h>

#define WDT_MAX_TIMEOUT         16
#define WDT_MIN_TIMEOUT         1
#define WDT_MODE_TIMEOUT(n)     ((n) << 3)
#define WDT_TIMEOUT_MASK        WDT_MODE_TIMEOUT(0x0F)

#define WDT_CTRL                0x00
#define WDT_MODE                0x04

#define WDT_MODE_RST_EN         (1 << 1)
#define WDT_MODE_EN             (1 << 0)

#define WDT_CTRL_RESTART        (1 << 0)
#define WDT_CTRL_RESERVED       (0x0a57 << 1)
#define DRV_NAME		"sunxi-wdt"
#define DRV_VERSION		"1.0"

static bool nowayout = WATCHDOG_NOWAYOUT;
static int heartbeat = WDT_MAX_TIMEOUT;

static const int wdt_timeout_map[] = {
	[1] = 0b0001,
	[2] = 0b0010,
	[3] = 0b0011,
	[4] = 0b0100,
	[5] = 0b0101,
	[6] = 0b0110,
	[8] = 0b0111,
	[10] = 0b1000,
	[12] = 0b1001,
	[14] = 0b1010,
	[16] = 0b1011,
};

static int sunxi_wdt_ping(struct watchdog_device *sunxi_wdt_dev)
{
	u32 reg;
	void __iomem *wdt_base = watchdog_get_drvdata(sunxi_wdt_dev);

	reg = ioread32(wdt_base + WDT_CTRL);
	reg |= (WDT_CTRL_RESTART | WDT_CTRL_RESERVED);
	iowrite32(reg, wdt_base + WDT_CTRL);

	return 0;
}

static int sunxi_wdt_set_timeout(struct watchdog_device *sunxi_wdt_dev,
		unsigned int timeout)
{
	u32 reg;
	void __iomem *wdt_base = watchdog_get_drvdata(sunxi_wdt_dev);

	if ((timeout > WDT_MAX_TIMEOUT) || (0 == wdt_timeout_map[timeout]))
		return -EINVAL;

	sunxi_wdt_dev->timeout = timeout;

	reg = ioread32(wdt_base + WDT_MODE);
	reg &= ~(WDT_TIMEOUT_MASK);
	reg |= WDT_MODE_TIMEOUT(wdt_timeout_map[sunxi_wdt_dev->timeout]);
	iowrite32(reg, wdt_base + WDT_MODE);

	sunxi_wdt_ping(sunxi_wdt_dev);

	return 0;
}

static int sunxi_wdt_stop(struct watchdog_device *sunxi_wdt_dev)
{
	u32 reg;
	void __iomem *wdt_base = watchdog_get_drvdata(sunxi_wdt_dev);

	reg = ioread32(wdt_base + WDT_MODE);
	reg &= ~(WDT_MODE_RST_EN | WDT_MODE_EN);
	iowrite32(reg, wdt_base + WDT_MODE);

	return 0;
}

static int sunxi_wdt_start(struct watchdog_device *sunxi_wdt_dev)
{
	u32 reg;
	int ret;
	void __iomem *wdt_base = watchdog_get_drvdata(sunxi_wdt_dev);

	ret = sunxi_wdt_set_timeout(sunxi_wdt_dev, sunxi_wdt_dev->timeout);
	if (ret < 0)
		return ret;

	reg = ioread32(wdt_base + WDT_MODE);
	reg |= (WDT_MODE_RST_EN | WDT_MODE_EN);
	iowrite32(reg, wdt_base + WDT_MODE);

	return 0;
}

static const struct watchdog_info sunxi_wdt_info = {
	.identity	= DRV_NAME,
	.options	= WDIOF_SETTIMEOUT |
			  WDIOF_KEEPALIVEPING |
			  WDIOF_MAGICCLOSE,
};

static const struct watchdog_ops sunxi_wdt_ops = {
	.owner		= THIS_MODULE,
	.start		= sunxi_wdt_start,
	.stop		= sunxi_wdt_stop,
	.ping		= sunxi_wdt_ping,
	.set_timeout	= sunxi_wdt_set_timeout,
};

static struct watchdog_device sunxi_wdt_dev = {
	.info		= &sunxi_wdt_info,
	.ops		= &sunxi_wdt_ops,
	.timeout	= WDT_MAX_TIMEOUT,
	.max_timeout	= WDT_MAX_TIMEOUT,
	.min_timeout	= WDT_MIN_TIMEOUT,
};

static int __init sunxi_wdt_probe(struct platform_device *pdev)
{
	int err;
	void __iomem *wdt_base;

	wdt_base = of_iomap(pdev->dev.of_node, 0);
	if (unlikely(!wdt_base)) {
		err = -ENOMEM;
		goto error_mem_out;
	}

	sunxi_wdt_dev.parent = &pdev->dev;
	watchdog_init_timeout(&sunxi_wdt_dev, heartbeat, &pdev->dev);
	watchdog_set_nowayout(&sunxi_wdt_dev, nowayout);

	err = watchdog_register_device(&sunxi_wdt_dev);
	if (unlikely(err))
		goto error_wdt;

	watchdog_set_drvdata(&sunxi_wdt_dev, wdt_base);

	pr_info("Watchdog enabled (heartbeat=%d sec, nowayout=%d)",
			sunxi_wdt_dev.timeout, nowayout);
	return 0;

error_wdt:
	iounmap(wdt_base);
error_mem_out:
	return err;
}

static int __exit sunxi_wdt_remove(struct platform_device *pdev)
{
	void __iomem *wdt_base = watchdog_get_drvdata(&sunxi_wdt_dev);

	sunxi_wdt_stop(&sunxi_wdt_dev);
	watchdog_unregister_device(&sunxi_wdt_dev);
	iounmap(wdt_base);

	return 0;
}

static const struct of_device_id sunxi_wdt_dt_ids[] = {
	{ .compatible = "allwinner,sun4i-wdt" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, sunxi_wdt_dt_ids);

static struct platform_driver sunxi_wdt_driver = {
	.probe		= sunxi_wdt_probe,
	.remove		= sunxi_wdt_remove,
	.driver		= {
		.owner		= THIS_MODULE,
		.name		= DRV_NAME,
		.of_match_table	= of_match_ptr(sunxi_wdt_dt_ids)
	},
};

module_platform_driver(sunxi_wdt_driver);

module_param(heartbeat, int, 0);
MODULE_PARM_DESC(heartbeat, "Watchdog heartbeat in seconds");

module_param(nowayout, bool, 0);
MODULE_PARM_DESC(nowayout, "Watchdog cannot be stopped once started "
		"(default=" __MODULE_STRING(WATCHDOG_NOWAYOUT) ")");

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Carlo Caione <carlo.caione@gmail.com>");
MODULE_AUTHOR("Henrik Nordstrom <henrik@henriknordstrom.net>");
MODULE_DESCRIPTION("sunxi WatchDog Timer Driver");
MODULE_VERSION(DRV_VERSION);
