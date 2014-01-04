/*
 * Generic platform ehci driver
 *
 * Copyright 2007 Steven Brown <sbrown@cortland.com>
 * Copyright 2010-2012 Hauke Mehrtens <hauke@hauke-m.de>
 *
 * Derived from the ohci-ssb driver
 * Copyright 2007 Michael Buesch <m@bues.ch>
 *
 * Derived from the EHCI-PCI driver
 * Copyright (c) 2000-2004 by David Brownell
 *
 * Derived from the ohci-pci driver
 * Copyright 1999 Roman Weissgaerber
 * Copyright 2000-2002 David Brownell
 * Copyright 1999 Linus Torvalds
 * Copyright 1999 Gregory P. Smith
 *
 * Licensed under the GNU/GPL. See COPYING for details.
 */
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/hrtimer.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/usb.h>
#include <linux/usb/hcd.h>
#include <linux/usb/ehci_pdriver.h>

#include "ehci.h"

#define DRIVER_DESC "EHCI generic platform driver"

struct ehci_platform_priv {
	struct clk *clk;
	struct phy *phy;
};

static const char hcd_name[] = "ehci-platform";

static int ehci_platform_reset(struct usb_hcd *hcd)
{
	struct platform_device *pdev = to_platform_device(hcd->self.controller);
	struct usb_ehci_pdata *pdata = dev_get_platdata(&pdev->dev);
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);
	int retval;

	hcd->has_tt = pdata->has_tt;
	ehci->has_synopsys_hc_bug = pdata->has_synopsys_hc_bug;
	ehci->big_endian_desc = pdata->big_endian_desc;
	ehci->big_endian_mmio = pdata->big_endian_mmio;

	if (pdata->pre_setup) {
		retval = pdata->pre_setup(hcd);
		if (retval < 0)
			return retval;
	}

	ehci->caps = hcd->regs + pdata->caps_offset;
	retval = ehci_setup(hcd);
	if (retval)
		return retval;

	if (pdata->no_io_watchdog)
		ehci->need_io_watchdog = 0;
	return 0;
}

static int ehci_platform_power_on(struct platform_device *dev)
{
	struct usb_hcd *hcd = platform_get_drvdata(dev);
	struct ehci_platform_priv *priv =
		(struct ehci_platform_priv *)hcd_to_ehci(hcd)->priv;
	int ret;

	if (!IS_ERR(priv->clk)) {
		ret = clk_prepare_enable(priv->clk);
		if (ret)
			return ret;
	}

	if (!IS_ERR(priv->phy)) {
		ret = phy_init(priv->phy);
		if (ret)
			goto err_disable_clk;

		ret = phy_power_on(priv->phy);
		if (ret)
			goto err_exit_phy;
	}

	return 0;

err_exit_phy:
	phy_exit(priv->phy);
err_disable_clk:
	if (!IS_ERR(priv->clk))
		clk_disable_unprepare(priv->clk);

	return ret;
}

static void ehci_platform_power_off(struct platform_device *dev)
{
	struct usb_hcd *hcd = platform_get_drvdata(dev);
	struct ehci_platform_priv *priv =
		(struct ehci_platform_priv *)hcd_to_ehci(hcd)->priv;

	if (!IS_ERR(priv->phy)) {
		phy_power_off(priv->phy);
		phy_exit(priv->phy);
	}
	if (!IS_ERR(priv->clk))
		clk_disable_unprepare(priv->clk);
}

static struct hc_driver __read_mostly ehci_platform_hc_driver;

static const struct ehci_driver_overrides platform_overrides __initconst = {
	.reset =		ehci_platform_reset,
	.extra_priv_size =	sizeof(struct ehci_platform_priv),
};

static struct usb_ehci_pdata ehci_platform_defaults = {
	.power_on = 		ehci_platform_power_on,
	.power_off = 		ehci_platform_power_off,
};

static int ehci_platform_probe(struct platform_device *dev)
{
	struct usb_hcd *hcd;
	struct resource *res_mem;
	struct usb_ehci_pdata *pdata;
	int irq;
	int err;

	if (usb_disabled())
		return -ENODEV;

	/*
	 * use reasonable defaults so platforms don't have to provide these.
	 * with DT probing on ARM, none of these are set.
	 */
	if (!dev_get_platdata(&dev->dev))
		dev->dev.platform_data = &ehci_platform_defaults;

	err = dma_coerce_mask_and_coherent(&dev->dev, DMA_BIT_MASK(32));
	if (err)
		return err;

	pdata = dev_get_platdata(&dev->dev);

	irq = platform_get_irq(dev, 0);
	if (irq < 0) {
		dev_err(&dev->dev, "no irq provided");
		return irq;
	}
	res_mem = platform_get_resource(dev, IORESOURCE_MEM, 0);
	if (!res_mem) {
		dev_err(&dev->dev, "no memory resource provided");
		return -ENXIO;
	}

	hcd = usb_create_hcd(&ehci_platform_hc_driver, &dev->dev,
			     dev_name(&dev->dev));
	if (!hcd)
		return -ENOMEM;

	if (pdata == &ehci_platform_defaults) {
		struct ehci_platform_priv *priv = (struct ehci_platform_priv *)
						  hcd_to_ehci(hcd)->priv;

		priv->phy = devm_phy_get(&dev->dev, "usb_phy");
		if (IS_ERR(priv->phy) && PTR_ERR(priv->phy) == -EPROBE_DEFER)
			return -EPROBE_DEFER;

		priv->clk = devm_clk_get(&dev->dev, "ehci_clk");
	}

	platform_set_drvdata(dev, hcd);
	if (pdata->power_on) {
		err = pdata->power_on(dev);
		if (err < 0)
			goto err_put_hcd;
	}

	hcd->rsrc_start = res_mem->start;
	hcd->rsrc_len = resource_size(res_mem);

	hcd->regs = devm_ioremap_resource(&dev->dev, res_mem);
	if (IS_ERR(hcd->regs)) {
		err = PTR_ERR(hcd->regs);
		goto err_power;
	}
	err = usb_add_hcd(hcd, irq, IRQF_SHARED);
	if (err)
		goto err_power;

	return err;

err_power:
	if (pdata->power_off)
		pdata->power_off(dev);
err_put_hcd:
	usb_put_hcd(hcd);

	return err;
}

static int ehci_platform_remove(struct platform_device *dev)
{
	struct usb_hcd *hcd = platform_get_drvdata(dev);
	struct usb_ehci_pdata *pdata = dev_get_platdata(&dev->dev);

	usb_remove_hcd(hcd);
	usb_put_hcd(hcd);

	if (pdata->power_off)
		pdata->power_off(dev);

	if (pdata == &ehci_platform_defaults)
		dev->dev.platform_data = NULL;

	return 0;
}

#ifdef CONFIG_PM

static int ehci_platform_suspend(struct device *dev)
{
	struct usb_hcd *hcd = dev_get_drvdata(dev);
	struct usb_ehci_pdata *pdata = dev_get_platdata(dev);
	struct platform_device *pdev =
		container_of(dev, struct platform_device, dev);
	bool do_wakeup = device_may_wakeup(dev);
	int ret;

	ret = ehci_suspend(hcd, do_wakeup);

	if (pdata->power_suspend)
		pdata->power_suspend(pdev);

	return ret;
}

static int ehci_platform_resume(struct device *dev)
{
	struct usb_hcd *hcd = dev_get_drvdata(dev);
	struct usb_ehci_pdata *pdata = dev_get_platdata(dev);
	struct platform_device *pdev =
		container_of(dev, struct platform_device, dev);

	if (pdata->power_on) {
		int err = pdata->power_on(pdev);
		if (err < 0)
			return err;
	}

	ehci_resume(hcd, false);
	return 0;
}

#else /* !CONFIG_PM */
#define ehci_platform_suspend	NULL
#define ehci_platform_resume	NULL
#endif /* CONFIG_PM */

static const struct of_device_id ehci_platform_ids[] = {
	{ .compatible = "via,vt8500-ehci", },
	{ .compatible = "wm,prizm-ehci", },
	{ .compatible = "platform-ehci", },
	{}
};

static const struct platform_device_id ehci_platform_table[] = {
	{ "ehci-platform", 0 },
	{ }
};
MODULE_DEVICE_TABLE(platform, ehci_platform_table);

static const struct dev_pm_ops ehci_platform_pm_ops = {
	.suspend	= ehci_platform_suspend,
	.resume		= ehci_platform_resume,
};

static struct platform_driver ehci_platform_driver = {
	.id_table	= ehci_platform_table,
	.probe		= ehci_platform_probe,
	.remove		= ehci_platform_remove,
	.shutdown	= usb_hcd_platform_shutdown,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "ehci-platform",
		.pm	= &ehci_platform_pm_ops,
		.of_match_table = ehci_platform_ids,
	}
};

static int __init ehci_platform_init(void)
{
	if (usb_disabled())
		return -ENODEV;

	pr_info("%s: " DRIVER_DESC "\n", hcd_name);

	ehci_init_driver(&ehci_platform_hc_driver, &platform_overrides);
	return platform_driver_register(&ehci_platform_driver);
}
module_init(ehci_platform_init);

static void __exit ehci_platform_cleanup(void)
{
	platform_driver_unregister(&ehci_platform_driver);
}
module_exit(ehci_platform_cleanup);

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_AUTHOR("Hauke Mehrtens");
MODULE_AUTHOR("Alan Stern");
MODULE_LICENSE("GPL");
