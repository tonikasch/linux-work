/*
 * Allwinner USB Host EHCI driver
 *
 * (C) Copyright 2007-2012
 * Allwinner Technology Co., Ltd. <www.allwinnertech.com>
 * Copyright 2012 Andrey Panov
 *
 * Based on ehci-spear.c which is
 * Based on various ehci-*.c drivers
 * Based on Allwinner sources
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License. See the file COPYING in the main directory of this archive for
 * more details.
 */

#include <linux/clk.h>
#include <linux/jiffies.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/pm.h>
#include <linux/io.h>

struct sunxi_ehci {
	struct ehci_hcd ehci;
	struct clk *clk;
	int vbus_pin;
};

//#define to_sunxi_ehci(hcd)	(struct sunxi_ehci *)hcd_to_ehci(hcd)

static void sunxi_start_ehci(struct sunxi_ehci *ehci)
{
//	clk_prepare_enable(ehci->clk);
}

static void sunxi_stop_ehci(struct sunxi_ehci *ehci)
{
//	clk_disable_unprepare(ehci->clk);
}

//static void sunxi_set_power(struct sunxi_ehci *ehci, int on)
//{
//	gpio_write_one_pin_value(ehci->vbus_pin, on, NULL);
//}

static int ehci_sunxi_setup(struct usb_hcd *hcd)
{
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);
	int retval = 0;

	ehci->caps = hcd->regs;

	retval = ehci_setup(hcd);

//	if (retval)
//		return retval;

	return retval;
}

static const struct hc_driver ehci_sunxi_hc_driver = {
	.description			= hcd_name,
	.product_desc			= "sunxi EHCI",
	.hcd_priv_size			= sizeof(struct sunxi_ehci),

	/* generic hardware linkage */
	.irq				= ehci_irq,
	.flags				= HCD_MEMORY | HCD_USB2,

	/* basic lifecycle operations */
	.reset				= ehci_sunxi_setup,
	.start				= ehci_run,
	.stop				= ehci_stop,
	.shutdown			= ehci_shutdown,

	/* managing i/o requests and associated device resources */
	.urb_enqueue			= ehci_urb_enqueue,
	.urb_dequeue			= ehci_urb_dequeue,
	.endpoint_disable		= ehci_endpoint_disable,
	.endpoint_reset			= ehci_endpoint_reset,

	/* scheduling support */
	.get_frame_number		= ehci_get_frame,

	/* root hub support */
	.hub_status_data		= ehci_hub_status_data,
	.hub_control			= ehci_hub_control,
	.bus_suspend			= ehci_bus_suspend,
	.bus_resume			= ehci_bus_resume,
	.relinquish_port		= ehci_relinquish_port,
	.port_handed_over		= ehci_port_handed_over,
	.clear_tt_buffer_complete	= ehci_clear_tt_buffer_complete,
};

#ifdef CONFIG_PM
static int ehci_sunxi_drv_suspend(struct device *dev)
{
	struct usb_hcd *hcd = dev_get_drvdata(dev);
	bool do_wakeup = device_may_wakeup(dev);

	return ehci_suspend(hcd, do_wakeup);
}

static int ehci_sunxi_drv_resume(struct device *dev)
{
	struct usb_hcd *hcd = dev_get_drvdata(dev);

	ehci_resume(hcd, false);
	return 0;
}
#endif /* CONFIG_PM */

static SIMPLE_DEV_PM_OPS(ehci_sunxi_pm_ops, ehci_sunxi_drv_suspend,
		ehci_sunxi_drv_resume);

static u64 sunxi_ehci_dma_mask = DMA_BIT_MASK(32);

static int sunxi_ehci_hcd_drv_probe(struct platform_device *pdev)
{
	struct device_node *dn = pdev->dev.of_node;
	int retval = 0, irq;
	struct usb_hcd *hcd ;
	struct resource res;
	struct sunxi_ehci *ehci;

	if (usb_disabled())
		return -ENODEV;

	hcd = usb_create_hcd(&ehci_sunxi_hc_driver, &pdev->dev, dev_name(&pdev->dev));
	if (!hcd) {
		retval = -ENOMEM;
		goto fail_create_hcd;
	}

	retval = of_address_to_resource(dn, 0, &res);
	if (retval)
		goto fail_get_resource;

	hcd->regs = ioremap(res.start, resource_size(&res));
	if (!hcd->regs){
		retval = -ENOMEM;
		goto fail_map_regs;
	}

	hcd->rsrc_start = res.start;
	hcd->rsrc_len = resource_size(&res);

	irq = irq_of_parse_and_map(dn,0);
	if (irq < 0){
		retval=irq;
		goto fail_get_irq;
	}

	dev_dbg(&pdev->dev, "DT IRQ=%u\n", irq);

	retval = usb_add_hcd(hcd, irq, IRQF_SHARED);
	if (retval)
		goto fail_add_hcd;

//	ehci = (struct sunxi_ehci *)hcd_to_ehci(hcd);
//	sunxi_start_ehci(ehci);

	return retval;

#if 0
	struct resource *res;
	struct clk *usbh_clk;
	const struct hc_driver *driver = &ehci_sunxi_hc_driver;
	int irq, retval;
//	char clk_name[20] = "usbh_clk";
//	static int instance = -1;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		retval = irq;
		goto fail_irq_get;
	}


	/*
	 * Right now device-tree probed devices don't get dma_mask set.
	 * Since shared usb code relies on it, set it here for now.
	 * Once we have dma capability bindings this can go away.
	 */
	if (!pdev->dev.dma_mask)
		pdev->dev.dma_mask = &sunxi_ehci_dma_mask;

	/*
	 * Increment the device instance, when probing via device-tree
	 */
/*
	if (pdev->id < 0)
		instance++;
	else
		instance = pdev->id;
	sprintf(clk_name, "usbh.%01d_clk", instance);

	usbh_clk = clk_get(NULL, clk_name);
	if (IS_ERR(usbh_clk)) {
		dev_err(&pdev->dev, "Error getting interface clock\n");
		retval = PTR_ERR(usbh_clk);
		goto fail_get_usbh_clk;
	}
*/

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		retval = -ENODEV;
		goto fail_request_resource;
	}

	hcd->rsrc_start = res->start;
	hcd->rsrc_len = resource_size(res);
	if (!request_mem_region(hcd->rsrc_start, hcd->rsrc_len,
				driver->description)) {
		retval = -EBUSY;
		goto fail_request_resource;
	}

	hcd->regs = ioremap(hcd->rsrc_start, hcd->rsrc_len);
	if (hcd->regs == NULL) {
		dev_dbg(&pdev->dev, "error mapping memory\n");
		retval = -ENOMEM;
		goto fail_ioremap;
	}

//	ehci->clk = usbh_clk;


fail_add_hcd:
	sunxi_stop_ehci(ehci);
	iounmap(hcd->regs);
fail_ioremap:
	release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
fail_request_resource:
	usb_put_hcd(hcd);
fail_create_hcd:
//	clk_put(usbh_clk);
//fail_get_usbh_clk:
fail_irq_get:
	dev_err(&pdev->dev, "init fail, %d\n", retval);

#endif
fail_add_hcd:
fail_get_irq:
	iounmap(hcd->regs);
	release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
fail_map_regs:
fail_get_resource:
fail_create_hcd:
	return retval;
}

static int sunxi_ehci_hcd_drv_remove(struct platform_device *pdev)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);
	struct sunxi_ehci *ehci_p = (struct sunxi_ehci *)hcd_to_ehci(hcd);

	if (!hcd)
		return 0;
	if (in_interrupt())
		BUG();
	usb_remove_hcd(hcd);

	if (ehci_p->clk)
		sunxi_stop_ehci(ehci_p);
	iounmap(hcd->regs);
	release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
	usb_put_hcd(hcd);

	if (ehci_p->clk)
		clk_put(ehci_p->clk);

	return 0;
}

static struct of_device_id sunxi_ehci_id_table[] = {
	{ .compatible = "allwinner,sun4i-ehci", },
	{ },
};

static struct platform_driver sunxi_ehci_hcd_driver = {
	.probe		= sunxi_ehci_hcd_drv_probe,
	.remove		= sunxi_ehci_hcd_drv_remove,
	.shutdown	= usb_hcd_platform_shutdown,
	.driver		= {
		.name = "sunxi-ehci",
		.bus = &platform_bus_type,
		.pm = &ehci_sunxi_pm_ops,
		.of_match_table = of_match_ptr(sunxi_ehci_id_table),
	}
};

MODULE_ALIAS("platform:sunxi-ehci");
