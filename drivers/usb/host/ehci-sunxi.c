/*
 * Copyright 2013 Arokux
 *
 * Arokux <arokux@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/of_gpio.h>

#define SW_VA_USB1_IO_BASE 0x01c14000
//#define SW_VA_USB1_IO_BASE 0x01c14000
#define SW_USB_PMU_IRQ_ENABLE   0x800

#define SW_VA_CCM_IO_BASE 			0x01c20000
#define SW_VA_CCM_AHBMOD_OFFSET		0x60
#define SW_VA_CCM_USBCLK_OFFSET     0xcc
#define SW_VA_DRAM_IO_BASE			0x01c01000
#define SW_SDRAM_REG_HPCR_USB1		(0x250 + ((1 << 2) * 4))
#define SW_SDRAM_REG_HPCR_USB2		(0x250 + ((1 << 2) * 5))


#define MEM(addr) ioremap(addr, 0x1000)

#define SUNXI_USB_DMA_ALIGN ARCH_DMA_MINALIGN


static void dbg_registers(void)
{
	printk("[%s]: probe, clock: "
		"SW_VA_CCM_AHBMOD_OFFSET(0x%x), "
		"SW_VA_CCM_USBCLK_OFFSET(0x%x); "
		"usb1: SW_USB_PMU_IRQ_ENABLE(0x%x), "
		"dram:(0x%x, 0x%x)\n",
		"sunxi-ehci", readl(MEM(SW_VA_CCM_IO_BASE + SW_VA_CCM_AHBMOD_OFFSET)),
		readl(MEM(SW_VA_CCM_IO_BASE + SW_VA_CCM_USBCLK_OFFSET)),
		readl(MEM(SW_VA_USB1_IO_BASE + SW_USB_PMU_IRQ_ENABLE)),
		readl(MEM(SW_VA_DRAM_IO_BASE + SW_SDRAM_REG_HPCR_USB1)),
		readl(MEM(SW_VA_DRAM_IO_BASE + SW_SDRAM_REG_HPCR_USB2)));
}

static void usb_passby(resource_size_t base, u32 enable)
{
	unsigned long reg_value = 0;
	unsigned long bits = 0;
	static DEFINE_SPINLOCK(lock);
	unsigned long flags = 0;
	void __iomem *addr = MEM(base + SW_USB_PMU_IRQ_ENABLE); //FIXME

	spin_lock_irqsave(&lock, flags);

	bits =	BIT(10) | /* AHB Master interface INCR8 enable */
			BIT(9)  | /* AHB Master interface burst type INCR4 enable */
			BIT(8)  | /* AHB Master interface INCRX align enable */
			BIT(0);   /* ULPI bypass enable */

	reg_value = readl(addr);

	if (enable)
		reg_value |= bits;
	else
		reg_value &= ~bits;

	writel(reg_value, addr);

	spin_unlock_irqrestore(&lock, flags);

	return;
}

struct sunxi_ehci_hcd {
	struct device *dev;
	struct usb_hcd *hcd;
	struct clk *phy_clk;
	struct clk *phy_rst_clk;
	struct clk *ahb_ehci_clk;
};

static const struct ehci_driver_overrides sunxi_overrides __initconst = {
	//.product_desc =	"Allwinner EHCI Controller",
	.reset =	NULL,
};

static struct hc_driver __read_mostly sunxi_ehci_hc_driver;

static int ehci_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct sunxi_ehci_hcd *sunxi_ehci;
	struct resource *res;
	struct usb_hcd *hcd;
	int irq;
	int ret;
	int gpio;

	dbg_registers();

	if (pdev->num_resources != 2) {
		dev_err(&pdev->dev, "hcd probe: invalid num_resources: %i\n",
		       pdev->num_resources);
		return -ENODEV;
	}

	if (pdev->resource[0].flags != IORESOURCE_MEM
			|| pdev->resource[1].flags != IORESOURCE_IRQ) {
		dev_err(&pdev->dev, "hcd probe: invalid resource type\n");
		return -ENODEV;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "Failed to get I/O memory\n");
		return -ENXIO;
	}
	usb_passby(res->start, 1);

	irq = platform_get_irq(pdev, 0);
	if (!irq) {
		dev_err(&pdev->dev, "Failed to get IRQ\n");
		return -ENODEV;
	}

	sunxi_ehci = devm_kzalloc(&pdev->dev, sizeof(struct sunxi_ehci_hcd),
								GFP_KERNEL);
	if (!sunxi_ehci) {
		return -ENOMEM;
	}

	sunxi_ehci->phy_clk = devm_clk_get(&pdev->dev, "usb_phy");
	if (IS_ERR(sunxi_ehci->phy_clk)) {
		dev_err(&pdev->dev, "Failed to get usb_phy clock\n");
		return PTR_ERR(sunxi_ehci->phy_clk);
	}
	ret = clk_prepare_enable(sunxi_ehci->phy_clk);
	if (ret)
		return ret;

	//needed?
	mdelay(10);

	sunxi_ehci->phy_rst_clk = devm_clk_get(&pdev->dev, "phy1_reset");
	if (IS_ERR(sunxi_ehci->phy_rst_clk)) {
		dev_err(&pdev->dev, "Failed to get phy1_reset clock\n");
		ret = PTR_ERR(sunxi_ehci->phy_rst_clk);
		goto fail1;
	}
	ret = clk_prepare_enable(sunxi_ehci->phy_rst_clk);
	if (ret)
		goto fail1;
	
	//this can be needed on some sunxi
	//aw_ccu_reg->UsbClk.OHCIClkSrc = 0;
	mdelay(10);

	sunxi_ehci->ahb_ehci_clk = devm_clk_get(&pdev->dev, "ahb_ehci0");
	if (IS_ERR(sunxi_ehci->ahb_ehci_clk)) {
		dev_err(&pdev->dev, "Failed to get ahb_ehci0 clock\n");
		ret = PTR_ERR(sunxi_ehci->ahb_ehci_clk);
		goto fail2;
	}
	ret = clk_prepare_enable(sunxi_ehci->ahb_ehci_clk);
	if (ret)
		goto fail2;

	mdelay(10);

	ehci_init_driver(&sunxi_ehci_hc_driver, &sunxi_overrides);

	hcd = usb_create_hcd(&sunxi_ehci_hc_driver, &pdev->dev,
							dev_name(&pdev->dev));

	if (!hcd) {
		dev_err(&pdev->dev, "Unable to create HCD\n");
		ret = -ENOMEM;
		goto fail3;
	}

	sunxi_ehci->hcd = hcd;

	hcd->rsrc_start = res->start;
	hcd->rsrc_len = resource_size(res);
	hcd->regs = devm_ioremap(&pdev->dev, res->start, hcd->rsrc_len);
	if (!hcd->regs) {
		dev_err(&pdev->dev, "Failed to remap I/O memory\n");
		ret = -ENOMEM;
		goto fail4;
	}
	hcd_to_ehci(hcd)->caps = hcd->regs;
	
	//seems to be enabled by default, of not it should be
	//enabled in U-Boot
    //hci_port_configure(1 /*port num*/, 1 /*enable*/);
		
	gpio = of_get_gpio(np, 0);
	if (!gpio_is_valid(gpio)) {
		ret = -1; //FIXME
		dev_err(&pdev->dev, "No GPIO in device tree\n");
		goto fail4;
	}
	ret = gpio_request_one(gpio, GPIOF_DIR_OUT, "usb_port_vbus");
	if (ret)
		goto fail4;
	gpio_set_value(gpio, 1);

	dbg_registers();

	ret = usb_add_hcd(hcd, irq, IRQF_SHARED | IRQF_DISABLED);
	if (ret) {
		dev_err(&pdev->dev, "Failed to add USB HCD\n");
		goto fail4;
	}

	mdelay(10);

	platform_set_drvdata(pdev, hcd);

	dbg_registers();

	return 0;

fail4:
	usb_put_hcd(hcd);
fail3:
	clk_disable_unprepare(sunxi_ehci->ahb_ehci_clk);
fail2:
	clk_disable_unprepare(sunxi_ehci->phy_rst_clk);
fail1:
	clk_disable_unprepare(sunxi_ehci->phy_clk);

	return ret;
}

static int ehci_remove(struct platform_device *pdev)
{
	printk("Buy from ehci!\n");
	return 0;
}

static const struct of_device_id ehci_of_match[] = {
	{.compatible = "allwinner,sunxi-ehci"},
	{},
};
//MODULE_DEVICE_TABLE(of, ehci_of_match);


static struct platform_driver ehci_sunxi_driver = {
	.driver = {
		//.owner = THIS_MODULE,
		.of_match_table = ehci_of_match,
		.name = "sunxi-ehci",
	},
	.probe = ehci_probe,
	.remove = ehci_remove,
#ifdef	CONFIG_PM
	//.suspend = ehci_suspend,
	//.resume = ehci_resume,
#endif
};
