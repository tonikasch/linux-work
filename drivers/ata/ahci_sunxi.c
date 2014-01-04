/*
 * Allwinner sunxi AHCI SATA platform driver
 * Copyright 2013 Olliver Schinagl <oliver@schinagl.nl>
 * Copyright 2014 Hans de Goede <hdegoede@redhat.com>
 *
 * based on the AHCI SATA platform driver by Jeff Garzik and Anton Vorontsov
 * Based on code from Allwinner Technology Co., Ltd. <www.allwinnertech.com>,
 * Daniel Wang <danielwang@allwinnertech.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 */

#include <linux/clk.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include "ahci.h"

#define AHCI_BISTAFR 0x00a0
#define AHCI_BISTCR 0x00a4
#define AHCI_BISTFCTR 0x00a8
#define AHCI_BISTSR 0x00ac
#define AHCI_BISTDECR 0x00b0
#define AHCI_DIAGNR0 0x00b4
#define AHCI_DIAGNR1 0x00b8
#define AHCI_OOBR 0x00bc
#define AHCI_PHYCS0R 0x00c0
#define AHCI_PHYCS1R 0x00c4
#define AHCI_PHYCS2R 0x00c8
#define AHCI_TIMER1MS 0x00e0
#define AHCI_GPARAM1R 0x00e8
#define AHCI_GPARAM2R 0x00ec
#define AHCI_PPARAMR 0x00f0
#define AHCI_TESTR 0x00f4
#define AHCI_VERSIONR 0x00f8
#define AHCI_IDR 0x00fc
#define AHCI_RWCR 0x00fc
#define AHCI_P0DMACR 0x0170
#define AHCI_P0PHYCR 0x0178
#define AHCI_P0PHYSR 0x017c

struct sunxi_ahci {
	struct ahci_host_priv hpriv;
	struct regulator *pwr;
	struct clk *sata_clk;
	struct clk *ahb_clk;
};

static void sunxi_clrbits(void __iomem *reg, u32 clr_val)
{
	u32 reg_val;

	reg_val = readl(reg);
	reg_val &= ~(clr_val);
	writel(reg_val, reg);
}

static void sunxi_setbits(void __iomem *reg, u32 set_val)
{
	u32 reg_val;

	reg_val = readl(reg);
	reg_val |= set_val;
	writel(reg_val, reg);
}

static void sunxi_clrsetbits(void __iomem *reg, u32 clr_val, u32 set_val)
{
	u32 reg_val;

	reg_val = readl(reg);
	reg_val &= ~(clr_val);
	reg_val |= set_val;
	writel(reg_val, reg);
}

static u32 sunxi_getbits(void __iomem *reg, u8 mask, u8 shift)
{
	return (readl(reg) >> shift) & mask;
}

static int sunxi_ahci_phy_init(struct device *dev, void __iomem *reg_base)
{
	u32 reg_val;
	int timeout;

	/* This magic is from the original code */
	writel(0, reg_base + AHCI_RWCR);
	mdelay(5);

	sunxi_setbits(reg_base + AHCI_PHYCS1R, BIT(19));
	sunxi_clrsetbits(reg_base + AHCI_PHYCS0R,
			 (0x7 << 24),
			 (0x5 << 24) | BIT(23) | BIT(18));
	sunxi_clrsetbits(reg_base + AHCI_PHYCS1R,
			 (0x3 << 16) | (0x1f << 8) | (0x3 << 6),
			 (0x2 << 16) | (0x6 << 8) | (0x2 << 6));
	sunxi_setbits(reg_base + AHCI_PHYCS1R, BIT(28) | BIT(15));
	sunxi_clrbits(reg_base + AHCI_PHYCS1R, BIT(19));
	sunxi_clrsetbits(reg_base + AHCI_PHYCS0R,
			 (0x7 << 20), (0x3 << 20));
	sunxi_clrsetbits(reg_base + AHCI_PHYCS2R,
			 (0x1f << 5), (0x19 << 5));
	mdelay(5);

	sunxi_setbits(reg_base + AHCI_PHYCS0R, (0x1 << 19));

	timeout = 0x100000;
	do {
		reg_val = sunxi_getbits(reg_base + AHCI_PHYCS0R, 0x7, 28);
	} while (--timeout && (reg_val != 0x2));
	if (!timeout) {
		dev_err(dev, "PHY power up failed.\n");
		return -EIO;
	}

	sunxi_setbits(reg_base + AHCI_PHYCS2R, (0x1 << 24));

	timeout = 0x100000;
	do {
		reg_val = sunxi_getbits(reg_base + AHCI_PHYCS2R, 0x1, 24);
	} while (--timeout && reg_val);
	if (!timeout) {
		dev_err(dev, "PHY calibration failed.\n");
		return -EIO;
	}
	mdelay(15);

	writel(0x7, reg_base + AHCI_RWCR);

	return 0;
}

void sunxi_ahci_pre_start_engine(struct ata_port *ap)
{
	struct ahci_host_priv *hpriv = ap->host->private_data;

	/* Setup DMA before DMA start */
	sunxi_clrsetbits(hpriv->mmio + AHCI_P0DMACR, 0x0000ff00, 0x00004400);
}

static int sunxi_ahci_enable_clks(struct sunxi_ahci *ahci)
{
	int ret;

	ret = clk_prepare_enable(ahci->sata_clk);
	if (ret)
		return ret;

	ret = clk_prepare_enable(ahci->ahb_clk);
	if (ret)
		clk_disable_unprepare(ahci->sata_clk);

	return ret;
}

static void sunxi_ahci_disable_clks(struct sunxi_ahci *ahci)
{
	clk_disable_unprepare(ahci->ahb_clk);
	clk_disable_unprepare(ahci->sata_clk);
}

static void sunxi_ahci_host_stop(struct ata_host *host)
{
	struct ahci_host_priv *hpriv = host->private_data;
	struct sunxi_ahci *ahci = hpriv->plat_data;

	if (!IS_ERR(ahci->pwr))
		regulator_disable(ahci->pwr);

	sunxi_ahci_disable_clks(ahci);
}

static struct ata_port_operations sunxi_ahci_platform_ops = {
	.inherits	= &ahci_ops,
	.host_stop	= sunxi_ahci_host_stop,
};

static const struct ata_port_info sunxiahci_port_info = {
	AHCI_HFLAGS(AHCI_HFLAG_32BIT_ONLY | AHCI_HFLAG_NO_MSI |
			  AHCI_HFLAG_NO_PMP | AHCI_HFLAG_YES_NCQ),
	.flags		= AHCI_FLAG_COMMON | ATA_FLAG_NCQ,
	.pio_mask	= ATA_PIO4,
	.udma_mask	= ATA_UDMA6,
	.port_ops	= &sunxi_ahci_platform_ops,
};

static struct scsi_host_template sunxi_ahci_platform_sht = {
	AHCI_SHT("sunxi_ahci"),
};

static int sunxi_ahci_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct ata_port_info *ppi[] = { &sunxiahci_port_info, NULL };
	struct sunxi_ahci *ahci;
	struct ata_host *host;
	int ret;

	ahci = devm_kzalloc(&pdev->dev, sizeof(*ahci), GFP_KERNEL);
	if (!ahci)
		return -ENOMEM;

	ahci->pwr = devm_regulator_get_optional(dev, "pwr");
	if (IS_ERR(ahci->pwr) && PTR_ERR(ahci->pwr) == -EPROBE_DEFER)
		return -EPROBE_DEFER;

	host = ata_host_alloc_pinfo(dev, ppi, 1);
	if (!host)
		return -ENOMEM;

	host->private_data = &ahci->hpriv;
	host->flags |= ATA_HOST_PARALLEL_SCAN;

	ahci->hpriv.flags = (unsigned long)ppi[0]->private_data;
	ahci->hpriv.plat_data = ahci;
	ahci->hpriv.pre_start_engine = sunxi_ahci_pre_start_engine;
	ahci->hpriv.mmio = devm_ioremap_resource(dev,
			      platform_get_resource(pdev, IORESOURCE_MEM, 0));
	if (IS_ERR(ahci->hpriv.mmio))
		return PTR_ERR(ahci->hpriv.mmio);

	ahci->ahb_clk = devm_clk_get(&pdev->dev, "ahb_sata");
	if (IS_ERR(ahci->ahb_clk))
		return PTR_ERR(ahci->ahb_clk);

	ahci->sata_clk = devm_clk_get(&pdev->dev, "pll6_sata");
	if (IS_ERR(ahci->sata_clk))
		return PTR_ERR(ahci->sata_clk);

	ret = sunxi_ahci_enable_clks(ahci);
	if (ret)
		return ret;

	if (!IS_ERR(ahci->pwr)) {
		ret = regulator_enable(ahci->pwr);
		if (ret) {
			sunxi_ahci_disable_clks(ahci);
			return ret;
		}
	}

	ret = sunxi_ahci_phy_init(dev, ahci->hpriv.mmio);
	if (ret) {
		sunxi_ahci_host_stop(host);
		return ret;
	}

	ahci_save_initial_config(dev, &ahci->hpriv, 0, 0);

	ret = ahci_reset_controller(host);
	if (ret) {
		sunxi_ahci_host_stop(host);
		return ret;
	}

	ahci_init_controller(host);
	ahci_print_info(host, "sunxi");

	ret = ata_host_activate(host, platform_get_irq(pdev, 0),
				ahci_interrupt, 0, &sunxi_ahci_platform_sht);
	if (ret)
		sunxi_ahci_host_stop(host);

	return ret;
}

#ifdef CONFIG_PM_SLEEP
static int sunxi_ahci_susp(struct device *dev)
{
	struct ata_host *host = dev_get_drvdata(dev);
	struct ahci_host_priv *hpriv = host->private_data;
	struct sunxi_ahci *ahci = hpriv->plat_data;
	int ret;

	/*
	 * AHCI spec rev1.1 section 8.3.3:
	 * Software must disable interrupts prior to requesting a
	 * transition of the HBA to D3 state.
	 */
	sunxi_clrbits(hpriv->mmio + HOST_CTL, HOST_IRQ_EN);

	ret = ata_host_suspend(host, PMSG_SUSPEND);
	if (ret)
		return ret;

	sunxi_ahci_disable_clks(ahci);

	return 0;
}

static int sunxi_ahci_resume(struct device *dev)
{
	struct ata_host *host = dev_get_drvdata(dev);
	struct ahci_host_priv *hpriv = host->private_data;
	struct sunxi_ahci *ahci = hpriv->plat_data;
	int ret;

	ret = sunxi_ahci_enable_clks(ahci);
	if (ret)
		return ret;

	if (dev->power.power_state.event == PM_EVENT_SUSPEND) {
		ret = ahci_reset_controller(host);
		if (ret)
			return ret;

		ahci_init_controller(host);
	}

	ata_host_resume(host);

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(sunxi_ahci_pmo, sunxi_ahci_susp, sunxi_ahci_resume);

static const struct of_device_id sunxi_ahci_of_match[] = {
	{ .compatible = "allwinner,sun4i-a10-ahci" },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, sunxi_ahci_of_match);

static struct platform_driver sunxi_ahci_driver = {
	.probe = sunxi_ahci_probe,
	.remove = ata_platform_remove_one,
	.driver = {
		.name = "sunxi-ahci",
		.owner = THIS_MODULE,
		.of_match_table = sunxi_ahci_of_match,
		.pm = &sunxi_ahci_pmo,
	},
};
module_platform_driver(sunxi_ahci_driver);

MODULE_DESCRIPTION("Allwinner sunxi AHCI SATA platform driver");
MODULE_AUTHOR("Olliver Schinagl <oliver@schinagl.nl>");
MODULE_LICENSE("GPL");
