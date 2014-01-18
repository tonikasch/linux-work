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

#include <linux/ahci_platform.h>
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

#ifdef CONFIG_AHCI_SUNXI

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

static int ahci_sunxi_phy_init(struct device *dev, void __iomem *reg_base)
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

static void ahci_sunxi_start_engine(struct ata_port *ap)
{
	void __iomem *port_mmio = ahci_port_base(ap);
	struct ahci_host_priv *hpriv = ap->host->private_data;

	/* Setup DMA before DMA start */
	sunxi_clrsetbits(hpriv->mmio + AHCI_P0DMACR, 0x0000ff00, 0x00004400);

	/* Start DMA */
	sunxi_setbits(port_mmio + PORT_CMD, PORT_CMD_START);
}

static int ahci_sunxi_init(struct device *dev, struct ahci_host_priv *hpriv,
			   void __iomem *reg_base)
{
	hpriv->start_engine = ahci_sunxi_start_engine;
	return ahci_sunxi_phy_init(dev, reg_base);
}

int ahci_sunxi_resume(struct device *dev)
{
	struct ata_host *host = dev_get_drvdata(dev);
	struct ahci_host_priv *hpriv = host->private_data;

	return ahci_sunxi_phy_init(dev, hpriv->mmio);
}

static const struct ata_port_info ahci_sunxi_port_info = {
	AHCI_HFLAGS(AHCI_HFLAG_32BIT_ONLY | AHCI_HFLAG_NO_MSI |
			  AHCI_HFLAG_NO_PMP | AHCI_HFLAG_YES_NCQ),
	.flags		= AHCI_FLAG_COMMON | ATA_FLAG_NCQ,
	.pio_mask	= ATA_PIO4,
	.udma_mask	= ATA_UDMA6,
	.port_ops	= &ahci_platform_ops,
};

struct ahci_platform_data ahci_sunxi_pdata = {
	.init = ahci_sunxi_init,
	.resume = ahci_sunxi_resume,
	.ata_port_info = &ahci_sunxi_port_info,
};

#endif
