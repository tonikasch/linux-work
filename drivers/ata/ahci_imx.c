/*
 * copyright (c) 2013 Freescale Semiconductor, Inc.
 * Freescale IMX AHCI SATA platform driver
 *
 * based on the AHCI SATA platform driver by Jeff Garzik and Anton Vorontsov
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/ahci_platform.h>
#include <linux/of_device.h>
#include <linux/mfd/syscon.h>
#include <linux/mfd/syscon/imx6q-iomuxc-gpr.h>
#include <linux/libata.h>
#include "ahci.h"

#ifdef CONFIG_AHCI_IMX

enum {
	PORT_PHY_CTL = 0x178,			/* Port0 PHY Control */
	PORT_PHY_CTL_PDDQ_LOC = 0x100000,	/* PORT_PHY_CTL bits */
	HOST_TIMER1MS = 0xe0,			/* Timer 1-ms */
};

enum {
	CLK_SATA,
	CLK_SATA_REF,
	CLK_AHB
};

struct imx_ahci_priv {
	struct platform_device *ahci_pdev;
	struct regmap *gpr;
	bool first_time;
};

static int ahci_imx_hotplug;
module_param_named(hotplug, ahci_imx_hotplug, int, 0644);
MODULE_PARM_DESC(hotplug, "AHCI IMX hot-plug support (0=Don't support, 1=support)");

static void ahci_imx_error_handler(struct ata_port *ap)
{
	u32 reg_val;
	struct ata_device *dev;
	struct ata_host *host = dev_get_drvdata(ap->dev);
	struct ahci_host_priv *hpriv = host->private_data;
	void __iomem *mmio = hpriv->mmio;
	struct imx_ahci_priv *imxpriv = hpriv->plat_data;
	int i;

	ahci_error_handler(ap);

	if (!(imxpriv->first_time) || ahci_imx_hotplug)
		return;

	imxpriv->first_time = false;

	ata_for_each_dev(dev, &ap->link, ENABLED)
		return;
	/*
	 * Disable link to save power.  An imx ahci port can't be recovered
	 * without full reset once the pddq mode is enabled making it
	 * impossible to use as part of libata LPM.
	 */
	reg_val = readl(mmio + PORT_PHY_CTL);
	writel(reg_val | PORT_PHY_CTL_PDDQ_LOC, mmio + PORT_PHY_CTL);
	regmap_update_bits(imxpriv->gpr, IOMUXC_GPR13,
			IMX6Q_GPR13_SATA_MPLL_CLK_EN,
			!IMX6Q_GPR13_SATA_MPLL_CLK_EN);

	for (i = CLK_AHB; i >= 0; i--) {
		clk_disable_unprepare(hpriv->clks[i]);
		/* Stop ahci_platform.c from trying to use the clks */
		clk_put(hpriv->clks[i]);
		hpriv->clks[i] = NULL;
	}
}

static struct ata_port_operations ahci_imx_ops = {
	.inherits	= &ahci_platform_ops,
	.error_handler	= ahci_imx_error_handler,
};

static const struct ata_port_info ahci_imx_port_info = {
	.flags		= AHCI_FLAG_COMMON,
	.pio_mask	= ATA_PIO4,
	.udma_mask	= ATA_UDMA6,
	.port_ops	= &ahci_imx_ops,
};

static int imx6q_sata_init(struct device *dev, struct ahci_host_priv *hpriv,
			   void __iomem *mmio)
{
	unsigned int reg_val;
	struct imx_ahci_priv *imxpriv;

	imxpriv = devm_kzalloc(dev, sizeof(*imxpriv), GFP_KERNEL);
	if (!imxpriv)
		return -ENOMEM;

	hpriv->plat_data = imxpriv;
	imxpriv->gpr =
		syscon_regmap_lookup_by_compatible("fsl,imx6q-iomuxc-gpr");
	if (IS_ERR(imxpriv->gpr)) {
		dev_err(dev, "failed to find fsl,imx6q-iomux-gpr regmap\n");
		return PTR_ERR(imxpriv->gpr);
	}

	/*
	 * set PHY Paremeters, two steps to configure the GPR13,
	 * one write for rest of parameters, mask of first write
	 * is 0x07ffffff, and the other one write for setting
	 * the mpll_clk_en.
	 */
	regmap_update_bits(imxpriv->gpr, 0x34, IMX6Q_GPR13_SATA_RX_EQ_VAL_MASK
			| IMX6Q_GPR13_SATA_RX_LOS_LVL_MASK
			| IMX6Q_GPR13_SATA_RX_DPLL_MODE_MASK
			| IMX6Q_GPR13_SATA_SPD_MODE_MASK
			| IMX6Q_GPR13_SATA_MPLL_SS_EN
			| IMX6Q_GPR13_SATA_TX_ATTEN_MASK
			| IMX6Q_GPR13_SATA_TX_BOOST_MASK
			| IMX6Q_GPR13_SATA_TX_LVL_MASK
			| IMX6Q_GPR13_SATA_MPLL_CLK_EN
			| IMX6Q_GPR13_SATA_TX_EDGE_RATE
			, IMX6Q_GPR13_SATA_RX_EQ_VAL_3_0_DB
			| IMX6Q_GPR13_SATA_RX_LOS_LVL_SATA2M
			| IMX6Q_GPR13_SATA_RX_DPLL_MODE_2P_4F
			| IMX6Q_GPR13_SATA_SPD_MODE_3P0G
			| IMX6Q_GPR13_SATA_MPLL_SS_EN
			| IMX6Q_GPR13_SATA_TX_ATTEN_9_16
			| IMX6Q_GPR13_SATA_TX_BOOST_3_33_DB
			| IMX6Q_GPR13_SATA_TX_LVL_1_025_V);
	regmap_update_bits(imxpriv->gpr, 0x34, IMX6Q_GPR13_SATA_MPLL_CLK_EN,
			IMX6Q_GPR13_SATA_MPLL_CLK_EN);
	usleep_range(100, 200);

	/*
	 * Configure the HWINIT bits of the HOST_CAP and HOST_PORTS_IMPL,
	 * and IP vendor specific register HOST_TIMER1MS.
	 * Configure CAP_SSS (support stagered spin up).
	 * Implement the port0.
	 * Get the ahb clock rate, and configure the TIMER1MS register.
	 */
	reg_val = readl(mmio + HOST_CAP);
	if (!(reg_val & HOST_CAP_SSS)) {
		reg_val |= HOST_CAP_SSS;
		writel(reg_val, mmio + HOST_CAP);
	}
	reg_val = readl(mmio + HOST_PORTS_IMPL);
	if (!(reg_val & 0x1)) {
		reg_val |= 0x1;
		writel(reg_val, mmio + HOST_PORTS_IMPL);
	}

	if (!hpriv->clks[CLK_AHB]) {
		dev_err(dev, "no ahb clk, need sata, sata_ref and ahb clks\n");
		return -ENOENT;
	}
	reg_val = clk_get_rate(hpriv->clks[CLK_AHB]) / 1000;
	writel(reg_val, mmio + HOST_TIMER1MS);

	return 0;
}

static void imx6q_sata_exit(struct device *dev)
{
	struct ata_host *host = dev_get_drvdata(dev);
	struct ahci_host_priv *hpriv = host->private_data;
	struct imx_ahci_priv *imxpriv = hpriv->plat_data;

	if (hpriv->clks[CLK_SATA])
		regmap_update_bits(imxpriv->gpr, 0x34,
			IMX6Q_GPR13_SATA_MPLL_CLK_EN,
			!IMX6Q_GPR13_SATA_MPLL_CLK_EN);
}

static void imx_ahci_suspend(struct device *dev)
{
	struct ata_host *host = dev_get_drvdata(dev);
	struct ahci_host_priv *hpriv = host->private_data;
	struct imx_ahci_priv *imxpriv = hpriv->plat_data;

	/* Check the CLKs have not been gated off in the initialization. */
	if (hpriv->clks[CLK_SATA])
		regmap_update_bits(imxpriv->gpr, IOMUXC_GPR13,
				IMX6Q_GPR13_SATA_MPLL_CLK_EN,
				!IMX6Q_GPR13_SATA_MPLL_CLK_EN);
}

static int imx_ahci_resume(struct device *dev)
{
	struct ata_host *host = dev_get_drvdata(dev);
	struct ahci_host_priv *hpriv = host->private_data;
	struct imx_ahci_priv *imxpriv = hpriv->plat_data;

	if (hpriv->clks[CLK_SATA]) {
		regmap_update_bits(imxpriv->gpr, IOMUXC_GPR13,
				IMX6Q_GPR13_SATA_MPLL_CLK_EN,
				IMX6Q_GPR13_SATA_MPLL_CLK_EN);
		usleep_range(1000, 2000);
	}

	return 0;
}

struct ahci_platform_data imx6q_sata_pdata = {
	.init = imx6q_sata_init,
	.exit = imx6q_sata_exit,
	.ata_port_info = &ahci_imx_port_info,
	.suspend = imx_ahci_suspend,
	.resume = imx_ahci_resume,
};

MODULE_AUTHOR("Richard Zhu <Hong-Xing.Zhu@freescale.com>");
MODULE_ALIAS("ahci:imx");

#endif
