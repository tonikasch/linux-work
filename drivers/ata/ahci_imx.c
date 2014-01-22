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

static void ahci_imx_host_stop(struct ata_host *host);

static void ahci_imx_error_handler(struct ata_port *ap)
{
	u32 reg_val;
	struct ata_device *dev;
	struct ata_host *host = dev_get_drvdata(ap->dev);
	struct ahci_host_priv *hpriv = host->private_data;
	void __iomem *mmio = hpriv->mmio;
	struct imx_ahci_priv *imxpriv = hpriv->plat_data;

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
	ahci_platform_disable_clks(hpriv);
	imxpriv->no_device = true;
}

static int ahci_imx_softreset(struct ata_link *link, unsigned int *class,
		       unsigned long deadline)
{
	struct ata_port *ap = link->ap;
	struct imx_ahci_priv *imxpriv = dev_get_drvdata(ap->dev->parent);
	int ret = -EIO;

	if (imxpriv->type == AHCI_IMX53)
		ret = ahci_pmp_retry_srst_ops.softreset(link, class, deadline);
	else if (imxpriv->type == AHCI_IMX6Q)
		ret = ahci_ops.softreset(link, class, deadline);

	return ret;
}

static struct ata_port_operations ahci_imx_ops = {
	.inherits	= &ahci_ops,
	.host_stop	= ahci_imx_host_stop,
	.error_handler	= ahci_imx_error_handler,
	.softreset	= ahci_imx_softreset,
};

static const struct ata_port_info ahci_imx_port_info = {
	.flags		= AHCI_FLAG_COMMON,
	.pio_mask	= ATA_PIO4,
	.udma_mask	= ATA_UDMA6,
	.port_ops	= &ahci_imx_ops,
};

static int imx_ahci_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct ahci_host_priv *hpriv;
	struct imx_ahci_priv *imxpriv;
	unsigned int reg_val;
	int rc;

	imxpriv = devm_kzalloc(dev, sizeof(*imxpriv), GFP_KERNEL);
	if (!imxpriv)
		return -ENOMEM;

	hpriv = ahci_platform_get_resources(pdev);
	if (IS_ERR(hpriv))
		return PTR_ERR(hpriv);
	if (!hpriv->clks[CLK_AHB]) {
		dev_err(dev, "no ahb clk, need sata, sata_ref and ahb clks\n");
		rc = -ENOENT;
		goto put_resources;
	}
	hpriv->plat_data = imxpriv;

	rc = ahci_platform_enable_resources(hpriv);
	if (rc)
		goto put_resources;

	imxpriv->gpr =
		syscon_regmap_lookup_by_compatible("fsl,imx6q-iomuxc-gpr");
	if (IS_ERR(imxpriv->gpr)) {
		dev_err(dev, "failed to find fsl,imx6q-iomux-gpr regmap\n");
		rc = PTR_ERR(imxpriv->gpr);
		goto disable_resources;
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
	reg_val = readl(hpriv->mmio + HOST_CAP);
	if (!(reg_val & HOST_CAP_SSS)) {
		reg_val |= HOST_CAP_SSS;
		writel(reg_val, hpriv->mmio + HOST_CAP);
	}
	reg_val = readl(hpriv->mmio + HOST_PORTS_IMPL);
	if (!(reg_val & 0x1)) {
		reg_val |= 0x1;
		writel(reg_val, hpriv->mmio + HOST_PORTS_IMPL);
	}

	reg_val = clk_get_rate(hpriv->clks[CLK_AHB]) / 1000;
	writel(reg_val, hpriv->mmio + HOST_TIMER1MS);

	rc = ahci_platform_init_host(pdev, hpriv, &ahci_imx_port_info, 0, 0);
	if (rc)
		goto disable_resources;

	return 0;

disable_resources:
	ahci_platform_disable_resources(hpriv);
put_resources:
	ahci_platform_put_resources(hpriv);
	return rc;
}

static void ahci_imx_host_stop(struct ata_host *host)
{
	struct ahci_host_priv *hpriv = host->private_data;
	struct imx_ahci_priv *imxpriv = hpriv->plat_data;

	if (!imxpriv->no_device) {
		regmap_update_bits(imxpriv->gpr, 0x34,
			IMX6Q_GPR13_SATA_MPLL_CLK_EN,
			!IMX6Q_GPR13_SATA_MPLL_CLK_EN);
		ahci_platform_disable_clks(hpriv);
	}
}

static void imx_ahci_suspend(struct device *dev)
{
	struct ata_host *host = dev_get_drvdata(dev);
	struct ahci_host_priv *hpriv = host->private_data;
	struct imx_ahci_priv *imxpriv = hpriv->plat_data;
	int rc;

	rc = ahci_platform_suspend_host(dev);
	if (rc)
		return rc;

	/*
	 * If no_device is set, The CLKs had been gated off in the
	 * initialization so don't do it again here.
	 */
	if (!imxpriv->no_device) {
		regmap_update_bits(imxpriv->gpr, IOMUXC_GPR13,
				IMX6Q_GPR13_SATA_MPLL_CLK_EN,
				!IMX6Q_GPR13_SATA_MPLL_CLK_EN);
		ahci_platform_disable_clks(hpriv);
	}

	return 0;
}

static int imx_ahci_resume(struct device *dev)
{
	struct ata_host *host = dev_get_drvdata(dev);
	struct ahci_host_priv *hpriv = host->private_data;
	struct imx_ahci_priv *imxpriv = hpriv->plat_data;
	int ret;

	if (!imxpriv->no_device) {
		ret = ahci_platform_enable_clks(hpriv);
		if (ret < 0) {
			dev_err(dev, "pre-enable sata_ref clock err:%d\n", ret);
			return ret;
		}

	if (!imxpriv->no_device)
		ret = imx_sata_clock_enable(dev);

	return ahci_platform_resume_host(dev);
}

static SIMPLE_DEV_PM_OPS(ahci_imx_pm_ops, imx_ahci_suspend, imx_ahci_resume);

static const struct of_device_id imx_ahci_of_match[] = {
	{ .compatible = "fsl,imx6q-ahci", },
	{},
};
MODULE_DEVICE_TABLE(of, imx_ahci_of_match);



static struct platform_driver imx_ahci_driver = {
	.probe = imx_ahci_probe,
	.remove = ata_platform_remove_one,
	.driver = {
		.name = "ahci-imx",
		.owner = THIS_MODULE,
		.of_match_table = imx_ahci_of_match,
		.pm = &ahci_imx_pm_ops,
	},
};
module_platform_driver(imx_ahci_driver);

MODULE_DESCRIPTION("Freescale i.MX AHCI SATA platform driver");
MODULE_AUTHOR("Richard Zhu <Hong-Xing.Zhu@freescale.com>");
MODULE_ALIAS("ahci:imx");

#endif
