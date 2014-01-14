/*
 * Allwinner sun4i USB phy driver
 *
 * Copyright (C) 2014 Hans de Goede <hdegoede@redhat.com>
 *
 * Based on code from
 * Allwinner Technology Co., Ltd. <www.allwinnertech.com>
 *
 * Modelled after: Samsung S5P/EXYNOS SoC series MIPI CSIS/DSIM DPHY driver
 * Copyright (C) 2013 Samsung Electronics Co., Ltd.
 * Author: Sylwester Nawrocki <s.nawrocki@samsung.com>
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

#include <linux/clk.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/reset.h>

#define REG_ISCR			0x00
#define REG_PHYCTL			0x04
#define REG_PHYBIST			0x08
#define REG_PHYTUNE			0x0c

#define SUNXI_AHB_ICHR8_EN		BIT(10)
#define SUNXI_AHB_INCR4_BURST_EN	BIT(9)
#define SUNXI_AHB_INCRX_ALIGN_EN	BIT(8)
#define SUNXI_ULPI_BYPASS_EN		BIT(0)

#define MAX_PHYS			3

struct sun4i_usb_phy_data {
	struct clk *clk;
	void __iomem *base;
	struct mutex mutex;
	int num_phys;
	u32 disc_thresh;
	struct sun4i_usb_phy {
		struct phy *phy;
		void __iomem *pmu_irq;
		struct regulator *vbus;
		struct reset_control *reset;
		int index;
	} phys[MAX_PHYS];
};

#define to_sun4i_usb_phy_data(phy) \
	container_of((phy), struct sun4i_usb_phy_data, phys[(phy)->index])

static void sun4i_usb_phy_write(struct sun4i_usb_phy *phy, u32 addr, u32 data,
				int len)
{
	struct sun4i_usb_phy_data *phy_data = to_sun4i_usb_phy_data(phy);
	u32 temp, usbc_bit = BIT(phy->index * 2);
	int i;

	mutex_lock(&phy_data->mutex);

	for (i = 0; i < len; i++) {
		temp = readl(phy_data->base + REG_PHYCTL);

		/* clear the address portion */
		temp &= ~(0xff << 8);

		/* set the address */
		temp |= ((addr + i) << 8);
		writel(temp, phy_data->base + REG_PHYCTL);

		/* set the data bit and clear usbc bit*/
		temp = readb(phy_data->base + REG_PHYCTL);
		if (data & 0x1)
			temp |= BIT(7);
		else
			temp &= ~BIT(7);
		temp &= ~usbc_bit;
		writeb(temp, phy_data->base + REG_PHYCTL);

		/* pulse usbc_bit */
		temp = readb(phy_data->base + REG_PHYCTL);
		temp |= usbc_bit;
		writeb(temp, phy_data->base + REG_PHYCTL);

		temp = readb(phy_data->base + REG_PHYCTL);
		temp &= ~usbc_bit;
		writeb(temp, phy_data->base + REG_PHYCTL);

		data >>= 1;
	}
	mutex_unlock(&phy_data->mutex);
}

static void sun4i_usb_phy_passby(struct sun4i_usb_phy *phy, int enable)
{
	u32 bits, reg_value;

	if (!phy->pmu_irq)
		return;

	bits = SUNXI_AHB_ICHR8_EN | SUNXI_AHB_INCR4_BURST_EN |
		SUNXI_AHB_INCRX_ALIGN_EN | SUNXI_ULPI_BYPASS_EN;

	reg_value = readl(phy->pmu_irq);

	if (enable)
		reg_value |= bits;
	else
		reg_value &= ~bits;

	writel(reg_value, phy->pmu_irq);
}

static int sun4i_usb_phy_init(struct phy *_phy)
{
	struct sun4i_usb_phy *phy = phy_get_drvdata(_phy);
	struct sun4i_usb_phy_data *data = to_sun4i_usb_phy_data(phy);
	int ret;

	ret = clk_prepare_enable(data->clk);
	if (ret)
		return ret;

	ret = reset_control_deassert(phy->reset);
	if (ret) {
		clk_disable_unprepare(data->clk);
		return ret;
	}

	/* Adjust PHY's magnitude and rate */
	sun4i_usb_phy_write(phy, 0x20, 0x14, 5);

	/* Disconnect threshold adjustment */
	sun4i_usb_phy_write(phy, 0x2a, data->disc_thresh, 2);

	sun4i_usb_phy_passby(phy, 1);

	return 0;
}

static int sun4i_usb_phy_exit(struct phy *_phy)
{
	struct sun4i_usb_phy *phy = phy_get_drvdata(_phy);
	struct sun4i_usb_phy_data *data = to_sun4i_usb_phy_data(phy);

	sun4i_usb_phy_passby(phy, 0);
	reset_control_assert(phy->reset);
	clk_disable_unprepare(data->clk);

	return 0;
}

static int sun4i_usb_phy_power_on(struct phy *_phy)
{
	struct sun4i_usb_phy *phy = phy_get_drvdata(_phy);
	int ret;

	if (phy->vbus) {
		ret = regulator_enable(phy->vbus);
		if (ret)
			return ret;

	}

	return 0;
}

static int sun4i_usb_phy_power_off(struct phy *_phy)
{
	struct sun4i_usb_phy *phy = phy_get_drvdata(_phy);

	if (phy->vbus)
		regulator_disable(phy->vbus);

	return 0;
}

static struct phy_ops sun4i_usb_phy_ops = {
	.init		= sun4i_usb_phy_init,
	.exit		= sun4i_usb_phy_exit,
	.power_on	= sun4i_usb_phy_power_on,
	.power_off	= sun4i_usb_phy_power_off,
	.owner		= THIS_MODULE,
};

static struct phy *sun4i_usb_phy_xlate(struct device *dev,
					struct of_phandle_args *args)
{
	struct sun4i_usb_phy_data *data = dev_get_drvdata(dev);

	if (WARN_ON(args->args[0] == 0 || args->args[0] >= data->num_phys))
		return ERR_PTR(-ENODEV);

	return data->phys[args->args[0]].phy;
}

static int sun4i_usb_phy_probe(struct platform_device *pdev)
{
	struct sun4i_usb_phy_data *data;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	void __iomem *pmu_irq = NULL;
	struct phy_provider *phy_provider;
	struct reset_control *reset;
	struct regulator *vbus;
	struct phy *phy;
	char name[16];
	int i;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	mutex_init(&data->mutex);
	if (of_device_is_compatible(np, "allwinner,sun4i-a10-usb-phy")) {
		data->num_phys = 3;
		data->disc_thresh = 3;
	} else if (of_device_is_compatible(np,
					"allwinner,sun5i-a13-usb-phy")) {
		data->num_phys = 2;
		data->disc_thresh = 2;
	} else { /* allwinner,sun7i-a20-usb-phy */
		data->num_phys = 3;
		data->disc_thresh = 2;
	}

	data->clk = devm_clk_get(dev, "usb_phy");
	if (IS_ERR(data->clk)) {
		dev_err(dev, "could not get usb_phy clock\n");
		return PTR_ERR(data->clk);
	}

	/* Skip 0, 0 is the phy for otg which is not yet supported. */
	for (i = 1; i < data->num_phys; i++) {
		snprintf(name, sizeof(name), "usb%d_vbus", i);
		vbus = devm_regulator_get_optional(dev, name);
		if (IS_ERR(vbus)) {
			if (PTR_ERR(vbus) == -EPROBE_DEFER)
				return -EPROBE_DEFER;
			vbus = NULL;
		}

		snprintf(name, sizeof(name), "usb%d_reset", i);
		reset = devm_reset_control_get(dev, name);
		if (IS_ERR(phy)) {
			dev_err(dev, "failed to get reset %s\n", name);
			return PTR_ERR(phy);
		}

		if (i) { /* No pmu_irq for usbc0 */
			pmu_irq = devm_ioremap_resource(dev,
			      platform_get_resource(pdev, IORESOURCE_MEM, i));
			if (IS_ERR(pmu_irq))
				return PTR_ERR(pmu_irq);
		}

		phy = devm_phy_create(dev, &sun4i_usb_phy_ops, NULL);
		if (IS_ERR(phy)) {
			dev_err(dev, "failed to create PHY %d\n", i);
			return PTR_ERR(phy);
		}

		data->phys[i].phy = phy;
		data->phys[i].pmu_irq = pmu_irq;
		data->phys[i].vbus = vbus;
		data->phys[i].reset = reset;
		data->phys[i].index = i;
		phy_set_drvdata(phy, &data->phys[i]);
	}

	data->base = devm_ioremap_resource(dev,
			platform_get_resource(pdev, IORESOURCE_MEM, 0));
	if (IS_ERR(data->base))
		return PTR_ERR(data->base);

	dev_set_drvdata(dev, data);
	phy_provider = devm_of_phy_provider_register(dev, sun4i_usb_phy_xlate);
	if (IS_ERR(phy_provider))
		return PTR_ERR(phy_provider);

	return 0;
}

static const struct of_device_id sun4i_usb_phy_of_match[] = {
	{ .compatible = "allwinner,sun4i-a10-usb-phy" },
	{ .compatible = "allwinner,sun5i-a13-usb-phy" },
	{ .compatible = "allwinner,sun7i-a20-usb-phy" },
	{ },
};
MODULE_DEVICE_TABLE(of, sun4i_usb_phy_of_match);

static struct platform_driver sun4i_usb_phy_driver = {
	.probe	= sun4i_usb_phy_probe,
	.driver = {
		.of_match_table	= sun4i_usb_phy_of_match,
		.name  = "sun4i-usb-phy",
		.owner = THIS_MODULE,
	}
};
module_platform_driver(sun4i_usb_phy_driver);

MODULE_DESCRIPTION("Allwinner sun4i USB phy driver");
MODULE_AUTHOR("Hans de Goede <hdegoede@redhat.com>");
MODULE_LICENSE("GPL");
