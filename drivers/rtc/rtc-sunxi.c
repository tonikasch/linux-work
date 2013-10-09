/*
 * An RTC driver for Allwinner A10/A20
 *
 * Copyright (c) 2013, Carlo Caione <carlo.caione@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/rtc.h>
#include <linux/types.h>

#define SUNXI_LOSC_CTRL				0x0000
#define SUNXI_LOSC_CTRL_RTC_HMS_ACC		BIT(8)
#define SUNXI_LOSC_CTRL_RTC_YMD_ACC		BIT(7)

#define SUNXI_RTC_YMD				0x0004

#define SUNXI_RTC_HMS				0x0008

#define SUNXI_ALRM_DHMS				0x000c

#define SUNXI_ALRM_EN				0x0014
#define SUNXI_ALRM_EN_CNT_EN			BIT(8)

#define SUNXI_ALRM_IRQ_EN			0x0018
#define SUNXI_ALRM_IRQ_EN_CNT_IRQ_EN		BIT(0)

#define SUNXI_ALRM_IRQ_STA			0x001c
#define SUNXI_ALRM_IRQ_STA_CNT_IRQ_PEND		BIT(0)

#define SUNXI_LOSC_CTRL_RTC_ACC (SUNXI_LOSC_CTRL_RTC_HMS_ACC | SUNXI_LOSC_CTRL_RTC_YMD_ACC)

#define SUNXI_MASK_DGD_TSH_ASH			0x0000001f
#define SUNXI_MASK_TGH_AGH			0x001f0000
#define SUNXI_MASK_TGS_AGS_TSM_ASM		0x0000003f
#define SUNXI_MASK_TGM				0x00003f00
#define SUNXI_MASK_DSM				0x0000000f
#define SUNXI_MASK_LY				0x00000001
#define SUNXI_MASK_ASD				0x00000ffe
#define SUNXI_MASK_GMD				0x00000f00

/* Get date values */
#define SUNXI_DATE_GET_DAY_VALUE(x)		((x) & SUNXI_MASK_DGD_TSH_ASH)
#define SUNXI_DATE_GET_MON_VALUE(x)		(((x) & SUNXI_MASK_GMD) >> 8)
#define SUNXI_DATE_GET_YEAR_VALUE(x, mask)	(((x) & mask) >> 16)

/* Get time values */
#define SUNXI_TIME_GET_SEC_VALUE(x)		((x) & SUNXI_MASK_TGS_AGS_TSM_ASM)
#define SUNXI_TIME_GET_MIN_VALUE(x)		(((x) & SUNXI_MASK_TGM) >> 8)
#define SUNXI_TIME_GET_HOUR_VALUE(x)		(((x) & SUNXI_MASK_TGH_AGH) >> 16)

/* Get alarm values */
#define SUNXI_ALARM_GET_SEC_VALUE(x)		((x) & SUNXI_MASK_TGS_AGS_TSM_ASM)
#define SUNXI_ALARM_GET_MIN_VALUE(x)		(((x) & SUNXI_MASK_TGM) >> 8)
#define SUNXI_ALARM_GET_HOUR_VALUE(x)		(((x) & SUNXI_MASK_TGH_AGH) >> 16)

/* Set date values */
#define SUNXI_DATE_SET_DAY_VALUE(x)		SUNXI_DATE_GET_DAY_VALUE(x)
#define SUNXI_DATE_SET_MON_VALUE(x)		(((x) & SUNXI_MASK_DSM) << 8)
#define SUNXI_DATE_SET_YEAR_VALUE(x, mask)	(((x) & mask) << 16)
#define SUNXI_LEAP_SET_VALUE(x, shift)		(((x) & SUNXI_MASK_LY) << shift)

/* Set time values */
#define SUNXI_TIME_SET_SEC_VALUE(x)		SUNXI_TIME_GET_SEC_VALUE(x)
#define SUNXI_TIME_SET_MIN_VALUE(x)		(((x) & SUNXI_MASK_TGS_AGS_TSM_ASM) << 8)
#define SUNXI_TIME_SET_HOUR_VALUE(x)		(((x) & SUNXI_MASK_DGD_TSH_ASH) << 16)

/* set alarm values */
#define SUNXI_ALARM_SET_SEC_VALUE(x)		SUNXI_ALARM_GET_SEC_VALUE(x)
#define SUNXI_ALARM_SET_MIN_VALUE(x)		(((x) & SUNXI_MASK_TGS_AGS_TSM_ASM) << 8)
#define SUNXI_ALARM_SET_HOUR_VALUE(x)		(((x) & SUNXI_MASK_DGD_TSH_ASH) << 16)
#define SUNXI_ALARM_SET_DAY_VALUE(x)		(((x) & SUNXI_MASK_ASD) << 21)

/* time unit conversions */
#define SEC_IN_MIN				60
#define SEC_IN_HOUR				(60 * SEC_IN_MIN)
#define SEC_IN_DAY				(24 * SEC_IN_HOUR)

struct sunxi_rtc_data_year {
	unsigned int min;
	unsigned int max;
	unsigned int off;
	unsigned int mask;
	unsigned char leap_shift;
};

static struct sunxi_rtc_data_year data_year_param[] = {
	[0] = {
		.min	= 1970,
		.max	= 2100,
		.off	= 0,
		.mask	= 0x00ff0000,
		.leap_shift	= 24,
	},
	[1] = {
		.min	= 2010,
		.max	= 2073,
		.off	= 110,
		.mask	= 0x003f0000,
		.leap_shift	= 22,
	},
};

struct sunxi_rtc_dev {
	struct rtc_device *rtc;
	struct device *dev;
	struct sunxi_rtc_data_year *data_year;
	void __iomem *base;
	int irq;
};

static irqreturn_t sunxi_rtc_alarmirq(int irq, void *id)
{
	struct sunxi_rtc_dev *chip = (struct sunxi_rtc_dev *) id;
	unsigned int val;

	val = readl(chip->base + SUNXI_ALRM_IRQ_STA);

	if (val & SUNXI_ALRM_IRQ_STA_CNT_IRQ_PEND) {

		val |= SUNXI_ALRM_IRQ_STA_CNT_IRQ_PEND;
		writel(val, chip->base + SUNXI_ALRM_IRQ_STA);

		rtc_update_irq(chip->rtc, 1, RTC_AF | RTC_IRQF);

		return IRQ_HANDLED;
	}

	return IRQ_NONE;
}

static void sunxi_rtc_setaie(int to, struct sunxi_rtc_dev *chip)
{
	unsigned int alarm_val = 0x0;
	unsigned int alarm_irq_val = 0x0;

	if (to) {
		alarm_val = readl(chip->base + SUNXI_ALRM_EN);
		alarm_val |= SUNXI_ALRM_EN_CNT_EN;

		alarm_irq_val = readl(chip->base + SUNXI_ALRM_IRQ_EN);
		alarm_irq_val |= SUNXI_ALRM_IRQ_EN_CNT_IRQ_EN;
	} else
		writel(SUNXI_ALRM_IRQ_STA_CNT_IRQ_PEND,
				chip->base + SUNXI_ALRM_IRQ_STA);

	writel(alarm_val, chip->base + SUNXI_ALRM_EN);
	writel(alarm_irq_val, chip->base + SUNXI_ALRM_IRQ_EN);
}

static int sunxi_rtc_getalarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct sunxi_rtc_dev *chip = dev_get_drvdata(dev);
	struct rtc_time *alrm_tm = &alrm->time;
	unsigned int alarm;
	unsigned int alarm_en;
	unsigned int date;

	alarm = readl(chip->base + SUNXI_ALRM_DHMS);
	date = readl(chip->base + SUNXI_RTC_YMD);

	alrm_tm->tm_sec = SUNXI_ALARM_GET_SEC_VALUE(alarm);
	alrm_tm->tm_min = SUNXI_ALARM_GET_MIN_VALUE(alarm);
	alrm_tm->tm_hour = SUNXI_ALARM_GET_HOUR_VALUE(alarm);

	alrm_tm->tm_mday = SUNXI_DATE_GET_DAY_VALUE(date);
	alrm_tm->tm_mon = SUNXI_DATE_GET_MON_VALUE(date);
	alrm_tm->tm_year = SUNXI_DATE_GET_YEAR_VALUE(date,
			chip->data_year->mask);

	alrm_tm->tm_year += chip->data_year->off;
	alrm_tm->tm_mon -= 1;

	alarm_en = readl(chip->base + SUNXI_ALRM_IRQ_EN);
	if (alarm_en & SUNXI_ALRM_EN_CNT_EN)
		alrm->enabled = 1;

	return 0;
}

static int sunxi_rtc_gettime(struct device *dev, struct rtc_time *rtc_tm)
{
	struct sunxi_rtc_dev *chip = dev_get_drvdata(dev);
	unsigned int date, time;
	int t;

	/* read again if the system was mid-updated */

	for (t = 0; t < 2; t++) {
		date = readl(chip->base + SUNXI_RTC_YMD);
		time = readl(chip->base + SUNXI_RTC_HMS);

		rtc_tm->tm_sec  = SUNXI_TIME_GET_SEC_VALUE(time);
		rtc_tm->tm_min  = SUNXI_TIME_GET_MIN_VALUE(time);
		rtc_tm->tm_hour = SUNXI_TIME_GET_HOUR_VALUE(time);

		rtc_tm->tm_mday = SUNXI_DATE_GET_DAY_VALUE(date);
		rtc_tm->tm_mon  = SUNXI_DATE_GET_MON_VALUE(date);
		rtc_tm->tm_year = SUNXI_DATE_GET_YEAR_VALUE(date,
				chip->data_year->mask);

		if (rtc_tm->tm_sec == 0)
			msleep(500);
		else
			break;
	}

	rtc_tm->tm_year += chip->data_year->off;
	rtc_tm->tm_mon  -= 1;

	return rtc_valid_tm(rtc_tm);
}

static int sunxi_rtc_setalarm(struct device *dev, struct rtc_wkalrm *alrm)
{
	struct sunxi_rtc_dev *chip = dev_get_drvdata(dev);
	struct rtc_time *alrm_tm = &alrm->time;
	struct rtc_time tm_now;
	unsigned int alarm = 0;
	unsigned long time_now = 0;
	unsigned long time_set = 0;
	unsigned long time_gap = 0;
	unsigned long time_gap_day = 0;
	unsigned long time_gap_hour = 0;
	unsigned long time_gap_min = 0;
	int ret = 0;

	ret = sunxi_rtc_gettime(dev, &tm_now);
	if (ret < 0) {
		dev_err(dev, "Error in getting time\n");
		return -EINVAL;
	}

	rtc_tm_to_time(alrm_tm, &time_set);
	rtc_tm_to_time(&tm_now, &time_now);
	if (time_set <= time_now) {
		dev_err(dev, "Date to set in the past\n");
		return -EINVAL;
	}

	time_gap = time_set - time_now;
	time_gap_day = time_gap / SEC_IN_DAY;
	time_gap -= time_gap_day * SEC_IN_DAY;
	time_gap_hour = time_gap / SEC_IN_HOUR;
	time_gap -= time_gap_hour * SEC_IN_HOUR;
	time_gap_min = time_gap / SEC_IN_MIN;
	time_gap -= time_gap_min * SEC_IN_MIN;

	if (time_gap_day > 255) {
		dev_err(dev, "Day must be in the range 0 - 255\n");
		return -EINVAL;
	}

	sunxi_rtc_setaie(0, chip);
	writel(0x0, chip->base + SUNXI_ALRM_DHMS);
	usleep_range(100, 300);

	alarm = SUNXI_ALARM_SET_SEC_VALUE(time_gap) |
		SUNXI_ALARM_SET_MIN_VALUE(time_gap_min) |
		SUNXI_ALARM_SET_HOUR_VALUE(time_gap_hour) |
		SUNXI_ALARM_SET_DAY_VALUE(time_gap_day);
	writel(alarm, chip->base + SUNXI_ALRM_DHMS);

	writel(0x0, chip->base + SUNXI_ALRM_IRQ_EN);
	writel(SUNXI_ALRM_IRQ_EN_CNT_IRQ_EN, chip->base + SUNXI_ALRM_IRQ_EN);

	sunxi_rtc_setaie(alrm->enabled, chip);

	return 0;
}

static int sunxi_rtc_settime(struct device *dev, struct rtc_time *rtc_tm)
{
	struct sunxi_rtc_dev *chip = dev_get_drvdata(dev);
	unsigned int date = 0;
	unsigned int time = 0;
	int year;
	int t;

	year = rtc_tm->tm_year + 1900;
	if (year < chip->data_year->min || year > chip->data_year->max) {
		dev_err(dev, "rtc only supports year in range %d - %d\n", chip->data_year->min,
				chip->data_year->max);
		return -EINVAL;
	}

	t = 3;
	while ((readl(chip->base + SUNXI_LOSC_CTRL) & SUNXI_LOSC_CTRL_RTC_ACC) && --t)
		msleep(500);

	rtc_tm->tm_year -= chip->data_year->off;
	rtc_tm->tm_mon += 1;

	date = SUNXI_DATE_SET_DAY_VALUE(rtc_tm->tm_mday) |
		SUNXI_DATE_SET_MON_VALUE(rtc_tm->tm_mon)  |
		SUNXI_DATE_SET_YEAR_VALUE(rtc_tm->tm_year,
				chip->data_year->mask);

	if (is_leap_year(year))
		date |= SUNXI_LEAP_SET_VALUE(1, chip->data_year->leap_shift);

	time = SUNXI_TIME_SET_SEC_VALUE(rtc_tm->tm_sec)  |
		SUNXI_TIME_SET_MIN_VALUE(rtc_tm->tm_min)  |
		SUNXI_TIME_SET_HOUR_VALUE(rtc_tm->tm_hour);

	writel(0x0, chip->base + SUNXI_RTC_HMS);
	writel(0x0, chip->base + SUNXI_RTC_YMD);

	writel(time, chip->base + SUNXI_RTC_HMS);
	t = 3;
	while ((readl(chip->base + SUNXI_LOSC_CTRL) & SUNXI_LOSC_CTRL_RTC_HMS_ACC) && --t)
		msleep(250);
	if (t == 0) {
		dev_err(dev, "Failed to set rtc time.\n");
		return -1;
	}

	writel(date, chip->base + SUNXI_RTC_YMD);
	t = 3;
	while ((readl(chip->base + SUNXI_LOSC_CTRL) & SUNXI_LOSC_CTRL_RTC_YMD_ACC) && --t)
		msleep(250);
	if (t == 0) {
		dev_err(dev, "Failed to set rtc date.\n");
		return -1;
	}

	usleep_range(70, 100);

	return 0;
}

static int sunxi_rtc_alarm_irq_enable(struct device *dev, unsigned int enabled)
{
	struct sunxi_rtc_dev *chip = dev_get_drvdata(dev);

	if (!enabled)
		sunxi_rtc_setaie(enabled, chip);

	return 0;
}

static const struct rtc_class_ops sunxi_rtc_ops = {
	.read_time		= sunxi_rtc_gettime,
	.set_time		= sunxi_rtc_settime,
	.read_alarm		= sunxi_rtc_getalarm,
	.set_alarm		= sunxi_rtc_setalarm,
	.alarm_irq_enable	= sunxi_rtc_alarm_irq_enable
};

static const struct of_device_id sunxi_rtc_dt_ids[] = {
	{ .compatible = "allwinner,sun4i-a10-rtc", .data = (void *) &data_year_param[0] },
	{ .compatible = "allwinner,sun7i-a20-rtc", .data = (void *) &data_year_param[1] },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, sunxi_rtc_dt_ids);


static int sunxi_rtc_probe(struct platform_device *pdev)
{
	struct sunxi_rtc_dev *chip;
	struct resource *res;
	const struct of_device_id *of_id;
	int ret;

	chip = devm_kzalloc(&pdev->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	platform_set_drvdata(pdev, chip);
	chip->dev = &pdev->dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	chip->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(chip->base))
		return PTR_ERR(chip->base);

	chip->irq = platform_get_irq(pdev, 0);
	if (chip->irq < 0) {
		dev_err(&pdev->dev, "No IRQ resource\n");
		return chip->irq;
	}
	ret = devm_request_irq(&pdev->dev, chip->irq, sunxi_rtc_alarmirq,
			0, dev_name(&pdev->dev), chip);
	if (ret) {
		dev_err(&pdev->dev, "Could not request IRQ\n");
		return ret;
	}

	of_id = of_match_device(sunxi_rtc_dt_ids, &pdev->dev);
	if (!of_id) {
		dev_err(&pdev->dev, "Unable to setup RTC data\n");
		return -ENODEV;
	}
	chip->data_year = (struct sunxi_rtc_data_year *) of_id->data;

	/* clear the alarm count value */
	writel(0x0, chip->base + SUNXI_ALRM_DHMS);

	/* disable alarm, not generate irq pending */
	writel(0x0, chip->base + SUNXI_ALRM_EN);

	/* disable alarm week/cnt irq, unset to cpu */
	writel(0x0, chip->base + SUNXI_ALRM_IRQ_EN);

	/* clear alarm week/cnt irq pending */
	writel(SUNXI_ALRM_IRQ_STA_CNT_IRQ_PEND, chip->base + SUNXI_ALRM_IRQ_STA);

	chip->rtc = rtc_device_register("rtc-sunxi", &pdev->dev,
			&sunxi_rtc_ops, THIS_MODULE);
	if (IS_ERR(chip->rtc)) {
		dev_err(&pdev->dev, "unable to register device\n");
		return PTR_ERR(chip->rtc);
	}

	dev_info(&pdev->dev, "RTC enabled\n");

	return 0;
}

static int sunxi_rtc_remove(struct platform_device *pdev)
{
	struct sunxi_rtc_dev *chip = platform_get_drvdata(pdev);

	rtc_device_unregister(chip->rtc);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

static struct platform_driver sunxi_rtc_driver = {
	.probe		= sunxi_rtc_probe,
	.remove		= sunxi_rtc_remove,
	.driver		= {
		.name		= "sunxi-rtc",
		.owner		= THIS_MODULE,
		.of_match_table = sunxi_rtc_dt_ids,
	},
};

module_platform_driver(sunxi_rtc_driver);

MODULE_DESCRIPTION("sunxi RTC driver");
MODULE_AUTHOR("Carlo Caione <carlo.caione@gmail.com");
MODULE_LICENSE("GPL");
