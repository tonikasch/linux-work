/*
 * Allwinner A1X SoCs i2c controller driver.
 *
 * Copyright (C) 2013 Maxime Ripard
 *
 * Maxime Ripard <maxime.ripard@free-electrons.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>

#define SUNXI_I2C_ADDR_REG		(0x00)
#define SUNXI_I2C_ADDR_ADDR(v)			((v & 0x7f) << 1)
#define SUNXI_I2C_XADDR_REG		(0x04)
#define SUNXI_I2C_DATA_REG		(0x08)
#define SUNXI_I2C_CNTR_REG		(0x0c)
#define SUNXI_I2C_CNTR_ASSERT_ACK		BIT(2)
#define SUNXI_I2C_CNTR_INT_FLAG			BIT(3)
#define SUNXI_I2C_CNTR_MASTER_STOP		BIT(4)
#define SUNXI_I2C_CNTR_MASTER_START		BIT(5)
#define SUNXI_I2C_CNTR_BUS_ENABLE		BIT(6)
#define SUNXI_I2C_CNTR_INT_ENABLE		BIT(7)
#define SUNXI_I2C_STA_REG		(0x10)
#define SUNXI_I2C_STA_BUS_ERROR			(0x00)
#define SUNXI_I2C_STA_START			(0x08)
#define SUNXI_I2C_STA_START_REPEAT		(0x10)
#define SUNXI_I2C_STA_MASTER_WADDR_ACK		(0x18)
#define SUNXI_I2C_STA_MASTER_WADDR_NAK		(0x20)
#define SUNXI_I2C_STA_MASTER_DATA_SENT_ACK	(0x28)
#define SUNXI_I2C_STA_MASTER_DATA_SENT_NAK	(0x30)
#define SUNXI_I2C_STA_MASTER_RADDR_ACK		(0x40)
#define SUNXI_I2C_STA_MASTER_RADDR_NAK		(0x48)
#define SUNXI_I2C_STA_MASTER_DATA_RECV_ACK	(0x50)
#define SUNXI_I2C_STA_MASTER_DATA_RECV_NAK	(0x58)
#define SUNXI_I2C_CCR_REG		(0x14)
#define SUNXI_I2C_CCR_DIV_N(val)		(val & 0x3)
#define SUNXI_I2C_CCR_DIV_M(val)		((val & 0xf) << 3)
#define SUNXI_I2C_SRST_REG		(0x18)
#define SUNXI_I2C_SRST_RESET			BIT(0)
#define SUNXI_I2C_EFR_REG		(0x1c)
#define SUNXI_I2C_LCR_REG		(0x20)

#define SUNXI_I2C_DONE			BIT(0)
#define SUNXI_I2C_ERROR			BIT(1)
#define SUNXI_I2C_NAK			BIT(2)
#define SUNXI_I2C_BUS_ERROR		BIT(3)

struct sunxi_i2c_dev {
	struct i2c_adapter	adapter;
	struct clk		*clk;
	struct device		*dev;
	struct completion	completion;
	unsigned int		irq;
	void __iomem		*membase;

	struct i2c_msg		*msg_cur;
	u8			*msg_buf;
	size_t			msg_buf_remaining;
	unsigned int		msg_err;
};

static void sunxi_i2c_write(struct sunxi_i2c_dev *i2c_dev, u16 reg, u8 value)
{
	writel(value, i2c_dev->membase + reg);
}

static u32 sunxi_i2c_read(struct sunxi_i2c_dev *i2c_dev, u16 reg)
{
	return readl(i2c_dev->membase + reg);
}

/*
 * This is where all the magic happens. The I2C controller works as a
 * state machine, each state being a step in the i2c protocol, with
 * the controller sending an interrupt at each state transition.
 *
 * The state we're in is stored in a register, which leads to a pretty
 * huge switch statement, all of this in the interrupt handler...
 */
static irqreturn_t sunxi_i2c_handler(int irq, void *data)
{
	struct sunxi_i2c_dev *i2c_dev = (struct sunxi_i2c_dev *)data;
	u32 status = sunxi_i2c_read(i2c_dev, SUNXI_I2C_CNTR_REG);
	u32 addr, val;

	if (!(status & SUNXI_I2C_CNTR_INT_FLAG))
		return IRQ_NONE;

	/* Read the current state we're in */
	status = sunxi_i2c_read(i2c_dev, SUNXI_I2C_STA_REG);

	switch (status & 0xff) {
	/* Start condition has been transmitted */
	case SUNXI_I2C_STA_START:
	/* A repeated start condition has been transmitted */
	case SUNXI_I2C_STA_START_REPEAT:
		addr = SUNXI_I2C_ADDR_ADDR(i2c_dev->msg_cur->addr);

		if (i2c_dev->msg_cur->flags & I2C_M_RD)
			addr |= 1;

		sunxi_i2c_write(i2c_dev, SUNXI_I2C_DATA_REG, addr);
		break;

	/*
	 * Address + Write bit have been transmitted, ACK has not been
	 * received.
	 */
	case SUNXI_I2C_STA_MASTER_WADDR_NAK:
	/*
	 * Data byte has been transmitted, ACK has not been
	 * received
	 */
	case SUNXI_I2C_STA_MASTER_DATA_SENT_NAK:
		if (!(i2c_dev->msg_cur->flags & I2C_M_IGNORE_NAK)) {
			i2c_dev->msg_err = SUNXI_I2C_NAK;
			goto out;
		}

	/*
	 * Address + Write bit have been transmitted, ACK has been
	 * received
	 */
	case SUNXI_I2C_STA_MASTER_WADDR_ACK:
	/* Data byte has been transmitted, ACK has been received */
	case SUNXI_I2C_STA_MASTER_DATA_SENT_ACK:
		if (i2c_dev->msg_buf_remaining) {
			sunxi_i2c_write(i2c_dev, SUNXI_I2C_DATA_REG,
					*i2c_dev->msg_buf);
			i2c_dev->msg_buf++;
			i2c_dev->msg_buf_remaining--;
			break;
		}

		if (i2c_dev->msg_buf_remaining == 0) {
			i2c_dev->msg_err = SUNXI_I2C_DONE;
			goto out;
		}

		break;

	/*
	 * Address + Read bit have been transmitted, ACK has not been
	 * received
	 */
	case SUNXI_I2C_STA_MASTER_RADDR_NAK:
		if (!(i2c_dev->msg_cur->flags & I2C_M_IGNORE_NAK)) {
			i2c_dev->msg_err = SUNXI_I2C_NAK;
			goto out;
		}

	/*
	 * Address + Read bit have been transmitted, ACK has been
	 * received
	 */
	case SUNXI_I2C_STA_MASTER_RADDR_ACK:
		/*
		 * We only need to send the ACK for the all the bytes
		 * but the last one
		 */
		if (i2c_dev->msg_buf_remaining > 1) {
			val = sunxi_i2c_read(i2c_dev, SUNXI_I2C_CNTR_REG);
			sunxi_i2c_write(i2c_dev, SUNXI_I2C_CNTR_REG,
					val | SUNXI_I2C_CNTR_ASSERT_ACK);
		}

		break;

	/*
	 * Data byte has been received, ACK has not been
	 * transmitted
	 */
	case SUNXI_I2C_STA_MASTER_DATA_RECV_NAK:
		if (i2c_dev->msg_buf_remaining == 1) {
			val = sunxi_i2c_read(i2c_dev, SUNXI_I2C_DATA_REG);
			*i2c_dev->msg_buf = val & 0xff;
			i2c_dev->msg_buf_remaining--;
			i2c_dev->msg_err = SUNXI_I2C_DONE;
			goto out;
		}

		if (!(i2c_dev->msg_cur->flags & I2C_M_IGNORE_NAK)) {
			i2c_dev->msg_err = SUNXI_I2C_NAK;
			goto out;
		}

	/* Data byte has been received, ACK has been transmitted */
	case SUNXI_I2C_STA_MASTER_DATA_RECV_ACK:
		val = sunxi_i2c_read(i2c_dev, SUNXI_I2C_DATA_REG) & 0xff;
		*i2c_dev->msg_buf = val;
		i2c_dev->msg_buf++;
		i2c_dev->msg_buf_remaining--;

		/* If there's only one byte left, disable the ACK */
		if (i2c_dev->msg_buf_remaining == 1) {
			val = sunxi_i2c_read(i2c_dev, SUNXI_I2C_CNTR_REG);
			sunxi_i2c_write(i2c_dev, SUNXI_I2C_CNTR_REG,
					val & ~SUNXI_I2C_CNTR_ASSERT_ACK);

		};

		break;

	case SUNXI_I2C_STA_BUS_ERROR:
		i2c_dev->msg_err = SUNXI_I2C_BUS_ERROR;
		goto out;

	default:
		i2c_dev->msg_err = SUNXI_I2C_ERROR;
		goto out;
	}

	val = sunxi_i2c_read(i2c_dev, SUNXI_I2C_CNTR_REG);
	sunxi_i2c_write(i2c_dev, SUNXI_I2C_CNTR_REG,
			val & ~SUNXI_I2C_CNTR_INT_FLAG);

	return IRQ_HANDLED;

out:
	val = sunxi_i2c_read(i2c_dev, SUNXI_I2C_CNTR_REG);
	sunxi_i2c_write(i2c_dev, SUNXI_I2C_CNTR_REG,
			val | SUNXI_I2C_CNTR_MASTER_STOP);

	val = sunxi_i2c_read(i2c_dev, SUNXI_I2C_CNTR_REG);
	sunxi_i2c_write(i2c_dev, SUNXI_I2C_CNTR_REG,
			val & ~SUNXI_I2C_CNTR_INT_FLAG);

	complete(&i2c_dev->completion);
	return IRQ_HANDLED;
}

static int sunxi_i2c_xfer_msg(struct sunxi_i2c_dev *i2c_dev,
			      struct i2c_msg *msg)
{
	int time_left;
	u32 val;

	i2c_dev->msg_cur = msg;
	i2c_dev->msg_buf = msg->buf;
	i2c_dev->msg_buf_remaining = msg->len;
	i2c_dev->msg_err = 0;
	INIT_COMPLETION(i2c_dev->completion);

	val = sunxi_i2c_read(i2c_dev, SUNXI_I2C_CNTR_REG);
	val |= SUNXI_I2C_CNTR_MASTER_START;
	sunxi_i2c_write(i2c_dev, SUNXI_I2C_CNTR_REG, val);

	time_left = wait_for_completion_timeout(&i2c_dev->completion,
						i2c_dev->adapter.timeout);
	if (!time_left) {
		dev_err(i2c_dev->dev, "i2c transfer timed out\n");
		return -ETIMEDOUT;
	}

	if (likely(i2c_dev->msg_err == SUNXI_I2C_DONE))
		return 0;

	dev_dbg(i2c_dev->dev, "i2c transfer failed: %x\n", i2c_dev->msg_err);

	return -EIO;
}

static int sunxi_i2c_xfer(struct i2c_adapter *adap, struct i2c_msg *msgs,
			  int num)
{
	struct sunxi_i2c_dev *i2c_dev = i2c_get_adapdata(adap);
	int i, ret;

	for (i = 0; i < num; i++) {
		ret = sunxi_i2c_xfer_msg(i2c_dev, &msgs[i]);
		if (ret)
			return ret;
	}

	return i;
}

static u32 sunxi_i2c_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static const struct i2c_algorithm sunxi_i2c_algo = {
	.master_xfer	= sunxi_i2c_xfer,
	.functionality	= sunxi_i2c_func,
};

static int sunxi_i2c_probe(struct platform_device *pdev)
{
	struct sunxi_i2c_dev *i2c_dev;
	struct device_node *np;
	u32 freq, div_m, div_n;
	int ret;

	np = pdev->dev.of_node;
	if (!np)
		return -EINVAL;

	i2c_dev = devm_kzalloc(&pdev->dev, sizeof(*i2c_dev), GFP_KERNEL);
	if (!i2c_dev)
		return -ENOMEM;
	platform_set_drvdata(pdev, i2c_dev);
	i2c_dev->dev = &pdev->dev;

	init_completion(&i2c_dev->completion);

	i2c_dev->membase = of_iomap(np, 0);
	if (!i2c_dev->membase)
		return -EADDRNOTAVAIL;

	sunxi_i2c_write(i2c_dev, SUNXI_I2C_SRST_REG, SUNXI_I2C_SRST_RESET);

	i2c_dev->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(i2c_dev->clk)) {
		ret = PTR_ERR(i2c_dev->clk);
		goto out_iounmap;
	}
	clk_prepare_enable(i2c_dev->clk);

	ret = of_property_read_u32(np, "clock-frequency", &freq);
	if (ret < 0) {
		dev_warn(&pdev->dev, "Could not read clock-frequency property\n");
		freq = 100000;
	}

	/*
	 * Set the clock dividers. we don't need to be super smart
	 * here, the datasheet defines the value of the factors for
	 * the two supported frequencies, and only the M factor
	 * changes between 100kHz and 400kHz.
	 *
	 * The bus clock is generated from the parent clock with two
	 * different dividers. It is generated as such:
	 *     f0 = fclk / (2 ^ DIV_N)
	 *     fbus = f0 / (10 * (DIV_M + 1))
	 *
	 * With DIV_N being on 3 bits, and DIV_M on 4 bits.
	 * So DIV_N < 8, and DIV_M < 16.
	 *
	 * However, we can generate both the supported frequencies
	 * with f0 = 12MHz, and only change M to get back on our
	 * feet.
	 */
	div_n = ilog2(clk_get_rate(i2c_dev->clk) / 12000000);
	if (freq == 100000)
		div_m = 11;
	else if (freq == 400000)
		div_m = 2;
	else {
		dev_err(&pdev->dev, "Unsupported bus frequency\n");
		ret = -EINVAL;
		goto out_clk_dis;
	}

	sunxi_i2c_write(i2c_dev, SUNXI_I2C_CCR_REG,
			SUNXI_I2C_CCR_DIV_N(div_n) | SUNXI_I2C_CCR_DIV_M(div_m));

	i2c_dev->irq = irq_of_parse_and_map(np, 0);
	if (!i2c_dev->irq) {
		dev_err(&pdev->dev, "No IRQ resource\n");
		ret = -ENODEV;
		goto out_clk_dis;
	}

	ret = devm_request_irq(&pdev->dev, i2c_dev->irq, sunxi_i2c_handler,
			       IRQF_SHARED, dev_name(&pdev->dev), i2c_dev);
	if (ret) {
		dev_err(&pdev->dev, "Could not request IRQ\n");
		goto out_clk_dis;
	}

	i2c_set_adapdata(&i2c_dev->adapter, i2c_dev);

	i2c_dev->adapter.owner = THIS_MODULE;
	strlcpy(i2c_dev->adapter.name, "sunxi I2C adapter",
		sizeof(i2c_dev->adapter.name));
	i2c_dev->adapter.algo = &sunxi_i2c_algo;
	i2c_dev->adapter.dev.parent = &pdev->dev;

	sunxi_i2c_write(i2c_dev, SUNXI_I2C_CNTR_REG,
			SUNXI_I2C_CNTR_BUS_ENABLE | SUNXI_I2C_CNTR_INT_ENABLE);

	ret = i2c_add_adapter(&i2c_dev->adapter);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register i2c adapter\n");
		goto out_clk_dis;
	}

	return 0;

out_clk_dis:
	clk_disable_unprepare(i2c_dev->clk);
out_iounmap:
	iounmap(i2c_dev->membase);
	return ret;
}


static int sunxi_i2c_remove(struct platform_device *pdev)
{
	struct sunxi_i2c_dev *i2c_dev = platform_get_drvdata(pdev);

	i2c_del_adapter(&i2c_dev->adapter);
	clk_disable_unprepare(i2c_dev->clk);
	iounmap(i2c_dev->membase);

	return 0;
}

static const struct of_device_id sunxi_i2c_of_match[] = {
	{ .compatible = "allwinner,sun4i-i2c" },
	{},
};
MODULE_DEVICE_TABLE(of, sunxi_i2c_of_match);

static struct platform_driver sunxi_i2c_driver = {
	.probe		= sunxi_i2c_probe,
	.remove		= sunxi_i2c_remove,
	.driver		= {
		.name	= "i2c-sunxi",
		.owner	= THIS_MODULE,
		.of_match_table = sunxi_i2c_of_match,
	},
};
module_platform_driver(sunxi_i2c_driver);

MODULE_AUTHOR("Maxime Ripard <maxime.ripard@free-electrons.com");
MODULE_DESCRIPTION("Allwinner A1X I2C bus adapter");
MODULE_LICENSE("GPL");
