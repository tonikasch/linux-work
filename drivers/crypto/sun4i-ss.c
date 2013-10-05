#include <linux/clk.h>
#include <linux/crypto.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <crypto/hash.h>
#include <crypto/internal/hash.h>
#include <crypto/md5.h>

#include "sun4i-ss.h"

static struct sun4i_ss_ctx {
	void *base;
	struct clk *busclk;
	struct clk *ssclk;
} _ss_ctx, *ss_ctx = &_ss_ctx;

struct sun4i_req_ctx {
	u32 block[MD5_BLOCK_WORDS];
	u64 byte_count;
};

/*
 * sun4i_ss_send: sends 32 bits through the FIFO
 *
 * It maintains count of the number of free spaces on the queue
 * to keep a decent speed.
 */
static void sun4i_ss_send(struct sun4i_ss_ctx *ctx, u32 value)
{
	static int spaces = 0;
	u32 tmp;

	if (!spaces) {
		/* Wait until there are free spaces, then take note of
		 * how many of them there are */
		do {
			tmp = readl(ctx->base + SUNXI_SS_FCSR);
		} while (!(tmp & SUNXI_RXFIFO_FREE));
		spaces = SUNXI_RXFIFO_SPACES(tmp);
	}

	writel(value, ctx->base + SUNXI_SS_RXFIFO);
	spaces--;
}

static irqreturn_t sun4i_feed_irq(int irq, void *dev_id)
{
	u32 status;//, i;
	//u32 *src, *dst;

	status = readl(ss_ctx->base + SUNXI_SS_ICSR);
	if (status & SUNXI_RXFIFO_EMP_INT_PENDING) {
		#if 0 /* TODO! */
		omap_aes_write(dd, AES_REG_IRQ_ENABLE(dd), 0x0);

		BUG_ON(!dd->in_sg);

		BUG_ON(_calc_walked(in) > dd->in_sg->length);

		src = sg_virt(dd->in_sg) + _calc_walked(in);

		for (i = 0; i < AES_BLOCK_WORDS; i++) {
			omap_aes_write(dd, AES_REG_DATA_N(dd, i), *src);

			scatterwalk_advance(&dd->in_walk, 4);
			if (dd->in_sg->length == _calc_walked(in)) {
				dd->in_sg = scatterwalk_sg_next(dd->in_sg);
				if (dd->in_sg) {
					scatterwalk_start(&dd->in_walk,
							  dd->in_sg);
					src = sg_virt(dd->in_sg) +
					      _calc_walked(in);
				}
			} else {
				src++;
			}
		}
		#endif
	} else if (status & SUNXI_TXFIFO_AVA_INT_PENDING) {
		#if 0 /* TODO! */
		omap_aes_write(dd, AES_REG_IRQ_ENABLE(dd), 0x0);

		BUG_ON(!dd->out_sg);

		BUG_ON(_calc_walked(out) > dd->out_sg->length);

		dst = sg_virt(dd->out_sg) + _calc_walked(out);

		for (i = 0; i < AES_BLOCK_WORDS; i++) {
			*dst = omap_aes_read(dd, AES_REG_DATA_N(dd, i));
			scatterwalk_advance(&dd->out_walk, 4);
			if (dd->out_sg->length == _calc_walked(out)) {
				dd->out_sg = scatterwalk_sg_next(dd->out_sg);
				if (dd->out_sg) {
					scatterwalk_start(&dd->out_walk,
							  dd->out_sg);
					dst = sg_virt(dd->out_sg) +
					      _calc_walked(out);
				}
			} else {
				dst++;
			}
		}

		dd->total -= AES_BLOCK_SIZE;

		BUG_ON(dd->total < 0);

		if (!dd->total)
			/* All bytes read! */
			tasklet_schedule(&dd->done_task);
		else
			/* Enable DATA_IN interrupt for next block */
			omap_aes_write(dd, AES_REG_IRQ_ENABLE(dd), 0x2);
		#endif
	}

	/* Clear pending interrupt notifications */
	status &= ~SUNXI_RXFIFO_EMP_INT_PENDING;
	status &= ~SUNXI_TXFIFO_AVA_INT_PENDING;
	writel(status, ss_ctx->base + SUNXI_SS_ICSR);

	return IRQ_HANDLED;
}

/*
 * sun4i_md5_init: initialize request context
 *
 * This function is a noop for now
 */
static int sun4i_md5_init(struct ahash_request *req)
{
	const struct sun4i_req_ctx *req_ctx = ahash_request_ctx(req);
	u32 tmp = 0;

	/* Enable and configure SS for MD5, using IVs on hardware */
	tmp |= SUNXI_SS_ENABLED;
	tmp |= SUNXI_OP_MD5;
	tmp |= SUNXI_IV_CONSTANTS;
	writel(tmp, ss_ctx->base + SUNXI_SS_CTL);

	/* Enable interrupts for both FIFOs */
	tmp = 0;
	tmp |= SUNXI_TXFIFO_AVA_INT_ENABLE;
	tmp |= SUNXI_RXFIFO_EMP_INT_ENABLE;
	writel(tmp, ss_ctx->base + SUNXI_SS_ICSR);

	return 0;
}

/*
 * sun4i_hash_update: update hash engine
 *
 * This function is a noop for now
 */
static int sun4i_hash_update(struct ahash_request *req)
{
	const struct sun4i_req_ctx *req_ctx = ahash_request_ctx(req);

	/* TODO */
	return 0;
}

/*
 * sun4i_hash_final: finalize hashing operation?
 *
 * This function is a noop for now
 */
static int sun4i_hash_final(struct ahash_request *req)
{
	const struct sun4i_req_ctx *req_ctx = ahash_request_ctx(req);

	/* Disable SS */
	writel(0, ss_ctx->base + SUNXI_SS_CTL);
	return 0;
}

struct ahash_alg sun4i_md5_alg = {
	.init = sun4i_md5_init,
	.update = sun4i_hash_update,
	.final = sun4i_hash_final,
	.halg = {
		.digestsize = MD5_DIGEST_SIZE,
		.base = {
			.cra_name = "md5",
			.cra_driver_name = "sun4i-md5",
			.cra_priority = 300,
			.cra_flags = CRYPTO_ALG_ASYNC,
			//.cra_blocksize = MD5_BLOCK_SIZE,
			.cra_ctxsize = sizeof(struct sun4i_req_ctx),
			.cra_module = THIS_MODULE,
		}
	}
};

static int __init sun4i_ss_probe(struct platform_device *pdev)
{
	struct resource *res;
	int irq, err;

	/* Map SS memory */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	ss_ctx->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(ss_ctx->base))
		return PTR_ERR(ss_ctx->base);

	/* Acquire SS bus clk */
	ss_ctx->busclk = of_clk_get(pdev->dev.of_node, 0);
	if(IS_ERR(ss_ctx->busclk))
		return PTR_ERR(ss_ctx->busclk);

	/* Acquire SS clk */
	ss_ctx->ssclk = of_clk_get(pdev->dev.of_node, 1);
	if(IS_ERR(ss_ctx->ssclk)) {
		clk_put(ss_ctx->ssclk);
		return PTR_ERR(ss_ctx->ssclk);
	}

	/* Acquire SS IRQ */
	irq = platform_get_irq(pdev, 0);
	err = devm_request_irq(&pdev->dev, irq, sun4i_feed_irq, 0,
			       dev_name(&pdev->dev), NULL);
	if (err) {
		clk_put(ss_ctx->busclk);
		clk_put(ss_ctx->ssclk);
		return err;
	}

	/* Enable the clocks */
	clk_set_rate(ss_ctx->ssclk, 150000000);
	clk_prepare_enable(ss_ctx->ssclk);
	clk_prepare_enable(ss_ctx->busclk);

	return crypto_register_ahash(&sun4i_md5_alg);
}

static int __exit sun4i_ss_remove(struct platform_device *pdev)
{
	clk_disable_unprepare(ss_ctx->busclk);
	clk_disable_unprepare(ss_ctx->ssclk);

	crypto_unregister_ahash(&sun4i_md5_alg);

	return 0;
}

static const struct of_device_id sun4i_ss_dt_ids[] = {
	{ .compatible = "allwinner,sun4i-ss" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, sun4i_ss_dt_ids);

static struct platform_driver sun4i_ss_driver = {
	.probe		= sun4i_ss_probe,
	.remove		= sun4i_ss_remove,
	.driver		= {
		.owner		= THIS_MODULE,
		.name		= "sun4i-ss",
		.of_match_table	= of_match_ptr(sun4i_ss_dt_ids),
	},
};

module_platform_driver(sun4i_ss_driver);

MODULE_DESCRIPTION("Allwinner SUN4I Security System");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Emilio López <emilio@elopez.com.ar>");
