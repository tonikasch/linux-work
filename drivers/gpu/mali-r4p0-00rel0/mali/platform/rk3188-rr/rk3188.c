#include "mali_utgard.h"
#include <mach/irqs.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <asm/io.h>
#include "mali_kernel_common.h"
#include <linux/dma-mapping.h>
#ifdef CONFIG_PM_RUNTIME
#include <linux/pm_runtime.h>
#endif

#define MALI_BASE_ADDR 0x10090000UL
#define IRQ_GP_3D IRQ_GPU_GP
#define IRQ_GPMMU_3D IRQ_GPU_MMU
#define IRQ_PP0_3D IRQ_GPU_PP
#define IRQ_PP1_3D IRQ_GPU_PP
#define IRQ_PP2_3D IRQ_GPU_PP
#define IRQ_PP3_3D IRQ_GPU_PP
#define IRQ_PPMMU0_3D IRQ_GPU_MMU
#define IRQ_PPMMU1_3D IRQ_GPU_MMU
#define IRQ_PPMMU2_3D IRQ_GPU_MMU
#define IRQ_PPMMU3_3D IRQ_GPU_MMU

static struct clk *mali_clk_aclk_gpu=NULL;
static struct clk *mali_clk_pd_gpu=NULL;

static struct resource mali_gpu_resources_m400_mp4[] = {
		MALI_GPU_RESOURCES_MALI400_MP4(MALI_BASE_ADDR,
				IRQ_GP_3D,
				IRQ_GPMMU_3D,
				IRQ_PP0_3D,
				IRQ_PPMMU0_3D,
				IRQ_PP1_3D,
				IRQ_PPMMU1_3D,
				IRQ_PP2_3D,
				IRQ_PPMMU2_3D,
				IRQ_PP3_3D,
				IRQ_PPMMU3_3D) };

static struct mali_gpu_device_data mali_gpu_data = {
	.shared_mem_size = 256 * 1024 * 1024, /* 256MB */
	.fb_start = 0x40000000,
	.fb_size  = 0x40000000,
};

static struct platform_device mali_gpu_device = {
	.name = MALI_GPU_NAME_UTGARD,
	.id = 0,
	.resource = mali_gpu_resources_m400_mp4,
	.dev.platform_data = &mali_gpu_data,
	.dev.coherent_dma_mask = DMA_BIT_MASK(32),
	.num_resources = ARRAY_SIZE(mali_gpu_resources_m400_mp4),
	.resource = mali_gpu_resources_m400_mp4,
};

/*
 TODO: Find a device, and not to hardcode it here
 */
extern struct platform_device *g_fb_pdev;

int mali_platform_device_register (void)
{
	int err;

	err = platform_device_register(&mali_gpu_device);
	if(err)
		return err;

	mali_clk_aclk_gpu = clk_get(NULL,"gpu");
	if(!mali_clk_aclk_gpu){
		MALI_DEBUG_PRINT(4, ("Can't get aclk_gpu clk\n"));
		return -ENOENT;
	}

	mali_clk_pd_gpu = clk_get(NULL,"pd_gpu");
	if(!mali_clk_pd_gpu){
		MALI_DEBUG_PRINT(4, ("Can't get pd_gpu clk\n"));
		clk_put(mali_clk_aclk_gpu);
		return -ENOENT;
	}

	clk_enable(mali_clk_aclk_gpu);
	clk_enable(mali_clk_pd_gpu);
	MALI_DEBUG_PRINT(4, ("Mali clk enabled\n"));

#ifdef CONFIG_PM_RUNTIME
	pm_runtime_set_autosuspend_delay(&(mali_gpu_device.dev), 1000);
	pm_runtime_use_autosuspend(&(mali_gpu_device.dev));
	pm_runtime_enable(&(mali_gpu_device.dev));
#endif
	return err;
}

int mali_platform_device_unregister(void)
{
	clk_disable(mali_clk_aclk_gpu);
	clk_put(mali_clk_aclk_gpu);
	mali_clk_aclk_gpu=NULL;

	clk_disable(mali_clk_pd_gpu);
	clk_put(mali_clk_pd_gpu);
	mali_clk_pd_gpu=NULL;

	platform_device_unregister(&mali_gpu_device);
	platform_device_put(&mali_gpu_device);
	return 0;;
}
