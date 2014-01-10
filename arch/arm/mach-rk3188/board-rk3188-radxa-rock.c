/*
 *
 * Copyright (C) 2012 ROCKCHIP, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/skbuff.h>
#include <linux/spi/spi.h>
#include <linux/mmc/host.h>
#include <linux/ion.h>
#include <linux/cpufreq.h>
#include <linux/clk.h>
#include <mach/dvfs.h>

#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/flash.h>
#include <asm/hardware/gic.h>

#include <mach/board.h>
#include <mach/hardware.h>
#include <mach/io.h>
#include <mach/gpio.h>
#include <mach/iomux.h>
#include <mach/irqs.h>
#include <linux/rk_fb.h>
#include <linux/regulator/machine.h>
#include <linux/earlysuspend.h>
#include <linux/regulator/act8846.h>
#include <linux/regulator/rk29-pwm-regulator.h>

#include <plat/efuse.h>

#include <linux/display-sys.h>
#include <linux/rk_fb.h>

#ifdef CONFIG_RK29_VMAC
#include "../mach-rk30/board-rk31-vmac.c"
#endif

#ifdef CONFIG_FB_ROCKCHIP

#if defined(CONFIG_LCDC0_RK3188) || defined(CONFIG_LCDC0_RK3188_MODULE)
struct rk29fb_info lcdc0_screen_info = {
	.prop			= EXTEND,
	.lcd_info		= NULL,
	.set_screen_info	= set_lcd_info,

};
#endif

#if defined(CONFIG_LCDC1_RK3188) || defined(CONFIG_LCDC1_RK3188_MODULE)
struct rk29fb_info lcdc1_screen_info = {
	.prop			= PRMRY,
	.set_screen_info	= set_lcd_info,
};
#endif

static struct resource resource_fb[] = {
	[0] = {
		.name  = "fb0 buf",
		.start = 0,
		.end   = 0,//RK30_FB0_MEM_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.name  = "ipp buf",  //for rotate
		.start = 0,
		.end   = 0,//RK30_FB0_MEM_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[2] = {
		.name  = "fb2 buf",
		.start = 0,
		.end   = 0,//RK30_FB0_MEM_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
};

static struct platform_device device_fb = {
	.name		= "rk-fb",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(resource_fb),
	.resource	= resource_fb,
};
#endif

#if defined(CONFIG_LCDC0_RK3188) || defined(CONFIG_LCDC0_RK3188_MODULE)
static struct resource resource_lcdc0[] = {
	[0] = {
		.name  = "lcdc0 reg",
		.start = RK30_LCDC0_PHYS,
		.end   = RK30_LCDC0_PHYS + RK30_LCDC0_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	
	[1] = {
		.name  = "lcdc0 irq",
		.start = IRQ_LCDC0,
		.end   = IRQ_LCDC0,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device device_lcdc0 = {
	.name		  = "rk30-lcdc",
	.id		  = 0,
	.num_resources	  = ARRAY_SIZE(resource_lcdc0),
	.resource	  = resource_lcdc0,
	.dev 		= {
		.platform_data = &lcdc0_screen_info,
	},
};
#endif
#if defined(CONFIG_LCDC1_RK3188) || defined(CONFIG_LCDC1_RK3188_MODULE)
static struct resource resource_lcdc1[] = {
	[0] = {
		.name  = "lcdc1 reg",
		.start = RK30_LCDC1_PHYS,
		.end   = RK30_LCDC1_PHYS + RK30_LCDC1_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.name  = "lcdc1 irq",
		.start = IRQ_LCDC1,
		.end   = IRQ_LCDC1,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device device_lcdc1 = {
	.name		  = "rk30-lcdc",
	.id		  = 1,
	.num_resources	  = ARRAY_SIZE(resource_lcdc1),
	.resource	  = resource_lcdc1,
	.dev 		= {
		.platform_data = &lcdc1_screen_info,
	},
};
#endif

#ifdef CONFIG_ION
#define ION_RESERVE_SIZE        (120 * SZ_1M)
static struct ion_platform_data rk30_ion_pdata = {
	.nr = 1,
	.heaps = {
		{
			.type = ION_HEAP_TYPE_CARVEOUT,
			.id = 0, //ION_NOR_HEAP_ID,
			.name = "norheap",
			.size = ION_RESERVE_SIZE,
		}
	},
};

static struct platform_device device_ion = {
	.name = "ion-rockchip",
	.id = 0,
	.dev = {
		.platform_data = &rk30_ion_pdata,
	},
};
#endif

/**************************************************************************************************
 * SDMMC devices,  include the module of SD,MMC,and sdio.noted by xbw at 2012-03-05
**************************************************************************************************/
#ifdef CONFIG_SDMMC_RK29
#include "board-rk3188-ds1006h-sdmmc-config.c"
#include "../plat-rk/rk-sdmmc-ops.c"
#endif //endif ---#ifdef CONFIG_SDMMC_RK29

#ifdef CONFIG_SDMMC0_RK29
static int rk29_sdmmc0_cfg_gpio(void)
{
	rk29_sdmmc_set_iomux(0, 0xFFFF);

    #if defined(CONFIG_SDMMC0_RK29_SDCARD_DET_FROM_GPIO)
        #if SDMMC_USE_NEW_IOMUX_API
        iomux_set_gpio_mode(iomux_gpio_to_mode(RK29SDK_SD_CARD_DETECT_N));
        #else
        rk30_mux_api_set(RK29SDK_SD_CARD_DETECT_PIN_NAME, RK29SDK_SD_CARD_DETECT_IOMUX_FGPIO);
        #endif
    #else
        #if SDMMC_USE_NEW_IOMUX_API       
        iomux_set(MMC0_DETN);
        #else
        rk30_mux_api_set(RK29SDK_SD_CARD_DETECT_PIN_NAME, RK29SDK_SD_CARD_DETECT_IOMUX_FMUX);
        #endif
    #endif	

#if defined(CONFIG_SDMMC0_RK29_WRITE_PROTECT)
	gpio_request(SDMMC0_WRITE_PROTECT_PIN, "sdmmc-wp");
	gpio_direction_input(SDMMC0_WRITE_PROTECT_PIN);
#endif

	return 0;
}

struct rk29_sdmmc_platform_data default_sdmmc0_data = {
	.host_ocr_avail =
	    (MMC_VDD_25_26 | MMC_VDD_26_27 | MMC_VDD_27_28 | MMC_VDD_28_29 |
	     MMC_VDD_29_30 | MMC_VDD_30_31 | MMC_VDD_31_32 | MMC_VDD_32_33 |
	     MMC_VDD_33_34 | MMC_VDD_34_35 | MMC_VDD_35_36),
	.host_caps =
	    (MMC_CAP_4_BIT_DATA | MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED),
	.io_init = rk29_sdmmc0_cfg_gpio,
	.set_iomux = rk29_sdmmc_set_iomux,
	.dma_name = "sd_mmc",
	.use_dma = 1,

#if defined(CONFIG_WIFI_COMBO_MODULE_CONTROL_FUNC) && defined(CONFIG_USE_SDMMC0_FOR_WIFI_DEVELOP_BOARD)
    .status = rk29sdk_wifi_mmc0_status,
    .register_status_notify = rk29sdk_wifi_mmc0_status_register,
#endif

#if defined(RK29SDK_SD_CARD_PWR_EN) || (INVALID_GPIO != RK29SDK_SD_CARD_PWR_EN)
    .power_en = RK29SDK_SD_CARD_PWR_EN,
    .power_en_level = RK29SDK_SD_CARD_PWR_EN_LEVEL,
#else
    .power_en = INVALID_GPIO,
    .power_en_level = GPIO_LOW,
#endif    
	.enable_sd_wakeup = 0,

#if defined(CONFIG_SDMMC0_RK29_WRITE_PROTECT)
	.write_prt = SDMMC0_WRITE_PROTECT_PIN,
	.write_prt_enalbe_level = SDMMC0_WRITE_PROTECT_ENABLE_VALUE;
#else
	.write_prt = INVALID_GPIO,
#endif

    .det_pin_info = {    
    #if defined(RK29SDK_SD_CARD_DETECT_N) || (INVALID_GPIO != RK29SDK_SD_CARD_DETECT_N)  
        .io             = RK29SDK_SD_CARD_DETECT_N, //INVALID_GPIO,
        .enable         = RK29SDK_SD_CARD_INSERT_LEVEL,
        #ifdef RK29SDK_SD_CARD_DETECT_PIN_NAME
        .iomux          = {
            .name       = RK29SDK_SD_CARD_DETECT_PIN_NAME,
            #ifdef RK29SDK_SD_CARD_DETECT_IOMUX_FGPIO
            .fgpio      = RK29SDK_SD_CARD_DETECT_IOMUX_FGPIO,
            #endif
            #ifdef RK29SDK_SD_CARD_DETECT_IOMUX_FMUX
            .fmux       = RK29SDK_SD_CARD_DETECT_IOMUX_FMUX,
            #endif
        },
        #endif
    #else
        .io             = INVALID_GPIO,
        .enable         = GPIO_LOW,
    #endif    
    }, 

};
#endif // CONFIG_SDMMC0_RK29

#ifdef CONFIG_SDMMC1_RK29
static int rk29_sdmmc1_cfg_gpio(void)
{

#if defined(CONFIG_SDMMC1_RK29_WRITE_PROTECT)
	gpio_request(SDMMC1_WRITE_PROTECT_PIN, "sdio-wp");
	gpio_direction_input(SDMMC1_WRITE_PROTECT_PIN);
#endif

	return 0;
}

struct rk29_sdmmc_platform_data default_sdmmc1_data = {
	.host_ocr_avail =
	    (MMC_VDD_25_26 | MMC_VDD_26_27 | MMC_VDD_27_28 | MMC_VDD_28_29 |
	     MMC_VDD_29_30 | MMC_VDD_30_31 | MMC_VDD_31_32 | MMC_VDD_32_33 |
	     MMC_VDD_33_34),

#if !defined(CONFIG_USE_SDMMC1_FOR_WIFI_DEVELOP_BOARD)
	.host_caps = (MMC_CAP_4_BIT_DATA | MMC_CAP_SDIO_IRQ |
		      MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED),
#else
	.host_caps =
	    (MMC_CAP_4_BIT_DATA | MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED),
#endif

	.io_init = rk29_sdmmc1_cfg_gpio,
	.set_iomux = rk29_sdmmc_set_iomux,
	.dma_name = "sdio",
	.use_dma = 1,

#if defined(CONFIG_SDMMC1_RK29_WRITE_PROTECT)
	.write_prt = SDMMC1_WRITE_PROTECT_PIN,
	    .write_prt_enalbe_level = SDMMC1_WRITE_PROTECT_ENABLE_VALUE;
#else
	.write_prt = INVALID_GPIO,
#endif

    #if defined(CONFIG_RK29_SDIO_IRQ_FROM_GPIO)
        .sdio_INT_gpio = RK29SDK_WIFI_SDIO_CARD_INT,
    #endif

    .det_pin_info = {    
#if defined(CONFIG_USE_SDMMC1_FOR_WIFI_DEVELOP_BOARD)
     #if defined(RK29SDK_SD_CARD_DETECT_N) || (INVALID_GPIO != RK29SDK_SD_CARD_DETECT_N)  
        .io             = RK29SDK_SD_CARD_DETECT_N,
     #else
         .io             = INVALID_GPIO,
     #endif   

        .enable         = RK29SDK_SD_CARD_INSERT_LEVEL,
        #ifdef RK29SDK_SD_CARD_DETECT_PIN_NAME
        .iomux          = {
            .name       = RK29SDK_SD_CARD_DETECT_PIN_NAME,
            #ifdef RK29SDK_SD_CARD_DETECT_IOMUX_FGPIO
            .fgpio      = RK29SDK_SD_CARD_DETECT_IOMUX_FGPIO,
            #endif
            #ifdef RK29SDK_SD_CARD_DETECT_IOMUX_FMUX
            .fmux       = RK29SDK_SD_CARD_DETECT_IOMUX_FMUX,
            #endif
        },
        #endif
 #else
        .io             = INVALID_GPIO,
        .enable         = GPIO_LOW,
#endif
    },
   
	.enable_sd_wakeup = 0,
};
#endif //endif--#ifdef CONFIG_SDMMC1_RK29

/**************************************************************************************************
 * the end of setting for SDMMC devices
**************************************************************************************************/

#ifdef CONFIG_RK30_PWM_REGULATOR
static int pwm_voltage_map[] = {
	 800000,  825000,  850000,  875000,  900000,  925000,
	 950000,  975000, 1000000, 1025000, 1050000, 1075000, 
	1100000, 1125000, 1150000, 1175000, 1200000, 1225000, 
	1250000, 1275000, 1300000, 1325000, 1350000, 1375000
    // Omegamoon >> Set max voltage from 1375000 to 1425000
    , 1400000, 1425000 

};

static struct regulator_consumer_supply pwm_dcdc1_consumers[] = {
	{
		.supply = "vdd_cpu",
	}
};

struct regulator_init_data pwm_regulator_init_dcdc[1] =
{
	{
		.constraints = {
			.name = "PWM_DCDC1",
			.min_uV = 600000,
			.max_uV = 1800000,	//0.6-1.8V
			.apply_uV = true,
			.valid_ops_mask = REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_VOLTAGE,
		},
		.num_consumer_supplies = ARRAY_SIZE(pwm_dcdc1_consumers),
		.consumer_supplies = pwm_dcdc1_consumers,
	},
};

static struct pwm_platform_data pwm_regulator_info[1] = {
	{
		.pwm_id = 1,
		.pwm_gpio = RK30_PIN3_PD4,
		.pwm_iomux_pwm = PWM1,
		.pwm_iomux_gpio = GPIO3_D4,
		.pwm_voltage = 1100000,
		.suspend_voltage = 1000000,
		.min_uV = 800000,
		// Omegamoon >> Set max voltage from 1375000 to 1425000
		.max_uV	= 1425000,
		.coefficient = 575,	//57.5%
		.pwm_voltage_map = pwm_voltage_map,
		.init_data	= &pwm_regulator_init_dcdc[0],
	},
};

struct platform_device pwm_regulator_device[1] = {
	{
		.name = "pwm-voltage-regulator",
		.id = 0,
		.dev		= {
			.platform_data = &pwm_regulator_info[0],
		}
	},
};
#endif

/*
 * Codec for the ASoC Rockchip HDMI machine driver
 */
#if defined(CONFIG_SND_SOC_RK_HDMI_CODEC) || defined(CONFIG_SND_SOC_RK_HDMI_CODEC_MODULE)
static struct platform_device rockchip_hdmi_codec = {
	.name	= "rockchip-hdmi-codec",
	.id	= -1,
};
#endif
/*
 * Device for the ASoC Rockchip HDMI machine driver
 */
#if defined(CONFIG_SND_RK_SOC_HDMI) || defined(CONFIG_SND_RK_SOC_HDMI_MODULE)
static struct platform_device rockchip_hdmi_audio = {
	.name	= "rockchip-hdmi-audio",
	.id	= -1,
};
#endif

#if defined(CONFIG_MALI) || defined(CONFIG_MALI_MODULE)
#include "mali_utgard.h"
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
/*
static struct clock *mali_clk=NULL;

static int mali_gpu_probe (struct platform_device *pdev)
{
	mali_clk=clk_get(NULL,"aclk_gpu");
	return 0;
}
*/
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

static struct platform_device mali_gpu_device = {
		.name = MALI_GPU_NAME_UTGARD,
		.id = 0,
		.resource = mali_gpu_resources_m400_mp4,
};
#endif

static struct platform_device *devices[] __initdata = {

#ifdef CONFIG_ION
	&device_ion,
#endif

#if defined(CONFIG_SND_SOC_RK_HDMI_CODEC) || defined(CONFIG_SND_SOC_RK_HDMI_CODEC_MODULE)
	&rockchip_hdmi_codec,
#endif
#if defined(CONFIG_SND_RK_SOC_HDMI) || defined(CONFIG_SND_RK_SOC_HDMI_MODULE)
	&rockchip_hdmi_audio,
#endif
#if defined(CONFIG_FB_ROCKCHIP) || defined(CONFIG_FB_ROCKCHIP_MODULE)
	&device_fb,
#endif
#if defined(CONFIG_LCDC1_RK3188) || defined(CONFIG_LCDC1_RK3188_MODULE)
	&device_lcdc1,
#endif
#if defined(CONFIG_LCDC0_RK3188) || defined(CONFIG_LCDC0_RK3188_MODULE)
	&device_lcdc0,
#endif
#if defined(CONFIG_MALI) || defined(CONFIG_MALI_MODULE)
	&mali_gpu_device,
#endif
};

static struct rkdisplay_platform_data hdmi_data = {
	.property	= DISPLAY_MAIN,
	.video_source 	= DISPLAY_SOURCE_LCDC1,
	.io_pwr_pin 	= INVALID_GPIO,
	.io_reset_pin 	= RK30_PIN3_PB2,
};

#if defined(CONFIG_RK1000_TVOUT) || defined(CONFIG_MFD_RK1000) || defined(CONFIG_RK1000_TVOUT_MODULE) || defined(CONFIG_MFD_RK1000_MODULE)
static struct rkdisplay_platform_data tv_data = {
	.property	= DISPLAY_AUX,
	.video_source 	= DISPLAY_SOURCE_LCDC0,
	.io_pwr_pin 	= INVALID_GPIO,
	.io_reset_pin 	= RK30_PIN_PD7,
	.io_switch_pin	= INVALID_GPIO,
};
#endif

// i2c
#ifdef CONFIG_I2C0_RK30
static struct i2c_board_info __initdata i2c0_info[] = {

#if defined(CONFIG_SND_SOC_RK1000) || defined(CONFIG_SND_SOC_RK1000_MODULE)
	{
		.type          = "rk1000_i2c_codec",
		.addr          = 0x60,
		.flags         = 0,
	},
	{
		.type          = "rk1000_control",
		.addr          = 0x40,
		.flags         = 0,
	},
#endif

};
#endif

int __sramdata g_pmic_type =  0;
#ifdef CONFIG_I2C1_RK30

#if defined(CONFIG_REGULATOR_ACT8846) || defined(CONFIG_REGULATOR_ACT8846_MODULE)
#define PMU_POWER_SLEEP RK30_PIN0_PA1
#define PMU_VSEL RK30_PIN3_PD3
#define ACT8846_HOST_IRQ                RK30_PIN0_PB3

static struct pmu_info  act8846_dcdc_info[] = {
	{
		.name		= "act_dcdc1",   //ddr
		.min_uv          = 1200000,
		.max_uv         = 1200000,
		.suspend_vol  =   1200000,
	},
	{
		.name          = "vdd_core",    //logic
		.min_uv          = 1000000,
		.max_uv         = 1000000,
		#ifdef CONFIG_ACT8846_SUPPORT_RESET
		.suspend_vol  =  1200000,
		#else
		.suspend_vol  =  900000,
		#endif
	},
	{
		.name          = "vdd_cpu",   //arm
		.min_uv          = 1000000,
		.max_uv         = 1000000,
		#ifdef CONFIG_ACT8846_SUPPORT_RESET
		.suspend_vol  =  1200000,
		#else
		.suspend_vol  =  900000,
		#endif
	},
	{
		.name          = "act_dcdc4",   //vccio
//SAW special voltage for QX1, from Leolas
#ifdef CONFIG_ACT8846_DCDC4_30V
		.min_uv		= 3000000,
		.max_uv		= 3000000,
#else
		.min_uv          = 3300000,
		.max_uv         = 3300000,
#endif
		#ifdef CONFIG_ACT8846_SUPPORT_RESET
		.suspend_vol  =  3000000,
		#else
		.suspend_vol  =  2800000,
		#endif
	},
	
};
static  struct pmu_info  act8846_ldo_info[] = {
	{
		.name          = "act_ldo1",   //vdd10
		.min_uv          = 1000000,
		.max_uv         = 1000000,
	},
	{
		.name          = "act_ldo2",    //vdd12
		.min_uv          = 1200000,
		.max_uv         = 1200000,
	},
	{
		.name          = "act_ldo3",   //vcc18_cif
		.min_uv          = 1800000,
		.max_uv         = 1800000,
	},
	{
		.name          = "act_ldo4",   //vcca33
		.min_uv          = 3300000,
		.max_uv         = 3300000,
	},
	{
		.name          = "act_ldo5",   //vcc_rmii
		.min_uv          = 3300000,
		.max_uv         = 3300000,
	},
	{
		.name          = "act_ldo6",   //vcc_jetta
//SAW volt set via kernel config, default 3300000, mk908 and some others
//need 1800000 to get wifi/bt working properly
#ifdef CONFIG_ACT8846_LDO6_18V
		.min_uv          = 1800000,
		.max_uv         = 1800000,
#else
		.min_uv		= 3300000,
		.max_uv		= 3300000,
#endif
	},
	{
		.name          = "act_ldo7",   //vcc18
		.min_uv          = 1800000,
		.max_uv         = 1800000,
	},
	{
		.name          = "act_ldo8",   //vcc28_cif
		.min_uv          = 2800000,
		.max_uv         = 2800000,
	},
 };

#include "../mach-rk30/board-pmu-act8846.c"
#endif

static struct i2c_board_info __initdata i2c1_info[] = {
#if defined(CONFIG_REGULATOR_ACT8846) || defined(CONFIG_REGULATOR_ACT8846_MODULE)
	{
		.type  		= "act8846",
		.addr           = 0x5a, 
		.flags		= 0,
		.irq            = ACT8846_HOST_IRQ,
		.platform_data=&act8846_data,
	},
#endif
#if defined (CONFIG_RTC_HYM8563)
	{
		.type		 = "rtc_hym8563",
		.addr           = 0x51,
		.flags          = 0,
		.irq            = RK30_PIN0_PB5,
	},
#endif
};
#endif

void __sramfunc board_pmu_suspend(void)
{
       #if defined(CONFIG_REGULATOR_ACT8846) || defined(CONFIG_REGULATOR_ACT8846_MODULE)
       if(pmic_is_act8846())
       board_pmu_act8846_suspend(); 
       #endif
}

void __sramfunc board_pmu_resume(void)
{
       #if defined(CONFIG_REGULATOR_ACT8846) || defined(CONFIG_REGULATOR_ACT8846_MODULE)
       if(pmic_is_act8846())
       board_pmu_act8846_resume(); 
       #endif
}

int __sramdata gpio3d6_iomux,gpio3d6_do,gpio3d6_dir,gpio3d6_en;

#define grf_readl(offset)	readl_relaxed(RK30_GRF_BASE + offset)
#define grf_writel(v, offset)	do { writel_relaxed(v, RK30_GRF_BASE + offset); dsb(); } while (0)
 
void __sramfunc rk30_pwm_logic_suspend_voltage(void)
{
#ifdef CONFIG_RK30_PWM_REGULATOR

//	int gpio0d7_iomux,gpio0d7_do,gpio0d7_dir,gpio0d7_en;
	sram_udelay(10000);
	gpio3d6_iomux = grf_readl(GRF_GPIO3D_IOMUX);
	gpio3d6_do = grf_readl(GRF_GPIO3H_DO);
	gpio3d6_dir = grf_readl(GRF_GPIO3H_DIR);
	gpio3d6_en = grf_readl(GRF_GPIO3H_EN);

	grf_writel((1<<28), GRF_GPIO3D_IOMUX);
	grf_writel((1<<30)|(1<<14), GRF_GPIO3H_DIR);
	grf_writel((1<<30)|(1<<14), GRF_GPIO3H_DO);
	grf_writel((1<<30)|(1<<14), GRF_GPIO3H_EN);
#endif 
}
void __sramfunc rk30_pwm_logic_resume_voltage(void)
{
#ifdef CONFIG_RK30_PWM_REGULATOR
	grf_writel((1<<28)|gpio3d6_iomux, GRF_GPIO3D_IOMUX);
	grf_writel((1<<30)|gpio3d6_en, GRF_GPIO3H_EN);
	grf_writel((1<<30)|gpio3d6_dir, GRF_GPIO3H_DIR);
	grf_writel((1<<30)|gpio3d6_do, GRF_GPIO3H_DO);
	sram_udelay(10000);

#endif

}
extern void pwm_suspend_voltage(void);
extern void pwm_resume_voltage(void);
void  rk30_pwm_suspend_voltage_set(void)
{
#ifdef CONFIG_RK30_PWM_REGULATOR
	pwm_suspend_voltage();
#endif
}
void  rk30_pwm_resume_voltage_set(void)
{
#ifdef CONFIG_RK30_PWM_REGULATOR
	pwm_resume_voltage();
#endif
}


#ifdef CONFIG_I2C2_RK30
static struct i2c_board_info __initdata i2c2_info[] = {

#ifdef CONFIG_IT66121
{
		.type		= "it66121",
		.addr		= 0x4c,
		.flags		= 0,
		.irq		= RK30_PIN2_PD6,
		.platform_data 	= &hdmi_data,
},
#endif

};
#endif

#ifdef CONFIG_I2C3_RK30
static struct i2c_board_info __initdata i2c3_info[] = {
};
#endif

#ifdef CONFIG_I2C4_RK30
static struct i2c_board_info __initdata i2c4_info[] = {

#if defined (CONFIG_MFD_RK1000) || defined (CONFIG_MFD_RK1000_MODULE)
	{
		.type		= "rk1000_control",
		.addr		= 0x40,
		.flags		= 0,
		.platform_data = &tv_data,
	},
#ifdef CONFIG_RK1000_TVOUT
    {
		.type           = "rk1000_tvout",
		.addr           = 0x42,
		.flags          = 0,
		.platform_data = &tv_data,
    },
#endif
#if defined(CONFIG_SND_SOC_RK1000) || defined(CONFIG_SND_SOC_RK1000_MODULE)
    {
		.type           = "rk1000_i2c_codec",
		.addr           = 0x60,
		.flags          = 0,
		.platform_data = &tv_data,
    },
#endif
#endif
};
#endif

#ifdef CONFIG_I2C_GPIO_RK30
#define I2C_SDA_PIN     INVALID_GPIO// RK30_PIN2_PD6   //set sda_pin here
#define I2C_SCL_PIN     INVALID_GPIO//RK30_PIN2_PD7   //set scl_pin here
static int rk30_i2c_io_init(void)
{
        //set iomux (gpio) here
        //rk30_mux_api_set(GPIO2D7_I2C1SCL_NAME, GPIO2D_GPIO2D7);
        //rk30_mux_api_set(GPIO2D6_I2C1SDA_NAME, GPIO2D_GPIO2D6);

        return 0;
}
struct i2c_gpio_platform_data default_i2c_gpio_data = {
       .sda_pin = I2C_SDA_PIN,
       .scl_pin = I2C_SCL_PIN,
       .udelay = 5, // clk = 500/udelay = 100Khz
       .timeout = 100,//msecs_to_jiffies(100),
       .bus_num    = 5,
       .io_init = rk30_i2c_io_init,
};
static struct i2c_board_info __initdata i2c_gpio_info[] = {
};
#endif

static void __init rk30_i2c_register_board_info(void)
{
#ifdef CONFIG_I2C0_RK30
	i2c_register_board_info(0, i2c0_info, ARRAY_SIZE(i2c0_info));
#endif
#ifdef CONFIG_I2C1_RK30
	i2c_register_board_info(1, i2c1_info, ARRAY_SIZE(i2c1_info));
#endif
#ifdef CONFIG_I2C2_RK30
	i2c_register_board_info(2, i2c2_info, ARRAY_SIZE(i2c2_info));
#endif
#ifdef CONFIG_I2C3_RK30
	i2c_register_board_info(3, i2c3_info, ARRAY_SIZE(i2c3_info));
#endif
#ifdef CONFIG_I2C4_RK30
	i2c_register_board_info(4, i2c4_info, ARRAY_SIZE(i2c4_info));
#endif
#ifdef CONFIG_I2C_GPIO_RK30
	i2c_register_board_info(5, i2c_gpio_info, ARRAY_SIZE(i2c_gpio_info));
#endif
}
//end of i2c

#define POWER_ON_PIN RK30_PIN0_PA0   //power_hold
static void rk30_pm_power_off(void)
{
	printk(KERN_ERR "rk30_pm_power_off start...\n");
#if defined(CONFIG_REGULATOR_ACT8846)
       if (pmic_is_act8846()) {
               if(gpio_get_value (RK30_PIN0_PB2) == GPIO_LOW)
               {
                       arm_pm_restart(0, NULL);
               }
       }
#endif
	gpio_direction_output(POWER_ON_PIN, GPIO_LOW);
	while (1);
}

static void __init machine_rk30_board_init(void)
{
	avs_init();
	gpio_request(POWER_ON_PIN, "poweronpin");
	gpio_direction_output(POWER_ON_PIN, GPIO_HIGH);
#ifdef CONFIG_POWERON_LED_ENABLE
	int pwm_gpio;
	pwm_gpio = iomux_mode_to_gpio(PWM0);
	if (gpio_request(pwm_gpio, NULL)) {
		printk("func %s, line %d: request gpio fail\n", __FUNCTION__, __LINE__);
		return -1;
	}
	gpio_direction_output(pwm_gpio, GPIO_HIGH);
#endif
	pm_power_off = rk30_pm_power_off;
	
	gpio_direction_output(POWER_ON_PIN, GPIO_HIGH);


	rk30_i2c_register_board_info();
	platform_add_devices(devices, ARRAY_SIZE(devices));
	board_usb_detect_init(RK30_PIN0_PA7);

}

static void __init rk30_reserve(void)
{
#ifdef CONFIG_ION
	rk30_ion_pdata.heaps[0].base = board_mem_reserve_add("ion", ION_RESERVE_SIZE);
#endif

#ifdef CONFIG_FB_ROCKCHIP
	resource_fb[0].start = board_mem_reserve_add("fb0 buf", get_fb_size());
	resource_fb[0].end = resource_fb[0].start + get_fb_size()- 1;

#if defined(CONFIG_FB_ROTATE) || !defined(CONFIG_THREE_FB_BUFFER)
	resource_fb[1].start = board_mem_reserve_add("fb1 buf", get_fb_size());
	resource_fb[1].end = resource_fb[0].start + get_fb_size()- 1;

	resource_fb[2].start = board_mem_reserve_add("fb2 buf",get_fb_size());
	resource_fb[2].end = resource_fb[2].start + get_fb_size() - 1;
#endif
#endif


#ifdef CONFIG_VIDEO_RK29
	rk30_camera_request_reserve_mem();
#endif
	
	board_mem_reserved();
}

/**
 * dvfs_cpu_logic_table: table for arm and logic dvfs 
 * @frequency	: arm frequency
 * @cpu_volt	: arm voltage depend on frequency
 * @logic_volt	: logic voltage arm requests depend on frequency
 * comments	: min arm/logic voltage
 */
static struct cpufreq_frequency_table dvfs_arm_table[] = {
#ifdef CONFIG_RK_CPU_312
        {.frequency = 312 * 1000,       .index = CONFIG_RK_CPU_312_VOLT * 1000},
#endif
#ifdef CONFIG_RK_CPU_504
        {.frequency = 504 * 1000,       .index = CONFIG_RK_CPU_504_VOLT * 1000},
#endif
#ifdef CONFIG_RK_CPU_816
        {.frequency = 816 * 1000,       .index = CONFIG_RK_CPU_816_VOLT * 1000},
#endif
#ifdef CONFIG_RK_CPU_1008
        {.frequency = 1008 * 1000,      .index = CONFIG_RK_CPU_1008_VOLT * 1000},
#endif
#ifdef CONFIG_RK_CPU_1200
        {.frequency = 1200 * 1000,      .index = CONFIG_RK_CPU_1200_VOLT * 1000},
#endif
#ifdef CONFIG_RK_CPU_1416
        {.frequency = 1416 * 1000,      .index = CONFIG_RK_CPU_1416_VOLT * 1000},
#endif
#ifdef CONFIG_RK_CPU_1608
        {.frequency = 1608 * 1000,      .index = CONFIG_RK_CPU_1608_VOLT * 1000},
#endif
#ifdef CONFIG_RK_CPU_1704
        {.frequency = 1704 * 1000,      .index = CONFIG_RK_CPU_1704_VOLT * 1000},
#endif
#ifdef CONFIG_RK_CPU_1800
        {.frequency = 1800 * 1000,      .index = CONFIG_RK_CPU_1800_VOLT * 1000},
#endif
#ifdef CONFIG_RK_CPU_1896
        {.frequency = 1896 * 1000,      .index = CONFIG_RK_CPU_1896_VOLT * 1000},
#endif
#ifdef CONFIG_RK_CPU_1920
        {.frequency = 1920 * 1000,      .index = CONFIG_RK_CPU_1920_VOLT * 1000},
#endif
#ifdef CONFIG_RK_CPU_2016
        {.frequency = 2016 * 1000,      .index = CONFIG_RK_CPU_2016_VOLT * 1000},
#endif
	{.frequency = CPUFREQ_TABLE_END},
};

static struct cpufreq_frequency_table dvfs_gpu_table[] = {
#ifdef CONFIG_RK_GPU_133
	   {.frequency = 133 * 1000,       .index = CONFIG_RK_GPU_133_VOLT * 1000},
#endif
#ifdef CONFIG_RK_GPU_200
       {.frequency = 200 * 1000,       .index = CONFIG_RK_GPU_200_VOLT * 1000},  
#endif
#ifdef CONFIG_RK_GPU_266
       {.frequency = 266 * 1000,       .index = CONFIG_RK_GPU_266_VOLT * 1000},  
#endif
#ifdef CONFIG_RK_GPU_300
       {.frequency = 300 * 1000,       .index = CONFIG_RK_GPU_300_VOLT * 1000},  
#endif
#ifdef CONFIG_RK_GPU_400
       {.frequency = 400 * 1000,       .index = CONFIG_RK_GPU_400_VOLT * 1000},
#endif
#ifdef CONFIG_RK_GPU_600
       {.frequency = 600 * 1000,       .index = CONFIG_RK_GPU_600_VOLT * 1000},
#endif
#ifdef CONFIG_RK_GPU_798
       {.frequency = 798 * 1000,       .index = CONFIG_RK_GPU_798_VOLT * 1000},
#endif
	{.frequency = CPUFREQ_TABLE_END},
};

static struct cpufreq_frequency_table dvfs_ddr_table[] = {
	{.frequency = 400 * 1000 + DDR_FREQ_IDLE,       .index = 1000 * 1000},
	{.frequency = 400 * 1000 + DDR_FREQ_SUSPEND,    .index = 1000 * 1000},
	{.frequency = 400 * 1000 + DDR_FREQ_VIDEO,      .index = 1000 * 1000},
#ifdef CONFIG_RK_DDR_300
	{.frequency = 300 * 1000 + DDR_FREQ_NORMAL,     .index = CONFIG_RK_DDR_300_VOLT * 1000},
#endif
#ifdef CONFIG_RK_DDR_360
	{.frequency = 360 * 1000 + DDR_FREQ_NORMAL,     .index = CONFIG_RK_DDR_360_VOLT * 1000},
#endif
#ifdef CONFIG_RK_DDR_400
	{.frequency = 400 * 1000 + DDR_FREQ_NORMAL,     .index = CONFIG_RK_DDR_400_VOLT * 1000},
#endif
#ifdef CONFIG_RK_DDR_500
	{.frequency = 500 * 1000 + DDR_FREQ_NORMAL,     .index = CONFIG_RK_DDR_500_VOLT * 1000},
#endif
#ifdef CONFIG_RK_DDR_536
	{.frequency = 536 * 1000 + DDR_FREQ_NORMAL,     .index = CONFIG_RK_DDR_536_VOLT * 1000},
#endif
#ifdef CONFIG_RK_DDR_600
	{.frequency = 600 * 1000 + DDR_FREQ_NORMAL,     .index = CONFIG_RK_DDR_600_VOLT * 1000},
#endif
#ifdef CONFIG_RK_DDR_640
	{.frequency = 640 * 1000 + DDR_FREQ_NORMAL,     .index = CONFIG_RK_DDR_640_VOLT * 1000},
#endif
#ifdef CONFIG_RK_DDR_672
	{.frequency = 672 * 1000 + DDR_FREQ_NORMAL,     .index = CONFIG_RK_DDR_672_VOLT * 1000},
#endif
#ifdef CONFIG_RK_DDR_700
	{.frequency = 700 * 1000 + DDR_FREQ_NORMAL,     .index = CONFIG_RK_DDR_700_VOLT * 1000},
#endif
#ifdef CONFIG_RK_DDR_720
	{.frequency = 720 * 1000 + DDR_FREQ_NORMAL,     .index = CONFIG_RK_DDR_720_VOLT * 1000},
#endif
#ifdef CONFIG_RK_DDR_768
	{.frequency = 768 * 1000 + DDR_FREQ_NORMAL,     .index = CONFIG_RK_DDR_768_VOLT * 1000},
#endif
#ifdef CONFIG_RK_DDR_800
	{.frequency = 800 * 1000 + DDR_FREQ_NORMAL,     .index = CONFIG_RK_DDR_800_VOLT * 1000},
#endif
	{.frequency = CPUFREQ_TABLE_END},
};

static struct cpufreq_frequency_table dvfs_ddr_table_t[] = {
	{.frequency = 200 * 1000 + DDR_FREQ_SUSPEND,    .index = 950 * 1000},
	{.frequency = 460 * 1000 + DDR_FREQ_NORMAL,     .index = 1150 * 1000},
	{.frequency = CPUFREQ_TABLE_END},
};

int get_max_freq(struct cpufreq_frequency_table *table)
{
	int i,temp=0;
	
	for(i=0;table[i].frequency!= CPUFREQ_TABLE_END;i++)
	{
		if(temp<table[i].frequency)
			temp=table[i].frequency;
	}	
	printk("get_max_freq=%d\n",temp);
	return temp;
}

void __init board_clock_init(void)
{
	u32 flags=RK30_CLOCKS_DEFAULT_FLAGS;
	rk30_clock_data_init(periph_pll_default, codec_pll_default, flags);
	//dvfs_set_arm_logic_volt(dvfs_cpu_logic_table, cpu_dvfs_table, dep_cpu2core_table);
	dvfs_set_freq_volt_table(clk_get(NULL, "cpu"), dvfs_arm_table);
	dvfs_set_freq_volt_table(clk_get(NULL, "gpu"), dvfs_gpu_table);
	if (rk_pll_flag() == 0)
		dvfs_set_freq_volt_table(clk_get(NULL, "ddr"), dvfs_ddr_table);
	else
		dvfs_set_freq_volt_table(clk_get(NULL, "ddr"), dvfs_ddr_table_t);
}

MACHINE_START(RK30, "Radxa Rock")
	.boot_params	= PLAT_PHYS_OFFSET + 0x800,
	.fixup		= rk30_fixup,
	.reserve	= &rk30_reserve,
	.map_io		= rk30_map_io,
	.init_irq	= rk30_init_irq,
	.timer		= &rk30_timer,
	.init_machine	= machine_rk30_board_init,
MACHINE_END
