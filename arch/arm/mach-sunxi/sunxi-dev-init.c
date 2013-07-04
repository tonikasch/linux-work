/*
 * Initialisation routines for devices without drivers
 * This file should go away when all drivers are ready
 *
 * Taken from various Allwinner sources
 *
 * Copyright (C) 2012 Allwinner Technology
 *
 * Andrey Panov <rockford@yandex.ru>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>

#define SW_CCMU_BASE				0x01c20000
#define SW_CCMU_BASE_LEN			0x100
#define SW_CCMU_REG_AHB_GATING_REG0		0x60
#define SW_CCMU_REG_USB_CLK_REG			0xCC

/* ABH Gating Reg0 */
#define SW_CCMU_BP_AHB_GATING_USBC2		3 // OR it is 3 in docs
#define SW_CCMU_BP_AHB_GATING_USBC1		1
#define SW_CCMU_BP_AHB_GATING_USBC0		0

/* usb clock reg */
#define SW_CCMU_BP_USB_CLK_GATING_USBPHY	8
#define SW_CCMU_BP_USB_CLK_GATING_OHCI1		7
#define SW_CCMU_BP_USB_CLK_GATING_OHCI0		6
#define SW_CCMU_BP_USB_CLK_48M_SEL		4
#define SW_CCMU_BP_USB_CLK_USBPHY2_RST		2
#define SW_CCMU_BP_USB_CLK_USBPHY1_RST		1
#define SW_CCMU_BP_USB_CLK_USBPHY0_RST		0



#define SW_PA_PORTC_IO_BASE               0x01c20800
/* Do something NO documentation */
#define SW_USB0_CSR			0x01c13404

static u32 USBC_Phy_Write(u32 usbc_no, u32 addr, u32 data, u32 len)
{
	u32 temp = 0, dtmp = 0;
	u32 j = 0;
	void __iomem *csr;

	csr = ioremap_nocache(SW_USB0_CSR, 4);

	dtmp = data;
	for (j = 0; j < len; j++) {
		/* set  the bit address to be write */
		temp = readl(csr);
		temp &= ~(0xff << 8);
		temp |= ((addr + j) << 8);
		writel(temp, csr);

		temp = readb(csr);
		temp &= ~(0x1 << 7);
		temp |= (dtmp & 0x1) << 7;
		temp &= ~(0x1 << (usbc_no << 1));
		writeb(temp, csr);

		temp = readb(csr);
		temp |= (0x1 << (usbc_no << 1));
		writeb(temp, csr);

		temp = readb(csr);
		temp &= ~(0x1 << (usbc_no << 1));
		writeb(temp, csr);
		dtmp >>= 1;
	}

	return data;
}

static void UsbPhyInit(u32 usbc_no)
{
//      DMSG_INFO("csr1: usbc%d: 0x%x\n", usbc_no, (u32)USBC_Readl(USBC_Phy_GetCsr(usbc_no)));

	/* 调节45欧阻抗 */
	if (usbc_no == 0) {
		USBC_Phy_Write(usbc_no, 0x0c, 0x01, 1);
	}
//      DMSG_INFO("csr2-0: usbc%d: 0x%x\n", usbc_no, (u32)USBC_Phy_Read(usbc_no, 0x0c, 1));

	/* 调整 USB0 PHY 的幅度和速率 */
	USBC_Phy_Write(usbc_no, 0x20, 0x14, 5);

//      DMSG_INFO("csr2-1: usbc%d: 0x%x\n", usbc_no, (u32)USBC_Phy_Read(usbc_no, 0x20, 5));

	/* 调节 disconnect 域值 */
	USBC_Phy_Write(usbc_no, 0x2a, 3, 2);

//      DMSG_INFO("csr2: usbc%d: 0x%x\n", usbc_no, (u32)USBC_Phy_Read(usbc_no, 0x2a, 2));
//      DMSG_INFO("csr3: usbc%d: 0x%x\n", usbc_no, (u32)USBC_Readl(USBC_Phy_GetCsr(usbc_no)));

	return;
}

static void __init sunxi_init_usb(void)
{
	void __iomem *ccmu_base;
	u32 reg_value;

	ccmu_base = ioremap_nocache(SW_CCMU_BASE, SW_CCMU_BASE_LEN);
	pr_debug("USB Init HACK base=%p\n",ccmu_base);

	UsbPhyInit(0);
	UsbPhyInit(1);
	UsbPhyInit(2);

	//Gating AHB clock for USB_phy
	reg_value = readl(ccmu_base + SW_CCMU_REG_AHB_GATING_REG0);
	reg_value |= (1 << SW_CCMU_BP_AHB_GATING_USBC0);
	reg_value |= (1 << SW_CCMU_BP_AHB_GATING_USBC1);
	reg_value |= (1 << SW_CCMU_BP_AHB_GATING_USBC2);
	writel(reg_value, (ccmu_base + SW_CCMU_REG_AHB_GATING_REG0));

	//delay to wati SIE stable
	mdelay(5);

	//Enable module clock for USB phy
	reg_value = readl(ccmu_base + SW_CCMU_REG_USB_CLK_REG);
	reg_value |= (1 << 9);						/* NOT IN DOCUMENTATION */
	reg_value |= (1 << SW_CCMU_BP_USB_CLK_GATING_USBPHY);
	reg_value |= (1 << SW_CCMU_BP_USB_CLK_GATING_OHCI1);
	reg_value |= (1 << SW_CCMU_BP_USB_CLK_GATING_OHCI0);
	reg_value |= (1 << 5);						/* NOT IN DOCUMENTATION */
	reg_value |= (1 << SW_CCMU_BP_USB_CLK_USBPHY2_RST);
	reg_value |= (1 << SW_CCMU_BP_USB_CLK_USBPHY1_RST);
	reg_value |= (1 << SW_CCMU_BP_USB_CLK_USBPHY0_RST);
	writel(reg_value, (ccmu_base + SW_CCMU_REG_USB_CLK_REG));

	//delay some time
	mdelay(5);

	/* GPIO PH3 & PH6 select for output */
//	writel(0x100010000,(void __iomem *)0xf1c208fc);
	/* GPIO PH3 & PH6 set high */
//	writel(0x1001000,(void __iomem *)0xf1c2090c);
}

static int __init sunxi_dev_init(void)
{
	sunxi_init_usb();
	return 0;
}

late_initcall(sunxi_dev_init);
