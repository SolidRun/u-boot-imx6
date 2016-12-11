/*
 * Copyright (C) 2012 Freescale Semiconductor, Inc.
 * Copyright (C) 2013 SolidRun ltd.
 * Copyright (C) 2013 Jon Nettleton <jon.nettleton@gmail.com>.
 *
 * Author: Fabio Estevam <fabio.estevam@freescale.com>
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <asm/arch/clock.h>
#include <asm/arch/crm_regs.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/iomux.h>
#include <asm/arch/mx6-pins.h>
#include <asm/arch/mxc_hdmi.h>
#include <asm/arch/sys_proto.h>
#include <asm/errno.h>
#include <asm/gpio.h>
#include <asm/imx-common/iomux-v3.h>
#include <asm/imx-common/boot_mode.h>
#include <asm/io.h>
#include <mmc.h>
#include <fsl_esdhc.h>
#include <miiphy.h>
#include <netdev.h>
#include <ipu_pixfmt.h>
#include "../../../drivers/video/mxcfb.h"

DECLARE_GLOBAL_DATA_PTR;

#define MX6QDL_SET_PAD(p, q) \
	if (is_cpu_type(MXC_CPU_MX6Q)) \
		imx_iomux_v3_setup_pad(MX6Q_##p | q);\
	else \
		imx_iomux_v3_setup_pad(MX6DL_##p | q)

#define UART_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define USDHC_PAD_CTRL (PAD_CTL_PUS_47K_UP |			\
	PAD_CTL_SPEED_LOW | PAD_CTL_DSE_80ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define USDHC_PAD_CLK_CTRL (PAD_CTL_SPEED_LOW |			\
	PAD_CTL_DSE_80ohm | PAD_CTL_SRE_FAST |			\
	PAD_CTL_HYS)

#define USDHC_PAD_GPIO_CTRL (PAD_CTL_PUS_22K_UP |		\
	PAD_CTL_SPEED_LOW | PAD_CTL_DSE_40ohm |			\
	PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define ENET_PAD_CTRL  (PAD_CTL_PUS_100K_UP |			\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS)

#define ENET_PAD_CTRL_OD  (PAD_CTL_PUS_100K_UP |		\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS | PAD_CTL_ODE)

#define ENET_PAD_CTRL_PD  (PAD_CTL_PUS_100K_DOWN |		\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS)

#define ENET_PAD_CTRL_CLK  (PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | \
	PAD_CTL_SRE_FAST)

#define LED IMX_GPIO_NR(4, 29)

int hb_cuboxi_ = 0; /* 2 is HummingBoard2, 1 is HummingBoard, 0 is CuBox-i */
int ver_15_ = 0; /* 0 is original som, 1 is the rev 1.5 */

int dram_init(void)
{
	gd->ram_size = imx_ddr_size();

	return 0;
}

#if defined(CONFIG_MX6Q) || defined(CONFIG_MX6DL)
iomux_v3_cfg_t const uart1_pads[] = {
	MX6_PAD_CSI0_DAT10__UART1_TXD | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_CSI0_DAT11__UART1_RXD | MUX_PAD_CTRL(UART_PAD_CTRL),
};


iomux_v3_cfg_t const usdhc2_pads[] = {
	MX6_PAD_SD2_CLK__USDHC2_CLK	| MUX_PAD_CTRL(USDHC_PAD_CLK_CTRL),
	MX6_PAD_SD2_CMD__USDHC2_CMD	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DAT0__USDHC2_DAT0	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DAT1__USDHC2_DAT1	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DAT2__USDHC2_DAT2	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DAT3__USDHC2_DAT3	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
        MX6_PAD_GPIO_4__USDHC2_CD       | MUX_PAD_CTRL(USDHC_PAD_GPIO_CTRL),
};
iomux_v3_cfg_t const hb_cbi_sense[] = {
	/* Following is only for sensing if it's CuBox-i or HummingBoard */
	MX6_PAD_KEY_ROW1__GPIO_4_9      | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_EIM_DA4__GPIO_3_4       | MUX_PAD_CTRL(UART_PAD_CTRL),
	PAD_SD4_DAT0__GPIO_2_8		| MUX_PAD_CTRL(UART_PAD_CTRL),
	PAD_CSI0_DAT14__GPIO_6_0	| MUX_PAD_CTRL(UART_PAD_CTRL),
	PAD_CSI0_DAT18__GPIO_6_4	| MUX_PAD_CTRL(UART_PAD_CTRL),
};
#endif

static void setup_iomux_uart(void)
{
#if defined(CONFIG_MX6Q) || defined(CONFIG_MX6DL)
	imx_iomux_v3_setup_multiple_pads(uart1_pads, ARRAY_SIZE(uart1_pads));
#endif
#if defined(CONFIG_MX6QDL)
	MX6QDL_SET_PAD(PAD_CSI0_DAT10__UART1_TXD, MUX_PAD_CTRL(UART_PAD_CTRL));
	MX6QDL_SET_PAD(PAD_CSI0_DAT11__UART1_RXD, MUX_PAD_CTRL(UART_PAD_CTRL));
#endif
}

#ifdef CONFIG_FSL_ESDHC
struct fsl_esdhc_cfg usdhc2_cfg = {
        { USDHC2_BASE_ADDR }};

struct fsl_esdhc_cfg usdhc3_cfg = {
	{ USDHC3_BASE_ADDR }};

int board_mmc_getcd(struct mmc *mmc)
{
        struct fsl_esdhc_cfg *cfg = (struct fsl_esdhc_cfg *)mmc->priv;

        if (cfg->esdhc_base == USDHC2_BASE_ADDR) {
                return !gpio_get_value(IMX_GPIO_NR(1, 4));
        } else if (cfg->esdhc_base == USDHC3_BASE_ADDR) {
		/*
		 * USDHC3 is typically MMC which doesn't have card detect
		 * signal
		 */
		return -1;
	}
        return 0;
}

int board_mmc_init(bd_t *bis)
{
	int status = 0;
	uint soc_sbmr = readl(SRC_BASE_ADDR + 0x4);
	uint bt_mem_ctl = (soc_sbmr & 0x000000FF) >> 4 ;
	usdhc2_cfg.sdhc_clk = mxc_get_clock(MXC_ESDHC2_CLK);
	usdhc2_cfg.max_bus_width = 4;
#if defined(CONFIG_MX6Q) || defined(CONFIG_MX6DL)
        imx_iomux_v3_setup_multiple_pads(usdhc2_pads, ARRAY_SIZE(usdhc2_pads));
#endif
#if defined(CONFIG_MX6QDL)
	MX6QDL_SET_PAD(PAD_SD2_CLK__USDHC2_CLK   , MUX_PAD_CTRL(USDHC_PAD_CTRL));
	MX6QDL_SET_PAD(PAD_SD2_CMD__USDHC2_CMD   , MUX_PAD_CTRL(USDHC_PAD_CTRL));
	MX6QDL_SET_PAD(PAD_SD2_DAT0__USDHC2_DAT0 , MUX_PAD_CTRL(USDHC_PAD_CTRL));
	MX6QDL_SET_PAD(PAD_SD2_DAT1__USDHC2_DAT1 , MUX_PAD_CTRL(USDHC_PAD_CTRL));
	MX6QDL_SET_PAD(PAD_SD2_DAT2__USDHC2_DAT2 , MUX_PAD_CTRL(USDHC_PAD_CTRL));
	MX6QDL_SET_PAD(PAD_SD2_DAT3__USDHC2_DAT3 , MUX_PAD_CTRL(USDHC_PAD_CTRL));
	/* Carrier MicroSD Card Detect */
	MX6QDL_SET_PAD(PAD_GPIO_4__USDHC2_CD	, MUX_PAD_CTRL(NO_PAD_CTRL));
#endif
        gpio_direction_input(IMX_GPIO_NR(1, 4));
        usdhc2_cfg.sdhc_clk = mxc_get_clock(MXC_ESDHC2_CLK);

	if (bt_mem_ctl != 0x6) {
		/* Only in case of booting from eMMC, register the micro SD later */
		status = fsl_esdhc_initialize(bis, &usdhc2_cfg);
	}

	if (hb_cuboxi_ == 2) { /* HummingBoard 2 */
		MX6QDL_SET_PAD(PAD_SD3_CLK__USDHC3_CLK   , MUX_PAD_CTRL(USDHC_PAD_CTRL));
		MX6QDL_SET_PAD(PAD_SD3_CMD__USDHC3_CMD   , MUX_PAD_CTRL(USDHC_PAD_CTRL));
		MX6QDL_SET_PAD(PAD_SD3_DAT0__USDHC3_DAT0 , MUX_PAD_CTRL(USDHC_PAD_CTRL));
		MX6QDL_SET_PAD(PAD_SD3_DAT1__USDHC3_DAT1 , MUX_PAD_CTRL(USDHC_PAD_CTRL));
		MX6QDL_SET_PAD(PAD_SD3_DAT2__USDHC3_DAT2 , MUX_PAD_CTRL(USDHC_PAD_CTRL));
		MX6QDL_SET_PAD(PAD_SD3_DAT3__USDHC3_DAT3 , MUX_PAD_CTRL(USDHC_PAD_CTRL));
		MX6QDL_SET_PAD(PAD_SD3_DAT4__USDHC3_DAT4 , MUX_PAD_CTRL(USDHC_PAD_CTRL));
		MX6QDL_SET_PAD(PAD_SD3_DAT5__USDHC3_DAT5 , MUX_PAD_CTRL(USDHC_PAD_CTRL));
		MX6QDL_SET_PAD(PAD_SD3_DAT6__USDHC3_DAT6 , MUX_PAD_CTRL(USDHC_PAD_CTRL));
		MX6QDL_SET_PAD(PAD_SD3_DAT7__USDHC3_DAT7 , MUX_PAD_CTRL(USDHC_PAD_CTRL));
		MX6QDL_SET_PAD(PAD_SD3_RST__USDHC3_RST , MUX_PAD_CTRL(USDHC_PAD_CTRL));
		usdhc3_cfg.max_bus_width = 8;
		usdhc3_cfg.sdhc_clk = mxc_get_clock(MXC_ESDHC3_CLK);
		fsl_esdhc_initialize(bis, &usdhc3_cfg);
	}
	if (bt_mem_ctl == 0x6) {
		status = fsl_esdhc_initialize(bis, &usdhc2_cfg);
	}
	return status;
}
#endif

#ifdef CONFIG_FEC_MXC
#if defined(CONFIG_MX6Q) || defined(CONFIG_MX6DL)
iomux_v3_cfg_t const enet_pads[] = {
	MX6_PAD_ENET_MDIO__ENET_MDIO		| MUX_PAD_CTRL(ENET_PAD_CTRL_OD),
	MX6_PAD_ENET_MDC__ENET_MDC		| MUX_PAD_CTRL(ENET_PAD_CTRL),		
	/* AR8035 reset */
	MX6_PAD_KEY_ROW4__GPIO_4_15		| MUX_PAD_CTRL(ENET_PAD_CTRL_PD),		
	/* AR8035 interrupt */
	MX6_PAD_DI0_PIN2__GPIO_4_18		| MUX_PAD_CTRL(NO_PAD_CTRL),		
	/* GPIO16 -> AR8035 25MHz */
	MX6_PAD_GPIO_16__ENET_ANATOP_ETHERNET_REF_OUT	| MUX_PAD_CTRL(NO_PAD_CTRL),		
	MX6_PAD_RGMII_TXC__ENET_RGMII_TXC		| MUX_PAD_CTRL(NO_PAD_CTRL),		
	MX6_PAD_RGMII_TD0__ENET_RGMII_TD0		| MUX_PAD_CTRL(ENET_PAD_CTRL),		
	MX6_PAD_RGMII_TD1__ENET_RGMII_TD1		| MUX_PAD_CTRL(ENET_PAD_CTRL),		
	MX6_PAD_RGMII_TD2__ENET_RGMII_TD2		| MUX_PAD_CTRL(ENET_PAD_CTRL),		
	MX6_PAD_RGMII_TD3__ENET_RGMII_TD3		| MUX_PAD_CTRL(ENET_PAD_CTRL),		
	MX6_PAD_RGMII_TX_CTL__RGMII_TX_CTL		| MUX_PAD_CTRL(ENET_PAD_CTRL),		
	/* AR8035 CLK_25M --> ENET_REF_CLK (V22) */
	MX6_PAD_ENET_REF_CLK__ENET_TX_CLK		| MUX_PAD_CTRL(ENET_PAD_CTRL_CLK),		
        MX6_PAD_RGMII_RXC__ENET_RGMII_RXC       | MUX_PAD_CTRL(ENET_PAD_CTRL),
        MX6_PAD_RGMII_RD0__ENET_RGMII_RD0       | MUX_PAD_CTRL(ENET_PAD_CTRL_PD),
        MX6_PAD_RGMII_RD1__ENET_RGMII_RD1       | MUX_PAD_CTRL(ENET_PAD_CTRL_PD),
        MX6_PAD_RGMII_RD2__ENET_RGMII_RD2       | MUX_PAD_CTRL(ENET_PAD_CTRL),
        MX6_PAD_RGMII_RD3__ENET_RGMII_RD3       | MUX_PAD_CTRL(ENET_PAD_CTRL),
        MX6_PAD_RGMII_RX_CTL__RGMII_RX_CTL      | MUX_PAD_CTRL(ENET_PAD_CTRL_PD),
};
#endif

static void setup_iomux_enet(void)
{
#if defined(CONFIG_MX6Q) || defined(CONFIG_MX6DL)
	imx_iomux_v3_setup_multiple_pads(enet_pads, ARRAY_SIZE(enet_pads));
#endif
#if defined(CONFIG_MX6QDL)
	MX6QDL_SET_PAD(PAD_ENET_MDIO__ENET_MDIO, MUX_PAD_CTRL(ENET_PAD_CTRL_OD));
	MX6QDL_SET_PAD(PAD_ENET_MDC__ENET_MDC, MUX_PAD_CTRL(ENET_PAD_CTRL));
	/* AR8035 reset */
	MX6QDL_SET_PAD(PAD_KEY_ROW4__GPIO_4_15, MUX_PAD_CTRL(ENET_PAD_CTRL_PD));
	/* AR8035 interrupt */
	MX6QDL_SET_PAD(PAD_DI0_PIN2__GPIO_4_18, MUX_PAD_CTRL(NO_PAD_CTRL));
	/* GPIO16 -> AR8035 25MHz */
	MX6QDL_SET_PAD(PAD_GPIO_16__ENET_ANATOP_ETHERNET_REF_OUT, MUX_PAD_CTRL(NO_PAD_CTRL));
	MX6QDL_SET_PAD(PAD_RGMII_TXC__ENET_RGMII_TXC, MUX_PAD_CTRL(NO_PAD_CTRL));
	MX6QDL_SET_PAD(PAD_RGMII_TD0__ENET_RGMII_TD0, MUX_PAD_CTRL(ENET_PAD_CTRL));
	MX6QDL_SET_PAD(PAD_RGMII_TD1__ENET_RGMII_TD1, MUX_PAD_CTRL(ENET_PAD_CTRL));
	MX6QDL_SET_PAD(PAD_RGMII_TD2__ENET_RGMII_TD2, MUX_PAD_CTRL(ENET_PAD_CTRL));
	MX6QDL_SET_PAD(PAD_RGMII_TD3__ENET_RGMII_TD3, MUX_PAD_CTRL(ENET_PAD_CTRL));
	MX6QDL_SET_PAD(PAD_RGMII_TX_CTL__RGMII_TX_CTL, MUX_PAD_CTRL(ENET_PAD_CTRL));
	/* AR8035 CLK_25M --> ENET_REF_CLK (V22) */
	MX6QDL_SET_PAD(PAD_ENET_REF_CLK__ENET_TX_CLK, MUX_PAD_CTRL(ENET_PAD_CTRL_CLK));
	MX6QDL_SET_PAD(PAD_RGMII_RXC__ENET_RGMII_RXC, MUX_PAD_CTRL(ENET_PAD_CTRL));
	MX6QDL_SET_PAD(PAD_RGMII_RD0__ENET_RGMII_RD0, MUX_PAD_CTRL(ENET_PAD_CTRL_PD));
	MX6QDL_SET_PAD(PAD_RGMII_RD1__ENET_RGMII_RD1, MUX_PAD_CTRL(ENET_PAD_CTRL_PD));
	MX6QDL_SET_PAD(PAD_RGMII_RD2__ENET_RGMII_RD2, MUX_PAD_CTRL(ENET_PAD_CTRL));
	MX6QDL_SET_PAD(PAD_RGMII_RD3__ENET_RGMII_RD3, MUX_PAD_CTRL(ENET_PAD_CTRL));
	MX6QDL_SET_PAD(PAD_RGMII_RX_CTL__RGMII_RX_CTL, MUX_PAD_CTRL(ENET_PAD_CTRL_PD));
	MX6QDL_SET_PAD(PAD_ENET_RXD0__GPIO_1_27, MUX_PAD_CTRL(ENET_PAD_CTRL_PD));
	MX6QDL_SET_PAD(PAD_ENET_RXD1__GPIO_1_26, MUX_PAD_CTRL(ENET_PAD_CTRL_PD));
#endif
	/*
	 * Reset AR8035 PHY. Since it runs 25MHz reference clock, it
	 * requires two resets.
	 */
	gpio_direction_output(IMX_GPIO_NR(4, 15), 0);
	udelay(1000 * 2);
	gpio_set_value(IMX_GPIO_NR(4, 15), 1);
	udelay(1000 * 2);
	gpio_set_value(IMX_GPIO_NR(4, 15), 0);
	udelay(1000 * 2);
	gpio_set_value(IMX_GPIO_NR(4, 15), 1);
	udelay(1000 * 2);
}
int fecmxc_initialize(bd_t *bd)
{
	/*
	 * Initialize the phy in address 0x0 or 0x4 (0x11 phy mask).
	 * The LED_ACT pin on the carrier-one boards had a pull down that
	 * forces the phy address to 0x0; where on CuBox-i and the production
	 * HummingBoard that pin is connected directly to LED that depending
	 * on the pull down strength of the LED it might be sampled
	 * as '0' or '1' thus the phy address might appear as either address
	 * 0x0 or 0x4.
	 */
	return fecmxc_initialize_multi(bd, -1, 0x11, IMX_FEC_BASE);
}

int board_phy_config(struct phy_device *phydev)
{
	if (phydev->drv->config)
		phydev->drv->config(phydev);

	return 0;
}

int enable_fec_anatop_clock(void)
{
	u32 reg = 0;
	s32 timeout = 100000;

	struct anatop_regs __iomem *anatop =
	(struct anatop_regs __iomem *)ANATOP_BASE_ADDR;

	reg = readl(&anatop->pll_enet);
	reg &= 0xfffffffc; /* Set PLL to generate 25MHz */
	writel(reg, &anatop->pll_enet);
	if ((reg & BM_ANADIG_PLL_ENET_POWERDOWN) ||
	    (!(reg & BM_ANADIG_PLL_ENET_LOCK))) {
		reg &= ~BM_ANADIG_PLL_ENET_POWERDOWN;
		writel(reg, &anatop->pll_enet);
		while (timeout--) {
			if (readl(&anatop->pll_enet) & BM_ANADIG_PLL_ENET_LOCK)
				break;
		}
		if (timeout < 0)
			return -ETIMEDOUT;
	}
	/* Enable FEC clock */
	reg |= BM_ANADIG_PLL_ENET_ENABLE;
	reg &= ~BM_ANADIG_PLL_ENET_BYPASS;
	writel(reg, &anatop->pll_enet);

	return 0;
}
int board_eth_init(bd_t *bis)
{
	int ret;
        struct iomuxc_base_regs *const iomuxc_regs
                = (struct iomuxc_base_regs *) IOMUXC_BASE_ADDR;
	struct anatop_regs __iomem *anatop =
                (struct anatop_regs __iomem *)ANATOP_BASE_ADDR;
	s32 timeout = 100000;

	enable_fec_anatop_clock();
	/* set gpr1[21] */
        clrsetbits_le32(&iomuxc_regs->gpr[1], 0, (1 << 21));

	while (timeout--) {
        	if (readl(&anatop->pll_enet) & BM_ANADIG_PLL_ENET_LOCK)
			break;
	}

	setup_iomux_enet();

	ret = cpu_eth_init(bis);
	if (ret)
		printf("FEC MXC: %s:failed\n", __func__);

	return 0;
}
#endif

#ifdef CONFIG_VIDEO_IPUV3
static struct fb_videomode const hdmi = {
        .name           = "HDMI",
        .refresh        = 60,
        .xres           = 1024,
        .yres           = 768,
        .pixclock       = 15384,
        .left_margin    = 160,
        .right_margin   = 24,
        .upper_margin   = 29,
        .lower_margin   = 3,
        .hsync_len      = 136,
        .vsync_len      = 6,
        .sync           = FB_SYNC_EXT,
        .vmode          = FB_VMODE_NONINTERLACED
};

static int detect_hdmi(void)
{
        struct hdmi_regs *hdmi  = (struct hdmi_regs *)HDMI_ARB_BASE_ADDR;
        return readb(&hdmi->phy_stat0) & HDMI_DVI_STAT;
}

int board_video_skip(void)
{
        int ret;

        ret = ipuv3_fb_init(&hdmi, 0, IPU_PIX_FMT_RGB24);

        if (ret)
                printf("HDMI cannot be configured: %d\n", ret);

	if (detect_hdmi())
		imx_enable_hdmi_phy();

        return ret;
}

static void setup_display(void)
{
        struct mxc_ccm_reg *mxc_ccm = (struct mxc_ccm_reg *)CCM_BASE_ADDR;
        int reg;
	s32 timeout = 100000;

        enable_ipu_clock();
        imx_setup_hdmi();

	/* set video pll to 455MHz (24MHz * (37+11/12) / 2) */
	reg = readl(&mxc_ccm->analog_pll_video);
	reg |= BM_ANADIG_PLL_VIDEO_POWERDOWN;
	writel(reg, &mxc_ccm->analog_pll_video);

	reg &= ~BM_ANADIG_PLL_VIDEO_DIV_SELECT;
	reg |= BF_ANADIG_PLL_VIDEO_DIV_SELECT(37);
	reg &= ~BM_ANADIG_PLL_VIDEO_POST_DIV_SELECT;
	reg |= BF_ANADIG_PLL_VIDEO_POST_DIV_SELECT(1);
	writel(reg, &mxc_ccm->analog_pll_video);

	writel(BF_ANADIG_PLL_VIDEO_NUM_A(11), &mxc_ccm->analog_pll_video_num);
	writel(BF_ANADIG_PLL_VIDEO_DENOM_B(12), &mxc_ccm->analog_pll_video_denom);

	reg &= ~BM_ANADIG_PLL_VIDEO_POWERDOWN;
	writel(reg, &mxc_ccm->analog_pll_video);

	while (timeout--)
		if (readl(&mxc_ccm->analog_pll_video) & BM_ANADIG_PLL_VIDEO_LOCK)
			break;
	if (timeout < 0)
		printf("Warning: video pll lock timeout!\n");

	reg = readl(&mxc_ccm->analog_pll_video);
	reg |= BM_ANADIG_PLL_VIDEO_ENABLE;
	reg &= ~BM_ANADIG_PLL_VIDEO_BYPASS;
	writel(reg, &mxc_ccm->analog_pll_video);

	/* select video pll for ldb_di0_clk */
	reg = readl(&mxc_ccm->cs2cdr);
	reg &= ~(MXC_CCM_CS2CDR_LDB_DI0_CLK_SEL_MASK);
	writel(reg, &mxc_ccm->cs2cdr);

	/* select ldb_di0_clk / 7 for ldb_di0_ipu_clk */
	reg = readl(&mxc_ccm->cscmr2);
	reg |= MXC_CCM_CSCMR2_LDB_DI0_IPU_DIV;
	writel(reg, &mxc_ccm->cscmr2);

	/* select ldb_di0_ipu_clk for ipu1_di0_clk -> 65MHz pixclock */
	reg = readl(&mxc_ccm->chsccdr);
	reg |= (CHSCCDR_CLK_SEL_LDB_DI0
		<< MXC_CCM_CHSCCDR_IPU1_DI0_CLK_SEL_OFFSET);
	writel(reg, &mxc_ccm->chsccdr);
}
#endif /* CONFIG_VIDEO_IPUV3 */

#ifdef CONFIG_USB_EHCI_MX6
#define USB_OTG_EN IMX_GPIO_NR(3, 22)
#define USB_H1_EN IMX_GPIO_NR(1, 0)
#if defined(CONFIG_MX6Q) || defined(CONFIG_MX6DL)
iomux_v3_cfg_t const usb_en_pads[] = {
	MX6_PAD_EIM_D22__GPIO_3_22 | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_GPIO_0__GPIO_1_0 | MUX_PAD_CTRL(UART_PAD_CTRL),
};
iomux_v3_cfg_t const usb_id_pad[] = {
	MX6_PAD_GPIO_1__USB_OTG_ID,
};
#endif

int board_ehci_hcd_init(int port)
{
        return 0;
}
#endif

char config_sys_prompt_cuboxi[] = "CuBox-i U-Boot > ";
char config_sys_prompt_hummingboard[] = "HummingBoard U-Boot > ";
char config_sys_prompt_hummingboard2[] = "HummingBoard2 U-Boot > ";
char *config_sys_prompt = config_sys_prompt_cuboxi;
static void detect_board(void)
{
	int val1,val2,val3,val4,val5;
#if defined(CONFIG_MX6Q) || defined(CONFIG_MX6DL)
	imx_iomux_v3_setup_multiple_pads(hb_cbi_sense, ARRAY_SIZE(hb_cbi_sense));
#endif
#if defined(CONFIG_MX6QDL)
	MX6QDL_SET_PAD(PAD_KEY_ROW1__GPIO_4_9, MUX_PAD_CTRL(UART_PAD_CTRL));
	MX6QDL_SET_PAD(PAD_EIM_DA4__GPIO_3_4, MUX_PAD_CTRL(UART_PAD_CTRL));
	MX6QDL_SET_PAD(PAD_SD4_DAT0__GPIO_2_8, MUX_PAD_CTRL(UART_PAD_CTRL));
	MX6QDL_SET_PAD(PAD_CSI0_DAT14__GPIO_6_0, MUX_PAD_CTRL(UART_PAD_CTRL));
	MX6QDL_SET_PAD(PAD_CSI0_DAT18__GPIO_6_4, MUX_PAD_CTRL(UART_PAD_CTRL));
#endif
	gpio_direction_input(IMX_GPIO_NR(4, 9));
	gpio_direction_input(IMX_GPIO_NR(3, 4));
	gpio_direction_input(IMX_GPIO_NR(2, 8));
	gpio_direction_input(IMX_GPIO_NR(6, 0));
	gpio_direction_input(IMX_GPIO_NR(6, 4));

	val1 = gpio_get_value(IMX_GPIO_NR(4, 9));
	val2 = gpio_get_value(IMX_GPIO_NR(3, 4));
	val3 = gpio_get_value(IMX_GPIO_NR(2, 8));
	val4 = gpio_get_value(IMX_GPIO_NR(6, 0));
	val5 = gpio_get_value(IMX_GPIO_NR(6, 4));

	/*
	 * Machine selection -
	 * Machine        val1, val2
	 * ----------------------------
	 * HB2            x     x    0
	 * HB rev 3.x     x     0    x
	 * CBi            0     1    x
	 * HB             1     1    x
	 */

	if (val3 == 0) {
		hb_cuboxi_ = 2;
		config_sys_prompt = config_sys_prompt_hummingboard2;
	} else if (val2 == 0) {
		hb_cuboxi_ = 1;
		config_sys_prompt = config_sys_prompt_hummingboard;
	} else if (val1 == 0) {
		hb_cuboxi_ = 0;
		config_sys_prompt = config_sys_prompt_cuboxi;
	} else {
		hb_cuboxi_ = 1;
		config_sys_prompt = config_sys_prompt_hummingboard;
	}

	if (val4 == 1 && val5 == 0) {
		ver_15_ = 1;
	} else {
		ver_15_ = 0;
	}
}

int board_early_init_f(void)
{
	setup_iomux_uart();
	detect_board();
#ifdef CONFIG_VIDEO_IPUV3
	setup_display();
#endif
	return 0;
}

#if defined(CONFIG_MX6Q) || defined(CONFIG_MX6DL)
iomux_v3_cfg_t const led_pads[] = {
	MX6_PAD_DISP0_DAT8__GPIO_4_29 | MUX_PAD_CTRL(UART_PAD_CTRL),
};
#endif

#ifdef CONFIG_CMD_SATA
int setup_sata(void)
{
	struct iomuxc_base_regs *const iomuxc_regs
		= (struct iomuxc_base_regs *) IOMUXC_BASE_ADDR;
	int ret = enable_sata_clock();
	if (ret)
		return ret;
	clrsetbits_le32(&iomuxc_regs->gpr[13],
			IOMUXC_GPR13_SATA_MASK,
			IOMUXC_GPR13_SATA_PHY_8_RXEQ_3P0DB
			|IOMUXC_GPR13_SATA_PHY_7_SATA2M
			|IOMUXC_GPR13_SATA_SPEED_3G
			|(3<<IOMUXC_GPR13_SATA_PHY_6_SHIFT)
			|IOMUXC_GPR13_SATA_SATA_PHY_5_SS_DISABLED
			|IOMUXC_GPR13_SATA_SATA_PHY_4_ATTEN_9_16
			|IOMUXC_GPR13_SATA_PHY_3_TXBOOST_0P00_DB
			|IOMUXC_GPR13_SATA_PHY_2_TX_1P104V
			|IOMUXC_GPR13_SATA_PHY_1_SLOW);
	return 0;
}
#endif

int board_init(void)
{
	/* address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;
	detect_board();
	/*
	 * The reason hb_cuboxi_ is not checked here is because it seems to be
	 * overwritten somewhere, somehow
	 */
#ifdef CONFIG_USB_EHCI_MX6
#if defined(CONFIG_MX6QDL)
	MX6QDL_SET_PAD(PAD_EIM_D22__GPIO_3_22 , MUX_PAD_CTRL(UART_PAD_CTRL));
	MX6QDL_SET_PAD(PAD_GPIO_0__GPIO_1_0 , MUX_PAD_CTRL(UART_PAD_CTRL));
	if (hb_cuboxi_ == 0) /* CuBox-i */
		MX6QDL_SET_PAD(PAD_GPIO_1__USB_OTG_ID, 0);
	else /* HummingBoard 1 and 2 */ {
		MX6QDL_SET_PAD(PAD_GPIO_5__CCM_CLKO, MUX_PAD_CTRL(UART_PAD_CTRL));
	}
#else
	/* Setup USB OTG ID */
	if (hb_cuboxi_ == 0) /* CuBox-i */
		imx_iomux_v3_setup_multiple_pads(usb_id_pad, ARRAY_SIZE(usb_id_pad));
	/* Setup enable pads */
	imx_iomux_v3_setup_multiple_pads(usb_en_pads, ARRAY_SIZE(usb_en_pads));
#endif
	/* Enable USB OTG and H1 current limiter */
	gpio_direction_output(USB_OTG_EN, 1);
	gpio_direction_output(USB_H1_EN, 1);
#endif

	if (config_sys_prompt == config_sys_prompt_cuboxi) {
		gd->bd->bi_arch_number = 4821; /* CuBox-i machine ID */
		/* Enable front LED */
		gpio_direction_output(LED, 0);
	} else {
		gd->bd->bi_arch_number = 4773; /* HummingBoard machine ID */
	}
#ifdef CONFIG_CMD_SATA
	setup_sata();
#endif

	return 0;
}

static char const *board_type = "uninitialized";
static char const *som_rev = "";

int checkboard(void)
{
	if (hb_cuboxi_ == 0) {
		puts("Board: MX6-CuBox-i\n");
		board_type = "mx6-cubox-i";
	} else if (hb_cuboxi_ == 1) {
		puts("Board: MX6-HummingBoard\n");
		board_type = "mx6-hummingboard";
	} else {
		puts("Board: MX6-HummingBoard2\n");
		board_type = "mx6-hummingboard2";
	}

	if (ver_15_ == 1) {
		som_rev = "-som-v15";
	}
	return 0;
}

#ifdef CONFIG_CMD_BMODE
static const struct boot_mode board_boot_modes[] = {
	/* 4 bit bus width */
	{"mmc0", MAKE_CFGVAL(0x40, 0x28, 0x00, 0x00)},
	{NULL,	 0},
};
#endif

int board_late_init(void)
{
        int cpurev = get_cpu_rev();
        setenv("cpu",get_imx_type((cpurev & 0xFF000) >> 12));
        setenv("board",board_type);
        setenv("somrev",som_rev);

#ifdef CONFIG_CMD_BMODE
        add_board_boot_modes(board_boot_modes);
#endif

	return 0;
}
