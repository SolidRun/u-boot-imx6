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

DECLARE_GLOBAL_DATA_PTR;

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

#define ENET_PAD_CTRL_PD  (PAD_CTL_PUS_100K_DOWN |		\
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_HYS)

#define ENET_PAD_CTRL_CLK  (PAD_CTL_PUS_100K_UP & ~PAD_CTL_PKE | \
	PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm | PAD_CTL_SRE_FAST)

int dram_init(void)
{
	gd->ram_size = ((phys_size_t)CONFIG_DDR_MB * 1024 * 1024);

	return 0;
}

iomux_v3_cfg_t const uart1_pads[] = {
	MX6_PAD_CSI0_DAT10__UART1_TXD | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_CSI0_DAT11__UART1_RXD | MUX_PAD_CTRL(UART_PAD_CTRL),
};

static void setup_iomux_uart(void)
{
	imx_iomux_v3_setup_multiple_pads(uart1_pads, ARRAY_SIZE(uart1_pads));
}

iomux_v3_cfg_t const usdhc2_pads[] = {
	MX6_PAD_SD2_CLK__USDHC2_CLK	| MUX_PAD_CTRL(USDHC_PAD_CLK_CTRL),
	MX6_PAD_SD2_CMD__USDHC2_CMD	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DAT0__USDHC2_DAT0	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DAT1__USDHC2_DAT1	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DAT2__USDHC2_DAT2	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DAT3__USDHC2_DAT3	| MUX_PAD_CTRL(USDHC_PAD_CTRL),
        MX6_PAD_GPIO_4__USDHC2_CD       | MUX_PAD_CTRL(USDHC_PAD_GPIO_CTRL),
        MX6_PAD_GPIO_2__USDHC2_WP       | MUX_PAD_CTRL(USDHC_PAD_GPIO_CTRL),
};

#ifdef CONFIG_FSL_ESDHC
struct fsl_esdhc_cfg usdhc_cfg[1] = {
        { USDHC2_BASE_ADDR },
};

int board_mmc_getcd(struct mmc *mmc)
{
        struct fsl_esdhc_cfg *cfg = (struct fsl_esdhc_cfg *)mmc->priv;

        if (cfg->esdhc_base == USDHC2_BASE_ADDR) {
                return !gpio_get_value(IMX_GPIO_NR(1, 4));
        }

        return 0;
}

int board_mmc_init(bd_t *bis)
{
        /*
         * Only one USDHC controller on titianium
         */
        imx_iomux_v3_setup_multiple_pads(usdhc2_pads, ARRAY_SIZE(usdhc2_pads));
        gpio_direction_input(IMX_GPIO_NR(1, 4));
        usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC2_CLK);

        return fsl_esdhc_initialize(bis, &usdhc_cfg[0]);
}
#endif

#ifdef CONFIG_FEC_MXC
iomux_v3_cfg_t const enet_pads[] = {
	MX6_PAD_ENET_MDIO__ENET_MDIO		| MUX_PAD_CTRL(ENET_PAD_CTRL),		
	MX6_PAD_ENET_MDC__ENET_MDC		| MUX_PAD_CTRL(ENET_PAD_CTRL),		
	/* AR8035 reset */
	MX6_PAD_KEY_ROW4__GPIO_4_15		| MUX_PAD_CTRL(ENET_PAD_CTRL_PD),		
	/* AR8035 interrupt */
	MX6_PAD_DI0_PIN2__GPIO_4_18		| MUX_PAD_CTRL(NO_PAD_CTRL),		
	/* GPIO16 -> AR8035 25MHz */
	MX6_PAD_GPIO_16__ENET_ANATOP_ETHERNET_REF_OUT	| MUX_PAD_CTRL(NO_PAD_CTRL),		
	MX6_PAD_RGMII_TX_CTL__RGMII_TX_CTL		| MUX_PAD_CTRL(NO_PAD_CTRL),		
	MX6_PAD_RGMII_TD0__ENET_RGMII_TD0		| MUX_PAD_CTRL(ENET_PAD_CTRL),		
	MX6_PAD_RGMII_TD1__ENET_RGMII_TD1		| MUX_PAD_CTRL(ENET_PAD_CTRL),		
	MX6_PAD_RGMII_TD2__ENET_RGMII_TD2		| MUX_PAD_CTRL(ENET_PAD_CTRL),		
	MX6_PAD_RGMII_TD3__ENET_RGMII_TD3		| MUX_PAD_CTRL(ENET_PAD_CTRL),		
	MX6_PAD_RGMII_TXC__ENET_RGMII_TXC		| MUX_PAD_CTRL(ENET_PAD_CTRL),		
	/* AR8035 CLK_25M --> ENET_REF_CLK (V22) */
	MX6_PAD_ENET_REF_CLK__ENET_TX_CLK		| MUX_PAD_CTRL(ENET_PAD_CTRL_CLK),		
        MX6_PAD_RGMII_RXC__ENET_RGMII_RXC       | MUX_PAD_CTRL(ENET_PAD_CTRL),
        MX6_PAD_RGMII_RD0__ENET_RGMII_RD0       | MUX_PAD_CTRL(ENET_PAD_CTRL_PD),
        MX6_PAD_RGMII_RD1__ENET_RGMII_RD1       | MUX_PAD_CTRL(ENET_PAD_CTRL_PD),
        MX6_PAD_RGMII_RD2__ENET_RGMII_RD2       | MUX_PAD_CTRL(ENET_PAD_CTRL),
        MX6_PAD_RGMII_RD3__ENET_RGMII_RD3       | MUX_PAD_CTRL(ENET_PAD_CTRL),
        MX6_PAD_RGMII_RX_CTL__RGMII_RX_CTL      | MUX_PAD_CTRL(ENET_PAD_CTRL_PD),
};

static void setup_iomux_enet(void)
{
	imx_iomux_v3_setup_multiple_pads(enet_pads, ARRAY_SIZE(enet_pads));

	/* Reset AR8035 PHY */
	gpio_direction_output(IMX_GPIO_NR(4, 15), 0);
	udelay(1000 * 2);    /* Wait 2 ms before reset */
	gpio_set_value(IMX_GPIO_NR(4, 15), 1);

	udelay(100);
}

int board_phy_config(struct phy_device *phydev)
{
	if (phydev->drv->config)
		phydev->drv->config(phydev);

	return 0;
}

int board_eth_init(bd_t *bis)
{
	int ret;
        struct iomuxc_base_regs *const iomuxc_regs
                = (struct iomuxc_base_regs *) IOMUXC_BASE_ADDR;
	struct anatop_regs __iomem *anatop =
                (struct anatop_regs __iomem *)ANATOP_BASE_ADDR;
	u32 reg = 0;
	s32 timeout = 100000;

     	reg = readl(&anatop->pll_enet);
	reg |= 0x00002000;
	writel(reg, &anatop->pll_enet);

	/* clear gpr1[21] */
        clrsetbits_le32(&iomuxc_regs->gpr[1], (1 << 21), 1);

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
        .pixclock       = 15385,
        .left_margin    = 220,
        .right_margin   = 40,
        .upper_margin   = 21,
        .lower_margin   = 7,
        .hsync_len      = 60,
        .vsync_len      = 10,
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

        enable_ipu_clock();
        imx_setup_hdmi();

        reg = readl(&mxc_ccm->chsccdr);
        reg |= (CHSCCDR_CLK_SEL_LDB_DI0
                << MXC_CCM_CHSCCDR_IPU1_DI0_CLK_SEL_OFFSET);
        writel(reg, &mxc_ccm->chsccdr);
}
#endif /* CONFIG_VIDEO_IPUV3 */

#ifdef CONFIG_USB_EHCI_MX6
int board_ehci_hcd_init(int port)
{
        return 0;
}
#endif

int board_early_init_f(void)
{
	setup_iomux_uart();

	return 0;
}

/*
 * Do not overwrite the console
 * Use always serial for U-Boot console
 */
int overwrite_console(void)
{
        return 1;
}

int board_init(void)
{
	/* address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;

#ifdef CONFIG_VIDEO_IPUV3
	setup_display();
#endif
	return 0;
}

static char const *board_type = "uninitialized";

int checkboard(void)
{
	puts("Board: MX6-C1\n");
	board_type = "mx6-c1";
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

#ifdef CONFIG_CMD_BMODE
        add_board_boot_modes(board_boot_modes);
#endif

	return 0;
}
