/*
 * Author: Tungyi Lin <tungyilin1127@gmail.com>
 *
 * Derived from EDM_CF_IMX6 code by TechNexion,Inc
 * Ported to SolidRun microSOM by Rabeeh Khoury <rabeeh@solid-run.com>
 *
 * SPDX-License-Identifier:     GPL-2.0+
 */
#include <common.h>
#include <asm/io.h>
#include <asm/arch/sys_proto.h>
#ifdef CONFIG_SPL
#include <spl.h>
#endif

#define CONFIG_SPL_STACK	0x0091FFB8

DECLARE_GLOBAL_DATA_PTR;

#if defined(CONFIG_SPL_BUILD)

static enum boot_device boot_dev;
enum boot_device get_boot_device(void);

static inline void setup_boot_device(void)
{
	uint soc_sbmr = readl(SRC_BASE_ADDR + 0x4);
	uint bt_mem_ctl = (soc_sbmr & 0x000000FF) >> 4 ;
	uint bt_mem_type = (soc_sbmr & 0x00000008) >> 3;
	uint bt_mem_mmc = (soc_sbmr & 0x00001000) >> 12;

	switch (bt_mem_ctl) {
	case 0x0:
		if (bt_mem_type)
			boot_dev = MX6_ONE_NAND_BOOT;
		else
			boot_dev = MX6_WEIM_NOR_BOOT;
		break;
	case 0x2:
			boot_dev = MX6_SATA_BOOT;
		break;
	case 0x3:
		if (bt_mem_type)
			boot_dev = MX6_I2C_BOOT;
		else
			boot_dev = MX6_SPI_NOR_BOOT;
		break;
	case 0x4:
	case 0x5:
		if (bt_mem_mmc)
			boot_dev = MX6_SD0_BOOT;
		else
			boot_dev = MX6_SD1_BOOT;
		break;
	case 0x6:
	case 0x7:
		boot_dev = MX6_MMC_BOOT;
		break;
	case 0x8 ... 0xf:
		boot_dev = MX6_NAND_BOOT;
		break;
	default:
		boot_dev = MX6_SD1_BOOT;
		break;
	}
}

enum boot_device get_boot_device(void) {
	return boot_dev;
}

#include "asm/arch/mx6_ddr_regs.h"

static void spl_dram_init_mx6solo_512mb(void);
static void spl_dram_init_mx6dl_1g(void);
static void spl_dram_init_mx6dq_1g(void);
static void spl_dram_init_mx6dq_2g(void);
static void spl_dram_init(void);

static void spl_mx6q_dram_setup_iomux(void)
{
	volatile struct mx6qd_iomux_ddr_regs *mx6q_ddr_iomux;
	volatile struct mx6qd_iomux_grp_regs *mx6q_grp_iomux;

	mx6q_ddr_iomux = (struct mx6dq_iomux_ddr_regs *) MX6DQ_IOM_DDR_BASE;
	mx6q_grp_iomux = (struct mx6dq_iomux_grp_regs *) MX6DQ_IOM_GRP_BASE;

	mx6q_grp_iomux->grp_ddr_type = (u32)0x000c0000;
	mx6q_grp_iomux->grp_ddrpke = (u32)0x00000000;
	mx6q_ddr_iomux->dram_sdclk_0 = (u32)0x00020030;
	mx6q_ddr_iomux->dram_sdclk_1 = (u32)0x00020030;
	mx6q_ddr_iomux->dram_cas = (u32)0x00020030;
	mx6q_ddr_iomux->dram_ras = (u32)0x00020030;
	mx6q_grp_iomux->grp_addds = (u32)0x00000030;
	mx6q_ddr_iomux->dram_reset = (u32)0x00020030;
	mx6q_ddr_iomux->dram_sdcke0 = (u32)0x00003000;
	mx6q_ddr_iomux->dram_sdcke1 = (u32)0x00003000;
	mx6q_ddr_iomux->dram_sdba2 = (u32)0x00000000;
	mx6q_ddr_iomux->dram_sdodt0 = (u32)0x00003030;
	mx6q_ddr_iomux->dram_sdodt1 = (u32)0x00003030;
	mx6q_grp_iomux->grp_ctlds = (u32)0x00000030;
	mx6q_grp_iomux->grp_ddrmode_ctl = (u32)0x00020000;
	mx6q_ddr_iomux->dram_sdqs0 = (u32)0x00000030;
	mx6q_ddr_iomux->dram_sdqs1 = (u32)0x00000030;
	mx6q_ddr_iomux->dram_sdqs2 = (u32)0x00000030;
	mx6q_ddr_iomux->dram_sdqs3 = (u32)0x00000030;
	mx6q_ddr_iomux->dram_sdqs4 = (u32)0x00000030;
	mx6q_ddr_iomux->dram_sdqs5 = (u32)0x00000030;
	mx6q_ddr_iomux->dram_sdqs6 = (u32)0x00000030;
	mx6q_ddr_iomux->dram_sdqs7 = (u32)0x00000030;
	mx6q_grp_iomux->grp_ddrmode = (u32)0x00020000;
	mx6q_grp_iomux->grp_b0ds = (u32)0x00000030;
	mx6q_grp_iomux->grp_b1ds = (u32)0x00000030;
	mx6q_grp_iomux->grp_b2ds = (u32)0x00000030;
	mx6q_grp_iomux->grp_b3ds = (u32)0x00000030;
	mx6q_grp_iomux->grp_b4ds = (u32)0x00000030;
	mx6q_grp_iomux->grp_b5ds = (u32)0x00000030;
	mx6q_grp_iomux->grp_b6ds = (u32)0x00000030;
	mx6q_grp_iomux->grp_b7ds = (u32)0x00000030;
	mx6q_ddr_iomux->dram_dqm0 = (u32)0x00020030;
	mx6q_ddr_iomux->dram_dqm1 = (u32)0x00020030;
	mx6q_ddr_iomux->dram_dqm2 = (u32)0x00020030;
	mx6q_ddr_iomux->dram_dqm3 = (u32)0x00020030;
	mx6q_ddr_iomux->dram_dqm4 = (u32)0x00020030;
	mx6q_ddr_iomux->dram_dqm5 = (u32)0x00020030;
	mx6q_ddr_iomux->dram_dqm6 = (u32)0x00020030;
	mx6q_ddr_iomux->dram_dqm7 = (u32)0x00020030;
}

static void spl_mx6dl_dram_setup_iomux(void)
{
	volatile struct mx6sdl_iomux_ddr_regs *mx6dl_ddr_iomux;
	volatile struct mx6sdl_iomux_grp_regs *mx6dl_grp_iomux;

	mx6dl_ddr_iomux = (struct mx6sdl_iomux_ddr_regs *) MX6SDL_IOM_DDR_BASE;
	mx6dl_grp_iomux = (struct mx6sdl_iomux_grp_regs *) MX6SDL_IOM_GRP_BASE;

	mx6dl_grp_iomux->grp_ddr_type = (u32)0x000c0000;
	mx6dl_grp_iomux->grp_ddrpke = (u32)0x00000000;
	mx6dl_ddr_iomux->dram_sdclk_0 = (u32)0x00000028;
	mx6dl_ddr_iomux->dram_sdclk_1 = (u32)0x00000028;
	mx6dl_ddr_iomux->dram_cas = (u32)0x00000028;
	mx6dl_ddr_iomux->dram_ras = (u32)0x00000028;
	mx6dl_grp_iomux->grp_addds = (u32)0x00000028;
	mx6dl_ddr_iomux->dram_reset = (u32)0x000c0028;
	mx6dl_ddr_iomux->dram_sdcke0 = (u32)0x00003000;
	mx6dl_ddr_iomux->dram_sdcke1 = (u32)0x00003000;
	mx6dl_ddr_iomux->dram_sdba2 = (u32)0x00000000;
	mx6dl_ddr_iomux->dram_sdodt0 = (u32)0x00003030;
	mx6dl_ddr_iomux->dram_sdodt1 = (u32)0x00003030;
	mx6dl_grp_iomux->grp_ctlds = (u32)0x00000028;
	mx6dl_grp_iomux->grp_ddrmode_ctl = (u32)0x00020000;
	mx6dl_ddr_iomux->dram_sdqs0 = (u32)0x00000028;
	mx6dl_ddr_iomux->dram_sdqs1 = (u32)0x00000028;
	mx6dl_ddr_iomux->dram_sdqs2 = (u32)0x00000028;
	mx6dl_ddr_iomux->dram_sdqs3 = (u32)0x00000028;
	mx6dl_ddr_iomux->dram_sdqs4 = (u32)0x00000028;
	mx6dl_ddr_iomux->dram_sdqs5 = (u32)0x00000028;
	mx6dl_ddr_iomux->dram_sdqs6 = (u32)0x00000028;
	mx6dl_ddr_iomux->dram_sdqs7 = (u32)0x00000028;
	mx6dl_grp_iomux->grp_ddrmode = (u32)0x00020000;
	mx6dl_grp_iomux->grp_b0ds = (u32)0x00000028;
	mx6dl_grp_iomux->grp_b1ds = (u32)0x00000028;
	mx6dl_grp_iomux->grp_b2ds = (u32)0x00000028;
	mx6dl_grp_iomux->grp_b3ds = (u32)0x00000028;
	mx6dl_grp_iomux->grp_b4ds = (u32)0x00000028;
	mx6dl_grp_iomux->grp_b5ds = (u32)0x00000028;
	mx6dl_grp_iomux->grp_b6ds = (u32)0x00000028;
	mx6dl_grp_iomux->grp_b7ds = (u32)0x00000028;
	mx6dl_ddr_iomux->dram_dqm0 = (u32)0x00000028;
	mx6dl_ddr_iomux->dram_dqm1 = (u32)0x00000028;
	mx6dl_ddr_iomux->dram_dqm2 = (u32)0x00000028;
	mx6dl_ddr_iomux->dram_dqm3 = (u32)0x00000028;
	mx6dl_ddr_iomux->dram_dqm4 = (u32)0x00000028;
	mx6dl_ddr_iomux->dram_dqm5 = (u32)0x00000028;
	mx6dl_ddr_iomux->dram_dqm6 = (u32)0x00000028;
	mx6dl_ddr_iomux->dram_dqm7 = (u32)0x00000028;
}

/* microSOM with Solo processor 512MByte memory */
static void spl_dram_init_mx6solo_512mb(void)
{
	volatile struct mmdc_p_regs *mmdc_p0;
	volatile struct mmdc_p_regs *mmdc_p1;
	mmdc_p0 = (struct mmdc_p_regs *) MMDC_P0_BASE_ADDR;
	mmdc_p1 = (struct mmdc_p_regs *) MMDC_P1_BASE_ADDR;

	/* ZQ */
	mmdc_p0->mpzqhwctrl = (u32)0xa1390003;
	mmdc_p1->mpzqhwctrl = (u32)0xa1390003;
	/* Write leveling */
	mmdc_p0->mpwldectrl0 = (u32)0x0045004D;
	mmdc_p0->mpwldectrl1 = (u32)0x003A0047;

	mmdc_p0->mpdgctrl0 = (u32)0x023C0224;
	mmdc_p0->mpdgctrl1 = (u32)0x02000220;
	mmdc_p0->mprddlctl = (u32)0x44444846;
	mmdc_p0->mpwrdlctl = (u32)0x32343032;
	/* Read data bit delay */
	mmdc_p0->mprddqby0dl = (u32)0x33333333;
	mmdc_p0->mprddqby1dl = (u32)0x33333333;
	mmdc_p0->mprddqby2dl = (u32)0x33333333;
	mmdc_p0->mprddqby3dl = (u32)0x33333333;
	mmdc_p1->mprddqby0dl = (u32)0x33333333;
	mmdc_p1->mprddqby1dl = (u32)0x33333333;
	mmdc_p1->mprddqby2dl = (u32)0x33333333;
	mmdc_p1->mprddqby3dl = (u32)0x33333333;
	/* Complete calibration by forced measurement */
	mmdc_p0->mpmur0 = (u32)0x00000800;
	mmdc_p0->mdpdc = (u32)0x0002002d;
	mmdc_p0->mdotc = (u32)0x00333040;
	mmdc_p0->mdcfg0 = (u32)0x3f435313;//0x3F4352F3;//0x3f435313
	mmdc_p0->mdcfg1 = (u32)0xb66e8b63;//0xB66D8B63;//0xb66e8b63
	mmdc_p0->mdcfg2 = (u32)0x01ff00db;
	mmdc_p0->mdmisc = (u32)0x00011740;
	mmdc_p0->mdscr = (u32)0x00008000;
	mmdc_p0->mdrwd = (u32)0x000026d2;
	mmdc_p0->mdor = (u32)0x00431023;
	mmdc_p0->mdasp = (u32)0x00000017;//0x00000017
	mmdc_p0->mdctl = (u32)0x83190000;//0x83190000 <-- this should be it
	mmdc_p0->mdscr = (u32)0x02008032;
	mmdc_p0->mdscr = (u32)0x00008033;
	mmdc_p0->mdscr = (u32)0x04008031;
	mmdc_p0->mdscr = (u32)0x05208030;
	mmdc_p0->mdscr = (u32)0x04008040;
	mmdc_p0->mdref = (u32)0x00007800;
	mmdc_p0->mpodtctrl = (u32)0x00011117;
	mmdc_p1->mpodtctrl = (u32)0x00011117;
	mmdc_p0->mdpdc = (u32)0x0002556d;
	mmdc_p1->mdpdc = (u32)0x00011006;
	mmdc_p0->mdscr = (u32)0x00000000;
}

/* microSOM with Dual lite processor 1GByte memory */
static void spl_dram_init_mx6dl_1g(void)
{
	volatile struct mmdc_p_regs *mmdc_p0;
	volatile struct mmdc_p_regs *mmdc_p1;
	mmdc_p0 = (struct mmdc_p_regs *) MMDC_P0_BASE_ADDR;
	mmdc_p1 = (struct mmdc_p_regs *) MMDC_P1_BASE_ADDR;

	/* Calibrations */
	/* ZQ */
	mmdc_p0->mpzqhwctrl = (u32)0xa1390003;
	mmdc_p1->mpzqhwctrl = (u32)0xa1390003;
	/* write leveling */
	mmdc_p0->mpwldectrl0 = (u32)0x0045004D;
	mmdc_p0->mpwldectrl1 = (u32)0x003A0047;
	mmdc_p1->mpwldectrl0 = (u32)0x001F001F;
	mmdc_p1->mpwldectrl1 = (u32)0x00210035;
	/* DQS gating, read delay, write delay calibration values
	based on calibration compare of 0x00ffff00 */
	mmdc_p0->mpdgctrl0 = (u32)0x023C0224;
	mmdc_p0->mpdgctrl1 = (u32)0x02000220;
	mmdc_p1->mpdgctrl0 = (u32)0x02200220;
	mmdc_p1->mpdgctrl1 = (u32)0x02040208;
	mmdc_p0->mprddlctl = (u32)0x44444846;
	mmdc_p1->mprddlctl = (u32)0x4042463C;
	mmdc_p0->mpwrdlctl = (u32)0x32343032;
	mmdc_p1->mpwrdlctl = (u32)0x36363430;
	/* read data bit delay */
	mmdc_p0->mprddqby0dl = (u32)0x33333333;
	mmdc_p0->mprddqby1dl = (u32)0x33333333;
	mmdc_p0->mprddqby2dl = (u32)0x33333333;
	mmdc_p0->mprddqby3dl = (u32)0x33333333;
	mmdc_p1->mprddqby0dl = (u32)0x33333333;
	mmdc_p1->mprddqby1dl = (u32)0x33333333;
	mmdc_p1->mprddqby2dl = (u32)0x33333333;
	mmdc_p1->mprddqby3dl = (u32)0x33333333;
	/* Complete calibration by forced measurment */
	mmdc_p0->mpmur0 = (u32)0x00000800;
	mmdc_p1->mpmur0 = (u32)0x00000800;
	/* MMDC init:
	 in DDR3, 64-bit mode, only MMDC0 is initiated: */
	mmdc_p0->mdpdc = (u32)0x0002002d;
	mmdc_p0->mdotc = (u32)0x00333040;

	mmdc_p0->mdcfg0 = (u32)0x3F4352F3;
	mmdc_p0->mdcfg1 = (u32)0xB66D8B63;

	mmdc_p0->mdcfg2 = (u32)0x01ff00db;
	mmdc_p0->mdmisc = (u32)0x00011740;
	mmdc_p0->mdscr = (u32)0x00008000;
	mmdc_p0->mdrwd = (u32)0x000026d2;
	mmdc_p0->mdor = (u32)0x00431023;
	mmdc_p0->mdasp = (u32)0x00000027;
	mmdc_p0->mdctl = (u32)0x831A0000;

	/* Initialize 2GB DDR3 - Micron MT41J128M */
	/* MR2 */
	mmdc_p0->mdscr = (u32)0x02008032;
	mmdc_p0->mdscr = (u32)0x0200803a;
	/* MR3 */
	mmdc_p0->mdscr = (u32)0x00008033;
	mmdc_p0->mdscr = (u32)0x0000803b;
	/* MR1 */
	mmdc_p0->mdscr = (u32)0x04008031;
	mmdc_p0->mdscr = (u32)0x04008039;
	/* MR0 */
	mmdc_p0->mdscr = (u32)0x05208030;
	mmdc_p0->mdscr = (u32)0x05208038;
	/* ZQ calibration */
	mmdc_p0->mdscr = (u32)0x04008040;
	mmdc_p0->mdscr = (u32)0x04008040;
	/* final DDR setup */
	mmdc_p0->mdref = (u32)0x00007800;
	mmdc_p0->mpodtctrl = (u32)0x00000007;
	mmdc_p1->mpodtctrl = (u32)0x00000007;
	mmdc_p0->mdpdc = (u32)0x0002556d;
	mmdc_p1->mapsr = (u32)0x00011006;
	mmdc_p0->mdscr = (u32)0x00000000;
}

/* microSOM with Dual processor 1GByte memory */
static void spl_dram_init_mx6dq_1g(void)
{
	volatile struct mmdc_p_regs *mmdc_p0;
	volatile struct mmdc_p_regs *mmdc_p1;
	mmdc_p0 = (struct mmdc_p_regs *) MMDC_P0_BASE_ADDR;
	mmdc_p1 = (struct mmdc_p_regs *) MMDC_P1_BASE_ADDR;

	/* Calibrations */
	/* ZQ */
	mmdc_p0->mpzqhwctrl = (u32)0xa1390003;
	/* write leveling */
	mmdc_p0->mpwldectrl0 = (u32)0x00000000;
	mmdc_p0->mpwldectrl1 = (u32)0x00000000;
	mmdc_p1->mpwldectrl0 = (u32)0x00000000;
	mmdc_p1->mpwldectrl1 = (u32)0x00000000;
	/* DQS gating, read delay, write delay calibration values
	 based on calibration compare of 0x00ffff00  */
	mmdc_p0->mpdgctrl0 = (u32)0x0314031c;
	mmdc_p0->mpdgctrl1 = (u32)0x023e0304;
	mmdc_p1->mpdgctrl0 = (u32)0x03240330;
	mmdc_p1->mpdgctrl1 = (u32)0x03180260;
	
	mmdc_p0->mprddlctl = (u32)0x3630323c;
	mmdc_p1->mprddlctl = (u32)0x3436283a;

	mmdc_p0->mpwrdlctl = (u32)0x36344038;
	mmdc_p1->mpwrdlctl = (u32)0x422a423c;

	mmdc_p0->mprddqby0dl = (u32)0x33333333;
	mmdc_p0->mprddqby1dl = (u32)0x33333333;
	mmdc_p0->mprddqby2dl = (u32)0x33333333;
	mmdc_p0->mprddqby3dl = (u32)0x33333333;
	mmdc_p1->mprddqby0dl = (u32)0x33333333;
	mmdc_p1->mprddqby1dl = (u32)0x33333333;
	mmdc_p1->mprddqby2dl = (u32)0x33333333;
	mmdc_p1->mprddqby3dl = (u32)0x33333333;

	mmdc_p0->mpmur0 = (u32)0x00000800;
	mmdc_p1->mpmur0 = (u32)0x00000800;
	/* MMDC init:
	  in DDR3, 64-bit mode, only MMDC0 is initiated: */
	mmdc_p0->mdpdc = (u32)0x00025576;
	mmdc_p0->mdotc = (u32)0x09444040;
	mmdc_p0->mdcfg0 = (u32)0x54597955;
	mmdc_p0->mdcfg1 = (u32)0xFF328F64;
	mmdc_p0->mdcfg2 = (u32)0x01ff00db;
	mmdc_p0->mdmisc = (u32)0x00011740;

	mmdc_p0->mdscr = (u32)0x00008000;
	mmdc_p0->mdrwd = (u32)0x000026d2;
	mmdc_p0->mdor = (u32)0x005B0E21;
	
	/* 1G */
	mmdc_p0->mdasp = (u32)0x00000027;
	mmdc_p0->mdctl = (u32)0x831a0000;

	mmdc_p0->mdscr = (u32)0x02088032;
	mmdc_p0->mdscr = (u32)0x00008033;
	mmdc_p0->mdscr = (u32)0x00408031;
	mmdc_p0->mdscr = (u32)0x09408030;
	mmdc_p0->mdscr = (u32)0x04008040;

	mmdc_p0->mdref = (u32)0x00005800;
	
	mmdc_p0->mpodtctrl = (u32)0x00000007;
	mmdc_p1->mpodtctrl = (u32)0x00000007;

	mmdc_p0->mdpdc = (u32)0x00025576;
	mmdc_p0->mapsr = (u32)0x00011006;
	mmdc_p0->mdscr = (u32)0x00000000;
}

/* microSOM with Quad processor 2GByte memory */
static void spl_dram_init_mx6dq_2g(void)
{
	volatile struct mmdc_p_regs *mmdc_p0;
	volatile struct mmdc_p_regs *mmdc_p1;
	mmdc_p0 = (struct mmdc_p_regs *) MMDC_P0_BASE_ADDR;
	mmdc_p1 = (struct mmdc_p_regs *) MMDC_P1_BASE_ADDR;

	/* Calibrations */
	/* ZQ */
	mmdc_p0->mpzqhwctrl = (u32)0xa1390003;
	/* write leveling */
	mmdc_p0->mpwldectrl0 = (u32)0x00000000;
	mmdc_p0->mpwldectrl1 = (u32)0x00000000;
	mmdc_p1->mpwldectrl0 = (u32)0x00000000;
	mmdc_p1->mpwldectrl1 = (u32)0x00000000;
	/* DQS gating, read delay, write delay calibration values
	 based on calibration compare of 0x00ffff00  */
	mmdc_p0->mpdgctrl0 = (u32)0x0314031c;
	mmdc_p0->mpdgctrl1 = (u32)0x023e0304;
	mmdc_p1->mpdgctrl0 = (u32)0x03240330;
	mmdc_p1->mpdgctrl1 = (u32)0x03180260;
	
	mmdc_p0->mprddlctl = (u32)0x3630323c;
	mmdc_p1->mprddlctl = (u32)0x3436283a;

	mmdc_p0->mpwrdlctl = (u32)0x36344038;
	mmdc_p1->mpwrdlctl = (u32)0x422a423c;

	mmdc_p0->mprddqby0dl = (u32)0x33333333;
	mmdc_p0->mprddqby1dl = (u32)0x33333333;
	mmdc_p0->mprddqby2dl = (u32)0x33333333;
	mmdc_p0->mprddqby3dl = (u32)0x33333333;
	mmdc_p1->mprddqby0dl = (u32)0x33333333;
	mmdc_p1->mprddqby1dl = (u32)0x33333333;
	mmdc_p1->mprddqby2dl = (u32)0x33333333;
	mmdc_p1->mprddqby3dl = (u32)0x33333333;

	mmdc_p0->mpmur0 = (u32)0x00000800;
	mmdc_p1->mpmur0 = (u32)0x00000800;
	/* MMDC init:
	  in DDR3, 64-bit mode, only MMDC0 is initiated: */
	mmdc_p0->mdpdc = (u32)0x00025576;
	mmdc_p0->mdotc = (u32)0x09444040;
	mmdc_p0->mdcfg0 = (u32)0x898E7975;
	mmdc_p0->mdcfg1 = (u32)0xFF328F64;
	mmdc_p0->mdcfg2 = (u32)0x01ff00db;
	mmdc_p0->mdmisc = (u32)0x00011740;

	mmdc_p0->mdscr = (u32)0x00008000;
	mmdc_p0->mdrwd = (u32)0x000026d2;
	mmdc_p0->mdor = (u32)0x005B0E21;
	
	/* 2G */ 
	mmdc_p0->mdasp = (u32)0x00000047;
	mmdc_p0->mdctl = (u32)0x841a0000;
	
	mmdc_p0->mdscr = (u32)0x02088032;
	mmdc_p0->mdscr = (u32)0x00008033;
	mmdc_p0->mdscr = (u32)0x00408031;
	mmdc_p0->mdscr = (u32)0x09408030;
	mmdc_p0->mdscr = (u32)0x04008040;

	mmdc_p0->mdref = (u32)0x00005800;
	
	mmdc_p0->mpodtctrl = (u32)0x00000007;
	mmdc_p1->mpodtctrl = (u32)0x00000007;

	mmdc_p0->mdpdc = (u32)0x00025576;
	mmdc_p0->mapsr = (u32)0x00011006;
	mmdc_p0->mdscr = (u32)0x00000000;
}

static void spl_dram_init(void)
{	
	u32 cpurev, imxtype;
	
	cpurev = get_cpu_rev();
	imxtype = (cpurev & 0xFF000) >> 12;

	get_imx_type(imxtype);	

	switch (imxtype){
	case MXC_CPU_MX6SOLO:
		spl_mx6dl_dram_setup_iomux();
		spl_dram_init_mx6solo_512mb();
		break;
	case MXC_CPU_MX6Q:
	{
		/* Read first the snoop control unit config register */
		u32 scu_config = *(u32 *)(SCU_BASE_ADDR + 0x4);
		spl_mx6q_dram_setup_iomux();
		if ((scu_config & 0x3) == 0x3) /* Quad core */
			spl_dram_init_mx6dq_2g();
		else /* Dual core */
			spl_dram_init_mx6dq_1g();
		break;
	}
	case MXC_CPU_MX6DL:
	default:
		spl_mx6dl_dram_setup_iomux();
		spl_dram_init_mx6dl_1g();
		break;	
	}
}

void board_init_f(ulong dummy)
{	
	/* Set the stack pointer. */
	asm volatile("mov sp, %0\n" : : "r"(CONFIG_SPL_STACK));

	spl_dram_init();	
	
	arch_cpu_init();

	/* Clear the BSS. */
	memset(__bss_start, 0, __bss_end - __bss_start);

	/* Set global data pointer. */
	gd = &gdata;

	board_early_init_f();	

	timer_init();
	preloader_console_init();

	board_init_r(NULL, 0);
}

void spl_board_init(void)
{
	setup_boot_device();
}

#ifdef CONFIG_SPL_OS_BOOT
int spl_start_uboot(void)
{
    return 0;
}
#endif

u32 spl_boot_device(void)
{
	puts("Boot Device: ");
	switch (get_boot_device()) {
	case MX6_SD0_BOOT:
		printf("SD0\n");
		return BOOT_DEVICE_MMC1;
	case MX6_SD1_BOOT:
		printf("SD1\n");
		return BOOT_DEVICE_MMC2;
	case MX6_MMC_BOOT:
		printf("MMC\n");
		return BOOT_DEVICE_MMC2;
	case MX6_NAND_BOOT:
		printf("NAND\n");
		return BOOT_DEVICE_NAND;
	case MX6_SATA_BOOT:
		printf("SATA\n");
		return BOOT_DEVICE_SATA;
	case MX6_UNKNOWN_BOOT:
	default:
		printf("UNKNOWN..\n");
		return BOOT_DEVICE_MMC1;
	}
}

u32 spl_boot_mode(void)
{
	switch (spl_boot_device()) {
	case BOOT_DEVICE_MMC1:
	case BOOT_DEVICE_MMC2:
	case BOOT_DEVICE_MMC2_2:
		return MMCSD_MODE;
		break;
	case BOOT_DEVICE_SATA:
		return SATA_MODE;
		break;
	//case BOOT_DEVICE_NAND:
	//	return 0;
	//	break;
	default:
		puts("spl: ERROR:  unsupported device\n");
		hang();
	}
}

void reset_cpu(ulong addr)
{
	__REG16(WDOG1_BASE_ADDR) = 4;
}
#endif

