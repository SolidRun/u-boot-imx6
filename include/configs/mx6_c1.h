/*
 * Copyright (C) 2013 Freescale Semiconductor, Inc.
 * Copyright (C) 2013 SolidRun ltd.
 * Copyright (C) 2013 Jon Nettleton <jon.nettleton@gmail.com>
 *
 * Configuration settings for the SolidRun carrier-1 (c1) board.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __CONFIG_H
#define __CONFIG_H

#include "mx6_common.h"
#include <asm/arch/imx-regs.h>
#include <asm/imx-common/gpio.h>
#include <asm/sizes.h>

#undef DEBUG

#define CONFIG_MX6
#define CONFIG_DISPLAY_CPUINFO
#define CONFIG_DISPLAY_BOARDINFO

#define CONFIG_MACH_TYPE		4773

#define CONFIG_CMDLINE_TAG
#define CONFIG_SETUP_MEMORY_TAGS
#define CONFIG_INITRD_TAG
#define CONFIG_REVISION_TAG

/* Size of malloc() pool */
#define CONFIG_SYS_MALLOC_LEN		(8 * SZ_1M)

#define CONFIG_BOARD_EARLY_INIT_F
#define CONFIG_BOARD_LATE_INIT
#define CONFIG_MXC_GPIO

#define CONFIG_CMD_FUSE
#ifdef CONFIG_CMD_FUSE
#define CONFIG_MXC_OCOTP
#endif

#define CONFIG_MXC_UART
#define CONFIG_MXC_UART_BASE		UART1_BASE

/* allow to overwrite serial and ethaddr */
#define CONFIG_ENV_OVERWRITE
#define CONFIG_CONS_INDEX		1
#define CONFIG_BAUDRATE			115200

/* Command definition */
#include <config_cmd_default.h>

#undef CONFIG_CMD_IMLS
#undef CONFIG_CMD_I2C

#define CONFIG_CMD_BMODE
#define CONFIG_CMD_SETEXPR
#define CONFIG_CMD_MEMTEST
#define CONFIG_BOOTDELAY		3

#define CONFIG_SYS_MEMTEST_START	0x10000000
#define CONFIG_SYS_MEMTEST_END		(CONFIG_SYS_MEMTEST_START + 500 * SZ_1M)

#define CONFIG_LOADADDR			0x10800000
#define CONFIG_SYS_TEXT_BASE		0x27800000

/* MMC Configuration */
#define CONFIG_FSL_ESDHC
#define CONFIG_FSL_USDHC
#define CONFIG_SYS_FSL_USDHC_NUM	1
#define CONFIG_SYS_FSL_ESDHC_ADDR	0

#define CONFIG_MMC
#define CONFIG_CMD_MMC
#define CONFIG_GENERIC_MMC
#define CONFIG_BOUNCE_BUFFER
#define CONFIG_CMD_EXT2
#define CONFIG_CMD_FAT
#define CONFIG_DOS_PARTITION

/* Ethernet Configuration */
#define CONFIG_FEC_MXC
#ifdef CONFIG_FEC_MXC
#define CONFIG_CMD_PING
#define CONFIG_CMD_DHCP
#define CONFIG_CMD_MII
#define CONFIG_CMD_NET
#define CONFIG_FEC_MXC
#define CONFIG_MII
#define IMX_FEC_BASE			ENET_BASE_ADDR
#define CONFIG_FEC_XCV_TYPE		RGMII
#define CONFIG_ETHPRIME			"FEC"
#define CONFIG_FEC_MXC_PHYADDR		0	
#define CONFIG_PHYLIB
#define CONFIG_PHY_ATHEROS
#endif

/* Framebuffer */
#define CONFIG_VIDEO
#ifdef CONFIG_VIDEO
#define CONFIG_VIDEO_IPUV3
#define CONFIG_IPUV3_CLK 260000000
#define CONFIG_CFB_CONSOLE
#define CONFIG_CFB_CONSOLE_ANSI
#define CONFIG_VGA_AS_SINGLE_DEVICE
#define CONFIG_VIDEO_BMP_RLE8
#define CONFIG_IMX_HDMI

#undef CONFIG_SPLASH_SCREEN
#undef CONFIG_SPLASH_SCREEN_ALIGN
#define CONFIG_BMP_16BPP
#undef CONFIG_CMD_BMP
#define CONFIG_VIDEO_LOGO
#define CONFIG_VIDEO_BMP_LOGO
#define CONFIG_CONSOLE_MUX
#endif

/* USB Configs */
#define CONFIG_CMD_USB
#ifdef CONFIG_CMD_USB
#define CONFIG_CMD_FAT
#define CONFIG_USB_EHCI
#define CONFIG_USB_EHCI_MX6
#define CONFIG_EHCI_IS_TDI
#define CONFIG_USB_STORAGE
#define CONFIG_USB_HOST_ETHER
#define CONFIG_USB_ETHER_ASIX
#define CONFIG_USB_ETHER_SMSC95XX
#define CONFIG_MXC_USB_PORT     1
#define CONFIG_MXC_USB_PORTSC   (PORT_PTS_UTMI | PORT_PTS_PTW)
#define CONFIG_MXC_USB_FLAGS    0
#define CONFIG_USB_KEYBOARD
#define CONFIG_SYS_USB_EVENT_POLL
#define CONFIG_PREBOOT "usb start"
#endif

#define CONFIG_SYS_CONSOLE_IS_IN_ENV

#define CONFIG_EXTRA_ENV_SETTINGS \
	"script=boot.scr\0" \
	"bootfile=uImage\0" \
	"bootenv=uEnv.txt\0" \
	"console=ttymxc0\0" \
	"stdin=serial,usbkbd\0" \
	"stdout=serial,vga\0" \
	"stderr=serial,vga\0" \
	"splashpos=m,m\0" \
	"fdt_high=0xffffffff\0" \
	"initrd_high=0xffffffff\0" \
	"fdt_addr=0x11000000\0" \
	"boot_fdt=no\0" \
        "ip_dyn=yes\0" \
        "mmcdev=" __stringify(CONFIG_SYS_MMC_ENV_DEV) "\0" \
        "mmcpart=1\0" \
        "mmcroot=/dev/mmcblk0p2 rootwait rw\0" \
        "update_sd_firmware_filename=u-boot.imx\0" \
        "update_sd_firmware=" \
                "if test ${ip_dyn} = yes; then " \
                        "setenv get_cmd dhcp; " \
                "else " \
                        "setenv get_cmd tftp; " \
                "fi; " \
                "if mmc dev ${mmcdev}; then "   \
                        "if ${get_cmd} ${update_sd_firmware_filename}; then " \
                                "setexpr fw_sz ${filesize} / 0x200; " \
                                "setexpr fw_sz ${fw_sz} + 1; "  \
                                "mmc write ${loadaddr} 0x2 ${fw_sz}; " \
                        "fi; "  \
                "fi\0" \
        "mmcargs=setenv bootargs console=${console},${baudrate} " \
                "root=${mmcroot}\0" \
        "fatloadbootscript=" \
                "fatload mmc ${mmcdev}:${mmcpart} ${loadaddr} ${script};\0" \
        "ext2loadbootscript=" \
                "ext2load mmc ${mmcdev}:${mmcpart} ${loadaddr} ${script};\0" \
        "bootscript=echo Running bootscript from mmc ...; " \
                "source\0" \
        "fatloadbootenv=fatload mmc ${mmcdev}:${mmcpart} ${loadaddr} ${bootenv}\0" \
        "fatloadbootfile=fatload mmc ${mmcdev}:${mmcpart} ${loadaddr} ${bootfile}\0" \
        "fatloadfdt=fatload mmc ${mmcdev}:${mmcpart} ${fdt_addr} ${fdt_file}\0" \
        "ext2loadbootenv=ext2load mmc ${mmcdev}:${mmcpart} ${loadaddr} ${bootenv}\0" \
        "ext2loadbootfile=ext2load mmc ${mmcdev}:${mmcpart} ${loadaddr} ${bootfile}\0" \
        "ext2loadfdt=ext2load mmc ${mmcdev}:${mmcpart} ${fdt_addr} ${fdt_file}\0" \
        "importbootenv=echo Importing environment from mmc${mmcdev} ...; " \
                "env import -t ${loadaddr} ${filesize}\0" \
        "fatmmcboot=echo Booting from mmc ...; " \
                "run mmcargs; " \
                "if test ${boot_fdt} = yes || test ${boot_fdt} = try; then " \
                        "if run fatloadfdt; then " \
                                "if test ${bootfile} = zImage; then " \
                                	"bootz ${loadaddr} - ${fdt_addr}; " \
                                "else " \
                                	"bootm ${loadaddr} - ${fdt_addr}; " \
                                "fi; " \
                        "else " \
                                "if test ${boot_fdt} = try; then " \
                                	"if test ${bootfile} = zImage; then " \
                                		"bootz; " \
                                	"else " \
                                		"bootm; " \
                                	"fi; " \
                                "else " \
                                        "echo WARN: Cannot load the DT; " \
                                "fi; " \
                        "fi; " \
                "else " \
                        "if test ${bootfile} = zImage; then " \
                        	"bootz; " \
                        "else " \
                        	"bootm; " \
                        "fi; " \
                "fi;\0" \
        "ext2mmcboot=echo Booting from mmc ...; " \
                "run mmcargs; " \
                "if test ${boot_fdt} = yes || test ${boot_fdt} = try; then " \
                        "if run ext2loadfdt; then " \
                                "if test ${bootfile} = zImage; then " \
                                	"bootz ${loadaddr} - ${fdt_addr}; " \
                                "else " \
                                	"bootm ${loadaddr} - ${fdt_addr}; " \
                                "fi; " \
                        "else " \
                                "if test ${boot_fdt} = try; then " \
                                	"if test ${bootfile} = zImage; then " \
                                		"bootz; " \
                                	"else " \
                                		"bootm; " \
                                	"fi; " \
                                "else " \
                                        "echo WARN: Cannot load the DT; " \
                                "fi; " \
                        "fi; " \
                "else " \
                        "if test ${bootfile} = zImage; then " \
                        	"bootz; " \
                        "else " \
                        	"bootm; " \
                        "fi; " \
                "fi;\0" \
        "netargs=setenv bootargs console=${console},${baudrate} " \
                "root=/dev/nfs " \
        "ip=dhcp nfsroot=${serverip}:${nfsroot},v3,tcp\0" \
        "netboot=echo Booting from net ...; " \
                "if test ${bootfile} = zImage; then " \
                        "setenv realfile zImage; " \
                "else " \
                        "setenv realfile uImage; " \
                "fi; " \
                "run netargs; " \
                "if test ${ip_dyn} = yes; then " \
                        "setenv get_cmd dhcp; " \
                "else " \
                        "setenv get_cmd tftp; " \
                "fi; " \
                "${get_cmd} ${bootfile}; " \
                "if test ${boot_fdt} = yes || test ${boot_fdt} = try; then " \
                        "if ${get_cmd} ${fdt_addr} ${fdt_file}; then " \
                                "if test ${realfile} = zImage; then " \
                                	"bootz ${loadaddr} - ${fdt_addr}; " \
                                "else " \
                                	"bootm ${loadaddr} - ${fdt_addr}; " \
                                "fi; " \
                        "else " \
                                "if test ${boot_fdt} = try; then " \
                                	"if test ${bootfile} = zImage; then " \
                                		"bootz; " \
                                	"else " \
                                		"bootm; " \
                                	"fi; " \
                                "else " \
                                        "echo WARN: Cannot load the DT; " \
                                "fi; " \
                        "fi; " \
                "else " \
                        "if test ${bootfile} = zImage; then " \
                        	"bootz; " \
                        "else " \
                        	"bootm; " \
                        "fi; " \
                "fi;\0"

#define CONFIG_BOOTCOMMAND \
	   "mmc dev ${mmcdev}; if mmc rescan; then " \
		   "if run ext2loadbootscript; then " \
			   "run bootscript; " \
		   "elif run fatloadbootscript; then " \
			   "run bootscript; " \
		   "else " \
			   "if run ext2loadbootenv || run fatloadbootenv; then " \
				   "run importbootenv; " \
			   "fi; " \
			   "if run ext2loadbootfile; then " \
				   "run ext2mmcboot; " \
			   "elif run fatloadbootfile; then " \
				   "run fatmmcboot; " \
			   "else run netboot; " \
			   "fi; " \
		   "fi; " \
	   "else run netboot; fi"

/* Miscellaneous configurable options */
#define CONFIG_SYS_LONGHELP
#define CONFIG_SYS_HUSH_PARSER
#define CONFIG_SYS_PROMPT	       "C1 U-Boot > "
#define CONFIG_AUTO_COMPLETE
#define CONFIG_SYS_CBSIZE		1024

/* Print Buffer Size */
#define CONFIG_SYS_PBSIZE (CONFIG_SYS_CBSIZE + sizeof(CONFIG_SYS_PROMPT) + 16)
#define CONFIG_SYS_MAXARGS	       16
#define CONFIG_SYS_BARGSIZE CONFIG_SYS_CBSIZE

#define CONFIG_SYS_LOAD_ADDR		CONFIG_LOADADDR
#define CONFIG_SYS_HZ			1000

#define CONFIG_CMDLINE_EDITING

/* Physical Memory Map */
#define CONFIG_NR_DRAM_BANKS		1
#define PHYS_SDRAM			MMDC0_ARB_BASE_ADDR

#define CONFIG_SYS_SDRAM_BASE		PHYS_SDRAM
#define CONFIG_SYS_INIT_RAM_ADDR	IRAM_BASE_ADDR
#define CONFIG_SYS_INIT_RAM_SIZE	IRAM_SIZE

#define CONFIG_SYS_INIT_SP_OFFSET \
	(CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_INIT_SP_ADDR \
	(CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)

/* FLASH and environment organization */
#define CONFIG_SYS_NO_FLASH

#define CONFIG_ENV_SIZE			(8 * 1024)

#define CONFIG_ENV_IS_IN_MMC
#define CONFIG_ENV_OFFSET		(6 * 64 * 1024)
#define CONFIG_SYS_MMC_ENV_DEV		0

#define CONFIG_OF_LIBFDT
#define CONFIG_CMD_BOOTZ

#ifndef CONFIG_SYS_DCACHE_OFF
#define CONFIG_CMD_CACHE
#endif

#endif			       /* __CONFIG_H * */
