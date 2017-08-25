/*
 * (C) Copyright 2010
 * Texas Instruments, <www.ti.com>
 *
 * Aneesh V <aneesh@ti.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
#include <common.h>
#include <spl.h>
#include <asm/u-boot.h>
#include <mmc.h>
#include <fat.h>
#include <ext4fs.h>
#include <version.h>
#include <image.h>

DECLARE_GLOBAL_DATA_PTR;

static int mmc_load_image_raw(struct mmc *mmc, unsigned long sector)
{
	unsigned long err;
	u32 image_size_sectors;
	struct image_header *header;

	header = (struct image_header *)(CONFIG_SYS_TEXT_BASE -
						sizeof(struct image_header));

	/* read image header to find the image size & load address */
	err = mmc->block_dev.block_read(0, sector, 1, header);
	if (err == 0)
		goto end;

	if (image_get_magic(header) != IH_MAGIC)
		return -1;

	spl_parse_image_header(header);

	/* convert size to sectors - round up */
	image_size_sectors = (spl_image.size + mmc->read_bl_len - 1) /
				mmc->read_bl_len;

	/* Read the header too to avoid extra memcpy */
	err = mmc->block_dev.block_read(0, sector, image_size_sectors,
					(void *)spl_image.load_addr);

end:
	if (err == 0)
		printf("spl: mmc blk read err - %lu\n", err);

	return (err == 0);
}

#ifdef CONFIG_SPL_OS_BOOT
static int mmc_load_image_raw_os(struct mmc *mmc)
{
	if (!mmc->block_dev.block_read(0,
				       CONFIG_SYS_MMCSD_RAW_MODE_ARGS_SECTOR,
				       CONFIG_SYS_MMCSD_RAW_MODE_ARGS_SECTORS,
				       (void *)spl_image.args_addr)) {
		printf("mmc args blk read error\n");
		return -1;
	}

	return mmc_load_image_raw(mmc, CONFIG_SYS_MMCSD_RAW_MODE_KERNEL_SECTOR);
}
#endif

#ifdef CONFIG_SPL_FAT_SUPPORT
static int mmc_load_image_fat(struct mmc *mmc, const char *filename)
{
	int err;
	struct image_header *header;

	header = (struct image_header *)(CONFIG_SYS_TEXT_BASE -
						sizeof(struct image_header));

	err = file_fat_read(filename, header, sizeof(struct image_header));
	if (err <= 0)
		goto end;

	spl_parse_image_header(header);

	err = file_fat_read(filename, (u8 *)spl_image.load_addr, 0);
end:
	if (err <= 0)
		printf("spl: error reading image %s, err - %d\n",
		       filename, err);

	return (err <= 0);
}
#endif

#ifdef CONFIG_SPL_FAT_SUPPORT
#ifdef CONFIG_SPL_OS_BOOT
static int mmc_load_image_fat_os(struct mmc *mmc)
{
	int err;
	struct image_header *header;

	err = file_fat_read(spl_image.args,
			    (void *)spl_image.args_addr, 0);
	if (err <= 0) {
		printf("spl: error reading image %s, err - %d\n",
		       spl_image.args, err);
		goto end;
	}
	header = (struct image_header *)(CONFIG_SYS_TEXT_BASE -
						sizeof(struct image_header));

	err = file_fat_read(spl_image.args, header, sizeof(struct image_header));
	if (err <= 0)
		goto end;
	spl_parse_image_header(header);

	return mmc_load_image_fat(mmc, spl_image.os_image);

end:
	return err;
}
#endif
#endif

#ifdef CONFIG_SPL_EXT_SUPPORT
static int mmc_load_image_ext(struct mmc *mmc, const char *filename)
{
	int err;
	struct image_header *header;

	header = (struct image_header *)(CONFIG_SYS_TEXT_BASE -
						sizeof(struct image_header));
	
	err = ext4_read_file(filename, header, 0, sizeof(struct image_header));
	if (err <= 0)
		goto end;

	spl_parse_image_header(header);

	err = ext4_read_file(filename, (u8 *)spl_image.load_addr, 0, 0);

end:
	if (err <= 0)
		printf("spl: error reading image %s, err - %d\n",
		       filename, err);
	else
		printf("loading %s from MMC EXT...\n", filename);	

	return (err <= 0);
}

#ifdef CONFIG_SPL_OS_BOOT
static int mmc_load_image_ext_os(struct mmc *mmc)
{
	int err;
	struct image_header *header;

	err = file_fat_read(spl_image.args,
			    (void *)spl_image.args_addr, 0);
	if (err <= 0) {
		printf("spl: error reading image %s, err - %d\n",
		       spl_image.args, err);
		goto end;
	}
	header = (struct image_header *)(CONFIG_SYS_TEXT_BASE -
						sizeof(struct image_header));

	err = file_fat_read(spl_image.args, header, sizeof(struct image_header));
	if (err <= 0)
		goto end;
	spl_parse_image_header(header);

	return mmc_load_image_ext(mmc, spl_image.os_image);

end:
	return err;
}
#endif
#endif 

static void spl_mmc_config(void)
{
	if (!spl_image.os_image)
		spl_image.os_image = CONFIG_SPL_FAT_LOAD_KERNEL_NAME;
	if (!spl_image.second_stage)
		spl_image.second_stage = CONFIG_SPL_FAT_LOAD_PAYLOAD_NAME;
	if (!spl_image.args)
		spl_image.args = CONFIG_SPL_FAT_LOAD_ARGS_NAME;
	if (!spl_image.args_addr)
		spl_image.args_addr = CONFIG_SYS_SPL_ARGS_ADDR;

	debug("Config to be loaded:\nOS: %s\nSTAGE2: %s\nARGS: %s\nARGS_ADDR: 0x%x\n",
			spl_image.os_image, spl_image.second_stage,
			spl_image.args, spl_image.args_addr);
}

void spl_mmc_load_image(void)
{
	struct mmc *mmc;
	int err;
	u32 boot_mode;

	mmc_initialize(gd->bd);
	/* We register only one device. So, the dev id is always 0 */
	mmc = find_mmc_device(0);
	if (!mmc) {
		puts("spl: mmc device not found!!\n");
		hang();
	}

	err = mmc_init(mmc);
	if (err) {
		printf("spl: mmc init failed: err - %d\n", err);
		hang();
	}

	spl_mmc_config();

#ifdef CONFIG_SPL_FAT_SUPPORT
	/* FAT filesystem */
	err = fat_register_device(&mmc->block_dev,
			  CONFIG_SYS_MMC_SD_FAT_BOOT_PARTITION);
	/*if (err) {
		printf("spl: fat register err - %d\n", err);
	}*/
#ifdef CONFIG_SPL_OS_BOOT
	if (spl_start_uboot() || mmc_load_image_fat_os(mmc))
#endif
	err = mmc_load_image_fat(mmc, spl_image.second_stage);
#endif

#ifdef CONFIG_SPL_EXT_SUPPORT
	/* EXT filesystem */
	if (err) {
		printf("Load image from EXT...\n");
		disk_partition_t info;
		if (get_partition_info(&mmc->block_dev, CONFIG_SYS_MMC_SD_FAT_BOOT_PARTITION, &info)) {
			printf("Cannot find partition %d\n", CONFIG_SYS_MMC_SD_FAT_BOOT_PARTITION);
		}
		if (ext4fs_probe(&mmc->block_dev, &info)) {
			printf("ext4fs probe failed \n");
		}
#ifdef CONFIG_SPL_OS_BOOT
		if (spl_start_uboot() || mmc_load_image_ext_os(mmc))
#endif
		err = mmc_load_image_ext(mmc, spl_image.second_stage);
	}
#endif

	if (err) {
		printf("Load image from RAW...\n");
#ifdef CONFIG_SPL_OS_BOOT_RAW_SUPPORT
		if (spl_start_uboot() || mmc_load_image_raw_os(mmc))
#endif
		err = mmc_load_image_raw(mmc, CONFIG_SYS_MMCSD_RAW_MODE_U_BOOT_SECTOR);
#ifdef CONFIG_SYS_MMCSD_RAW_MODE_U_BOOT_SECTOR_ALT
		if (err)
			err = mmc_load_image_raw(mmc, CONFIG_SYS_MMCSD_RAW_MODE_U_BOOT_SECTOR_ALT);
#endif
		if (err) {
			printf("spl: wrong MMC boot mode\n");
			hang();
		}
	}
}
