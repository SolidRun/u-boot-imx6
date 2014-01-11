/*
 * (C) Copyright 2013
 * Richard Hu, Technexion Ltd, richard.hu@technexion.com

 * (C) Copyright 2006-2008
 * Stefan Roese, DENX Software Engineering, sr@denx.de.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <nand.h>
#include <asm/io.h>
#include <linux/mtd/nand_ecc.h>

static nand_info_t mtd;
static struct nand_chip nand_chip;

#define CONFIG_SYS_NAND_RESET_CNT 200000


static int nand_command_lp(int block, int page, uint32_t column, u8 command)
{
	struct nand_chip *chip = mtd.priv;
	int page_addr = page + block * CONFIG_SYS_NAND_PAGE_COUNT;
	uint32_t rst_sts_cnt = CONFIG_SYS_NAND_RESET_CNT;

	/* Emulate NAND_CMD_READOOB */
	if (command == NAND_CMD_READOOB) {
		column += mtd.writesize;
		command = NAND_CMD_READ0;
	}

	/* Command latch cycle */
	chip->cmd_ctrl(&mtd, command & 0xff,
		       NAND_NCE | NAND_CLE | NAND_CTRL_CHANGE);

	if (column != -1 || page_addr != -1) {
		int ctrl = NAND_CTRL_CHANGE | NAND_NCE | NAND_ALE;

		/* Serially input address */
		if (column != -1) {
			/* Adjust columns for 16 bit buswidth */
			if (chip->options & NAND_BUSWIDTH_16)
				column >>= 1;
			chip->cmd_ctrl(&mtd, column, ctrl);
			ctrl &= ~NAND_CTRL_CHANGE;
			chip->cmd_ctrl(&mtd, column >> 8, ctrl);
		}
		if (page_addr != -1) {
			chip->cmd_ctrl(&mtd, page_addr, ctrl);
			chip->cmd_ctrl(&mtd, page_addr >> 8,
					NAND_NCE | NAND_ALE);
			#ifdef CONFIG_SYS_NAND_5_ADDR_CYCLE
			chip->cmd_ctrl(&mtd, page_addr >> 16,
					NAND_NCE | NAND_ALE);
			#endif
		}
	}
	chip->cmd_ctrl(&mtd, NAND_CMD_NONE, NAND_NCE | NAND_CTRL_CHANGE);

	/*
	 * Program and erase have their own busy handlers status, sequential
	 * in, and deplete1 need no delay.
	 */
	switch (command) {

	case NAND_CMD_CACHEDPROG:
	case NAND_CMD_PAGEPROG:
	case NAND_CMD_ERASE1:
	case NAND_CMD_ERASE2:
	case NAND_CMD_SEQIN:
	case NAND_CMD_RNDIN:
	case NAND_CMD_STATUS:
	case NAND_CMD_DEPLETE1:
		return;

	case NAND_CMD_STATUS_ERROR:
	case NAND_CMD_STATUS_ERROR0:
	case NAND_CMD_STATUS_ERROR1:
	case NAND_CMD_STATUS_ERROR2:
	case NAND_CMD_STATUS_ERROR3:
		/* Read error status commands require only a short delay */
		udelay(chip->chip_delay);
		return;

	case NAND_CMD_RESET:
		if (chip->dev_ready)
			break;
		udelay(chip->chip_delay);
		chip->cmd_ctrl(&mtd, NAND_CMD_STATUS,
			       NAND_NCE | NAND_CLE | NAND_CTRL_CHANGE);
		chip->cmd_ctrl(&mtd, NAND_CMD_NONE,
			       NAND_NCE | NAND_CTRL_CHANGE);
		while (!(chip->read_byte(&mtd) & NAND_STATUS_READY) &&
			(rst_sts_cnt--));
		return;

	case NAND_CMD_RNDOUT:
		/* No ready / busy check necessary */
		chip->cmd_ctrl(&mtd, NAND_CMD_RNDOUTSTART,
			       NAND_NCE | NAND_CLE | NAND_CTRL_CHANGE);
		chip->cmd_ctrl(&mtd, NAND_CMD_NONE,
			       NAND_NCE | NAND_CTRL_CHANGE);
		return;

	case NAND_CMD_READ0:
		chip->cmd_ctrl(&mtd, NAND_CMD_READSTART,
			       NAND_NCE | NAND_CLE | NAND_CTRL_CHANGE);
		chip->cmd_ctrl(&mtd, NAND_CMD_NONE,
			       NAND_NCE | NAND_CTRL_CHANGE);

		/* This applies to read commands */
	default:
		/*
		 * If we don't have access to the busy pin, we apply the given
		 * command delay.
		 */
		if (!chip->dev_ready) {
			udelay(chip->chip_delay);
			return;
		}
	}

	/*
	 * Apply this short delay always to ensure that we do wait tWB in
	 * any case on any machine.
	 */
	ndelay(100);

	while (!chip->dev_ready(&mtd))
	;
}


static int nand_is_bad_block(int block)
{
	struct nand_chip *this = mtd.priv;
	struct mxs_nand_info *nand_info = nand_chip.priv;
	char bad_mark;
	
	nand_command_lp(block, 0, CONFIG_SYS_NAND_BAD_BLOCK_POS, NAND_CMD_READOOB);
	bad_mark = this->read_byte(&mtd);
	if (bad_mark != 0xff)	{
		printf("nand_is_bad_block: %lx is bad block:%x !!! \r\n", block, bad_mark);
		return 1;
	}

	return 0;
}

static int nand_read_page(int block, int page, void *dst)
{
	struct nand_chip *this = mtd.priv;

	void (*hwctrl)(struct mtd_info *mtd, int cmd,
			unsigned int ctrl) = this->cmd_ctrl;

	nand_command_lp(block, page, 0, NAND_CMD_READ0);
	this->ecc.read_page(&mtd, &nand_chip, dst, 0, page);
}


int nand_spl_load_image(uint32_t offs, unsigned int size, void *dst)
{
	unsigned int block, lastblock;
	unsigned int page;

	/*
	 * offs has to be aligned to a page address!
	 */
	block = offs / CONFIG_SYS_NAND_BLOCK_SIZE;
	lastblock = (offs + size - 1) / CONFIG_SYS_NAND_BLOCK_SIZE;
	page = (offs % CONFIG_SYS_NAND_BLOCK_SIZE) / CONFIG_SYS_NAND_PAGE_SIZE;

	nand_read_page(block, page, dst);

	
	while (block <= lastblock) {
		if (!nand_is_bad_block(block)) {
			//
			// Skip bad blocks
			//
			while (page < CONFIG_SYS_NAND_PAGE_COUNT) {
				nand_read_page(block, page, dst);
				dst += CONFIG_SYS_NAND_PAGE_SIZE;
				page++;
			}

			page = 0;
		
		} else {
			lastblock++;
		}
		

		block++;
	}
	

	return 0;
}

/* nand_init() - initialize data to make nand usable by SPL */
void nand_init(void)
{
	/*
	 * Init board specific nand support
	 */
	mtd.priv = &nand_chip;

	board_nand_init(&nand_chip);
	mtd.priv = &nand_chip;	

	if (nand_chip.select_chip)
		nand_chip.select_chip(&mtd, 0);
	
	mtd.writesize = CONFIG_SYS_NAND_PAGE_SIZE;
	mtd.oobsize = CONFIG_SYS_NAND_OOBSIZE;

	//Actually, not scan and create bbt, just for setting flash memory geometry to BCH
	nand_chip.scan_bbt(&mtd); 

	nand_chip.buffers = memalign(ARCH_DMA_MINALIGN, sizeof(*nand_chip.buffers));
	nand_chip.oob_poi = nand_chip.buffers->databuf + mtd.writesize;	

}

/* Unselect after operation */
void nand_deselect(void)
{
	if (nand_chip.select_chip)
		nand_chip.select_chip(&mtd, -1);
}

