/*
 * (C) Copyright 2010
 * Texas Instruments, <www.ti.com>
 *
 * Aneesh V <aneesh@ti.com>
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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */
#include <common.h>
#include <spl.h>
#include <asm/u-boot.h>
#include <asm/utils.h>
#include <mmc.h>
#include <fat.h>
#include <version.h>

#include <asm/arch/amp_config.h>
#include <asm/io.h>

DECLARE_GLOBAL_DATA_PTR;

#ifdef CONFIG_SPL_BUILD
extern u32 get_core_id(void);
extern void load_cpu0_global_data_to_cpu1_r8(void);
extern void restore_cpu1_r8(void);
#endif

static struct mmc *dw_mmc=0;
static u32 offset = -1;

static void mmc_load_image_raw(struct mmc *mmc)
{
	u32 image_size_sectors, err;
	const struct image_header *header;

	header = (struct image_header *)(CONFIG_SYS_TEXT_BASE -
						sizeof(struct image_header));

	/* read image header to find the image size & load address */
	err = mmc->block_dev.block_read(0,
			CONFIG_SYS_MMCSD_RAW_MODE_U_BOOT_SECTOR, 1,
			(void *)header);

	if (err <= 0)
		goto end;

	spl_parse_image_header(header);

	/* convert size to sectors - round up */
	image_size_sectors = (spl_image.size + mmc->read_bl_len - 1) /
				mmc->read_bl_len;

	/* Read the header too to avoid extra memcpy */
	err = mmc->block_dev.block_read(0,
			CONFIG_SYS_MMCSD_RAW_MODE_U_BOOT_SECTOR,
			image_size_sectors, (void *)spl_image.load_addr);

end:
	if (err <= 0) {
		printf("spl: mmc blk read err - %d\n", err);
		hang();
	}
}

static u32 spl_mmc_get_preloader_offset_in_mbr(struct mmc *mmc)
{
#define MBR_SIGNATURE				(0xAA55)
#define MBR_SIGNATURE_OFFSET			(0x1FE)
#define MBR_NUM_OF_PARTITIONS			(4)
#define MBR_PARTITION_OFFSET			(0x1BE)
#define MBR_PARTITION_TYPE_OFFSET		(4)
#define MBR_PARTITION_START_SECTOR_OFFSET	(8)
#define MBR_PARTITION_ENTRY_SIZE		(16)
#define CONFIG_SYS_MMCSD_MBR_MODE_U_BOOT_PARTITION_ID	(0xA2)
	u32 i;
	u32 part_type;
	u32 offset;
	u16 sign;
	u8 *p_entry;
	const struct image_header *header;
	s32 err;

	header = (struct image_header *)(CONFIG_SYS_TEXT_BASE -
						sizeof(struct image_header));

	/* search the MBR to get the parition offset */
	/* Read MBR */
	err = mmc->block_dev.block_read(0, 0, 1, (void *)header);
	if (err <= 0)
		goto end;
	memcpy(&sign, (u8 *)header + MBR_SIGNATURE_OFFSET, sizeof(sign));
	if (sign != MBR_SIGNATURE) {
		printf("spl: No MBR signature is found\n");
		hang();
	}

	/* Lookup each partition entry for partition ID. */
	p_entry = (u8 *)header + MBR_PARTITION_OFFSET;
	for (i = 0; i < MBR_NUM_OF_PARTITIONS; i++) {
		part_type = p_entry[MBR_PARTITION_TYPE_OFFSET];

		if (part_type ==
			CONFIG_SYS_MMCSD_MBR_MODE_U_BOOT_PARTITION_ID){
			/* Partition found */
			memcpy(&offset,
				p_entry + MBR_PARTITION_START_SECTOR_OFFSET,
				sizeof(offset));
			debug("spl: Partition offset 0x%x sectors\n", offset);
			break;
		}
		p_entry += MBR_PARTITION_ENTRY_SIZE;
	}
	return offset;
	
end:
	if (err <= 0) {
		printf("spl: mmc blk read err - %d\n", err);
		hang();
	}

}

static void mmc_load_image_mbr(struct mmc *mmc)
{
	s32 err;
	u32 image_size_sectors;
	const struct image_header *header;
	
	header = (struct image_header *)(CONFIG_SYS_TEXT_BASE -
						sizeof(struct image_header));

	/* read image header to find the image size & load address */
	err = mmc->block_dev.block_read(0,
			(offset + CONFIG_SYS_MMCSD_RAW_MODE_U_BOOT_SECTOR), 1,
			(void *)header);

	if (err <= 0)
		goto end;

	spl_parse_image_header(header);

	/* convert size to sectors - round up */
	image_size_sectors = (spl_image.size + mmc->read_bl_len - 1) /
				mmc->read_bl_len;

	/* Read the header too to avoid extra memcpy */
	err = mmc->block_dev.block_read(0,
			(offset + CONFIG_SYS_MMCSD_RAW_MODE_U_BOOT_SECTOR),
			image_size_sectors, (void *)spl_image.load_addr);

end:
	if (err <= 0) {
		printf("spl: mmc blk read err - %d\n", err);
		hang();
	}
}

void spl_mmc_load_bm_image_mbr(void)
{
	s32 err;
	u32 image_size_sectors;

	/* convert size to sectors - round up */
	image_size_sectors = (CONFIG_PRELOADER_SDMMC_BARE_METAL_IMAGE_SIZE + 
						dw_mmc->read_bl_len - 1) /dw_mmc->read_bl_len;
	
	/* read image header to find the image size & load address */
	err = dw_mmc->block_dev.block_read(0,
			(offset + CONFIG_SYS_MMCSD_RAW_MODE_BARE_METAL_SECTOR), 
			image_size_sectors, (void *)AMPPHY_START);
	if (err <= 0) {
		printf("spl: mmc blk read err - %d\n", err);
		hang();
	}
}
/* 
 * we reuse the preloader tf card driver in the bare metal, and we save
 * the tf card probe and read functions in the share parameter memory region, and reuse
 * it in the bare metal for loading the fpga image.
 */
void spl_mmc_save_func(void)
{
	writel((u32)dw_mmc, &(asp->sd_bi.preloader_sd_saved_mmc));
	writel((u32)offset, &(asp->sd_bi.preloader_sd_saved_offset));

	if(dw_mmc)
	{
		writel((u32)(dw_mmc->block_dev.block_read), &(asp->sd_bi.preloader_sd_read_fn));
		asp->sd_bi.sd_read_bl_len = dw_mmc->read_bl_len;
	}

}


#ifdef CONFIG_SPL_FAT_SUPPORT
static void mmc_load_image_fat(struct mmc *mmc)
{
	s32 err;
	struct image_header *header;

	header = (struct image_header *)(CONFIG_SYS_TEXT_BASE -
						sizeof(struct image_header));

	err = fat_register_device(&mmc->block_dev,
				CONFIG_SYS_MMC_SD_FAT_BOOT_PARTITION);
	if (err) {
		printf("spl: fat register err - %d\n", err);
		hang();
	}

	err = file_fat_read(CONFIG_SPL_FAT_LOAD_PAYLOAD_NAME,
				(u8 *)header, sizeof(struct image_header));
	if (err <= 0)
		goto end;

	spl_parse_image_header(header);

	err = file_fat_read(CONFIG_SPL_FAT_LOAD_PAYLOAD_NAME,
				(u8 *)spl_image.load_addr, 0);

end:
	if (err <= 0) {
		printf("spl: error reading image %s, err - %d\n",
			CONFIG_SPL_FAT_LOAD_PAYLOAD_NAME, err);
		hang();
	}
}
#endif

struct mmc * spl_mmc_probe(void)
{
	struct mmc *mmc;
	int err;
	u32 id;

	mmc_initialize(gd->bd);

	/* We register only one device. So, the dev id is always 0 */
	mmc = find_mmc_device(0);
	if (!mmc) {
		printf("spl: mmc device not found!!\n");
		hang();
	}

	err = mmc_init(mmc);
	if (err) {
		printf("spl: mmc init failed: err - %d\n", err);
		hang();
	}

	dw_mmc = mmc;
	offset = spl_mmc_get_preloader_offset_in_mbr(dw_mmc);
	
	return mmc;
}

void spl_mmc_load_image(void)
{
	int err;
	u32 boot_mode;

	boot_mode = spl_boot_mode();
	if (boot_mode == MMCSD_MODE_RAW) {
		debug("boot mode - RAW\n");
		mmc_load_image_raw(dw_mmc);
	} else if (boot_mode == MMCSD_MODE_MBR) {
		debug("boot mode - MBR\n");
		mmc_load_image_mbr(dw_mmc);
#ifdef CONFIG_SPL_FAT_SUPPORT
	} else if (boot_mode == MMCSD_MODE_FAT) {
		debug("boot mode - FAT\n");
		mmc_load_image_fat(dw_mmc);
#endif
	} else {
		printf("spl: wrong MMC boot mode\n");
		hang();
	}
}
