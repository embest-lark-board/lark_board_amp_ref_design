/*
 *
 * Copyright (C) 2014 Embest Technology Co., Ltd. <http://www.embest-tech.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  - Neither the name of Embest Technology Co., Ltd. nor the
 *    names of its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS", 
 * it is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 */


#include <sd_load.h>
#include <io.h>
#include <bmlog.h>

struct mmc;

typedef u32	(*preloader_block_read)(s32 dev, u32 start, u32 blkcnt, void *buffer);

preloader_block_read bm_sd_block_read;

extern struct amp_share_param *asp;

s32 bm_sd_load_rbf(void)
{
	struct mmc * mmc;
	s32 err;
	u32 image_size_sectors;
	u32 offset;

	bm_sd_block_read = (preloader_block_read)readl(&(asp->sd_bi.preloader_sd_read_fn));

	mmc = readl(&(asp->sd_bi.preloader_sd_saved_mmc));
	if(!mmc)
	{
		uartprintf("BM probe sd card failed!\n");
		return -1;
	}
	
	offset = readl(&(asp->sd_bi.preloader_sd_saved_offset));
	image_size_sectors = (CONFIG_PRELOADER_SDMMC_RBF_IMAGE_SIZE + 
						asp->sd_bi.sd_read_bl_len - 1) /asp->sd_bi.sd_read_bl_len;

	/* read image header to find the image size & load address */
	err = bm_sd_block_read(0,
						(offset + CONFIG_SYS_MMCSD_RAW_MODE_RBF_SECTOR), 
						image_size_sectors, 
						(void *)FPGA_RBF_BASE);
	if (err <= 0) {
		uartprintf("spl: mmc blk read err - %d\n", err);
		return -2;
	}
	

	return 0;
}


