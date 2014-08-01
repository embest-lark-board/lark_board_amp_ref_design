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


#ifndef _BM_SD_LOAD_H
#define _BM_SD_LOAD_H

#include <bm.h>

/* SD layout:
 * absolute offset    name                 size					  relative offset at raw(0xA2) partition
 *    0x100000      preloader(s)      64KB/per   256KB(total)	; offset is 0x0
 *    0x140000      u-boot.img       768KB(max)				; offset is 0x40000
 *    0x200000      bm.bin             1MB(max)				; offset is 0x100000
 *    0x300000      rbf.bin             14MB(max)				; offset is 0x200000
 *
 *    It must be generated from the make_sd_image_bm.sh
 */

#define CONFIG_PRELOADER_SDMMC_RBF_IMAGE			(0x200000)

#define CONFIG_SYS_MMCSD_RAW_MODE_RBF_SECTOR \
	(CONFIG_PRELOADER_SDMMC_RBF_IMAGE / 512)
	
#define CONFIG_PRELOADER_SDMMC_RBF_IMAGE_SIZE		(0xa00000)



s32 bm_sd_load_rbf(void);


#endif
