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


#ifndef _BM_BOOT_MODE_H
#define _BM_BOOT_MODE_H

#include <bm.h>

#define BOOTSEL_MODE_MASK					(0x7)

#define BOOTSEL_MODE_NAND_1_8V			(0b010)
#define BOOTSEL_MODE_NAND_3_3V			(0b011)
#define BOOTSEL_MODE_SD_1_8V				(0b100)
#define BOOTSEL_MODE_SD_3_3V				(0b101)
#define BOOTSET_MODE_QSPI_1_8V			(0b110)
#define BOOTSET_MODE_QSPI_3_3V			(0b111)

u32 bm_get_boot_mode(void);

#endif



