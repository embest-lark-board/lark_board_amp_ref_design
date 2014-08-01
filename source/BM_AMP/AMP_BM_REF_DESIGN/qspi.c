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


#include "qspi.h"
#include "io.h"

struct spi_flash;

typedef struct spi_flash * (*spi_flash_probe_fn)(unsigned int bus, unsigned int cs,
		unsigned int max_hz, unsigned int spi_mode);

typedef int (*spi_flash_read_fn)(struct spi_flash *flash, u32 offset,
		u32 len, void *buf);

spi_flash_probe_fn sf_probe;
spi_flash_read_fn sf_read;

extern struct amp_share_param *asp;

s32 bm_qspi_load_rbf(void)
{
	struct spi_flash *flash;

	sf_probe = (spi_flash_probe_fn)asp->qspi_bi.preloader_qspi_probe_fn;
	sf_read = (spi_flash_read_fn)asp->qspi_bi.preloader_qspi_read_fn;

	
	flash = sf_probe(CONFIG_SPL_SPI_BUS, CONFIG_SPL_SPI_CS,
				CONFIG_SF_DEFAULT_SPEED, SPI_MODE_3);
	if(!flash)
	{
		uartprintf("BM probe spi flash failed!\n");
		return -1;
	}
	/* load the bm.bin from the CONFIG_QSPI_RBF_OFF */
	
	sf_read(flash, CONFIG_QSPI_RBF_OFF,
				FPGA_RBF_SIZE, (void *)FPGA_RBF_BASE);

	return 0;

}



