#ifndef _BM_QSPI_H
#define _BM_QSPI_H

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


#include <bm.h>
#include <bmlog.h>

/* It comes from the u-boot's socfpga_cyclone5.h */

#define CONFIG_BM_BASE_ADDRESS		(0x1e000000)
#define CONFIG_BM_MAX_SIZE			(0x20000)	/* 128KB */
#define CONFIG_BM_QSPI_ADDERSS		(0x600000)
#define CONFIG_LOAD_BM_STAMP_START	(CONFIG_BM_BASE_ADDRESS + 0x100000 + 0x80000)
#define CONFIG_LOAD_BM_STAMP_END		(CONFIG_LOAD_BM_STAMP_START + 0x4)
#define CONFIG_QSPI_FUNC_START		(CONFIG_LOAD_BM_STAMP_END + 0x10)
#define CONFIG_QSPI_PROBE_FUNC		(CONFIG_QSPI_FUNC_START + 0x4)
#define CONFIG_QSPI_READ_FUNC			(CONFIG_QSPI_PROBE_FUNC + 0x4)
#define CONFIG_PRELOADER_WAIT_BM_LOAD_FPGA		(CONFIG_LOAD_BM_STAMP_END + 0x4)
#define CONFIG_PRELOADER_WAIT_BM_LOAD_FPGA_STATUS	(CONFIG_PRELOADER_WAIT_BM_LOAD_FPGA + 0x4)

/* QSPI layout:
  *  preloader:	0x00000000   256K  (0x0 ~ 0x3ffff) max 4 preloaders, per is 64KB
  *  env: 		0x40000      	64KB
  *  fdt: 			0x50000      	64KB
  *  u-boot:		0x60000    	256K           
  *  uImage:		0xa0000    	5MB max
  *  bm:   		0x600000    	128K
  *  rbf:  			0x640000		10MB max
  *  rootfs:		0x1200000	(128 - 18) 110MB or so, you must modify the kernel dts
 */

#define CONFIG_QSPI_RBF_OFF			(0x640000)


#define CONFIG_SPL_SPI_BUS				0
#define CONFIG_SPL_SPI_CS				0
#define CONFIG_SF_DEFAULT_SPEED		(50000000)

#define	SPI_CPHA						0x01			/* clock phase */
#define	SPI_CPOL						0x02			/* clock polarity */
#define	SPI_MODE_0						(0|0)			/* (original MicroWire) */
#define	SPI_MODE_1						(0|SPI_CPHA)
#define	SPI_MODE_2						(SPI_CPOL|0)
#define	SPI_MODE_3						(SPI_CPOL|SPI_CPHA)

					


struct spi_flash;

typedef struct spi_flash *(*spi_flash_probe_fn)(unsigned int bus, unsigned int cs,
		unsigned int max_hz, unsigned int spi_mode);

typedef int (*spi_flash_read_fn)(struct spi_flash *flash, u32 offset,
		u32 len, void *buf);


#endif


