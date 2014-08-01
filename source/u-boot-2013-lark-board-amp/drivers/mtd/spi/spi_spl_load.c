/*
 * Copyright (C) 2011 OMICRON electronics GmbH
 *
 * based on drivers/mtd/nand/nand_spl_load.c
 *
 * Copyright (C) 2011
 * Heiko Schocher, DENX Software Engineering, hs@denx.de.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <common.h>
#include <spi_flash.h>
#include <spl.h>
#include <asm/io.h>

#include <asm/arch/amp_config.h>

/*
 * The main entry for SPI booting. It's necessary that SDRAM is already
 * configured and available since this code loads the main U-Boot image
 * from SPI into SDRAM and starts it from there.
 */
void spl_spi_load_image(void)
{
	struct spi_flash *flash;
	struct image_header *header;

	/*
	 * Load U-Boot image from SPI flash into RAM
	 */

	flash = spi_flash_probe(CONFIG_SPL_SPI_BUS, CONFIG_SPL_SPI_CS,
				CONFIG_SF_DEFAULT_SPEED, SPI_MODE_3);
	if (!flash) {
		puts("SPI probe failed.\n");
		hang();
	}


#ifdef CONFIG_SPL_SPI_XIP
	if (spi_flash_xip_enter(flash)) {
		puts("SPI enter XIP mode failed.\n");
		hang();
	}
	puts("Both SPI and serial NOR flash in XIP mode\n");
	/* expected mkimage header at CONFIG_SPL_SPI_XIP_ADDR - 0x40 */
	header = (struct image_header *)(CONFIG_SPL_SPI_XIP_ADDR -
		sizeof(struct image_header));
	spl_parse_image_header(header);
	return;
#endif

	/* use CONFIG_SYS_TEXT_BASE as temporary storage area */
	header = (struct image_header *)(CONFIG_SYS_TEXT_BASE);

	/* Load u-boot, mkimage header is 64 bytes. */
	spi_flash_read(flash, CONFIG_SYS_SPI_U_BOOT_OFFS, 0x40,
			(void *) header);
	spl_parse_image_header(header);
	spi_flash_read(flash, CONFIG_SYS_SPI_U_BOOT_OFFS,
		       spl_image.size, (void *)spl_image.load_addr);
}

void spl_spi_load_bm(void)
{
	struct spi_flash *flash;

	/* probe the spi flash */
	flash = spi_flash_probe(CONFIG_SPL_SPI_BUS, CONFIG_SPL_SPI_CS,
				CONFIG_SF_DEFAULT_SPEED, SPI_MODE_3);
	if(!flash) {
		puts("SPI probe failed.\n");
		hang();
	}

	/* load teh bm.bin from the CONFIG_BM_QSPI_ADDERSS */
	spi_flash_read(flash, CONFIG_BM_QSPI_ADDERSS,
				CONFIG_BM_MAX_SIZE, AMPPHY_START);

}

extern struct amp_share_param *asp;
/* 
 * we reuse the preloader qspi driver in the bare metal, and we save
 * the qspi probe and read functions in the share parameter memory region, and reuse
 * it in the bare metal for load the fpga image.
 */
void spl_spi_save_func(void)
{
	writel(spi_flash_probe, &(asp->qspi_bi.preloader_qspi_probe_fn));
	writel(spi_flash_read, &(asp->qspi_bi.preloader_qspi_read_fn));

}


