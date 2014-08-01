/*
 *
 * Copyright (C) 2012 Altera Corporation <www.altera.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  - Neither the name of the Altera Corporation nor the
 *    names of its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL ALTERA CORPORATION BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include <reset_config.h>
#include <types.h>
#include <fpga_manager.h>
#include <nic301.h>
#include <reset_manager.h>
#include <regs.h>
#include <io.h>
#include <bmlog.h>


#define DEBUG_MEMORY

#ifdef DEBUG_FPGA
#define FPGA_DEBUG(format...) bmlog(format)
#else
#define FPGA_DEBUG(format...) 
#endif

static const struct socfpga_reset_manager *reset_manager_base =
		(void *)SOCFPGA_RSTMGR_ADDRESS;

/* Disable the watchdog (toggle reset to watchdog) */
void watchdog_disable(void)
{
	/* assert reset for watchdog */
	setbits_le32(&reset_manager_base->per_mod_reset,
		(1<<RSTMGR_PERMODRST_L4WD0_LSB));
	/* deassert watchdog from reset (watchdog in not running state) */
	clrbits_le32(&reset_manager_base->per_mod_reset,
		(1<<RSTMGR_PERMODRST_L4WD0_LSB));
}

/* Check whether Watchdog in reset state? */
int is_wdt_in_reset(void)
{
	unsigned long val;
	val = readl(&reset_manager_base->per_mod_reset);
	val = ((val >> RSTMGR_PERMODRST_L4WD0_LSB) & 0x1);
	/* return 0x1 if watchdog in reset */
	return val;
}

/* Write the reset manager register to cause reset */
void reset_cpu(ulong addr)
{
	/* request a warm reset */
	writel( (1 << RSTMGR_CTRL_SWWARMRSTREQ_LSB),
		(volatile void *)&reset_manager_base->ctrl);
	/*
	 * infinite loop here as watchdog will trigger and reset
	 * the processor
	 */
	while (1)
		;
}

/* Release all peripherals from reset through reset manager */
void reset_deassert_all_peripherals(void)
{
	writel(0x0, (volatile void *)&reset_manager_base->per_mod_reset);
}

/* Release all bridges from reset through reset manager */
void reset_deassert_all_bridges(void)
{
	/* check signal from FPGA */
	if (is_fpgamgr_fpga_ready() == 0) {
		/* FPGA not ready. Not much can be done but let WD timeout */
		for(;;)
			;
	}
	/* brdmodrst */
	writel(0, (volatile void *)&reset_manager_base->brg_mod_reset);

	/* remap the bridges into memory map */
	setbits_le32(SOCFPGA_L3REGS_ADDRESS,
		(L3REGS_REMAP_LWHPS2FPGA_MASK | L3REGS_REMAP_HPS2FPGA_MASK));
}

/* Release second processor cpu1 from reset through reset manager */
void reset_deassert_cpu1(void)
{
	writel(0, (volatile void *)&reset_manager_base->mpu_mod_reset);
}

/* Release L4 OSC1 Timer0 from reset through reset manager */
void reset_deassert_osc1timer0(void)
{
	clrbits_le32(&reset_manager_base->per_mod_reset,
		(1<<RSTMGR_PERMODRST_OSC1TIMER0_LSB));
}

/* Release L4 OSC1 Watchdog Timer 0 from reset through reset manager */
void reset_deassert_osc1wd0(void)
{
	clrbits_le32(&reset_manager_base->per_mod_reset,
		(1<<RSTMGR_PERMODRST_L4WD0_LSB));
}

/* Assert reset to all peripherals through reset manager */
void reset_assert_all_peripherals(void)
{
	writel(~0, (volatile void *)&reset_manager_base->per_mod_reset);
}

/* Assert reset to all bridges through reset manager */
void reset_assert_all_bridges(void)
{
}

/* Assert reset to all peripherals through reset manager */
void reset_assert_all_peripherals_except_l4wd0(void)
{
	writel(~(1<<RSTMGR_PERMODRST_L4WD0_LSB),
		(volatile void *)&reset_manager_base->per_mod_reset);
}

/* Release peripherals from reset based on handoff */
void reset_deassert_peripherals_handoff(void)
{
	unsigned val = 0;

	/* permodrst */
	val |= RSTMGR_PERMODRST_EMAC0_SET(CONFIG_HPS_RESET_ASSERT_EMAC0);
	val |= RSTMGR_PERMODRST_EMAC1_SET(CONFIG_HPS_RESET_ASSERT_EMAC1);
	val |= RSTMGR_PERMODRST_USB0_SET(CONFIG_HPS_RESET_ASSERT_USB0);
	val |= RSTMGR_PERMODRST_USB1_SET(CONFIG_HPS_RESET_ASSERT_USB1);
	val |= RSTMGR_PERMODRST_NAND_SET(CONFIG_HPS_RESET_ASSERT_NAND);
	val |= RSTMGR_PERMODRST_QSPI_SET(CONFIG_HPS_RESET_ASSERT_QSPI);
	val |= RSTMGR_PERMODRST_L4WD1_SET(CONFIG_HPS_RESET_ASSERT_L4WD1);
	val |= RSTMGR_PERMODRST_OSC1TIMER1_SET(
		CONFIG_HPS_RESET_ASSERT_OSC1TIMER1);
	val |= RSTMGR_PERMODRST_SPTIMER0_SET(CONFIG_HPS_RESET_ASSERT_SPTIMER0);
	val |= RSTMGR_PERMODRST_SPTIMER1_SET(CONFIG_HPS_RESET_ASSERT_SPTIMER1);
	val |= RSTMGR_PERMODRST_I2C0_SET(CONFIG_HPS_RESET_ASSERT_I2C0);
	val |= RSTMGR_PERMODRST_I2C1_SET(CONFIG_HPS_RESET_ASSERT_I2C1);
	val |= RSTMGR_PERMODRST_I2C2_SET(CONFIG_HPS_RESET_ASSERT_I2C2);
	val |= RSTMGR_PERMODRST_I2C3_SET(CONFIG_HPS_RESET_ASSERT_I2C3);
	val |= RSTMGR_PERMODRST_UART0_SET(CONFIG_HPS_RESET_ASSERT_UART0);
	val |= RSTMGR_PERMODRST_UART1_SET(CONFIG_HPS_RESET_ASSERT_UART1);
	val |= RSTMGR_PERMODRST_SPIM0_SET(CONFIG_HPS_RESET_ASSERT_SPIM0);
	val |= RSTMGR_PERMODRST_SPIM1_SET(CONFIG_HPS_RESET_ASSERT_SPIM1);
	val |= RSTMGR_PERMODRST_SPIS0_SET(CONFIG_HPS_RESET_ASSERT_SPIS0);
	val |= RSTMGR_PERMODRST_SPIS1_SET(CONFIG_HPS_RESET_ASSERT_SPIS1);
	val |= RSTMGR_PERMODRST_SDMMC_SET(CONFIG_HPS_RESET_ASSERT_SDMMC);
	val |= RSTMGR_PERMODRST_CAN0_SET(CONFIG_HPS_RESET_ASSERT_CAN0);
	val |= RSTMGR_PERMODRST_CAN1_SET(CONFIG_HPS_RESET_ASSERT_CAN1);
	val |= RSTMGR_PERMODRST_GPIO0_SET(CONFIG_HPS_RESET_ASSERT_GPIO0);
	val |= RSTMGR_PERMODRST_GPIO1_SET(CONFIG_HPS_RESET_ASSERT_GPIO1);
	val |= RSTMGR_PERMODRST_GPIO2_SET(CONFIG_HPS_RESET_ASSERT_GPIO2);
	val |= RSTMGR_PERMODRST_DMA_SET(CONFIG_HPS_RESET_ASSERT_DMA);
	val |= RSTMGR_PERMODRST_SDR_SET(CONFIG_HPS_RESET_ASSERT_SDR);
	writel(val, (volatile void *)&reset_manager_base->per_mod_reset);

	/* permodrst2 */
	val = 0;
	val |= RSTMGR_PER2MODRST_DMAIF0_SET(CONFIG_HPS_RESET_ASSERT_FPGA_DMA0);
	val |= RSTMGR_PER2MODRST_DMAIF1_SET(CONFIG_HPS_RESET_ASSERT_FPGA_DMA1);
	val |= RSTMGR_PER2MODRST_DMAIF2_SET(CONFIG_HPS_RESET_ASSERT_FPGA_DMA2);
	val |= RSTMGR_PER2MODRST_DMAIF3_SET(CONFIG_HPS_RESET_ASSERT_FPGA_DMA3);
	val |= RSTMGR_PER2MODRST_DMAIF4_SET(CONFIG_HPS_RESET_ASSERT_FPGA_DMA4);
	val |= RSTMGR_PER2MODRST_DMAIF5_SET(CONFIG_HPS_RESET_ASSERT_FPGA_DMA5);
	val |= RSTMGR_PER2MODRST_DMAIF6_SET(CONFIG_HPS_RESET_ASSERT_FPGA_DMA6);
	val |= RSTMGR_PER2MODRST_DMAIF7_SET(CONFIG_HPS_RESET_ASSERT_FPGA_DMA7);
	writel(val, (volatile void *)&reset_manager_base->per2_mod_reset);

	/* warm reset handshake support */
	DEBUG_MEMORY
#if (CONFIG_HPS_RESET_WARMRST_HANDSHAKE_FPGA==1)
	setbits_le32(&reset_manager_base->ctrl, RSTMGR_CTRL_FPGAHSEN_MASK);
#else
	clrbits_le32(&reset_manager_base->ctrl, RSTMGR_CTRL_FPGAHSEN_MASK);
#endif

#if (CONFIG_HPS_RESET_WARMRST_HANDSHAKE_ETR==1)
	setbits_le32(&reset_manager_base->ctrl, RSTMGR_CTRL_ETRSTALLEN_MASK);
#else
	clrbits_le32(&reset_manager_base->ctrl, RSTMGR_CTRL_ETRSTALLEN_MASK);
#endif

#if (CONFIG_HPS_RESET_WARMRST_HANDSHAKE_SDRAM==1)
	setbits_le32(&reset_manager_base->ctrl, RSTMGR_CTRL_SDRSELFREFEN_MASK);
#else
	clrbits_le32(&reset_manager_base->ctrl, RSTMGR_CTRL_SDRSELFREFEN_MASK);
#endif
}


