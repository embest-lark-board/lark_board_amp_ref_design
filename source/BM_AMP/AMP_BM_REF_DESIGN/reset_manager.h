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

#ifndef	_RESET_MANAGER_H_
#define	_RESET_MANAGER_H_

void watchdog_disable(void);
int is_wdt_in_reset(void);
void reset_cpu(ulong addr);
void reset_deassert_all_peripherals(void);
void reset_deassert_all_bridges(void);
void reset_deassert_cpu1(void);
void reset_deassert_osc1timer0(void);
void reset_deassert_osc1wd0(void);
void reset_assert_all_peripherals(void);
void reset_assert_all_peripherals_except_l4wd0(void);
void reset_assert_all_bridges(void);
void reset_deassert_peripherals_handoff(void);
void reset_deassert_bridges_handoff(void);

#if defined(CONFIG_SOCFPGA_VIRTUAL_TARGET)
struct socfpga_reset_manager {
	u32	padding1;
	u32	ctrl;
	u32	padding2;
	u32	padding3;
	u32	mpu_mod_reset;
	u32	per_mod_reset;
};
#else
struct socfpga_reset_manager {
	u32	status;
	u32	ctrl;
	u32	counts;
	u32	padding1;
	u32	mpu_mod_reset;
	u32	per_mod_reset;
	u32	per2_mod_reset;
	u32	brg_mod_reset;
};
#endif


#define RSTMGR_CTRL_SWWARMRSTREQ_LSB 1
#define RSTMGR_PERMODRST_OSC1TIMER0_LSB 8

#define RSTMGR_PERMODRST_L4WD0_LSB 6
#define RSTMGR_PERMODRST_SDR_LSB 29

/* Warm Reset mask */
#define RSTMGR_STAT_L4WD1RST_MASK 0x00008000
#define RSTMGR_STAT_L4WD0RST_MASK 0x00004000
#define RSTMGR_STAT_MPUWD1RST_MASK 0x00002000
#define RSTMGR_STAT_MPUWD0RST_MASK 0x00001000
#define RSTMGR_STAT_SWWARMRST_MASK 0x00000400
#define RSTMGR_STAT_FPGAWARMRST_MASK 0x00000200
#define RSTMGR_STAT_NRSTPINRST_MASK 0x00000100
#define RSTMGR_WARMRST_MASK	(\
	RSTMGR_STAT_L4WD1RST_MASK | \
	RSTMGR_STAT_L4WD0RST_MASK | \
	RSTMGR_STAT_MPUWD1RST_MASK | \
	RSTMGR_STAT_MPUWD0RST_MASK | \
	RSTMGR_STAT_SWWARMRST_MASK | \
	RSTMGR_STAT_FPGAWARMRST_MASK | \
	RSTMGR_STAT_NRSTPINRST_MASK)
#define RSTMGR_CTRL_SDRSELFREFEN_MASK 0x00000010
#define RSTMGR_CTRL_FPGAHSEN_MASK 0x00010000
#define RSTMGR_CTRL_ETRSTALLEN_MASK 0x00100000

#define RSTMGR_PERMODRST_EMAC0_SET(x) \
(((x) << 0) & 0x00000001)
#define RSTMGR_PERMODRST_EMAC1_SET(x) \
(((x) << 1) & 0x00000002)
#define RSTMGR_PERMODRST_USB0_SET(x) \
(((x) << 2) & 0x00000004)
#define RSTMGR_PERMODRST_USB1_SET(x) \
(((x) << 3) & 0x00000008)
#define RSTMGR_PERMODRST_NAND_SET(x) \
(((x) << 4) & 0x00000010)
#define RSTMGR_PERMODRST_QSPI_SET(x) \
(((x) << 5) & 0x00000020)
#define RSTMGR_PERMODRST_L4WD1_SET(x) \
(((x) << 7) & 0x00000080)
#define RSTMGR_PERMODRST_OSC1TIMER1_SET(x) \
(((x) << 9) & 0x00000200)
#define RSTMGR_PERMODRST_SPTIMER0_SET(x) \
(((x) << 10) & 0x00000400)
#define RSTMGR_PERMODRST_SPTIMER1_SET(x) \
(((x) << 11) & 0x00000800)
#define RSTMGR_PERMODRST_I2C0_SET(x) \
(((x) << 12) & 0x00001000)
#define RSTMGR_PERMODRST_I2C1_SET(x) \
(((x) << 13) & 0x00002000)
#define RSTMGR_PERMODRST_I2C2_SET(x) \
(((x) << 14) & 0x00004000)
#define RSTMGR_PERMODRST_I2C3_SET(x) \
(((x) << 15) & 0x00008000)
#define RSTMGR_PERMODRST_UART0_SET(x) \
(((x) << 16) & 0x00010000)
#define RSTMGR_PERMODRST_UART1_SET(x) \
(((x) << 17) & 0x00020000)
#define RSTMGR_PERMODRST_SPIM0_SET(x) \
(((x) << 18) & 0x00040000)
#define RSTMGR_PERMODRST_SPIM1_SET(x) \
(((x) << 19) & 0x00080000)
#define RSTMGR_PERMODRST_SPIS0_SET(x) \
(((x) << 20) & 0x00100000)
#define RSTMGR_PERMODRST_SPIS1_SET(x) \
(((x) << 21) & 0x00200000)
#define RSTMGR_PERMODRST_SDMMC_SET(x) \
(((x) << 22) & 0x00400000)
#define RSTMGR_PERMODRST_CAN0_SET(x) \
(((x) << 23) & 0x00800000)
#define RSTMGR_PERMODRST_CAN1_SET(x) \
(((x) << 24) & 0x01000000)
#define RSTMGR_PERMODRST_GPIO0_SET(x) \
(((x) << 25) & 0x02000000)
#define RSTMGR_PERMODRST_GPIO1_SET(x) \
(((x) << 26) & 0x04000000)
#define RSTMGR_PERMODRST_GPIO2_SET(x) \
(((x) << 27) & 0x08000000)
#define RSTMGR_PERMODRST_DMA_SET(x) \
(((x) << 28) & 0x10000000)
#define RSTMGR_PERMODRST_SDR_SET(x) \
(((x) << 29) & 0x20000000)

#define RSTMGR_PER2MODRST_DMAIF0_SET(x) \
(((x) << 0) & 0x00000001)
#define RSTMGR_PER2MODRST_DMAIF1_SET(x) \
(((x) << 1) & 0x00000002)
#define RSTMGR_PER2MODRST_DMAIF2_SET(x) \
(((x) << 2) & 0x00000004)
#define RSTMGR_PER2MODRST_DMAIF3_SET(x) \
(((x) << 3) & 0x00000008)
#define RSTMGR_PER2MODRST_DMAIF4_SET(x) \
(((x) << 4) & 0x00000010)
#define RSTMGR_PER2MODRST_DMAIF5_SET(x) \
(((x) << 5) & 0x00000020)
#define RSTMGR_PER2MODRST_DMAIF6_SET(x) \
(((x) << 6) & 0x00000040)
#define RSTMGR_PER2MODRST_DMAIF7_SET(x) \
(((x) << 7) & 0x00000080)

#define RSTMGR_BRGMODRST_HPS2FPGA_SET(x) \
(((x) << 0) & 0x00000001)
#define RSTMGR_BRGMODRST_LWHPS2FPGA_SET(x) \
(((x) << 1) & 0x00000002)
#define RSTMGR_BRGMODRST_FPGA2HPS_SET(x) \
(((x) << 2) & 0x00000004)

#endif /* _RESET_MANAGER_H_ */

