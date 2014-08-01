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


#ifndef REGS_H
#define REGS_H

#define SOCFPGA_EMAC0_ADDRESS 0xff700000
#define SOCFPGA_EMAC1_ADDRESS 0xff702000
#define SOCFPGA_SDMMC_ADDRESS 0xff704000
#define SOCFPGA_QSPIREGS_ADDRESS 0xff705000
#define SOCFPGA_L3REGS_ADDRESS 0xff800000
#define SOCFPGA_QSPIREGS_ADDRESS 0xff705000
#define SOCFPGA_FPGAMGRREGS_ADDRESS 0xff706000
#define SOCFPGA_QSPIDATA_ADDRESS 0xffa00000
#define SOCFPGA_FPGAMGRDATA_ADDRESS 0xffb90000
#define SOCFPGA_UART0_ADDRESS 0xffc02000
#define SOCFPGA_UART1_ADDRESS 0xffc03000
#define SOCFPGA_SDR_ADDRESS 0xffc20000
#define SOCFPGA_OSC1TIMER0_ADDRESS 0xffd00000
#define SOCFPGA_OSC1TIMER1_ADDRESS 0xffd01000
#define SOCFPGA_L4WD0_ADDRESS 0xffd02000
#define SOCFPGA_CLKMGR_ADDRESS 0xffd04000
#define SOCFPGA_RSTMGR_ADDRESS 0xffd05000
#define SOCFPGA_SYSMGR_ADDRESS 0xffd08000
#define SOCFPGA_SCANMGR_ADDRESS 0xfff02000
#define SOCFPGA_MPUSCU_ADDRESS 0xfffec000
#define SOCFPGA_MPUL2_ADDRESS 0xfffef000
#define HPS_GPIO1_BASE_ADDR		0xff709000
#define HPS_GPIO2_BASE_ADDR		0xff70A000
#define HPS_DMAC_BASE			0xFFE01000

/*  HPS GPIO OFFSET */
#define HPS_GPIO_DAT_OFFSET		0x0
#define HPS_GPIO_DIR_OFFSET		0x4

#define BITBANG_IIC_PINMUX	0xFFD084BC
#define HPS_GPIO1_PINMUX	0xFFD084C0



/* ACP ID Mapper Register */
#define ACP_ID_MAP_REG_BASE					((u32)0xFF707000)
#define ACP_REG_ID_MAP(x)					((volatile u32 *)(ACP_ID_MAP_REG_BASE + x))
#define ACP_REG_ID_2_RD						(0x0)	/* fixed for DAP */
#define ACP_REG_ID_2_WR						(0x4)	/* fixed for DAP */
#define ACP_REG_ID_3_RD						(0x8)
#define ACP_REG_ID_3_WR						(0xc)
#define ACP_REG_ID_4_RD						(0x10)
#define ACP_REG_ID_4_WR						(0x14)
#define ACP_REG_ID_5_RD						(0x18)
#define ACP_REG_ID_5_WR						(0x1c)
#define ACP_REG_ID_6_RD						(0x20)
#define ACP_REG_ID_6_WR						(0x24)
#define ACP_REG_ID_7_RD						(0x28)
#define ACP_REG_ID_7_WR						(0x2c)

/* 12bit master id */
/* 
	Interconnect Master           | ID            
	----------------------------------------------------
	L2M0                               | 12'b0xxxxxxxx010
	DMA                                | 12'b00000xxxx001
	EMAC0                             | 12'b10000xxxx001
	EMAC1                             | 12'b10000xxxx010
	USB0                               | 12'b100000000011
      NAND                               | 12'b1xxxxxxxx100
	TMC                                | 12'b100000000000
      DAP                                 | 12'b000000000100
	SDMMC                            | 12'b100000000101
	FPGA2HPS bridge               | 12'b0xxxxxxxx000
      USB1                               | 12'b100000000110
      ----------------------------------------------------
      where:
      for dma: when the  DMA channel thread accesses the DMA interface, xxxx=0yyy, where yyy is chnannel number
                  When the DMA manager thread accesses the DMA interface, xxxx=1000 = number of channels
      Please refer to the newest the cyclone5_handbook.pdf.
*/

#define ACP_ID_DMA_CHANNEL_THREAD(x)						(0x1 | ((x & 0xf) << 3))
#define ACP_ID_DMA_MANAGER_THREAD						(0x1 | ((0x8 << 3))


#define  GIC_CPU_INTF_BASE    (u32)0xFFFEC100

#define  GIC_INT_REG_ICCICR              (*(volatile u32 *)(GIC_CPU_INTF_BASE + 0x00))
#define  GIC_INT_REG_ICCPMR              (*(volatile u32 *)(GIC_CPU_INTF_BASE + 0x04))
#define  GIC_INT_REG_ICCBPR              (*(volatile u32 *)(GIC_CPU_INTF_BASE + 0x08))
#define  GIC_INT_REG_ICCIAR              (*(volatile u32 *)(GIC_CPU_INTF_BASE + 0x0C))
#define  GIC_INT_REG_ICCEOIR             (*(volatile u32 *)(GIC_CPU_INTF_BASE + 0x10))
#define  GIC_INT_REG_ICCRPR              (*(volatile u32 *)(GIC_CPU_INTF_BASE + 0x14))
#define  GIC_INT_REG_ICCHPIR             (*(volatile u32 *)(GIC_CPU_INTF_BASE + 0x18))
#define  GIC_INT_REG_ICCABPR             (*(volatile u32 *)(GIC_CPU_INTF_BASE + 0x1C))
#define  GIC_INT_REG_ICCIIDR             (*(volatile u32 *)(GIC_CPU_INTF_BASE + 0xFC))


#define  GIC_INT_DIST_BASE         (u32)0xFFFED000

#define  GIC_INT_REG_ICDDCR              (*(volatile u32 *)(GIC_INT_DIST_BASE + 0x000))
#define  GIC_INT_REG_ICDICTR             (*(volatile u32 *)(GIC_INT_DIST_BASE + 0x004))
#define  GIC_INT_REG_ICDIIDR             (*(volatile u32 *)(GIC_INT_DIST_BASE + 0x008))
#define  GIC_INT_REG_ICDISR              (*(volatile u32 *)(GIC_INT_DIST_BASE + 0x080))
#define  GIC_INT_REG_ICDISER             (*(volatile u32 *)(GIC_INT_DIST_BASE + 0x100))
#define  GIC_INT_REG_ICDICER             (*(volatile u32 *)(GIC_INT_DIST_BASE + 0x180))
#define  GIC_INT_REG_ICDISPR             (*(volatile u32 *)(GIC_INT_DIST_BASE + 0x200))
#define  GIC_INT_REG_ICDICPR             (*(volatile u32 *)(GIC_INT_DIST_BASE + 0x280))
#define  GIC_INT_REG_ICDABR              (*(volatile u32 *)(GIC_INT_DIST_BASE + 0x300))
#define  GIC_INT_REG_ICDIPR              (*(volatile u32 *)(GIC_INT_DIST_BASE + 0x400))
#define  GIC_INT_REG_ICDIPTR             (*(volatile u32 *)(GIC_INT_DIST_BASE + 0x800))
#define  GIC_INT_REG_ICDICFR             (*(volatile u32 *)(GIC_INT_DIST_BASE + 0xC00))
#define  GIC_INT_REG_ICDSGIR             (*(volatile u32 *)(GIC_INT_DIST_BASE + 0xF00))

#endif
