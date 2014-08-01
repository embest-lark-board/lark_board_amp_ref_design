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
#ifndef AMP_CONFIG_H
#define AMP_CONFIG_H

#include <linux/semaphore.h>

#define DRAM_PHYS_START	(0x00000000)
#define BM_MEM_START	(0x1E000000)	//bm phys start memory
#define SH_MEM_START	(BM_MEM_START + 0x100000)	//share start memory
#define SH_MEM_SIZE	(0x1F00000)	//exclude BM context region

//0x1E000000~0x1E0FFFFF:	1MB	BM context region
//0x1E100000~0x1E1FFFFF:	1MB	share memory ioctl region
//0x1E200000~0x1FDFFFFF:	12MB	share buffer region
//0x1FE00000~0x1FFFFFFF:	2MB	DMA map share buffer

/*
*------------------------------------
*0xfc600000-0xfc6fffff:	BM context
*------------------------------------
*0xfc700000-0xfc7fffff:	sh_ioctl_region[MAX_SGI], sh_text_addr...
*------------------------------------
*0xfc800000-0xfe3fffff:	cache share buffer
*------------------------------------
*0xfe400000-0xfe5fffff:	dma map share buffer
*------------------------------------
*0xfe700000-0xfe70ffff: sram amp region
*------------------------------------
*0xfe720000-0xff50ffff: io map region
*/
#define DMABUF_SIZE		(2*1024*1024)

#define AMPMAP_OFFSET           (2*1024*1024) //extra 8MB gap and 32MB static fix map space for amp yejc
#define AMPMAP_START            (VMALLOC_END + AMPMAP_OFFSET) //0xfc600000
#define AMPMAP_END              0xfe600000UL
#define AMPMAP_SIZE             (AMPMAP_END - AMPMAP_START)
#define AMPPHY_START            0x1E000000UL
#define AMPPHY_END	        0x20000000UL

#define BM_CONTEXT_SIZE	        0x100000UL

//extra 6MB for sram map and io map & gap for amp
#define SRAMMAP_OFFSET          (1024*1024)
#define SRAMMAP_START           (AMPMAP_END + SRAMMAP_OFFSET)
#define SRAMMAP_END             0xfe710000UL
#define SRAMMAP_SIZE            (SRAMMAP_END - SRAMMAP_START)
#define SRAMPHY_START           0xFFFF0000UL

//amp io map start from 0xfeb20000, and the iomap table should be locate in arch/arm/mach-xxx/inclue/mach
#define IOMAP_START             (0xfe720000)
#define IOMAP_END               (0xff510000)
#define IOMAP_SIZE              (IOMAP_END - IOMAP_START)
#define IOPHY_START             (0xff200000)
#define IOMAP_OFFSET            (IOPHY_START - IOMAP_START)

#define AMP_SHARE_PARAM_START   (0xfc700000)
#define AMP_SHARE_BUFFER_START  (0xfc800000)
#define AMP_SHARE_DMABUF_START  (0xfe400000)
#define AMP_SHARE_SRAM_STRART	(0xfe700000)

#undef	NEED_SAVE_RESTORE_GIC_REGS_FROM_BM
#define AMP_GIC_DATA_OFFSET		(2*1024*1024) // 2MB offset for shadow of GIC register

#define SOCFGA_GPIO0_PHY_BASE           0xff708000
#define SOCFGA_GPIO0_VIRT_BASE          (SOCFGA_GPIO0_PHY_BASE - IOMAP_OFFSET)

#define SOCFGA_GPIO1_PHY_BASE           0xff709000
#define SOCFGA_GPIO1_VIRT_BASE          (SOCFGA_GPIO1_PHY_BASE - IOMAP_OFFSET)

#define SOCFGA_GPIO2_PHY_BASE           0xff70a000
#define SOCFGA_GPIO2_VIRT_BASE          (SOCFGA_GPIO2_PHY_BASE - IOMAP_OFFSET)

#define SOCFGA_UART0_PHY_BASE           0xffc02000
#define SOCFGA_UART0_VIRT_BASE          (SOCFGA_UART0_PHY_BASE - IOMAP_OFFSET)

#define RAW_SPIN_LOCK_OBJECT_NR	16

//amp io map start from 0xfeb20000, and the iomap table should be locate in arch/arm/mach-xxx/inclue/mach
enum req_status{
	COMPLETE,
	PENDING
};

struct sh_ioctl_region {
	u32 linux_cmd_dispatcher;
	u32 linux_cmd_args;
	u32 linux_cmd_status; //bit0:1 req in progress or pending, bit0:0 req complete.
	u32 sgi_l2b;
	u32 bm_cmd_dispatcher;
	u32 bm_cmd_args;
	u32 bm_cmd_status; //bit0:1 req in progress or pending, bit0:0 req complete.
	u32 sgi_b2l;
};

struct sh_text_addr {
	u32 printk_fn;

	u32 spinlock_trylock_fn;
	u32 spinlock_lock_fn;
	u32 spinlock_unlock_fn;
	u32 spin_lock_irqsave_fn;
	u32 spin_unlock_irqrestore_fn;

	u32 _down_trylock_fn;
	u32 _up_fn;
};

//should be power of 2
#define LOG_LEN 4096
#define DLOG_LEN 8192

struct amp_share_qspi_boot{
	u32 preloader_qspi_probe_fn;
	u32 preloader_qspi_read_fn;
};

struct amp_share_sd_boot{
	u32 sd_read_bl_len;	/* normally it is 512 */
	u32 preloader_sd_saved_mmc;/*new fixed*/
	u32 preloader_sd_saved_offset;/*new fixed*/
	u32 preloader_sd_read_fn;
};

struct amp_share_param {
	struct sh_ioctl_region sra[16]; //for 16 sgi
	struct sh_text_addr sta;
	struct raw_spinlock rslocks[RAW_SPIN_LOCK_OBJECT_NR];  //spinlock object
	struct semaphore semobj[16];
	u32 logindex;
	u32 logbuf_overlap;
	char logbuf[DLOG_LEN];		//actual 4KB logbuf, 4kB for overlap handle use

	u32 boot_start_stamp;
	u32 boot_end_stamp;
	u32 preloader_wait_bm_load_rbf;
	u32 load_bm_start;
	u32 load_bm_end;
	u32 bm_magic;
	struct amp_share_qspi_boot qspi_bi; /* qspi boot info */
	struct amp_share_sd_boot   sd_bi;	 /* sd boot info */
	struct raw_spinlock *gic_dist_lock;
};

#define DRAM_BUF_SIZE	(0x10000)	//64KB
#define SRAM_BUF_SIZE	(0x8000)	//32KB

#define CPU_DATA_PATERN0	(0x43505530)	//'CPU0'
#define CPU_DATA_PATERN1	(0x30555043)	//'0UPC'
#define CPU_DATA_PATERN2	(0x414d5331)	//'AMS1'
#define CPU_DATA_PATERN3	(0x31534d41)	//'1SMA'


#define CMD_BM_REQ_LINUX_FILL_BUF	0xAA000001
#define CMD_BM_REQ_LINUX_CONSUME_BUF	0xAA000002
#define CMD_LINUX_REQ_BM_FILL_BUF	0xAA000003
#define CMD_LINUX_REQ_BM_CONSUME_BUF	0xAA000004


#define SGI_BM_REQ_LINUX_FILL_BUF	(0xC)
#define SGI_BM_REQ_LINUX_CONSUME_BUF	(0xD)
#define SGI_LINUX_REQ_BM_FILL_BUF	(0xE)
#define SGI_LINUX_REQ_BM_CONSUME_BUF	(0xF)
#define SGI_BRING_OTHER2CACHECOHERENCE	(0x8)

#define CPU0	(0x1)
#define CPU1	(0x2)

extern void gic_raise_softirq(const struct cpumask *mask, unsigned int irq);//yejc
extern struct amp_share_param *asp;

#define CPU_CORE_CACHE_COHERENCE_TEST
#define SPINLOCK_TEST



#define CORE_SHARE_BUFFER_TEST_COUNT	40000 /*yejc*/
#define SPINLOCK_TEST_COUNT		1000000

enum test_sequence{
	eSH_DRAM,
	eSH_SRAM,
	eSPINLOCK
};
















#endif
