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



#ifndef MMAP_H
#define MMAP_H


#include <types.h>


#define SDRAM_VIRT_BASE              0x1E000000UL
#define SDRAM_PHYS_BASE              0x1E000000UL
#define SDRAM_SIZE                   (32 * 1024 * 1024)

#define FPGA_SDRAM_VIRT_BASE         0x20000000UL
#define FPGA_SDRAM_PHYS_BASE         0xE0000000UL
#define FPGA_SDRAM_SIZE              (256 * 1024 * 1024)

#define HPS_SRAM_PHYS_BASE			0xFFFF0000UL
#define HPS_SRAM_SIZE				0x10000UL

#define FPGA_SRAM_PHYS_BASE			0xC0000000UL
#define FPGA_SRAM_SIZE				0x4000UL



#define MEM_STRONGLY_ORDERED		0x0
#define MEM_SHARE_DEVICE			0x1
#define MEM_OIWB_NWA				0x2
#define MEM_OIWT_NWA				0x3
#define MEM_OINC					0x4
#define MEM_OIWB_WA					0x7
#define MEM_NON_SHARE_DEVICE		0x8
#define MEM_CACHE_TYPE(outer, inner)	(0x10 | ((outer)<<2) | (inner))

#define MEM_NC				0x0
#define MEM_WBWA			0x1
#define MEM_WTNWA			0x2
#define MEM_WBNWA			0x3

#define AP_PNA_UNA			0x0
#define AP_PRW_UNA			0x1
#define AP_PRW_URO			0x2
#define AP_PRW_URW			0x3
#define AP_PRO_UNA			0x5
#define AP_PRO_URO			0x7

#define PGTL_L1_SECT_SIZE           0x100000

#define PGTL_L1_SECT_ENTRY(addr, mem_type , xn, ns, dom, ap, s, ng) \
	(((addr) & ~((1<< 20)-1)) | (((ns) & 1)<<19) | (((ng) & 1)<<17) | \
	(((s) & 1)<<16) | (((ap) & 0x4)<<15) | (((mem_type) & 0x1c)<<10) | \
	(((ap) & 3)<< 10) | (((dom) & 0xf)<<5) | (((mem_type) & 3)<<2) | 0x2)

/******* PGTL_L1_SECT_WBWA_ENTRY is use to map wbwa share memeroy region *********/
#define PGTL_L1_SECT_WBWA_ENTRY(addr,  xn, ns, dom, ap, s, ng) \
	(((addr) & ~((1<< 20)-1)) | (((ns) & 1)<<19) | (((ng) & 1)<<17) | \
	(((s) & 1)<<16) | (((ap) & 0x4)<<15) | ((0x1)<<12) | \
	(((ap) & 3)<< 10) | (((dom) & 0xf)<<5) | ((0x3)<<2) | 0x2)


#define PHYS_TO_VIRT(addr) \
	((u32)(addr) - (SDRAM_PHYS_BASE) + (SDRAM_VIRT_BASE))

#define VIRT_TO_PHYS(addr) \
        ((u32)(addr) - (SDRAM_VIRT_BASE) + (SDRAM_PHYS_BASE))

#define VADDR_TO_L1_INDEX(addr) ((addr)>> 20)

#endif 
