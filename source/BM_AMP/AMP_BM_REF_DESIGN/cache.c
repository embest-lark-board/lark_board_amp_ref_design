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


#include <types.h>
#include <regs.h>
#include <l2cache.h>
#include <io.h>
#include <mmap.h>
#include <amp_config.h>
#include <bmlog.h>


void L1DCacheFlush(void)
{
	register u32 Csid, C7;
	u32 CacheSize, NrWays, LineSize;
	u32 WayIndex, Way, NrSet, SetIndex, Set;

	/*
	*************************************************************
	* cortex-a9 are hierachical cache system, select cache level & D cache***
	*************************************************************
	*/
	__asm__ __volatile__("mcr p15, 2, %0, c0, c0, 0\n"
						: : "r"(0): );
	
	__asm__ __volatile__("isb\n");

	__asm__ __volatile__("mrc p15, 1, %0,  c0,  c0, 0\n"
						: "=r" (Csid):: );


	/*
	*************************************************************
	* extract cache infomation by CSID register                                     ***
	*************************************************************
	*/

	CacheSize = (Csid >> 13) & 0x1FF;
	CacheSize += 1;
	CacheSize = CacheSize<<7;  

	NrWays = (Csid & 0x3ff) >> 3;
	NrWays += 1;

	LineSize = (Csid & 0x07) + 4;

	NrSet = CacheSize/NrWays;
	NrSet /= (1 << LineSize);

	Set = 0x0;
	Way = 0x0;
	
	//bmlog("L1 Dcache Sizes %dbytes, %d Ways, CacheLineSize %d\n", 
	//					CacheSize, NrWays, 1<<LineSize);
	for (WayIndex = 0; WayIndex < NrWays; WayIndex++)
	{
		for (SetIndex = 0; SetIndex < NrSet; SetIndex++)
		{
			C7 = Way | Set;
			__asm__ __volatile__("mcr p15, 0, %0,  c7, c14, 2\n"
			 					:: "r" (C7));
			Set += (1 << LineSize);
		}
		Way += 0x40000000;
	}

	/* flush L1 STB */
	__asm__ __volatile__("dsb\n");
}

void L1DCacheInvalid(void)
{
	register u32 Csid, C7;
	u32 CacheSize, NrWays, LineSize;
	u32 WayIndex, Way, NrSet, SetIndex, Set;

	/*
	*************************************************************
	* cortex-a9 are hierachical cache system, select cache level & D cache***
	*************************************************************
	*/
	__asm__ __volatile__("mcr p15, 2, %0, c0, c0, 0\n"
						: : "r"(0): );
	
	__asm__ __volatile__("isb\n");

	__asm__ __volatile__("mrc p15, 1, %0,  c0,  c0, 0\n"
						: "=r" (Csid):: );


	/*
	*************************************************************
	* extract cache infomation by CSID regisger                                     ***
	*************************************************************
	*/

	CacheSize = (Csid >> 13) & 0x1FF;
	CacheSize += 1;
	CacheSize = CacheSize<<7;  

	NrWays = (Csid & 0x3ff) >> 3;
	NrWays += 1;

	LineSize = (Csid & 0x07) + 4;

	NrSet = CacheSize/NrWays;
	NrSet /= (1 << LineSize);

	Set = 0x0;
	Way = 0x0;
	
	//bmlog("L1 Dcache Sizes %dbytes, %d Ways, CacheLineSize %d\n", 
	//					CacheSize, NrWays, 1<<LineSize);
	for (WayIndex = 0; WayIndex < NrWays; WayIndex++)
	{
		for (SetIndex = 0; SetIndex < NrSet; SetIndex++)
		{
			C7 = Way | Set;
			__asm__ __volatile__("mcr p15, 0, %0,  c7,  c6, 2\n"
			 					:: "r" (C7));
			Set += (1 << LineSize);
		}
		Way += 0x40000000;
	}

	/* flush L1 STB */
	__asm__ __volatile__("dsb\n");
}


void L1ICacheInvalid(void)
{
	__asm__ __volatile__("mcr p15, 2, %0, c0, c0, 0\n"
						: : "r"(1): );
	__asm__ __volatile__("mcr p15, 0, %0, c7, c5, 0\n"
						: : "r"(0): );
	__asm__ __volatile__("dsb\n");
}

void L1DCacheDisable(void)
{
	register u32 Ctrl;

	L1DCacheFlush();


	__asm__ __volatile__("mrc p15, 0, %0, c1, c0, 0\n"
						: "=r" (Ctrl):: );

	Ctrl &= ~0x4;

	__asm__ __volatile__("mcr p15, 0, %0, c1, c0, 0\n"
						: : "r"(Ctrl): );
	__asm__ __volatile__("dsb\n");
}

void L1DCacheEnable(void)
{
	register u32 Ctrl;

	__asm__ __volatile__("mrc p15, 0, %0, c1, c0, 0\n"
						: "=r" (Ctrl):: );
	
	if (Ctrl & 0x4)
	{
		return;
	}

	L1DCacheInvalid();

	Ctrl |= 0x4;

	__asm__ __volatile__("mcr p15, 0, %0, c1, c0, 0\n"
						: : "r"(Ctrl): );
	__asm__ __volatile__("dsb\n");
}


void L1ICacheDisable(void)
{
	register u32 Ctrl;

	__asm__ __volatile__("dsb\n");

	__asm__ __volatile__("mcr p15, 0, %0, c7, c5, 0\n"
						: : "r"(0): );

	__asm__ __volatile__("mrc p15, 0, %0, c1, c0, 0\n"
						: "=r" (Ctrl):: );

	Ctrl &= ~0x1000;

	__asm__ __volatile__("mcr p15, 0, %0, c1, c0, 0\n"
						: : "r"(Ctrl): );
}

void L1ICacheEnable(void)
{
	register u32 Ctrl;

	__asm__ __volatile__("mrc p15, 0, %0, c1, c0, 0\n"
						: "=r" (Ctrl):: );
	
	if(Ctrl & 0x1000)
	{
		return;
	}

	__asm__ __volatile__("mcr p15, 0, %0, c7, c5, 0\n"
						: : "r"(0): );

	Ctrl |= 0x1000;

	__asm__ __volatile__("mcr p15, 0, %0, c1, c0, 0\n"
						: : "r"(Ctrl): );
}


void BML2CacheFlush(void)
{
	register unsigned int L2Ctrl;

	/*
	********************************************************************
	* flush BM L2 cache ways, don't touch linux cache ways                                  ***
	********************************************************************
	*/
	writel(0xFFAA, (void *)SOCFPGA_MPUL2_ADDRESS + L2X0_CLEAN_INV_WAY);

	//Wait ways finish
	do
	{
		L2Ctrl = readl((const volatile void *)SOCFPGA_MPUL2_ADDRESS + L2X0_CACHE_SYNC);
	} while (L2Ctrl != 0);

	__asm__ __volatile__("dsb\n");
}

extern void LockCodeData(u32 addr, u32 size);

void LockL2Ways(void)
{

	L1DCacheFlush();
	L1ICacheInvalid();
	L1DCacheDisable();
	L1ICacheDisable();
	//it would be better to hold the linux l2x0 spinlock to do L2 cache maintainment
	//spinlock(asp->l2x0);
	BML2CacheFlush();
	LockCodeData(SDRAM_VIRT_BASE, 0x20000);
	//spinunlock(asp->l2x0);
	L1ICacheEnable();
	L1DCacheEnable();
	bmlog("BM L2 Lock ways status:D:0x%08x I:0x%08x\n", 
	readl((const volatile void *)SOCFPGA_MPUL2_ADDRESS + L2X0_LOCKDOWN_WAY_D_BASE + 8),
	readl((const volatile void *)SOCFPGA_MPUL2_ADDRESS + L2X0_LOCKDOWN_WAY_I_BASE + 8));
}


