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
#include <twd.h>


#define  PVT_TMR_BASE				(u32   )0xFFFEC600

#define  PVT_TMR_LOAD			(*(volatile u32 *)(PVT_TMR_BASE + 0x00))
#define  PVT_TMR_COUNTER		(*(volatile u32 *)(PVT_TMR_BASE + 0x04))
#define  PVT_TMR_CONTROL		(*(volatile u32 *)(PVT_TMR_BASE + 0x08))
#define  PVT_TMR_INT_STATUS		(*(volatile u32 *)(PVT_TMR_BASE + 0x0C))

s8  pvt_init(u8 div)
{
	//u32  load_val;

	//per_clk = 800000000/4;
			
	PVT_TMR_CONTROL    = div<<8;     //Disable timer
			                         //Clear interrupt flag
	PVT_TMR_INT_STATUS = BIT0;

	PVT_TMR_COUNTER = 0xFFFFFFFF;	//counter init val
			
	PVT_TMR_LOAD    = 0xFFFFFFFF; //reload val

	BITSET32(PVT_TMR_CONTROL, BIT1);
			
    return true;
}

void  pvt_start (void)
{
	BITSET32(PVT_TMR_CONTROL, BIT0);
}

void  pvt_stop (void)
{
	BITCLR32(PVT_TMR_CONTROL, BIT0);
}

inline u32 pvt_read(void)
{
	return	PVT_TMR_COUNTER;
}
