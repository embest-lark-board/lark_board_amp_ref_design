#ifndef __IO_H__
#define __IO_H__

#include <types.h>


static inline u32 readl(const volatile void *addr)
{
	return *(const volatile u32 *) addr;
}


static inline void writel(u32 val, volatile void *addr)
{
	*(volatile u32 *) addr = val;
}


#endif
