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


#ifndef BM_TYPES_H
#define BM_TYPES_H

typedef unsigned char u8;
typedef unsigned int u32;
typedef unsigned short u16;
typedef unsigned long long u64;

typedef char s8;
typedef int s32;
typedef short s16;
typedef long long s64;

typedef u32 ulong;

//typedef void *caddr_t;
typedef int bool;

#define false			(0)
#define true			(1)

#define NULL			(0)

#define ACCESS_ONCE(x) (*(volatile typeof(x) *)&(x))


//we are in little endian
#undef __ARMEB__

typedef struct {
	union {
		u32 slock;
		struct __raw_tickets {
#ifdef __ARMEB__
			u16 next;
			u16 owner;
#else
			u16 owner;
			u16 next;
#endif
		} tickets;
	};
} arch_spinlock_t;


typedef struct raw_spinlock {
	arch_spinlock_t raw_lock;
} raw_spinlock_t;

struct list_head {
	struct list_head *next, *prev;
};

/* Please don't access any members of this structure directly */
struct semaphore {
	raw_spinlock_t		lock;
	unsigned int		count;
	struct list_head	wait_list;
};

typedef int (*raw_spinlock_fn)(raw_spinlock_t *lock);
typedef int (*raw_spinunlock_fn)(raw_spinlock_t *lock);
typedef int (*printk_fn)(const char *fmt, ...);

typedef int (*down_trylock_fn)(struct semaphore *sem);



extern void * memset_int(void *p, int val, int size);
extern void ledblink(void);
extern void ledblink1(void);


typedef struct {
	int counter;
} atomic_t;

#define sev()   __asm__ __volatile__ ("sev" : : : "memory")
#define wfe()   __asm__ __volatile__ ("wfe" : : : "memory")
#define wfi()   __asm__ __volatile__ ("wfi" : : : "memory")


#define isb() __asm__ __volatile__ ("isb" : : : "memory")
#define dsb() __asm__ __volatile__ ("dsb" : : : "memory")
#define dmb() __asm__ __volatile__ ("dmb" : : : "memory")

#define smp_mb()        dmb()
#define smp_rmb()       dmb()
#define smp_wmb()       dmb()


#define DEF_TRUE 1
#define DEF_FAIL 0
#define DEF_OK	1

#define  BITN       0x00u

#define  BIT0       0x01u
#define  BIT1       0x02u
#define  BIT2       0x04u
#define  BIT3       0x08u
#define  BIT4       0x10u
#define  BIT5       0x20u
#define  BIT6       0x40u
#define  BIT7       0x80u

#define  BIT8       0x0100u
#define  BIT9       0x0200u
#define  BIT10      0x0400u
#define  BIT11      0x0800u
#define  BIT12      0x1000u
#define  BIT13      0x2000u
#define  BIT14      0x4000u
#define  BIT15      0x8000u

#define  BIT16      0x00010000u
#define  BIT17      0x00020000u
#define  BIT18      0x00040000u
#define  BIT19      0x00080000u
#define  BIT20      0x00100000u
#define  BIT21      0x00200000u
#define  BIT22      0x00400000u
#define  BIT23      0x00800000u

#define  BIT24      0x01000000u
#define  BIT25      0x02000000u
#define  BIT26      0x04000000u
#define  BIT27      0x08000000u
#define  BIT28      0x10000000u
#define  BIT29      0x20000000u
#define  BIT30      0x40000000u
#define  BIT31      0x80000000u


#define  BITSET32(val, mask)                     ((val) = (u32)(((u32)(val)) | ((u32) (mask))))
#define  BITCLR32(val, mask)                     ((val) = (u32)(((u32)(val)) & ((u32)~(mask))))

#define setbits_le32(addr, mask)				 *(volatile u32 *)(addr) |= ((u32) (mask))
#define clrbits_le32(addr, mask)				 *(volatile u32 *)(addr) &= ((u32)~(mask))

/*!
 * This type definition enumerates the list of UARTs available on the system.
 */
typedef enum ALT_PERIPH_FPGA_DEVICE_e
{

    ALT_PERIPH_FPGA_0 = 0,

    ALT_PERIPH_FPGA_1 = 1,

	ALT_PERIPH_FPGA_2,
	ALT_PERIPH_FPGA_3,
	ALT_PERIPH_FPGA_4,
	ALT_PERIPH_FPGA_5,
	ALT_PERIPH_FPGA_6,
	ALT_PERIPH_FPGA_7,

    ALT_PERIPH_FPGA_MAX
}
ALT_PERIPH_FPGA_0_DEVICE_t;

/*!
 * This structure is used to represent a handle to a specific UART on the
 * system. The internal members are undocumented and should be not altered
 * outside of this API.
 */
typedef struct ALT_PERIPH_FPGA_HANDLE_s
{
    ALT_PERIPH_FPGA_0_DEVICE_t device;
    void *             location;
    u32         clock_freq;
    u32           data;
    u32           fcr;
}
ALT_PERIPH_FPGA_HANDLE_t;


#endif
