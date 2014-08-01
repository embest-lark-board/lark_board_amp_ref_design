#ifndef __ASM_ARM_HARDWARE_L2X0_H
#define __ASM_ARM_HARDWARE_L2X0_H


#include <types.h>


#define L2X0_CACHE_SYNC			0x730
#define L2X0_DUMMY_REG			0x740
#define L2X0_INV_LINE_PA		0x770
#define L2X0_INV_WAY			0x77C
#define L2X0_CLEAN_LINE_PA		0x7B0
#define L2X0_CLEAN_LINE_IDX		0x7B8
#define L2X0_CLEAN_WAY			0x7BC
#define L2X0_CLEAN_INV_LINE_PA	0x7F0
#define L2X0_CLEAN_INV_LINE_IDX	0x7F8
#define L2X0_CLEAN_INV_WAY		0x7FC


#define L2X0_LOCKDOWN_WAY_D_BASE	0x900
#define L2X0_LOCKDOWN_WAY_I_BASE	0x904


#endif

