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


#ifndef ARMV7_IRQ_H
#define ARMV7_IRQ_H

#define PSR_F_BIT 0x00000040
#define PSR_I_BIT 0x00000080

static inline unsigned long cpu_local_irq_save(void)
{
        unsigned long flags;

        asm volatile(
                "       mrs     %0, cpsr        @ cpu_local_irq_save\n"
                "       cpsid   i"
                : "=r" (flags) : : "memory", "cc");
        return flags;
}

static inline void cpu_local_irq_enable(void)
{
        asm volatile(
                "       cpsie i                 @ cpu_local_irq_enable"
                :
                :
                : "memory", "cc");
}

static inline void cpu_local_irq_disable(void)
{
        asm volatile(
                "       cpsid i                 @ cpu_local_irq_disable"
                :
                :
                : "memory", "cc");
}

#define local_fiq_enable()  __asm__("cpsie f    @ __stf" : : : "memory", "cc")
#define local_fiq_disable() __asm__("cpsid f    @ __clf" : : : "memory", "cc")

/*
 * Save the current interrupt enable state.
 */
static inline unsigned long cpu_local_save_flags(void)
{
        unsigned long flags;
        asm volatile(
                "       mrs     %0, cpsr        @ local_save_flags"
                : "=r" (flags) : : "memory", "cc");
        return flags;
}

/*
 * restore saved IRQ & FIQ state
 */
static inline void cpu_local_irq_restore(unsigned long flags)
{
        asm volatile(
                "       msr     cpsr_c, %0      @ local_irq_restore"
                :
                : "r" (flags)
                : "memory", "cc");
}

static inline int cpu_irqs_disabled_flags(unsigned long flags)
{
        return flags & PSR_I_BIT;
}



#define  CPU_ARM_EXCEPT_RST                               0u
#define  CPU_ARM_EXCEPT_UND                               1u
#define  CPU_ARM_EXCEPT_SWI                               2u
#define  CPU_ARM_EXCEPT_ABORT_PREFETCH                    3u
#define  CPU_ARM_EXCEPT_ABORT_DATA                        4u
#define  CPU_ARM_EXCEPT_RSVD                              5u
#define  CPU_ARM_EXCEPT_IRQ                               6u
#define  CPU_ARM_EXCEPT_FIQ                               7u

#define GIC_INT_POL_LEVEL_HIGH                 (u16)(0u)    /* High level polarity.                                 */
#define GIC_INT_POL_LEVEL_LOW                  (u16)(1u)    /* Low  level polarity.                                 */
#define GIC_INT_POL_EDGE_RISING                (u16)(2u)    /* Rising edge polarity.                                */
#define GIC_INT_POL_EDGE_POSITIVE              (u16)(2u)    
#define GIC_INT_POL_EDGE_FALLING               (u16)(3u)    /* Falling edge polarity.                               */
#define GIC_INT_POL_EDGE_NEGATIVE              (u16)(3u)  
#define GIC_INT_POL_EDGE_BOTH                  (u16)(4u)    /* Edge positive & negative polarity.                   */
#define GIC_INT_POL_NONE                       (u16)(5u)


#define  MAX_IRQ_NR                           (1020u)

typedef            void      (*irq_fn_ptr_t )(void *parg);
typedef  struct  gic_irq_vect {    
    irq_fn_ptr_t   irqFnPtr;                      
    void          *parg;                               
} gic_irq_vect_t;

#define	DIV_ROUND_UP(x,y)	(((x) + ((y) - 1)) / (y))

struct bm_gic_chip_data {
	u32 saved_spi_enable[DIV_ROUND_UP(MAX_IRQ_NR, 32)];
	u32 saved_spi_conf[DIV_ROUND_UP(MAX_IRQ_NR, 16)];
	u32 saved_spi_prio[DIV_ROUND_UP(MAX_IRQ_NR, 4)];
	u32 saved_spi_target[DIV_ROUND_UP(MAX_IRQ_NR, 4)];
	unsigned int gic_irqs;
};

inline void gic_raise_interrupt(u32 target_cpu, u32 sgi);
void  gic_Int_init (void);
void  gic_Int_dis (u16  irq);
void  gic_Int_clr (u16  irq);
void  gic_Int_en (u16  irq);
void  gic_Int_vect_reg (u16 irq, irq_fn_ptr_t isr_fn, void *parg);
void  gic_Int_cfg (u16 irq, u16 prio, u16 pol);
void gic_dist_save(void);
/*
*  when linux is runing, gic dist is in a content environment, registers which not support atomic bit
*  operation would need lock mechanism to protect.
*
*
*  exclusive version API
*/
void gic_Int_cfg_target_ex(u32 irq);
void gic_Int_cfg_ex(u16 irq, u16 prio, u16 pol);



#define  GIC_SGI0						   (u16)( 0u)
#define  GIC_SGI1						   (u16)( 1u)
#define  GIC_SGI2						   (u16)( 2u)
#define  GIC_SGI3						   (u16)( 3u)
#define  GIC_SGI4						   (u16)( 4u)
#define  GIC_SGI5						   (u16)( 5u)
#define  GIC_SGI6						   (u16)( 6u)
#define  GIC_SGI7						   (u16)( 7u)
#define  GIC_SGI8						   (u16)( 8u)
#define  GIC_SGI9						   (u16)( 9u)
#define  GIC_SGI10						   (u16)( 10u)
#define  GIC_SGI11						   (u16)( 11u)
#define  GIC_SGI12						   (u16)( 12u)
#define  GIC_SGI13						   (u16)( 13u)
#define  GIC_SGI14						   (u16)( 14u)
#define  GIC_SGI15						   (u16)( 15u)

#define	GIC_DMA0						   (u16)( 136u)
#define	GIC_DMA1						   (u16)( 137u)
#define	GIC_DMA2						   (u16)( 138u)
#define	GIC_DMA3						   (u16)( 139u)
#define	GIC_DMA4						   (u16)( 140u)
#define	GIC_DMA5						   (u16)( 141u)
#define	GIC_DMA6						   (u16)( 142u)
#define	GIC_DMA7						   (u16)( 143u)

#define  GIC_PFGA0						   (u16)( 72u)


#endif
