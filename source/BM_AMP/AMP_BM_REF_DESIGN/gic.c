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
#include <fpga_regs.h>
#include <io.h>
#include <irq.h>
#include <dma.h>
#include <twd.h>
#include <bmlog.h>
#include <amp_config.h>


extern u32 gINTtestDone;
extern u32 gDMAtestDone;

extern struct amp_share_param *asp;
extern raw_spinlock_fn _raw_spinlock;
extern raw_spinunlock_fn _raw_spinunlock;


static  gic_irq_vect_t  gic_vect_tbl[MAX_IRQ_NR]; 
void gic_Int_cfg (u16 irq, u16 prio, u16 pol);

inline void gic_raise_interrupt(u32 target_cpu, u32 sgi)
{
	GIC_INT_REG_ICDSGIR = (target_cpu << 16) | sgi;
}

#ifdef NEED_SAVE_RESTORE_GIC_REGS_FROM_BM
void gic_dist_save(void)
{
	int i, j, gic_irqs;
	struct bm_gic_chip_data *gic_data = (struct bm_gic_chip_data *)(AMPPHY_START + AMP_GIC_DATA_OFFSET);
	void *base = (void *)GIC_INT_DIST_BASE;

	gic_irqs = gic_data->gic_irqs;
	for (i = 32, j = 0; i < gic_irqs; i += 16, j++)
		gic_data->saved_spi_conf[j] = readl(base + 0xc00 + i * 4 / 16);

	for (i = 32, j = 0; i < gic_irqs; i += 4, j++)
		gic_data->saved_spi_target[j] = readl(base + 0x800 + i * 4 / 4);

	for (i = 32, j = 0; i < gic_irqs; i += 4, j++)
		gic_data->saved_spi_prio[j] = readl(base + 0x400 + i * 4 / 4);

	for (i = 32, j = 0; i < gic_irqs; i += 32, j++)
		gic_data->saved_spi_enable[j] = readl(base + 0x100 + i * 4 / 32);

	//uartprintf("--gic_irqs = %d\r\n", gic_irqs);
}

void gic_dist_restore(void)
{
	int i, gic_irqs;
	void *base = (void *)GIC_INT_DIST_BASE;
	struct bm_gic_chip_data *gic_data = (struct bm_gic_chip_data *)(AMPPHY_START + AMP_GIC_DATA_OFFSET);

	gic_irqs = gic_data->gic_irqs;
	for (i = 32; i < gic_irqs; i += 16)
		 writel(gic_data->saved_spi_conf[i], base + GIC_INT_REG_ICDICFR + i * 4 / 16);

	for (i = 32; i < gic_irqs; i += 4)
		 writel(gic_data->saved_spi_target[i], base + GIC_INT_REG_ICDIPTR + i * 4 / 4);

	for (i = 32; i < gic_irqs; i += 4)
		 writel(gic_data->saved_spi_prio[i], base + GIC_INT_REG_ICDICPR + i * 4 / 4);

	for (i = 32; i < gic_irqs; i += 32)
		 writel(gic_data->saved_spi_enable[i], base + GIC_INT_REG_ICDISER + i * 4 / 32);
}
#endif

void  gic_Int_vect_clr (gic_irq_vect_t  *pvect)
{
    pvect->irqFnPtr = (irq_fn_ptr_t)0;
    pvect->parg  = (void *)0;
}

void  gic_Int_vect_set (gic_irq_vect_t *pvect, irq_fn_ptr_t isr_fn, void *parg)
{
    pvect->irqFnPtr = isr_fn;
    pvect->parg  = parg;     
}

void  gic_Int_cb(gic_irq_vect_t  *pvect)
{
   if (pvect->irqFnPtr != (irq_fn_ptr_t)0)       
   		pvect->irqFnPtr(pvect->parg);
}

void  gic_Int_clr (u16  irq)
{
    GIC_INT_REG_ICCEOIR = irq;
}

void  gic_Int_dis (u16  irq)
{   
    u32  reg;  
    
    reg  =  (u32)&GIC_INT_REG_ICDICER;
	reg += ((irq / 32u) * 4u);

    (*(u32 *)reg) = (1u << (irq & 0x1F));
}

void  gic_Int_dis_all (void)
{
    BITCLR32(GIC_INT_REG_ICDDCR, BIT0);
}

void  gic_Int_en (u16  irq)
{
    u32  reg;
    
    reg  =  (u32)&GIC_INT_REG_ICDISER;
	reg += ((irq / 32u) * 4u);

    (*(u32 *)reg) = (1u << (irq & 0x1F));
}

void gic_Int_cfg_target(u32 irq)
{
	u32     reg, temp,shift;

	reg = ((u32)&GIC_INT_REG_ICDIPTR) + (irq/4)*4;
	temp = *(u32 *)reg;
	shift = (irq & 0x3) << 3;
	temp &= ~(BIT0 << shift);
	temp |= BIT1 << shift;

	(*(u32 *)reg) = temp;
}

/*
*  when linux is runing, gic dist is in a content environment, registers which not support atomic bit
*  operation would need lock mechanism to protect.
*
*
*  exclusive version API
*/
void gic_Int_cfg_target_ex(u32 irq)
{
	u32 	reg, temp,shift;

	_raw_spinlock(asp->gic_dist_lock);
	reg = ((u32)&GIC_INT_REG_ICDIPTR) + (irq/4)*4;
	temp = *(u32 *)reg;
	shift = (irq & 0x3) << 3;
	temp &= ~(BIT0 << shift);
	temp |= BIT1 << shift;
	(*(u32 *)reg) = temp;
	_raw_spinunlock(asp->gic_dist_lock);
}



void  gic_Int_init (void)
{
    u32     reg;
    u32     core_msk;
	u8     i;
    u16     irq;
    gic_irq_vect_t  *pvect;
	u32 flags;
	struct bm_gic_chip_data *gic_data = (struct bm_gic_chip_data *)(AMPPHY_START + AMP_GIC_DATA_OFFSET);
	void *base = (void *)GIC_INT_DIST_BASE;
	u32 gic_irqs;
	
    BITCLR32(GIC_INT_REG_ICDDCR, BIT0);    /* Disable the Global Interrupt Controller.             */
	//uartprintf("********####gic_irqs\r\n");
	gic_irqs = readl(base + 4) & 0x1f;
	gic_irqs = (gic_irqs + 1) * 32;
	//uartprintf("++gic_irqs = %d\r\n", gic_irqs);
	if (gic_irqs > 1020)
		gic_irqs = 1020;
	gic_data->gic_irqs = gic_irqs;

	
	for (i = 0u; i < (gic_irqs / 32u); i++) {        /* Disable all the GIC irq sources.                     */
	    reg  = (u32)&GIC_INT_REG_ICDICER;
 	    reg += (4u + (i * 4u));

        (*(u32 *)reg) = 0xFFFFFFFF;
	  
 	    reg  = (u32)&GIC_INT_REG_ICDICPR;
 	    reg +=  i * 4u;

        //(*(u32 *)reg) = 0xa0a0a0a0;
		(*(u32 *)reg) = 0xFFFFFFFF;
	}

	for (i = 32; i < (gic_irqs / 16); i += 16)
		writel(0, (void *)GIC_INT_DIST_BASE + 0xc00 + i * 4 / 16);
		 
	core_msk = (BIT0 | BIT8 | BIT16 | BIT24); /* bit0~7 for cpu0~cpu8  */
	//core_msk = (BIT1 | BIT9 | BIT17 | BIT25); 

    for (i = 0u; i < ((gic_irqs - 32u) / 4u); i++) {
		reg  = (u32)&GIC_INT_REG_ICDIPTR;    //target
 		reg += (((32u / 4u) + i) * 4u);

        (*(u32 *)reg) = core_msk;	

	  	reg  = (u32)&GIC_INT_REG_ICDIPR;     //priority
 	    reg += (((32u / 4u) + i) * 4u);

        (*(u32 *)reg) = 0xa0a0a0a0;	
	}

	gic_Int_cfg_target(GIC_DMA0);
	gic_Int_cfg_target(GIC_PFGA0);


	gic_Int_cfg(GIC_PFGA0, 0x80, GIC_INT_POL_EDGE_RISING);
                                                                /* Initialize the vector table. */
    for (irq = 0u; irq < MAX_IRQ_NR; irq++) {
        pvect = &gic_vect_tbl[irq];

        flags = cpu_local_irq_save();
        gic_Int_vect_clr(pvect);                             /* Initialize main vector table entry. */
        cpu_local_irq_restore(flags);
    }
	
    BITCLR32(GIC_INT_REG_ICCPMR, 0xffff);	 
    BITSET32(GIC_INT_REG_ICCPMR, BIT4|BIT5|BIT6|BIT7);    /* Set priority mask. */  
    BITSET32(GIC_INT_REG_ICCICR, BIT0);    /* Enable CPU interface.  */
	BITSET32(GIC_INT_REG_ICDDCR, BIT0);    /* Enable the Global Interrupt Controller.  */ 
}

void gic_Int_cfg(u16 irq, u16 prio, u16 pol)
{
    u32  reg;  

    reg  =  (u32)&GIC_INT_REG_ICDIPR;        /* Configure interrupt priority. */
	reg +=  (irq / 4u *4);

	*(u32 *) reg &=~(0xFF<<((irq%4)*8));
	*(u32 *) reg |= prio<<((irq%4)*8);
	 
    reg  =  (u32)&GIC_INT_REG_ICDICFR;                   
    reg +=  (irq / 16u);
	
	if (pol == GIC_INT_POL_LEVEL_HIGH || pol == GIC_INT_POL_LEVEL_LOW) { 
	    BITCLR32((*(u32 *)reg), (1u << (((irq % 16u) * 2u) + 1u)));
	} else {
	    BITSET32((*(u32 *)reg), (1u << (((irq % 16u) * 2u) + 1u))); 
	}
}

/*
*  when linux is runing, gic dist is in a content environment, registers which not support atomic bit
*  operation would need lock mechanism to protect.
*
*
*  exclusive version API
*/
void  gic_Int_cfg_ex(u16 irq, u16 prio, u16 pol)
{
    u32  reg;  

	_raw_spinlock(asp->gic_dist_lock);
    reg  =  (u32)&GIC_INT_REG_ICDIPR;        /* Configure interrupt priority. */
	reg +=  (irq / 4u *4);

	*(u32 *) reg &=~(0xFF<<((irq%4)*8));
	*(u32 *) reg |= prio<<((irq%4)*8);
	 
    reg  =  (u32)&GIC_INT_REG_ICDICFR;                   
    reg +=  (irq / 16u);
	
	if (pol == GIC_INT_POL_LEVEL_HIGH || pol == GIC_INT_POL_LEVEL_LOW) { 
	    BITCLR32((*(u32 *)reg), (1u << (((irq % 16u) * 2u) + 1u)));
	} else {
	    BITSET32((*(u32 *)reg), (1u << (((irq % 16u) * 2u) + 1u))); 
	}
	_raw_spinunlock(asp->gic_dist_lock);

}


void  gic_Int_vect_reg (u16 irq, irq_fn_ptr_t isr_fn, void *parg)
{
    gic_irq_vect_t  *pvect;
    u32 flags;
	
    pvect = (gic_irq_vect_t  *)&gic_vect_tbl[irq];
	
    flags = cpu_local_irq_save();

    gic_Int_vect_set((gic_irq_vect_t *)pvect,
                   (irq_fn_ptr_t  )isr_fn,
                   (void *        )parg);

    cpu_local_irq_restore(flags);
    
}

void  gic_Int_vect_unreg (u16  irq)
{
    gic_irq_vect_t  *pvect;
    u32 flags;

    pvect = (gic_irq_vect_t  *)&gic_vect_tbl[irq];

    flags = cpu_local_irq_save();
    gic_Int_vect_clr(pvect);
    cpu_local_irq_restore(flags);

}

void except_handler  (u8  ex_mode)
{
    switch (ex_mode) {
        case CPU_ARM_EXCEPT_RST:
        case CPU_ARM_EXCEPT_UND:
        case CPU_ARM_EXCEPT_SWI:
        case CPU_ARM_EXCEPT_ABORT_DATA:	 
        case CPU_ARM_EXCEPT_ABORT_PREFETCH:
             while (DEF_TRUE) {
                 ;
             }
    }
}

extern u32 *p_pvt;
extern u32 *p_fpga_dt;
extern u32 gFPGA_IRQ_req;
extern u32 gARM_IRQ_ack;

u32  identify_and_clear_source  (void)
{
    u32    irq;
	static u32 index = 0;
    register u32 dt;

        
    irq = GIC_INT_REG_ICCIAR;
    irq = irq & 0x000003FF;

   if(irq == GIC_PFGA0)
   {
	dt = pvt_read();
	p_pvt[index] = dt;

	//clear fpga irq0, let fpga deassert fpga irq0
	// wr 1 to ack irq ;
	writel(0x1, (void *)FPGA_INT_LATENCY_IP_ACK);        //wr 1 to ack irq ;
	readl((const volatile void *)FPGA_INT_LATENCY_IP_CYCLE_COUNT);

	index++;
	if(index > IL_TEST_COUNT)
	{
		gFPGA_IRQ_req = readl((const volatile void *)FPGA_INT_LATENCY_IP_REQ_COUNT);
		gARM_IRQ_ack = readl((const volatile void *)FPGA_INT_LATENCY_IP_ACK_COUNT);
		writel(0x1, (void *)FPGA_INT_LATENCY_IP_SOFT_RESET); //clear counter, dessert irq also
		gic_Int_dis(GIC_PFGA0);
		pvt_stop();
		index = 0;
		ACCESS_ONCE(gINTtestDone) = 1;
	}
    }
   else if(irq == GIC_DMA0)
   {
		//only level trigger, so we temporary mask this channel irq,
		//or it would cause serverl interrupt again when we re-enable irq
		//even we have clear dma irq request source here
		alt_dma_int_clear(ALT_DMA_EVENT_0);
		//alt_dma_int_clear(ALT_DMA_EVENT_0);
		//alt_dma_int_clear(ALT_DMA_EVENT_0);
		//alt_dma_int_clear(ALT_DMA_EVENT_0);
		//alt_dma_int_clear(ALT_DMA_EVENT_0);
		gic_Int_dis(GIC_DMA0);
   }

	//bmlog("irq =0x%x~~~\n", irq);
	gic_Int_clr(irq);
	return irq;
}

void C_irq_handler(u32 irq_nr)
{
	gic_Int_cb(&gic_vect_tbl[irq_nr]);
}

