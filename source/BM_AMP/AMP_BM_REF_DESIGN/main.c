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


#include <bm.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <dma.h>
#include <twd.h>
#include <bmlog.h>
#include <iic_bitbang.h>
#include <io.h>
#include <fpga_manager.h>
#include <qspi.h>
#include <boot_mode.h>
#include <sd_load.h>


//#define UARTDEBUG
#ifdef UARTDEBUG
#define UART_DEBUG(format...) uartprintf(format)
#else
#define UART_DEBUG(format...) 
#endif

#ifdef	LCD1602_DISP
static	char cDispBuf[2][16];
#endif
volatile u32 gCacheCoherence = 0;
struct amp_share_param *asp = (struct amp_share_param *)SH_MEM_START;
volatile int sgi15task_pending = 0;
typedef struct
{
	u8 *	src_mpu;
	u8 *   	dst_mpu;
	u32		size;
}dma_irq_info;

u32 gINTtestDone = 0;
volatile dma_irq_info	gDMAtestIrqInfoChan0;
u32 gDMAtestDone = 0;
u32 gDMAtestACPDone = 0;
u32 gFPGA_IRQ_req = 0;
u32 gARM_IRQ_ack = 0;
u32 *p_pvt = (u32 *)(SH_MEM_START + 0x300000); //offset 3MB
u32 *p_fpga_dt = (u32 *)(SH_MEM_START + 0x700000);				////offset 7MB


extern u32  PageTable[];

raw_spinlock_fn _raw_spinlock = NULL;
raw_spinunlock_fn _raw_spinunlock = NULL;

down_trylock_fn down_trylock = NULL;

//int dincr, dheap_end;
//we found that linux printk depends on the thread_info, and it get the theadinfo address 
//from the sp(r13) register, and bare metal has not such thread_info struct, it will corruct.
//so we can't use  the printk function now, and please use the bmlog function.
//would cause lock acquire and irq disable&enbale impace realtime performace
//use bm_log instead to enhance realtime performace
printk_fn printk = NULL;


extern void lock_code_data_section_to_L2(u32 start, u32 size);
typedef void (*l2lock_fn)(u32 start, u32 size);
l2lock_fn l2lock = NULL;


/*led function: it must depend the preloader pin mux */
static inline void led_direction_output(u32 led_base_addr, u32 pin_bit)
{
	*((volatile u32 *)(led_base_addr + 4)) |= (0x1 << (pin_bit));
}

/* it will according to your hardware design */
static inline void led_on_by_low_level(u32 led_base_addr, u32 pin_bit)
{
	led_direction_output(led_base_addr, pin_bit);
	*((volatile u32 *)(led_base_addr)) &= ~(0x1 << (pin_bit));
}

static inline void led_on_by_high_level(u32 led_base_addr, u32 pin_bit)
{
	led_direction_output(led_base_addr, pin_bit);
	*((volatile u32 *)(led_base_addr)) |= (0x1 << (pin_bit));
}

/* it will toggle the led status */
static inline void led_blink(u32 led_base_addr, u32 pin_bit)
{
	led_direction_output(led_base_addr, pin_bit);
	*((volatile u32 *)led_base_addr) ^= (0x1 << (pin_bit));
}

/* for lark board */
#define LDE27_SET_OUTPUT()	led_direction_output(HPS_GPIO1_BASE_ADDR, 24)
#define LED27_ON()			led_on_by_low_level(HPS_GPIO1_BASE_ADDR, 24)
#define LED27_BLINK()		led_blink(HPS_GPIO1_BASE_ADDR, 24)

#define LDE28_SET_OUTPUT()	led_direction_output(HPS_GPIO1_BASE_ADDR, 25)
#define LED28_ON()			led_on_by_low_level(HPS_GPIO1_BASE_ADDR, 25)
#define LED28_BLINK()		led_blink(HPS_GPIO1_BASE_ADDR, 25)

#define LDE30_SET_OUTPUT()	led_direction_output(HPS_GPIO2_BASE_ADDR, 4)
#define LED30_ON()			led_on_by_low_level(HPS_GPIO2_BASE_ADDR, 4)
#define LED30_BLINK()		led_blink(HPS_GPIO2_BASE_ADDR ,4)



void delayN1(void)
{
	int i = 0xF000;
	while(i>0) i--;
}

void delayN(void)
{
	int i = 0x7FFFFFFF;
	while(i>0) i--;
}


void  gic_sgi0to7_handler (void  *p_arg)
{
	bmlog("BM SGI#%d triggered!\n", (u32 *)p_arg);

}

void  gic_sgi8_handler (void  *p_arg)
{
	/*
	************************************************************************
	***** we are in SVC mode and interrupt enable in the ISR_Handler ****************
	***** then we can service other high priority IRQ when do current ISR_Handler ******
	************************************************************************
	*/
	
	/********** ISR Top Half  begin ************/
	/***************************************/
	/* place ur emerency TH bussiness here  ******/
	/********** ISR Top Half  begin ************/
	
	gCacheCoherence ++;
	if(gCacheCoherence == 2)
	{
		bmlog("lock_code_data_section_to_L2=0x%08x\n", lock_code_data_section_to_L2);
		memcpy((void *)AMP_SHARE_DMABUF_START, (void *)SDRAM_PHYS_BASE, 0x20000);
		/*
		********************************************************************************
		* lock code & data section to L2 cache function need to be run in non-cache memory region   ***
		********************************************************************************
		*/
		__asm__ __volatile__("dsb\n");
		l2lock = (l2lock_fn)(((u32)lock_code_data_section_to_L2) + (AMP_SHARE_DMABUF_START - AMPMAP_START));
		l2lock(SDRAM_PHYS_BASE, 0x20000);
		ACCESS_ONCE(*(u32 *)(AMP_SHARE_DMABUF_START + 0x100000)) = (('L'<<24) | ('O'<<16) | ('C'<<8)| 'K');

	}
	/********** Set Flags to indecate while{} loop block do the Buttom Half  ************/
	
         
}

void  gic_sgi15_handler (void  *p_arg)
{
	/*
	************************************************************************
	***** we are in SVC mode and interrupt enable in the ISR_Handler ****************
	***** then we can service other high priority IRQ when do current ISR_Handler ******
	************************************************************************
	*/


	/********** ISR Top Half  begin ************/
	/***************************************/
	/* place ur emerency TH bussiness here  ******/
	/********** ISR Top Half  begin ************/
	sgi15task_pending = 1;
	/********** Set Flags to indecate while{} loop block do the Buttom Half  ************/
}


void  gic_sgi_init (void)
{
	int i;
    
    gic_Int_vect_reg((u16)GIC_SGI8,
                     (irq_fn_ptr_t)gic_sgi8_handler,
                     (void *)0);

    gic_Int_en(GIC_SGI8);

	gic_Int_vect_reg((u16)GIC_SGI15,
                     (irq_fn_ptr_t)gic_sgi15_handler,
                     (void *)0);

    gic_Int_en(GIC_SGI15);

	//init sgi0~sgi7
	for(i = 0; i < 8; i++){
		gic_Int_vect_reg((u16)i,
                         (irq_fn_ptr_t)gic_sgi0to7_handler,
                         (void *)i);
    	gic_Int_en(i);
	}
	
}

static s32 check_dma_memory_result(u8 *src, u8 *dst, int size)
{
	while(size && size--)
	{
		if(*src != *dst)
			break;
		src++;
		dst++;
	}
	
	if(size)
	{
		bmlog("src =0x%p, *src=%u; dst = 0x%p, *dst = %u;size=%d\n", src, *src, dst, *dst, size);
		return ALT_E_ERROR;
	}
	else
		bmlog("DMA test memory(0x%x) to memory(0x%x) is ok!\n", virt_to_phy((u32)src), virt_to_phy((u32)dst));
	
	return ALT_E_SUCCESS;
}

static void init_dma_cfg(ALT_DMA_CFG_t *cfg)
{
	int i;
	
	cfg->manager_sec = ALT_DMA_SECURITY_SECURE;

	for( i = 0; i < 32; i++)
	{
		cfg->periph_sec[i] = ALT_DMA_SECURITY_SECURE;
	}

	for(i = 0; i < 8; i++)
	{
		cfg->irq_sec[i] = ALT_DMA_SECURITY_SECURE;
	}

	for(i = 0; i < 4; i++)
	{
		cfg->periph_mux[i] = ALT_DMA_PERIPH_MUX_FPGA;
	}

}

static int dma_init(void)
{
	s32 ret;
	ALT_DMA_CFG_t cfg;
	
	init_dma_cfg(&cfg);

	ret = alt_dma_init(&cfg);
	if(ret != ALT_E_SUCCESS)
	{
		bmlog("ERROR: dma_init error!\n");
		ret = ALT_E_ERROR;
	}

	return ALT_E_SUCCESS;
}

static void dma_handler_channel0(void *arg)
{
	volatile dma_irq_info *dma_info = (volatile dma_irq_info *) arg;/* gDMAtestIrqInfoChan0 */
	
	gic_Int_en(GIC_DMA0);
	check_dma_memory_result(dma_info->src_mpu, dma_info->dst_mpu, dma_info->size);
	ACCESS_ONCE(gDMAtestDone) = 1;
}

static void dma_init_interrupt_channel0(void)
{
	gic_Int_vect_reg((u16)GIC_DMA0,
                     (irq_fn_ptr_t)dma_handler_channel0,
                     (void *)&gDMAtestIrqInfoChan0);

	gic_Int_en(GIC_DMA0);

	alt_dma_event_int_select(ALT_DMA_EVENT_0, ALT_DMA_EVENT_SELECT_SIG_IRQ);
}

static int dma_mem2mem(void)
{
	s32 ret = ALT_E_SUCCESS;
	/* for DMAC PC, not cache-able */
	ALT_DMA_PROGRAM_t *program = (ALT_DMA_PROGRAM_t *)AMP_SHARE_DMABUF_START;
	u32 dma_phy = AMPPHY_START + AMPMAP_SIZE - DMABUF_SIZE;
	u32 src_phy = (dma_phy + 0x100000);//offset: 1MB
	u32 dst_phy = (src_phy + 0x80000);// 512K
	u8 * src_virt	= (u8 *)phy_to_virt(src_phy);
	u8 * dst_virt = (u8 *)phy_to_virt(dst_phy);
	int i, size = 0x4000;

	gDMAtestIrqInfoChan0.src_mpu  = src_virt;
	gDMAtestIrqInfoChan0.dst_mpu	= dst_virt;
	gDMAtestIrqInfoChan0.size		= size;

	memset(program, 0x0, sizeof(ALT_DMA_PROGRAM_t));

	ret = alt_dma_channel_alloc(ALT_DMA_CHANNEL_0);
	if(ret != ALT_E_SUCCESS)
	{
		bmlog("ERROR: dma_channel_alloc error!\n");
		return ret;
	}

	for(i = 0; i < size; i++)
	{
		src_virt[i] = i + 1;
		dst_virt[i] = 0x0;
	}

	dma_init_interrupt_channel0();
	
	program->program_phy = virt_to_phy((u32)(program->program));

	ret = alt_dma_memory_to_memory(ALT_DMA_CHANNEL_0,
		                                       program,
		                                       (void *)dst_phy,
		                                       (const void *)src_phy,
		                                       size,
		                                       true,
		                                       ALT_DMA_EVENT_0,
		                                       0);
	if(ret != ALT_E_SUCCESS)
	{
		bmlog("ERROR: dma memory_to_memory failed!\n");
		return ret;
	}
	
	return ret;

}

static s32 dma_mem2mem_done(void)
{
	s32 ret = ALT_E_SUCCESS;
	
	ret = alt_dma_channel_free(ALT_DMA_CHANNEL_0);
	if(ret != ALT_E_SUCCESS)
	{
		bmlog("ERROR: dma free channel error!\n");
		return ret;
	}

	memset((void *)&gDMAtestIrqInfoChan0, 0x0, sizeof(gDMAtestIrqInfoChan0));
	ACCESS_ONCE(gDMAtestDone) = 0;
	
	return ret;
}


/* Init ACP ID Mapper register */
static void dma_use_acp_init(ALT_DMA_CHANNEL_t channel_id)
{
	volatile u32 * reg;
	u32 val;
	/* sideband signal 	AXUSER[4..0]:		[4:1] inner attributes: 
										b0000 = Strongly-ordered
										b0001 = Device
										b0011 = Normal Memory Non-Cacheable
										b0110 = Write-Through
										b0111 = Write-Back no Write Allocate
										b1111 = Write-Back Write Allocate 
										[0] shared.
	 */
	val = (ACP_ID_DMA_CHANNEL_THREAD(channel_id) << 16) | (0x1 << 31)| (0x1f << 4);
	reg = ACP_REG_ID_MAP(ACP_REG_ID_3_RD);
	*reg = val;

	reg = ACP_REG_ID_MAP(ACP_REG_ID_3_WR);
	*reg = val;

}

/* Init ACP ID Mapper register */
static void dma_use_acp_init_fpga(ALT_DMA_CHANNEL_t channel_id)
{
	volatile u32 * reg;
	u32 val;
	/* sideband signal 	AXUSER[4..0]:		[4:1] inner attributes: 
										b0000 = Strongly-ordered
										b0001 = Device
										b0011 = Normal Memory Non-Cacheable
										b0110 = Write-Through
										b0111 = Write-Back no Write Allocate
										b1111 = Write-Back Write Allocate 
										[0] shared.
	 */
	val = (ACP_ID_DMA_CHANNEL_THREAD(channel_id) << 16) | (0x1 << 31)| (0x1f << 4) | (0x3<<12);
	reg = ACP_REG_ID_MAP(ACP_REG_ID_3_RD);
	*reg = val;

	val = (ACP_ID_DMA_CHANNEL_THREAD(channel_id) << 16) | (0x1 << 31)|(0x1f << 4);
	reg = ACP_REG_ID_MAP(ACP_REG_ID_3_WR);
	*reg = val;

}



static s32 dma_mem2mem_use_acp(void)
{
	s32 ret = ALT_E_SUCCESS;
	/* for DMAC PC, not cache-able */
	ALT_DMA_PROGRAM_t *program = (ALT_DMA_PROGRAM_t *)AMP_SHARE_DMABUF_START;
	u32 dma_phy = AMPPHY_START + 0x200000; //physical address: 0x1e200000
	u32 src_phy = (dma_phy + 0x0);			//source physical address for dma : offset: 0
	u32 dst_phy = (src_phy + 0x4000);			// destination physical address for dma: offset 0x4000
	u8 * src_virt	= (u8 *)phy_to_virt(src_phy);	//source virtual address for mpu, cache-able, WBWA shareable
	u8 * dst_virt = (u8 *)phy_to_virt(dst_phy);	// desination virtual address for mpu,  cache-able, WBWA shareable
	int i, size = 0x4000;

	gDMAtestIrqInfoChan0.src_mpu = src_virt;
	gDMAtestIrqInfoChan0.dst_mpu = dst_virt;
	gDMAtestIrqInfoChan0.size       = size;

	dma_use_acp_init(ALT_DMA_CHANNEL_0);

	memset(program, 0x0, sizeof(ALT_DMA_PROGRAM_t));
	ret = alt_dma_channel_alloc(ALT_DMA_CHANNEL_0);
	if(ret != ALT_E_SUCCESS)
	{
		bmlog("ERROR: dma_channel_alloc error!\n");
		return ret;
	}

	/* init the source memory and the destination memory for dma using ACP */
	for(i = 0; i < size; i++)
	{
		src_virt[i] = i + 1;
		dst_virt[i] = 0x0;	
	}

	dma_init_interrupt_channel0();

	/* Init the dma engine's micro code PC */
	program->program_phy = virt_to_phy((u32)(program->program));

	
	ret = alt_dma_memory_to_memory(ALT_DMA_CHANNEL_0,
		                                       program,
		                                       (void *)(dst_phy|(0x1 << 31)),
		                                       (const void *)(src_phy|(0x1 << 31)),
		                                       size,
		                                       true,
		                                       ALT_DMA_EVENT_0,
		                                       1);
	if(ret != ALT_E_SUCCESS)
	{
		bmlog("ERROR: dma memory_to_memory acp failed!\n");
		return ret;
	}
	return 	ret;

}

static s32 dma_mem2mem_use_acp_done(void)
{
	s32 ret = ALT_E_SUCCESS;
	
	ret = alt_dma_channel_free(ALT_DMA_CHANNEL_0);
	if(ret != ALT_E_SUCCESS)
	{
		bmlog("ERROR: dma free channel error!\n");
		return ret;
	}

	memset((void *)&gDMAtestIrqInfoChan0, 0x0, sizeof(gDMAtestIrqInfoChan0));
	ACCESS_ONCE(gDMAtestDone) = 0;
	
	return ret;
}


static int dma_ARMmem2FPGAmem(void)
{
	s32 ret = ALT_E_SUCCESS;
	/* for DMAC PC, not cache-able */
	ALT_DMA_PROGRAM_t *program = (ALT_DMA_PROGRAM_t *)AMP_SHARE_DMABUF_START;
	u32 dma_phy = AMPPHY_START + AMPMAP_SIZE - DMABUF_SIZE;
	u32 src_phy = (dma_phy + 0x100000);//offset: 1MB
	u32 dst_phy = FPGA_SDRAM_PHYS_BASE + (FPGA_SDRAM_SIZE/2);
	//non-cacherable, bufferable src
	u8 * src_virt	= (u8 *)phy_to_virt(src_phy);
	//non-cacherable, bufferable dest
	u8 * dst_virt = (u8 *)phy_to_virt(dst_phy);		
	int i, size = 0x4000;

	gDMAtestIrqInfoChan0.src_mpu  = src_virt;
	gDMAtestIrqInfoChan0.dst_mpu	= dst_virt;
	gDMAtestIrqInfoChan0.size		= size;

	memset(program, 0x0, sizeof(ALT_DMA_PROGRAM_t));

	ret = alt_dma_channel_alloc(ALT_DMA_CHANNEL_0);
	if(ret != ALT_E_SUCCESS)
	{
		bmlog("ERROR: dma_channel_alloc error!\n");
		return ret;
	}

	for(i = 0; i < size; i++)
	{
		src_virt[i] = i + 1;
		dst_virt[i] = 0x0;
	}

	dma_init_interrupt_channel0();
	
	program->program_phy = virt_to_phy((u32)(program->program));

	ret = alt_dma_memory_to_memory(ALT_DMA_CHANNEL_0,
		                                       program,
		                                       (void *)dst_phy,
		                                       (const void *)src_phy,
		                                       size,
		                                       true,
		                                       ALT_DMA_EVENT_0,
		                                       0);

	if(ret != ALT_E_SUCCESS)
	{
		bmlog("ERROR: dma memory_to_memory failed!\n");
		return ret;
	}
	
	return ret;

}


static s32 dma_FPGAmem2ARMmem_use_acp(void)
{
	s32 ret = ALT_E_SUCCESS;
	/* for DMAC PC, not cache-able */
	ALT_DMA_PROGRAM_t *program = (ALT_DMA_PROGRAM_t *)AMP_SHARE_DMABUF_START;
	//u32 dma_phy = AMPPHY_START + 0x200000; //physical address: 0x1e200000
	u32 src_phy = FPGA_SDRAM_PHYS_BASE;			//source physical address for dma : offset: 0
	u32 dst_phy = (SH_MEM_START + 0x200000);			// destination physical address for dma: offset 0x200000
	//source virtual address for mpu, cache-able, WBWA shareable
	u8 * src_virt	= (u8 *)phy_to_virt(src_phy);
	// desination virtual address for mpu,  cache-able, WBWA shareable
	u8 * dst_virt = (u8 *)phy_to_virt(dst_phy);	
	int i, size = 0x4000;

	gDMAtestIrqInfoChan0.src_mpu = src_virt;
	gDMAtestIrqInfoChan0.dst_mpu = dst_virt;
	gDMAtestIrqInfoChan0.size       = size;

	dma_use_acp_init_fpga(ALT_DMA_CHANNEL_0);

	memset(program, 0x0, sizeof(ALT_DMA_PROGRAM_t));
	ret = alt_dma_channel_alloc(ALT_DMA_CHANNEL_0);
	if(ret != ALT_E_SUCCESS)
	{
		bmlog("ERROR: dma_channel_alloc error!\n");
		return ret;
	}

	/* init the source memory and the destination memory for dma using ACP */
	for(i = 0; i < size; i++)
	{
		src_virt[i] = i + 1;
		dst_virt[i] = 0x0;	
	}

	dma_init_interrupt_channel0();

	/* Init the dma engine's micro code PC */
	program->program_phy = virt_to_phy((u32)(program->program));

	
	ret = alt_dma_memory_to_memory(ALT_DMA_CHANNEL_0,
		                                       program,
		                                       (void *)(dst_phy|(0x1 << 31)),
		                                       (const void *)(src_phy&(~(0x1 << 30))),
		                                       size,
		                                       true,
		                                       ALT_DMA_EVENT_0,
		                                       1);
	if(ret != ALT_E_SUCCESS)
	{
		bmlog("ERROR: dma memory_to_memory acp failed!\n");
		return ret;
	}

	return ret;
	

}

void  fire_fpga_irq(void)
{
   	//fpga timer frequency is 50M
    writel(0x1, (void *)FPGA_INT_LATENCY_IP_SOFT_RESET); //clear counter, dessert irq also
	writel(5000, (void *)FPGA_INT_LATENCY_IP_CYCLE);     //5000 = 100us in 50m 
    writel(0x1, (void *)FPGA_INT_LATENCY_IP_START);      //wr 1 start conter
}


void DMAC_regs_dump(int channel)
{
	void *base = (void *)HPS_DMAC_BASE;
	bmlog("*********************DMAC_regs_dump**********************\n");
	bmlog("DS:   0x%08x\n", readl(base));
	bmlog("FSM:  0x%08x\n", readl(base + 0x30));
	bmlog("FSC:  0x%08x\n", readl(base + 0x34));
	bmlog("FTM:  0x%08x\n", readl(base + 0x38));
	bmlog("FTC0: 0x%08x\n", readl(base + 0x40));
	bmlog("CS0:  0x%08x\n", readl(base + 0x100));
	bmlog("CPC0: 0x%08x\n", readl(base + 0x104));
	bmlog("SA_0: 0x%08x\n", readl(base + 0x400));
	bmlog("DA_0: 0x%08x\n", readl(base + 0x404));
	bmlog("CC_0: 0x%08x\n", readl(base + 0x408));
	bmlog("LC00: 0x%08x\n", readl(base + 0x40C));
	bmlog("LC10: 0x%08x\n", readl(base + 0x410));
	bmlog("*********************************************************\n");
}

void InitPeripheral(void)
{
	u32 reg;
	
	/* init GPIO */
	reg =  readl((const volatile void *)HPS_GPIO1_BASE_ADDR + HPS_GPIO_DIR_OFFSET);
	reg |= 0xf<<12;
	writel(reg, (void *)HPS_GPIO1_BASE_ADDR + HPS_GPIO_DIR_OFFSET);
	reg = readl((const volatile void *)HPS_GPIO1_BASE_ADDR + HPS_GPIO_DAT_OFFSET);
	reg |= 0xf<<12;
	writel(reg, (void *)HPS_GPIO1_BASE_ADDR + HPS_GPIO_DAT_OFFSET);

	LDE27_SET_OUTPUT();
	LDE28_SET_OUTPUT();
	LDE30_SET_OUTPUT();

#ifdef	LCD1602_DISP
	//i2c pinmux to gpio 63~64
	writel(0x0, (void *)BITBANG_IIC_PINMUX);
	writel(0x0, (void *)HPS_GPIO1_PINMUX);
#endif
}


u32 testIL = 1;
int  main (void)

{
	u32 count = 0;
	s32 i, j, k = 0;
	u32 max, sum;
	volatile u32 tmp;
	u32 *p;
	char *pstr;
	p = (u32 *)(DRAM_PHYS_START + 0x6000);
	u32  boot;
	u32  load_start, load_end;
	u32  boot_start, boot_end;
	u32 bm_load_duration_ns, bm_load_duration_ms;

	/* get the timestamp of the osc timer1 */
	boot_end = readl((const volatile void *)SOCFPGA_OSC1TIMER1_ADDRESS + 0x4);

	InitPeripheral();
	cpu_local_irq_disable();
	
	/*  asp is cleared by the preloader , and preloader will save
	  *  qspi probe, read function in it. Please don't clear
	  *  we reuse the preloader qspi driver at here, so we save
	  *  the probe and read function pointer into asp.
	  *  it at here.
	  */
	//memset(asp, 0, sizeof(struct amp_share_param));
	asp->bm_magic = ('b' << 8) | 'm';
	
	/**** interrupt initialize*******/
    gic_Int_init();
	/**** hook ISR callback ******/
	gic_sgi_init();
	
#ifdef NEED_SAVE_RESTORE_GIC_REGS_FROM_BM
	gic_dist_save();
#endif
	/**** interrupt ready *****/
	cpu_local_irq_enable();
	UART_DEBUG("start bm...... %x\r\n", p);
	UART_DEBUG("++start bm...... 12345\r\n");
	bmlog("start bm......%x\r\n", p);
	UART_DEBUG("start bm...... %x\r\n", 0x55aa);
	UART_DEBUG("--start bm...... 67890\r\n");

	/* auto detect the boot mode */
	boot = bm_get_boot_mode();
	if((BOOTSEL_MODE_SD_1_8V == boot) || (BOOTSEL_MODE_SD_3_3V == boot))
	{		
		if(!bm_sd_load_rbf())
			bmlog("load fpga rbf is ok!\n");
	}
	else if((BOOTSET_MODE_QSPI_1_8V == boot) || (BOOTSET_MODE_QSPI_3_3V == boot))
	{
		if(!bm_qspi_load_rbf())
			bmlog("load fpga rbf is ok!\n");
	}
	
	/* osc_timer1 is running at 25MHz, 40ns per cycle */
	asp->boot_end_stamp = boot_end;
	boot_start =  asp->boot_start_stamp;
	bm_load_duration_ns = (boot_start - boot_end) * 40;
	bm_load_duration_ms = bm_load_duration_ns/1000/1000;
	bmlog("start[0x%x],end[%x]\n, bm_load_ns = %u(ns), bm_load_ms= %u(ms)\n", 
										   boot_start,   
										   boot_end,
										   bm_load_duration_ns,
										   bm_load_duration_ms
										   );
	load_start = asp->load_bm_start;
	load_end   = asp->load_bm_end;
	bmlog("real loading bm time duration is %u(ms)(including qspi/sd init)\n", load_end - load_start);

	if(fpgamgr_program_fpga((const unsigned long *)FPGA_RBF_BASE, FPGA_RBF_SIZE) < 0)	
		UART_DEBUG("config fpga failed!\r\n");

	else
	{
		UART_DEBUG("config fpga OK!\r\n");
		writel(('R'<<16)|('B'<<8)|('F'<<0),&(asp->preloader_wait_bm_load_rbf));
	}


	pstr = (char *)FPGA_SDRAM_PHYS_BASE;
	sprintf(pstr, "%s\n", "i am from fpga ddr ram");
	bmlog("%s", pstr);
	
	pstr = (char *)FPGA_SRAM_PHYS_BASE;
	sprintf(pstr, "%s\n", "i am from fpga sram");
	bmlog("%s", pstr);

#ifdef	LCD1602_DISP
	IIC_InitIp();
	LCD_SetCursor(0);
	sprintf(cDispBuf[0], "fpga config done!");
	IIC_EXfer(LCD_ADDR, cDispBuf[0], strlen(cDispBuf[0])>15?16:strlen(cDispBuf[0]));
#endif
	
	while(!gCacheCoherence)
	{
		/** led blink for hand shake debug with linux **/
		k++;
		if((k&0x3FFFF) == 0)
			LED27_BLINK();//*(volatile u32 *)HPS_GPIO1_BASE_ADDR ^= (0x1<<12);
	}
	/*
	**************************************************************
	**** cause Linux peer use 2GB user space/2GB kernel space split *******
	**** so it is 2048 L1 entry here, if use 3GB user/1 GB space split ********
	**** it woule be 1024 here, and p should adjust to 0x7000 yejc    ********
	**************************************************************
	*/
	for(j = 0; j < 2048-16; j++) //16MB for IO map space
				PageTable[j+2048] = *p++;

	/*
	**************************************************************
	**** time to bring bm core to smp cache coherence environmence *******
	**************************************************************
	*/
	__asm__ __volatile__("dsb\n"
	"isb\n"
	"mrc    p15, 0, r1, c1, c0, 0\n"
	"ldr	r2, =0x40180d\n"
	"orr	r1, r1, r2\n"
	"mcr    p15, 0, r1, c1, c0, 0\n"
	"dsb\n"
	"isb\n"
	: : :"memory", "cc");


	bmlog("L1 L2 cache enabled, SCU enabled~~~\n");
	//asp = 0xFC700000;
	
	/*
	********************************************************************************
	** from this point on, you can free to invoke the linux kernel space text function from BM,  *****
	** only if the function would not cause the shceduler to action, or the CPU of BM would trap ***** 
	** in the cpu_idle(clone from linux cpu core) process if we can't not obtain the lock/mutex  *****
	** /semaphore                                                                                                              ****
	********************************************************************************
	*/
	//printk = (printk_fn)asp->sta.printk_fn;
	_raw_spinlock = (raw_spinlock_fn)asp->sta.spinlock_lock_fn;
	_raw_spinunlock = (raw_spinlock_fn)asp->sta.spinlock_unlock_fn;
	/* semaphore */
	down_trylock = (down_trylock_fn)asp->sta._down_trylock_fn;

	while(1)
	{
		if(sgi15task_pending)
		{	
			if(ACCESS_ONCE(asp->sra[SGI_LINUX_REQ_BM_CONSUME_BUF].linux_cmd_args) == 0)
			{
			/**************************************************************************************/
			/**** place interrupt here test worse case interrupt latency would be more accuracy ***/
			/**** casue we not only take care of 10000 interrupts/second from FPGA, but also    ***/
			/**** suffer more than 6000 extra interrupts/second from IPI and offen L1 data cahce miss **/
			/**************************************************************************************/
#ifdef TEST_IL_MORE_ACCURACY
			if(testIL)
			{
				bmlog("\n________________________________________________________________________________\n");
                bmlog("Now the BM CPU would suffer more than 16000 interrupts/second(10000i/s from\n");
                bmlog("FPGA_IRQ0 req(100us),more than 6000i/s from IPI(Inner Process Interrupt), and\n");
                bmlog("process several Gbps data per second, they are all concurrently, very intensive\n");
                bmlog("load for BM CPU core!\n");
                bmlog("__________________________________________________________________________________\n");
				testIL = 0;
				gic_Int_dis (GIC_PFGA0); //72 FPGA_IRQ0
				gic_Int_clr (GIC_PFGA0);
				pvt_init(1);
				fire_fpga_irq();
				pvt_start();
				gic_Int_en (GIC_PFGA0);
			}
#endif
			/************************************************************************/
			//p = (unsigned int *)0x1E200000;
			p = (unsigned int *)0xFC800000;
			for(i = 0; i < (DRAM_BUF_SIZE/4); i++)
			{
				if(*p != CPU_DATA_PATERN0)
					bmlog("check buf from linux failed! i=%d, *p=%x\n", i, *p);
				p++;
			}
			//bmlog("check buf from linux finish!\n");
			//p = (unsigned int *)0x1E200000;
			p = (unsigned int *)0xFC800000;
			memset_int(p, CPU_DATA_PATERN3, DRAM_BUF_SIZE);
			//now u can mesure blink frequency on hps  LED3 and multiply 2 to calculate the interrupte frequency
			//and then you can evaulate how fast the cpu do 2 times DRAM_SIZE write and 2 times DRAM_SIZE read
			//you sholud know that cpu spend must time to iter, compare in the read "for loop" and BM do interrupt 
			//handler&cpu mode switch Linux handle system tick&sgi intrrupt and sched task, so the actual memory system
			//band width is greater than this evaluate value
			//be aware 65536B is bigger than L1 Dcache but little than L2 Dcache
			//evaluate data process speed in our case is:
			//1842Hz * 2toggle * 2w * 2r&check * DRAM_SIZE(65536B) = 965738496B/s = 7.73Gbps
			

			//toggle hps led
			//*(volatile u32 *)HPS_GPIO1_BASE_ADDR ^= (0x1<<12);
			LED27_BLINK();
			sgi15task_pending = 0;
			gic_raise_interrupt(CPU0, GIC_SGI13);
				}
			else if(ACCESS_ONCE(asp->sra[SGI_LINUX_REQ_BM_CONSUME_BUF].linux_cmd_args) == 1)
				{
			p = (unsigned int *)0xfe700000;
			for(i = 0; i < (SRAM_BUF_SIZE/4); i++)
			{
				if(*p != CPU_DATA_PATERN0)
					bmlog("check buf from linux failed! i=%d, *p=%x\n", i, *p);
				p++;
			}
			//bmlog("check buf from linux finish!\n");
			p = (unsigned int *)0xfe700000;
			memset_int(p, CPU_DATA_PATERN3, SRAM_BUF_SIZE);
			//now u can mesure blink frequency on hps LED2  and multiply 2 to calculate the interrupte frequency
			//and then you can evaulate how fast the cpu do 2 times SRAM_SIZE write and 2 times SRAM_SIZE read
			//you sholud know that cpu spend must time to iter, compare in the read "for loop" and BM do interrupt 
			//handler&cpu mode switch Linux handle system tick&sgi intrrupt and sched task, so the actual memory system
			//band width is greater than this evaluate value
			//be aware 32768B is bigger than L1 Dcache but little than L2 Dcache
			//evaluate data process speed in our case is:
			//3230Hz * 2toggle * 2w * 2r&check * SRAM_SIZE(32768B) = 846725120B/s = 6.77Gbps //little then DRAM cause much more interrupt overhead
			

			//toggle hps led
			//*(volatile u32 *)HPS_GPIO1_BASE_ADDR ^= (0x2<<12);
			LED28_BLINK();
			sgi15task_pending = 0;
			gic_raise_interrupt(CPU0, GIC_SGI13);
				
		}
		//bmlog("CPU1#%04d:msg from cpu1 call~~~~~~~~~~~~~~~~\n", i);
	}
		if(asp->sra[SGI_LINUX_REQ_BM_CONSUME_BUF].linux_cmd_args == 2)
		{
					for(i = 0; i < SPINLOCK_TEST_COUNT; i++)
        			{
                		_raw_spinlock(&asp->rslocks[0]);
                		tmp = asp->sra[SGI_LINUX_REQ_BM_CONSUME_BUF].bm_cmd_status;
                		//dummy j++
                		j++;
                		asp->sra[SGI_LINUX_REQ_BM_CONSUME_BUF].bm_cmd_status += 2;
                		if((asp->sra[SGI_LINUX_REQ_BM_CONSUME_BUF].bm_cmd_status - tmp) != 2)
                        	bmlog("BM:spinlock test failed!\n");
                		_raw_spinunlock(&asp->rslocks[0]);
                		//dummy operation on j++ simulate the actual scenario to give another cpu chance
                		//to take lock, reduce starvation situation
                		j++;
        			}
					//bmlog("\nBM spinlock test:%d\n", tmp + 2);
					bmlog("\n----------------------------\n");
					bmlog("BM spinlock test:%d\n", tmp + 2);
					bmlog("----------------------------\n");


			/*************************************************************************************/
			/**** place interrupt test here test cpu data process speed would be more accuracy ***/
			/*************************************************************************************/
#ifdef TEST_DATA_PROCESS_SPEED_MORE_ACCURACY
			gic_Int_dis (GIC_PFGA0); //72 FPGA_IRQ0
			gic_Int_clr (GIC_PFGA0);
			pvt_init(1);
			fire_fpga_irq();
			pvt_start();
			gic_Int_en (GIC_PFGA0);
#endif
			/************************************************************************/
			while(!ACCESS_ONCE(gINTtestDone));
			ACCESS_ONCE(gINTtestDone) = 0;
			for(i = 0; i < IL_TEST_COUNT - 1; i++)
			{
				p_pvt[i] = p_pvt[i] - p_pvt[i + 1]; //delta t of pvt, be careful pvt counter overlap!
			}
			for(i = 0; i < IL_TEST_COUNT - 1; i++)
			{
				if(p_pvt[i] > PVT_100US_CYCLE)
					p_pvt[i] -= PVT_100US_CYCLE; /* pvt 10ns resolution, 10000 * 10ns = 100us*/ //IL(interrupt latency jitter)
				else
					p_pvt[i] = PVT_100US_CYCLE - p_pvt[i];
			}
			max = p_pvt[0];
			k = 0;
			for(i = 0; i < IL_TEST_COUNT - 1; i++)
				if(p_pvt[i] > max)
				{
					max = p_pvt[i];
					k = i;
				}
			max *= 10;
			bmlog("\n------------------------------------------------------------------------------------------\n");
			//bmlog("\ninterrupt latency test method 1(use private timer)\n");
			bmlog("interrupt latency test method 1(use private timer)\n");
			//bmlog("max interrupt latency jitter: %d ns\n", max);
			bmlog("max interrupt latency jitter: %d ns\n", max);
			sum = 0;
			for(i = 0; i < IL_TEST_COUNT - 1; i++)
			{
				sum += p_pvt[i];				//be carefule sum overflow
			}
			sum /= (IL_TEST_COUNT - 1);
			sum *= 10;
			//bmlog("average interrupt latency jitter: %d ns\n", sum);
			bmlog("average interrupt latency jitter: %d ns\n", sum);
			//90ns is measure from oscilloscope, see AMP Reference Design for detail
			bmlog("max interrupt latency: %d ns(use private timer@CPU core cluster)\n", max + 90);
			if(max>410)
			    bmlog("max interrupt latency: %d ns(use private timer@CPU core cluster),max_index=%d\n", max + 90, k); 
			bmlog("average interrupt latency: %d ns(use private timer@CPU core cluster)\n", sum + 90);
			bmlog("-----------------------------------------------------------------------------------------------\n");


			bmlog("\n-----------------------------------------------------------------------------------------------\n");
			//bmlog("\nfpga send %d times irq req to arm, arm ack %d times, lost irq %d times\n", gFPGA_IRQ_req, gARM_IRQ_ack, gFPGA_IRQ_req - gARM_IRQ_ack);
			bmlog("fpga send %d times irq req to arm, arm ack %d times, lost irq %d times\n", 
			ACCESS_ONCE(gFPGA_IRQ_req), ACCESS_ONCE(gARM_IRQ_ack), ACCESS_ONCE(gFPGA_IRQ_req) - ACCESS_ONCE(gARM_IRQ_ack));
			bmlog("-------------------------------------------------------------------------------------------------\n");

			testIL = 1;
#ifdef DMA_AND_ACP_TEST
			dma_init();
			//DMAC_regs_dump(0);
			dma_mem2mem();//new for test
			//DMAC_regs_dump(0);
			while(!ACCESS_ONCE(gDMAtestDone));
			ACCESS_ONCE(gDMAtestDone) = 0;
			dma_mem2mem_done();
			//DMAC_regs_dump(0);
			bmlog("DMA test done\n");
			bmlog("-----------------------------------------------\n\n");

			dma_mem2mem_use_acp();
			while(!ACCESS_ONCE(gDMAtestDone));
			ACCESS_ONCE(gDMAtestDone) = 0;
			dma_mem2mem_use_acp_done();
			bmlog("DMA test acp done\n");
			bmlog("-----------------------------------------------\n\n");

			dma_ARMmem2FPGAmem();
			while(!ACCESS_ONCE(gDMAtestDone));
			ACCESS_ONCE(gDMAtestDone) = 0;
			dma_mem2mem_done();
			bmlog("dma_ARMmem2FPGAmem test done\n");
			bmlog("-----------------------------------------------\n\n");

			dma_FPGAmem2ARMmem_use_acp();
			while(!ACCESS_ONCE(gDMAtestDone));
			ACCESS_ONCE(gDMAtestDone) = 0;
			dma_mem2mem_done();
			bmlog("dma_FPGAmem2ARMmem_use_acp test done\n");
			bmlog("-----------------------------------------------\n\n");
#endif

			count++;
#ifdef	LCD1602_DISP
			LCD_SetCursor(1);
			memset(cDispBuf[1], ' ', 16);
			sprintf(cDispBuf[1], "test:%d", count);
			IIC_EXfer(LCD_ADDR, cDispBuf[1], 16);
#endif
			
			asp->sra[SGI_LINUX_REQ_BM_CONSUME_BUF].linux_cmd_args = -1; //tell linux interrupt latency and dma test done!
	
		}
		//dummy k++ and toggle hps led1, waste cpu time..
		//indicate bm activity, u can measure the toggle freqency to evalute how much time spend per loop
		//loop cycle = 1s /(toggle freqency * 2 * 1048576)
		//in our case cpu = 800MHz, no interrupt to handle no work cmd,
		// loop cycle = 1/(42.384*2*1048576) * 10^9 = 11.25ns
		//if remov the K++, if((k&0xFFFFF) == 0){...} block, we can remove about 7 dissemable instrution~8 clock cycle
		// 11.25 * (6 +6)(instrution)/(6+6+8) total instrution = 6.75ns
		k++;
		if((k&0x1FFFFF) == 0)
			LED30_BLINK();
	}

}
