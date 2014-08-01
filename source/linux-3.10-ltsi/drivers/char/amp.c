/*
 *
 *  Copyright (C) 2013-2014 Embest Technology co.,ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/completion.h>
#include <linux/dcache.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/fcntl.h>
#include <linux/kref.h>
#include <linux/kthread.h>
#include <linux/limits.h>
#include <linux/module.h>
#include <linux/rwsem.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/string.h>
#include <linux/freezer.h>
#include <linux/utsname.h>
#include <linux/delay.h>
#include <linux/smp.h>
#include <linux/vmalloc.h>
#include <linux/mm.h>
#include <linux/mmzone.h>
#include <linux/sched.h>
#include <linux/fs.h>
#include <asm/mach/map.h>
#include <asm/amp_config.h>

#define DRIVER_NAME		"amp"
#define DRIVER_VERSION		"1 August 2013"


MODULE_DESCRIPTION("amp");
MODULE_AUTHOR("embest co.,ltd.yejc");
MODULE_LICENSE("Dual BSD/GPL");

static u32 terimate=0;
static u32 ipcflag;
static struct task_struct       *thread_handle[2];
static u32 *pbuf0 = (u32 *)AMP_SHARE_BUFFER_START;
//static u32 *pbuf1 = (u32 *)(AMP_SHARE_BUFFER_START + DRAM_BUF_SIZE);
static u32 *psram = (u32 *)AMP_SHARE_SRAM_STRART;

static DECLARE_COMPLETION(kthread_amp_done);
static DEFINE_MUTEX(amp_mutex);
static unsigned long useflags;
static unsigned long threadrun_flags;
static unsigned long preemptdis_flag = 0;

extern int irq_base;

static u32 walk_virt_to_phys(u32 addr)
{
        pmd_t *pmd;
        pte_t *ptep;

        pgd_t *pgd = pgd_offset(current->active_mm, addr);
        if (pgd_none(*pgd) || pgd_bad(*pgd))
                return 0;

        pmd = pmd_offset((pud_t *)pgd, addr);
        if (pmd_none(*pmd)) //allow section map pass yejc
                return 0;
	if(!pmd_bad(*pmd)) //page map
	{
        	ptep = pte_offset_map(pmd, addr);	//linux version pte
        	if (ptep && pte_present(*ptep))
		{
			ptep += 512;
                	return (PAGE_MASK & *ptep) | (~PAGE_MASK & addr);//hw version pte yejc
		}
	}
        return (pgd_val(*pgd)&SECTION_MASK) | (~SECTION_MASK & addr); //is section map yejc
}


/*
***************************************************************************
*                       Embest Tech co., ltd
*                        www.embest-tech.com
***************************************************************************
*
*two CPUs concurrenctly access the same memory adrress
*/
static int amp_test_thread(void *arg)
{
	int cpu;
	int i, tmp;
	int sh_buf_test_cnt;
	enum test_sequence ts;

	if(preemptdis_flag)
		preempt_disable();
	sh_buf_test_cnt = 0;
	ts = eSH_DRAM;
	cpu = 1;
	printk(KERN_INFO"amp test begin\n");
	//ACCESS_ONCE(asp->sra[SGI_LINUX_REQ_BM_CONSUME_BUF].linux_cmd_args) = eSH_DRAM;
	asp->sra[SGI_LINUX_REQ_BM_CONSUME_BUF].linux_cmd_args = eSH_DRAM;
	while(ts == eSH_DRAM)
	{
		if(ACCESS_ONCE(asp->sra[SGI_LINUX_REQ_BM_CONSUME_BUF].linux_cmd_status) == COMPLETE)
		{
			//for(i = 0; i < (DRAM_BUF_SIZE/4); i++)
			//	*(pbuf0 + i) = CPU_DATA_PATERN0;
			memset_int(pbuf0, CPU_DATA_PATERN0, DRAM_BUF_SIZE);
			ACCESS_ONCE(asp->sra[SGI_LINUX_REQ_BM_CONSUME_BUF].linux_cmd_status) = PENDING;
			sh_buf_test_cnt++;
			//request bm check buf
			gic_raise_softirq(cpumask_of(cpu), SGI_LINUX_REQ_BM_CONSUME_BUF);
		}
		if(sh_buf_test_cnt >=  CORE_SHARE_BUFFER_TEST_COUNT)
			break;
	}

	sh_buf_test_cnt = 0;
	if(preemptdis_flag)
                preempt_enable();
	//give some time to last gic issue and process buffer data before move next test case;
	printk("\nDRAM share test finished(test DRAM share access and cache coherence)\n");
	msleep(10);
	if(preemptdis_flag)
                preempt_disable();
	//ACCESS_ONCE(asp->sra[SGI_LINUX_REQ_BM_CONSUME_BUF].linux_cmd_args) = eSH_SRAM;
	asp->sra[SGI_LINUX_REQ_BM_CONSUME_BUF].linux_cmd_args = eSH_SRAM;
	ts++;
	//ACCESS_ONCE(asp->sra[SGI_LINUX_REQ_BM_CONSUME_BUF].linux_cmd_args) = eSH_SRAM;
	while(ts == eSH_SRAM)
        {
                if(ACCESS_ONCE(asp->sra[SGI_LINUX_REQ_BM_CONSUME_BUF].linux_cmd_status) == COMPLETE)
                {
                        //for(i = 0; i < (DRAM_BUF_SIZE/4); i++)
                        //      *(pbuf0 + i) = CPU_DATA_PATERN0;
                        memset_int(psram, CPU_DATA_PATERN0, SRAM_BUF_SIZE);
                        ACCESS_ONCE(asp->sra[SGI_LINUX_REQ_BM_CONSUME_BUF].linux_cmd_status) = PENDING;
                        sh_buf_test_cnt++;
                        //request bm check buf
                        gic_raise_softirq(cpumask_of(cpu), SGI_LINUX_REQ_BM_CONSUME_BUF);
                }
                if(sh_buf_test_cnt >=  CORE_SHARE_BUFFER_TEST_COUNT)
                        break;
        }

	//give some time to last gic issue and process buffer data before move next test case;
	if(preemptdis_flag)
                preempt_enable();
	printk("SRAM share test finished(test SRAM share access and cache coherence)\n");
        msleep(10);
	if(preemptdis_flag)
                preempt_disable();
	ts++;
	asp->sra[SGI_LINUX_REQ_BM_CONSUME_BUF].linux_cmd_args = eSPINLOCK;
	asp->sra[SGI_LINUX_REQ_BM_CONSUME_BUF].bm_cmd_status = 0;
	//isse gic notify BM do spinlock test
	for(i = 0; i < SPINLOCK_TEST_COUNT; i++)
	{
		raw_spin_lock(&asp->rslocks[0]);
		tmp = asp->sra[SGI_LINUX_REQ_BM_CONSUME_BUF].bm_cmd_status;
		//dummy sh_buf_test_cnt++
		sh_buf_test_cnt++;
		asp->sra[SGI_LINUX_REQ_BM_CONSUME_BUF].bm_cmd_status += 2;
		if((asp->sra[SGI_LINUX_REQ_BM_CONSUME_BUF].bm_cmd_status - tmp) != 2)
			printk("Linux:spinlock test failed!\n");
		raw_spin_unlock(&asp->rslocks[0]);
		//dummy operation on sh_buf_test_cnt++ simulate the actual scenario to give another cpu chance
		//to take lock, reduce starvation situation
		sh_buf_test_cnt++;
	}
	printk("Linux spinlock test:%d\n", asp->sra[SGI_LINUX_REQ_BM_CONSUME_BUF].bm_cmd_status);
	if(preemptdis_flag)
                preempt_enable();
	printk("waiting BM finish interrupt latency test and DMA test etc.\n");
	//preempt_disable();
	while(asp->sra[SGI_LINUX_REQ_BM_CONSUME_BUF].linux_cmd_args != -1)
	{
		//dummy, just put some interrupt load to bm
		//cpu = 1;
        	//gic_raise_softirq(cpumask_of(cpu), SGI_BRING_OTHER2CACHECOHERENCE);//yejc
		usleep_range(200, 210);
	}
	//preempt_enable();
	printk("BM finish BM finish interrupt latency test and DMA test\n");
	printk("use ""cat /sys/class/amp/amp/bm_log to check bmlog""\n");
	clear_bit(0, &threadrun_flags);	
	
	return 0;
}
static int thread_shced_test(void *arg)
{
        printk(KERN_INFO "++thread_shced_test\n");
        while(1)
        {
		//indicate linux activity
		*(volatile u32 *)SOCFGA_GPIO0_VIRT_BASE ^= 0x1<<18;
                msleep(20);
        }
        printk(KERN_INFO "--thread_shced_test\n");
        return 0;
}

static irqreturn_t sgi13handler(int irq, void *dev_id)
{

        int i;
	int size = 0;
	u32 *p = pbuf0;

	//TODO: u should move the check routine to BH in actual project
	//if(ACCESS_ONCE(asp->sra[SGI_LINUX_REQ_BM_CONSUME_BUF].linux_cmd_args) == eSH_DRAM)
	if(asp->sra[SGI_LINUX_REQ_BM_CONSUME_BUF].linux_cmd_args == eSH_DRAM)
	{
		p = pbuf0;
		size = DRAM_BUF_SIZE>>2;
	}
	else  if(asp->sra[SGI_LINUX_REQ_BM_CONSUME_BUF].linux_cmd_args == eSH_SRAM)
	//else  if(ACCESS_ONCE(asp->sra[SGI_LINUX_REQ_BM_CONSUME_BUF].linux_cmd_args) == eSH_SRAM)
	{
		p = psram;
		size = SRAM_BUF_SIZE>>2;
	}
	for(i = 0; i < size; i++)
		if(*(p + i) != CPU_DATA_PATERN3)
			printk("check buf from bm failed! addr=%08x, *addr=%08x~~~~\n", (u32)(p + i), *(p + i));
	
	//printk("check buf from bm finish!~~~~~\n");
	ACCESS_ONCE(asp->sra[SGI_LINUX_REQ_BM_CONSUME_BUF].linux_cmd_status) = COMPLETE;
        return IRQ_HANDLED;
}

static int amp_open(struct inode *inode, struct file *filp)
{
        int result = 0;

        mutex_lock(&amp_mutex);
        if (test_and_set_bit(0, &useflags)) {
                result = -EBUSY;
                /* this legacy device is always one per system and it doesn't
                 * know how to handle multiple concurrent clients.
                 */
                goto out;
        }
	/*
        result = request_irq(amp_interrupt, &amp_interrupt,
                             IRQF_DISABLED, "SGI", amp_interrupt);
        if (result == -EBUSY)
                printk(KERN_ERR "amp: Interrupt can't be reserved.\n");
	*/
out:
        mutex_unlock(&amp_mutex);
        return result;
}

static long amp_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	return 0;
}

static int amp_mmap(struct file *file, struct vm_area_struct *vma)
{
	size_t nm_size = vma->vm_end - vma->vm_start;
	size_t dma_size = 0;
	pgprot_t dma_page_prot;

	if(nm_size > (AMPMAP_SIZE - BM_CONTEXT_SIZE))
		return -EPERM;

	if(nm_size > (AMPMAP_SIZE - DMABUF_SIZE - BM_CONTEXT_SIZE)){ 
		//for security reason, don't expose bm ctx to user space.
		dma_size = nm_size - (AMPMAP_SIZE - DMABUF_SIZE - BM_CONTEXT_SIZE);
		nm_size = AMPMAP_SIZE - DMABUF_SIZE - BM_CONTEXT_SIZE;
	}

	/* Remap-pfn-range will mark the range VM_IO */
	if (remap_pfn_range(vma,
			    vma->vm_start,
			    (AMPPHY_START + BM_CONTEXT_SIZE)>>PAGE_SHIFT,
			    nm_size,
			    vma->vm_page_prot)) {
		return -EAGAIN;
	}
	if(dma_size > 0){
		dma_page_prot = pgprot_writecombine(vma->vm_page_prot);
		 if (remap_pfn_range(vma,
                            vma->vm_start + nm_size,
                            (AMPPHY_START + AMPMAP_SIZE - DMABUF_SIZE)>>PAGE_SHIFT,
                            dma_size,
                            dma_page_prot)) {
                	return -EAGAIN;
        	}
	}
	return 0;
}

static int amp_release(struct inode *inode, struct file *filp)
{
        //free_irq(amp_interrupt, amp_interrupt);
	clear_bit(0, &useflags);	

        return 0;
}

static const struct file_operations amp_fops = {
        .mmap = amp_mmap,
	.open = amp_open,
	.unlocked_ioctl = amp_ioctl,
        .release = amp_release,
};

static struct class *amp_class;
int amp_major = 0;

static ssize_t amp_test_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int i;
	if(strncmp(buf, "test", 4) == 0)
	{
		if(test_and_set_bit(0, &threadrun_flags))
			return -EBUSY;
		if((size > 7) && (strncmp(buf, "test_wpd", 8) == 0)) //test with preempt disable
			preemptdis_flag = 1;
		else
			preemptdis_flag = 0;

		for(i = 0; i < 5; i++)
		{
			thread_handle[0] = kthread_create(amp_test_thread, NULL, "amp_test:0:0");
			if (IS_ERR(thread_handle[0])) 
			{
				msleep(30);
				continue;
        		}
			else
			{
				wake_up_process(thread_handle[0]);
				break;
			}
		}
		if(i >= 5)
		{
			size = PTR_ERR(thread_handle[0]);
			printk("create amp_test kthread failed 0x%08x\n", size);
			clear_bit(0, &threadrun_flags);
                        return size;
		}
	}

        return size;
}

static ssize_t sgi_trigger_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	int sgi_nb = -1;
	int cpu = 1;
	char *endp;

	endp = (char *)buf + size;
	
	sgi_nb = simple_strtol(buf, &endp, 10);
	if((sgi_nb >= 0) && (sgi_nb < 8))
	{
		gic_raise_softirq(cpumask_of(cpu), sgi_nb);
	}
	else
		printk("invalid sgi number\n");

	return size;
}

static ssize_t
bm_log_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	
	if(asp->logbuf_overlap)
	{
		printk("overlap:logindex %d\n", asp->logindex);
		memcpy(buf, (void *)((char *)asp->logbuf + asp->logindex), LOG_LEN - asp->logindex);
		memcpy(buf + LOG_LEN - asp->logindex, asp->logbuf, asp->logindex);
		return LOG_LEN - 1; //limit it little than 1 page
	}
	else
	{
		printk("logindex = %d\n", asp->logindex);
		memcpy(buf, asp->logbuf, asp->logindex);
		return asp->logindex;
	}
}

static struct device_attribute amp_dev_attrs[] = {
	__ATTR(amp_test, 0220, NULL, amp_test_store),
	__ATTR(sgi_trigger, 0220, NULL, sgi_trigger_store),
	__ATTR(bm_log, 0440, bm_log_show, NULL),
	__ATTR_NULL
};
/*
***************************************************************************
*                       Embest Tech co., ltd
*                        www.embest-tech.com
***************************************************************************
*
*request ipi irq, launch thread
*/
static int __init amp_init(void)
{
	int rc=0;

	walk_virt_to_phys(AMP_SHARE_DMABUF_START);
	
	rc = register_chrdev(amp_major, "amp", &amp_fops);
	if(rc < 0)
	{
        	printk(KERN_INFO"failed to register amp dev\n");
		return rc;
	}
	amp_major = rc;

        amp_class = class_create(THIS_MODULE, "amp");
        if (IS_ERR(amp_class))
                return PTR_ERR(amp_class);

	amp_class->dev_attrs = amp_dev_attrs;

	if(!device_create(amp_class, NULL, MKDEV(amp_major, 0), NULL, "amp"))
		printk(KERN_INFO"amp:device_create failed!\n");

	rc = request_irq(irq_base/*irq_base derive from irq_gic.c*/ + SGI_BM_REQ_LINUX_CONSUME_BUF, sgi13handler, IRQF_SHARED, "SGI", &ipcflag);
	if(rc)
	{
		printk(KERN_INFO"%s:request irq failed!\r\n", __FUNCTION__);
		return rc;
	}
	/*
	thread_handle[0] = kthread_create(amp_test_thread, NULL, "amp_test:0:0");
	//thread_task = kthread_create_on_cpu(ipi_demo_thread, &gIpiCnt, 0 , "amp_test:1:0");
        if (IS_ERR(thread_handle[0])) {
                rc = PTR_ERR(thread_handle[0]);
		return rc;
        }*/
	thread_handle[1] = kthread_create(thread_shced_test, NULL, "sched_test:0:0");
        if (IS_ERR(thread_handle[1])) {
                rc = PTR_ERR(thread_handle[1]);
                return rc;
        }

	//wake_up_process(thread_handle[0]);
	wake_up_process(thread_handle[1]);
	//kthread_unpark(thread_task);

	printk(KERN_INFO "--amp_init\n");
	return rc;
}
module_init(amp_init);


static void __exit amp_cleanup(void)
{
	terimate = 1;
}
module_exit(amp_cleanup);
