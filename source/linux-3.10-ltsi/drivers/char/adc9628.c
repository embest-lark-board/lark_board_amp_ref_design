/*
 * 
 *
 */

#include <linux/init.h>
#include <linux/fs.h>
#include <linux/major.h>
#include <linux/blkdev.h>
#include <linux/module.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/gfp.h>
#include <linux/compat.h>
#include <linux/vmalloc.h>
#include <asm/uaccess.h>
#include <linux/delay.h>


#define LARK_ADC_MAJOR			(218)
#define ADC_RESET_BIT 0x08	
#define BSP_OK     0
#define BSP_ERROR  -1
#define BSP_ADC_ADDR   (0x00000020 + 0xff200000)
#define BSP_FIFO_BASE (0x20040000 + 0xc0000000)


static struct class *adc_class;
static struct raw_device_data *adc_devices;
static DEFINE_MUTEX(adc_mutex);
static int max_adc_minors = 1;

static void __iomem * gRegbase = 0;
static void __iomem * gFifobase = 0;


module_param(max_adc_minors, int, 0);


struct adc_priv_data{
	void __iomem * regbase;
	void __iomem * fifobase;
	u32* adc_buff;

};

static u8 ad9628_read(int reg)
{
    int i;
    unsigned int header;
    u8 result;  
    u8 val;

    header = (0x1<<15) | (reg);
    writew(0x4,gRegbase +(0x5*4)); 					/* cs = 0 */
    udelay(1);

    for(i = 0; i < 16; i++){
        if( (header>>(15-i)) & 0x1 )  				
        	writew(0x1,gRegbase + (0x4*4)); 		
        else
        	writew(0x1,gRegbase + (0x5*4)); 		/* pio_clr = bit0 */

        udelay(1);
        writew(0x2,gRegbase + (0x5*4)); 			/* sclk=1 */
        udelay(1);
        writew(0x2,gRegbase + (0x4*4)); 			/* sclk=0 */
    }
    writew(0x6,gRegbase + (0x1*4)); 				
    result = 0;
    udelay(1);
	
    for(i = 0; i < 8; i++){
    	writew(0x2,gRegbase + (0x5*4)); 			/* sclk=1 */
        udelay(1);
        val = readb(gRegbase);
        if (val  & 0x1)  							
            result = result | (0x1<<(7-i));

        writew(0x2,gRegbase + (0x4*4)); 			/* sclk=0 */
        udelay(1);
    }
    writew(0x4,gRegbase + (0x4*4)); 				/* cs =  1 */
    writew(0x7,gRegbase + (0x1*4)); 				

    return result;
}

static void ad9628_write(int reg, int val)
{
    int i;
    unsigned int header;


    header = ( (0x0<<23) | (reg<<8) | (val&0xff) );
    writew(0x4,gRegbase + (0x5*4)); 			/* cs = 0 */
    udelay(2);

    for(i = 0; i < 24; i++){
        if( (header>>(23-i)) & 0x1 )  			
        	writew(0x1,gRegbase + (0x4*4));		/* pio_set = bit0 */
        else
        	writew(0x1,gRegbase + (0x5*4)); 	/* pio_clr = bit0 */

        udelay(1);
        writew(0x2,gRegbase + (0x5*4)); 		/* sclk=1 */
        udelay(1);
        writew(0x2,gRegbase + (0x4*4)); 		/* sclk=0 */
    }
    udelay(1);
    writew(0x4,gRegbase + (0x4*4)); 			/* cs =  1 */

}


static void bsp_adc_cmos(void)
{
	ad9628_write(0x14, 0x01);
}

/*
*init the registers of ADC9628,
*write to Altera's PIO IP core to operate the gpio as spi pins
*/
static void bsp_adc_init(struct adc_priv_data *adc_data)
{
    int iadc_val  ;


    writew( 0x7,adc_data->regbase + (0x1*4)); 						/* pio_dir = 0x7 */
	writew( 0x6,adc_data->regbase + (0x4*4)); 						/* cs = 1  */
	writew( 0x2,adc_data->regbase + (0x4*4)); 						/* sclk=0 */
	mdelay(2);

    iadc_val = ad9628_read(0x1 );  									/*  chip ID, 0x89 */

    printk(KERN_INFO"current id = %x \n", iadc_val);

    iadc_val = ad9628_read(0x2 );  									/*  chip grade */
	iadc_val = ad9628_read(0x0b ); 									/* clock divide */
	iadc_val = 0x0;
	ad9628_write(0x0b, iadc_val);
	iadc_val = ad9628_read(0x0b );
    iadc_val = ad9628_read(0x09 );  								/* DUTY stabilty */
	iadc_val = ad9628_read(0x0b );  								/* clock ratio */
	ad9628_write(0x05, 0x03); 									    /*  Channel a b */
    iadc_val = ad9628_read(0x5 );
	ad9628_write(0x0d, 0x0);  										/*  normal mode */

    iadc_val = ad9628_read(0xd );
	iadc_val = ad9628_read(0x101 ); 							

	iadc_val |= (0x1<<7);
	ad9628_write(0x101, iadc_val);
	iadc_val = ad9628_read(0x101 );

	bsp_adc_cmos();
	iadc_val = ad9628_read(0x14 );

}


static int adc_open(struct inode *inode, struct file *filp)
{

	struct adc_priv_data *adc_data;

	adc_data = kzalloc(sizeof(struct adc_priv_data), GFP_KERNEL);
	if (!adc_data) {
		return -ENOMEM;
	}
	adc_data->adc_buff = kzalloc(sizeof(u32)*1024, GFP_KERNEL);
	if (!adc_data->adc_buff) {
		return -ENOMEM;
	}
	
	adc_data->regbase = gRegbase;
	adc_data->fifobase = gFifobase;

    bsp_adc_init(adc_data);
	filp->private_data = adc_data;

	return 0;
}


static int adc_release(struct inode *inode, struct file *filp)
{

	struct adc_priv_data *adc_data = filp->private_data;
	kfree(adc_data->adc_buff);
	kfree(adc_data);

	return 0;
}


ssize_t adc_read(struct file *filp, char __user *buf, size_t count, loff_t *off)
{
	struct adc_priv_data *adc_data = filp->private_data;
	u8 val = 0;
	int i;
	u32 tmp;

    val = readb(adc_data->regbase);
    val |= ADC_RESET_BIT;
    writew( ADC_RESET_BIT,adc_data->regbase + (0x1*4)); 	/* bit7: 1-out */
    writew( val,adc_data->regbase + (0x4*4)); 				/* pio_set 1,  bit8 */
    udelay(1);
    writew( val,adc_data->regbase + (0x5*4)); 				/* pio_clr 0,  bit8 */
    writew( val,adc_data->regbase + (0x4*4)); 				/* pio_set 1,  bit8 */
    
    mdelay(1000);											/* wait for FPGA complete writing the adc data */
	if(count > 1024*4)										/* by default fpga just write 1024 32-bits */
		count = 1024*4;
	
    for (i=0;i < count/4;i++){
		tmp = readl(adc_data->fifobase + i*4);
		adc_data->adc_buff[i] = tmp;
    }
	if (copy_to_user((void __user *)buf, adc_data->adc_buff, count))
		return 0;

	return count;
}

static const struct file_operations adc_fops = {
	.read		= adc_read,
	.open		= adc_open,
	.release		= adc_release,
	.owner		= THIS_MODULE,
};

static struct cdev adc_cdev;

static char *adc_devnode(struct device *dev, umode_t *mode)
{
	return kasprintf(GFP_KERNEL, "adc/%s", dev_name(dev));
}


static int __init adc9628_init(void)
{
	dev_t dev = MKDEV(LARK_ADC_MAJOR, 0);
	int ret;

	ret = register_chrdev_region(dev, max_adc_minors, "adc");
	if (ret)
		goto error;

	cdev_init(&adc_cdev, &adc_fops);
	ret = cdev_add(&adc_cdev, dev, max_adc_minors);
	if (ret) {
		goto error_region;
	}

	adc_class = class_create(THIS_MODULE, "adc");
	if (IS_ERR(adc_class)) {
		printk(KERN_ERR "Error creating adc class.\n");
		cdev_del(&adc_cdev);
		ret = PTR_ERR(adc_class);
		goto error_region;
	}
	adc_class->devnode = adc_devnode;
	device_create(adc_class, NULL, MKDEV(LARK_ADC_MAJOR, 0), NULL, "adc");

	if (!request_mem_region(BSP_ADC_ADDR,sizeof(u32),"adc9628")) {
		printk( "Memory region busy\n");
		return  -EBUSY;

	}
	gRegbase = ioremap_nocache(BSP_ADC_ADDR, sizeof(u32));
	if(!gRegbase) {
	    printk( KERN_INFO" ioremap failed\n");
		return -EIO;
	}
	
	if (!request_mem_region(BSP_FIFO_BASE,sizeof(u32)*1024,"adc9628_fifo")) {
		printk( KERN_INFO"Memory region busy\n");
		return  -EBUSY;

	}
	gFifobase = ioremap_nocache(BSP_FIFO_BASE, sizeof(u32)*1024);
	if(!gFifobase) {
	    printk( KERN_INFO" ioremap failed\n");
		return -EIO;
	}

	return 0;

error_region:
	unregister_chrdev_region(dev, max_adc_minors);
error:
	return ret;
}

static void __exit adc9628_exit(void)
{
	device_destroy(adc_class, MKDEV(LARK_ADC_MAJOR, 0));
	class_destroy(adc_class);
	cdev_del(&adc_cdev);
	unregister_chrdev_region(MKDEV(LARK_ADC_MAJOR, 0), max_adc_minors);
}

module_init(adc9628_init);
module_exit(adc9628_exit);
MODULE_LICENSE("GPL");

