/*
 *  larkboardfb.c -- Altera framebuffer driver
 *
 *  Based on vfb.c -- Virtual frame buffer device
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 * 		modified fy Embest
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/fb.h>
#include <linux/init.h>

#include <linux/delay.h>
#include <linux/kthread.h>


#define ALTERA_SGDMA_IO_EXTENT 0x400
#define ALTERA_SGDMA_STATUS 0
#define ALTERA_SGDMA_STATUS_BUSY_MSK (0x10)
#define ALTERA_SGDMA_CONTROL 16
#define ALTERA_SGDMA_CONTROL_RUN_MSK  (0x20)
#define ALTERA_SGDMA_CONTROL_SOFTWARERESET_MSK (0X10000)
#define ALTERA_SGDMA_CONTROL_PARK_MSK (0X20000)
#define ALTERA_SGDMA_NEXT_DESC_POINTER 32

/* SGDMA can only transfer this many bytes per descriptor */
#define DISPLAY_BYTES_PER_DESC 0xFF00UL
#define ALTERA_SGDMA_DESCRIPTOR_CONTROL_GENERATE_EOP_MSK (0x1)
#define ALTERA_SGDMA_DESCRIPTOR_CONTROL_GENERATE_SOP_MSK (0x4)
#define ALTERA_SGDMA_DESCRIPTOR_CONTROL_OWNED_BY_HW_MSK (0x80)
#define DISPLAY_DESC_COUNT(len) (((len) + DISPLAY_BYTES_PER_DESC - 1) \
				/ DISPLAY_BYTES_PER_DESC)
#define DISPLAY_DESC_SIZE(len) (DISPLAY_DESC_COUNT(len) \
				* sizeof(struct sgdma_desc))


#define PACKET_BANK_ADDRESSOFFSET 12
#define PB0_BASE_ADDRESSOFFSET 16
#define PB0_WORDS_ADDRESSOFFSET 20
#define PB0_SAMPLES_ADDRESSOFFSET 24
#define PB0_WIDTH_ADDRESSOFFSET 32
#define PB0_HEIGHT_ADDRESSOFFSET 36
#define PB0_INTERLACED_ADDRESSOFFSET 40

#define VIPFR_GO 0

#define VGA_IMAGE_WIDTH 1024
#define VGA_IMAGE_HEIGHT 768
#define LCD7_IMAGE_WIDTH 800
#define LCD7_IMAGE_HEIGHT 480
#define LCD4_3_IMAGE_WIDTH 480
#define LCD4_3_IMAGE_HEIGHT 272

#define CLOCK_CTR_BASE (0xff200000 + 0x00010040)


struct sgdma_desc {
	u32 read_addr;
	u32 read_addr_pad;

	u32 write_addr;
	u32 write_addr_pad;

	u32 next;
	u32 next_pad;

	u16 bytes_to_transfer;
	u8 read_burst;
	u8 write_burst;

	u16 actual_bytes_transferred;
	u8 status;
	u8 control;

} __attribute__ ((packed));


struct altfb_type;

struct altfb_dev {
    struct altfb_type *type;
    struct platform_device *pdev;
    struct fb_info *info;
    struct resource *reg_res;
	struct resource *itc_res;
    void __iomem *base;
	void __iomem *itc_base;
	void __iomem *ctr_base;
    int mem_word_width;
    void *priv;
};

struct altfb_type {
    const char *name;
    int (*of_setup)(struct altfb_dev *fbdev);
    int (*start_hw)(struct altfb_dev *fbdev);
};
enum FBMODE{
	VGA_MODE = 0,
	LCD4_3_MODE,
	LCD7_MODE,
};

static int gFbmode = VGA_MODE;

/*
 *  RAM we reserve for the frame buffer. This defines the maximum screen
 *  size
 *
 *  The default can be overridden if the driver is compiled as a module
 */

static struct fb_var_screeninfo altfb_default = {
	.activate = FB_ACTIVATE_NOW,
	.height = -1,
	.width = -1,
	.vmode = FB_VMODE_NONINTERLACED,
};

static struct fb_fix_screeninfo altfb_fix = {
	.id = "altfb",
	.type = FB_TYPE_PACKED_PIXELS,
	.visual = FB_VISUAL_TRUECOLOR, //FB_VISUAL_TRUECOLOR,
	.accel = FB_ACCEL_NONE,
};

static int altfb_setcolreg(unsigned regno, unsigned red, unsigned green,
			   unsigned blue, unsigned transp, struct fb_info *info)
{
	/*
	 *  Set a single color register. The values supplied have a 32/16 bit
	 *  magnitude.
	 *  Return != 0 for invalid regno.
	 */

	if (regno > 255)
		return 1;

    if(info->var.bits_per_pixel == 16) {
	    red >>= 11;
	    green >>= 10;
	    blue >>= 11;

	    if (regno < 255) {
		    ((u32 *) info->pseudo_palette)[regno] = ((red & 31) << 11) |
		        ((green & 63) << 5) | (blue & 31);
	    }
    } else {
	    red >>= 8;
	    green >>= 8;
	    blue >>= 8;

	    if (regno < 255) {
		    ((u32 *) info->pseudo_palette)[regno] = ((red & 255) << 16) |
		        ((green & 255) << 8) | (blue & 255);
	    }
    }

	return 0;
}

static struct fb_ops altfb_ops = {
	.owner = THIS_MODULE,
	.fb_fillrect = cfb_fillrect,
	.fb_copyarea = cfb_copyarea,
	.fb_imageblit = cfb_imageblit,
	.fb_setcolreg = altfb_setcolreg,
};

/*
 *  Initialization - SGDMA Based
 */
static int of_setup_sgdma(struct altfb_dev *fbdev)
{
	const __be32* val;

	val = of_get_property(fbdev->pdev->dev.of_node, "width", NULL);
	if (!val) {
		dev_err(&fbdev->pdev->dev, "Missing required parameter 'width'");
		return -ENODEV;
	}

	fbdev->info->var.xres = be32_to_cpup(val),
	fbdev->info->var.xres_virtual = fbdev->info->var.xres,

	val = of_get_property(fbdev->pdev->dev.of_node, "height", NULL);
	if (!val) {
		dev_err(&fbdev->pdev->dev, "Missing required parameter 'height'");
		return -ENODEV;
	}

	fbdev->info->var.yres = be32_to_cpup(val);
	fbdev->info->var.yres_virtual = fbdev->info->var.yres;

	val = of_get_property(fbdev->pdev->dev.of_node, "bpp", NULL);
	if (!val) {
		dev_err(&fbdev->pdev->dev, "Missing required parameter 'bpp'");
		return -ENODEV;
	}

	fbdev->info->var.bits_per_pixel = be32_to_cpup(val);
	if(fbdev->info->var.bits_per_pixel == 24) {
		dev_info(&fbdev->pdev->dev, "BPP is set to 24. Using 32 to align to 16bit addresses");
		fbdev->info->var.bits_per_pixel = 32;
	}
	return 0;
}

static int start_sgdma_hw(struct altfb_dev *fbdev) 
{
	unsigned long first_desc_phys, next_desc_phys;
	unsigned ctrl = ALTERA_SGDMA_DESCRIPTOR_CONTROL_OWNED_BY_HW_MSK;
    unsigned long start = fbdev->info->fix.smem_start;
    unsigned long len = fbdev->info->fix.smem_len;
    struct sgdma_desc *desc;
	struct sgdma_desc *descp = dma_alloc_coherent(NULL,
					DISPLAY_DESC_SIZE(fbdev->info->fix.smem_len),
					(void*)&first_desc_phys, GFP_KERNEL);
    if(!descp) {
        dev_err(&fbdev->pdev->dev, "Failed to allocate SGDMA descriptor memory\n");
        return -ENOMEM;
    }
	writel(ALTERA_SGDMA_CONTROL_SOFTWARERESET_MSK, \
	       fbdev->base + ALTERA_SGDMA_CONTROL);	/* halt current transfer */
	writel(0, fbdev->base + ALTERA_SGDMA_CONTROL);	/* disable interrupts */
	writel(0xff, fbdev->base + ALTERA_SGDMA_STATUS);	/* clear status */
	writel(first_desc_phys, fbdev->base + ALTERA_SGDMA_NEXT_DESC_POINTER);

    next_desc_phys = first_desc_phys;
    desc = descp;
	while (len) {
		unsigned long cc = min(len, DISPLAY_BYTES_PER_DESC);
		next_desc_phys += sizeof(struct sgdma_desc);
		desc->read_addr = start;
		desc->next = next_desc_phys;
		desc->bytes_to_transfer = cc;
		desc->control = ctrl;
		start += cc;
		len -= cc;
		desc++;
	}

	desc--;
	desc->next = first_desc_phys;
	desc->control = ctrl | ALTERA_SGDMA_DESCRIPTOR_CONTROL_GENERATE_EOP_MSK;
	desc = descp;
	desc->control = ctrl | ALTERA_SGDMA_DESCRIPTOR_CONTROL_GENERATE_SOP_MSK;
	writel(ALTERA_SGDMA_CONTROL_RUN_MSK | ALTERA_SGDMA_CONTROL_PARK_MSK, \
	       fbdev->base + ALTERA_SGDMA_CONTROL);	/* start */
	return 0;
}
/*
 *  Initialization - VIPFrameReader Based
 */



static int of_setup_vipfr(struct altfb_dev *fbdev) 
{
	 __be32* val;


	if(gFbmode == VGA_MODE){
		fbdev->info->var.xres = VGA_IMAGE_WIDTH;
		fbdev->info->var.yres = VGA_IMAGE_HEIGHT;
	}
	else if(gFbmode == LCD7_MODE){
		fbdev->info->var.xres = LCD7_IMAGE_WIDTH;
		fbdev->info->var.yres = LCD7_IMAGE_HEIGHT;

	}
	else if(gFbmode == LCD4_3_MODE){
		fbdev->info->var.xres = LCD4_3_IMAGE_WIDTH;
		fbdev->info->var.yres = LCD4_3_IMAGE_HEIGHT;

	}
	fbdev->info->var.xres_virtual = fbdev->info->var.xres,
	fbdev->info->var.yres_virtual = fbdev->info->var.yres;
	fbdev->info->var.bits_per_pixel = 32;
	
	val = of_get_property(fbdev->pdev->dev.of_node, "mem-word-width", NULL);
	if (!val) {
		dev_err(&fbdev->pdev->dev, "Missing required parameter 'mem-word-width'");
		return -ENODEV;
	}
	fbdev->mem_word_width = be32_to_cpup(val);		/* word midth is the master port width in Qsys */
	if(!(fbdev->mem_word_width >= 32 && fbdev->mem_word_width % 32 == 0)) {
		dev_err(&fbdev->pdev->dev, "mem-word-width is set to %i. must be >= 32 and multiple of 32.",fbdev->mem_word_width);
		return -ENODEV;
	}
					
	return 0;
}
static int start_vipfr_hw(struct altfb_dev *fbdev) 
{
	u8 ctrValue;

	ctrValue = readb(fbdev->ctr_base);	

	writew(0,fbdev->itc_base);							//stop video output
	writew(0,fbdev->itc_base + 28*4);					//mode1 disable


	if(gFbmode == VGA_MODE){
		printk(KERN_INFO"VGA mode...\n");
		
		writeb(ctrValue&0xf9,fbdev->ctr_base);			//chose 65Mhz
		
		writew(1920,fbdev->itc_base + 3*4);				//fife words
		writew(VGA_IMAGE_WIDTH,fbdev->itc_base + 6*4);				//picture width
		writew(VGA_IMAGE_HEIGHT,fbdev->itc_base + 7*4);				//picture height

		writew(24,fbdev->itc_base + 9*4);				//hfp
		writew(136,fbdev->itc_base + 10*4);				//hsw
		writew(320,fbdev->itc_base + 11*4);				//h blanking
		
		writew(3,fbdev->itc_base + 12*4);				//vfp
		writew(6,fbdev->itc_base + 13*4);				//vsw
		writew(38,fbdev->itc_base + 14*4);				//b blanking
		
	}
	else if(gFbmode == LCD7_MODE){
		printk(KERN_INFO"LCD7 mode...\n");
		
		writeb( (ctrValue&0xf9)|0x02 ,fbdev->ctr_base);	//chose 30Mhz

		writew(1920,fbdev->itc_base + 3*4);				//fife words
		writew(LCD7_IMAGE_WIDTH,fbdev->itc_base + 6*4);	//picture width
		writew(LCD7_IMAGE_HEIGHT,fbdev->itc_base + 7*4);//picture height

		writew(39,fbdev->itc_base + 9*4);				//hfp
		writew(47,fbdev->itc_base + 10*4);  			//hsw
		writew(125,fbdev->itc_base + 11*4);				//h blanking
		
		writew(13,fbdev->itc_base + 12*4);				//vfp
		writew(2,fbdev->itc_base + 13*4);				//vsw
		writew(44,fbdev->itc_base + 14*4);				//b blanking		
	}
	else if(gFbmode == LCD4_3_MODE){
		printk(KERN_INFO"LCD4_3 mode...\n");
		
		writeb( (ctrValue&0xf9)|0x04,fbdev->ctr_base);	//10M 
		
		writew(1920,fbdev->itc_base + 3*4);				//fife words
		writew(LCD4_3_IMAGE_WIDTH,fbdev->itc_base + 6*4);//picture width
		writew(LCD4_3_IMAGE_HEIGHT,fbdev->itc_base + 7*4);//picture height

		writew(2,fbdev->itc_base + 9*4);				//hfp
		writew(41,fbdev->itc_base + 10*4);  			//hsw
		writew(45,fbdev->itc_base + 11*4);				//h blanking
		
		writew(2,fbdev->itc_base + 12*4);				//vfp
		writew(10,fbdev->itc_base + 13*4);				//vsw
		writew(14,fbdev->itc_base + 14*4);				//b blanking		
	}


	writew(1,fbdev->itc_base + 28*4);					//mode1 enable


    writel(fbdev->info->fix.smem_start, fbdev->base + PB0_BASE_ADDRESSOFFSET);			//frame buff physic addr
    writel(fbdev->info->var.xres * fbdev->info->var.yres/(fbdev->mem_word_width/32),\
			fbdev->base + PB0_WORDS_ADDRESSOFFSET);										//frame words,word width is master port width
    writel(fbdev->info->var.xres * fbdev->info->var.yres, \
            fbdev->base + PB0_SAMPLES_ADDRESSOFFSET);									//The number of single-cycle color patterns
    writel(fbdev->info->var.xres, fbdev->base + PB0_WIDTH_ADDRESSOFFSET);				//picture width
    writel(fbdev->info->var.yres, fbdev->base + PB0_HEIGHT_ADDRESSOFFSET);				//picture height
    writel(3, fbdev->base + PB0_INTERLACED_ADDRESSOFFSET);						
    writel(0, fbdev->base + PACKET_BANK_ADDRESSOFFSET);									//chose frame0

	writel(1, fbdev->base);																//enable the frame read
	writew(1,fbdev->itc_base);															//enable the video output

    return 0;
}

/*
 *  Initialization - General
 */
static struct altfb_type altfb_sgdma = {
    .name = "SGDMA with video_sync_generator",
    .of_setup = of_setup_sgdma,
    .start_hw = start_sgdma_hw,
};

static struct altfb_type altfb_vipfr = {
    .name = "ALTVIP FrameReader",
    .of_setup = of_setup_vipfr,
    .start_hw = start_vipfr_hw,
};

static struct of_device_id altfb_match[] = {
	{ .compatible = "ALTR,altfb-12.1", .data = &altfb_sgdma },
	{ .compatible = "ALTR,altfb-1.0",  .data = &altfb_sgdma },
	{ .compatible = "ALTR,vip-frame-reader-9.1",  .data = &altfb_vipfr },
	{},
};
MODULE_DEVICE_TABLE(of, altfb_match);


static int altfb_probe(struct platform_device *pdev)
{
	struct fb_info *info;
	int retval = -ENOMEM;
	void *fbmem_virt;
    struct altfb_dev *fbdev;
    struct of_device_id *match;

	printk(KERN_INFO"altfb_probe..............................\n");
    match = of_match_node(altfb_match, pdev->dev.of_node);
    if(!match)
        return -ENODEV;
	
    fbdev = kzalloc(sizeof(struct altfb_dev), GFP_KERNEL);
    if(!fbdev)
        return -ENOMEM;
	
    fbdev->pdev = pdev;
    fbdev->type = match->data;

	fbdev->reg_res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "vfr_reg");
	if (!fbdev->reg_res)
	    goto err;

	fbdev->itc_res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "itc_reg");
	if (!fbdev->reg_res)
	    goto err;
	
	info = framebuffer_alloc(sizeof(u32) * 256, &pdev->dev);
	if (!info)
		goto err;

    fbdev->info = info;
	info->fbops = &altfb_ops;
	info->var = altfb_default;
	info->fix = altfb_fix;

    if(fbdev->type->of_setup(fbdev))
        goto err1;

	if(info->var.bits_per_pixel == 16) {
		info->var.red.offset = 11;
		info->var.red.length = 5;
		info->var.red.msb_right = 0;
		info->var.green.offset = 5;
		info->var.green.length = 6;
		info->var.green.msb_right = 0;
		info->var.blue.offset = 0;
		info->var.blue.length = 5;
		info->var.blue.msb_right = 0;
	} else {

		info->var.red.offset = 16;
		info->var.red.length = 8;
		info->var.red.msb_right = 0;
		info->var.green.offset = 8;
		info->var.green.length = 8;
		info->var.green.msb_right = 0;
		info->var.blue.offset = 0;
		info->var.blue.length = 8;
		info->var.blue.msb_right = 0;
	
	}

	info->fix.line_length = (info->var.xres * (info->var.bits_per_pixel >> 3));
	info->fix.smem_len = info->fix.line_length * info->var.yres;

	/* sgdma descriptor table is located at the end of display memory */
	fbmem_virt = dma_alloc_coherent(NULL,
					info->fix.smem_len,
					(void *)&(info->fix.smem_start),
					GFP_KERNEL);
	if (!fbmem_virt) {
		dev_err(&pdev->dev, "altfb: unable to allocate %ld Bytes fb memory\n",
			info->fix.smem_len + DISPLAY_DESC_SIZE(info->fix.smem_len));
		goto err2;
	}

	info->screen_base = fbmem_virt;
	info->pseudo_palette = info->par;
	info->par = NULL;
	info->flags = FBINFO_FLAG_DEFAULT;

	retval = fb_alloc_cmap(&info->cmap, 256, 0);
	if (retval < 0)
		goto err1;

	platform_set_drvdata(pdev, fbdev);

	if (!request_mem_region(fbdev->reg_res->start, resource_size(fbdev->reg_res), pdev->name)) {
		dev_err(&pdev->dev, "Memory region busy\n");
		retval = -EBUSY;
		goto err3;
	}
	fbdev->base = ioremap_nocache(fbdev->reg_res->start, resource_size(fbdev->reg_res));
	if(!fbdev->base) {
	    dev_err(&pdev->dev, "ioremap failed\n");
		retval = -EIO;
		goto err4;
	}

	if (!request_mem_region(fbdev->itc_res->start, resource_size(fbdev->itc_res), pdev->name)) {
		dev_err(&pdev->dev, "Memory region busy\n");
		retval = -EBUSY;
		goto err3;
	}
	fbdev->itc_base = ioremap_nocache(fbdev->itc_res->start, resource_size(fbdev->itc_res));
	if(!fbdev->itc_base) {
	    dev_err(&pdev->dev, "itc_base ioremap failed\n");
		retval = -EIO;
		goto err4;
	}

	if (!request_mem_region(CLOCK_CTR_BASE,sizeof(u32), pdev->name)) {
		dev_err(&pdev->dev, "Memory region busy\n");
		retval = -EBUSY;
		goto err3;
	}
	fbdev->ctr_base = ioremap_nocache(CLOCK_CTR_BASE, sizeof(u32));
	if(!fbdev->itc_base) {
	    dev_err(&pdev->dev, "ctr_base ioremap failed\n");
		retval = -EIO;
		goto err4;
	}

	if (fbdev->type->start_hw(fbdev))
		goto err5;

	printk(KERN_INFO "fb%d: %s frame buffer device at 0x%x+0x%x\n",
		info->node, info->fix.id, (unsigned)info->fix.smem_start,
		info->fix.smem_len);

	retval = register_framebuffer(info);
	if (retval < 0)
		goto err5;

	return 0;
err5:
	iounmap(fbdev->base);
err4:
	release_region(fbdev->reg_res->start, resource_size(fbdev->reg_res));
err3:
	dma_free_coherent(NULL, info->fix.smem_len, fbmem_virt,
			  info->fix.smem_start);
err2:
	fb_dealloc_cmap(&info->cmap);
err1:
	framebuffer_release(info);
err:
    kfree(fbdev);
	return retval;
}

static int altfb_remove(struct platform_device *dev)
{
	struct altfb_dev *fbdev = platform_get_drvdata(dev);

    if (fbdev) {
	    iounmap(fbdev->base);
	    release_region(fbdev->reg_res->start, resource_size(fbdev->reg_res));
	    if (fbdev->info) {
		    unregister_framebuffer(fbdev->info);
		    dma_free_coherent(NULL, fbdev->info->fix.smem_len, fbdev->info->screen_base,
				      fbdev->info->fix.smem_start);
		    framebuffer_release(fbdev->info);
	    }
	    kfree(fbdev);
    }
	return 0;
}


static int __init vfb_set_fb_mode(char *str)
{
	if( !strcmp(str,"VGA") ){
		gFbmode = VGA_MODE;
	}
	else if( !strcmp(str,"LCD4_3") ){
		gFbmode = LCD4_3_MODE;
	}
	else if( !strcmp(str,"LCD7") ){
		gFbmode = LCD7_MODE;
	}
	else{
		pr_err("bad fbmode,use defualt VGA mode\n");
	}

	return 0;
}

__setup("fbmode=",vfb_set_fb_mode);


static const struct of_device_id altfbvip_match[] = {
	{.compatible = "ALTR,vip-frame-reader-13.0",},
	{}
};

static struct platform_driver altfb_driver = {
	.probe = altfb_probe,
	.remove = altfb_remove,
	.driver = {
	    .owner = THIS_MODULE,
		.name = "altfb",
		.of_match_table = of_match_ptr(altfbvip_match),
    },
};

module_platform_driver(altfb_driver);

MODULE_DESCRIPTION("Altera framebuffer driver");
MODULE_AUTHOR("Thomas Chou <thomas@wytron.com.tw>");
MODULE_LICENSE("GPL");
