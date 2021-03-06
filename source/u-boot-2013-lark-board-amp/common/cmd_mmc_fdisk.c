/*
 * Copyright (c) 2010 Samsung Electronics Co., Ltd.
 *              http://www.samsung.com/
 *
 * fdisk command for U-boot
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <common.h>
#include <command.h>
#include <mmc.h>

#define		BLOCK_SIZE			512
#define		BLOCK_END			0xFFFFFFFF
#define		_1MB				(1024*1024)
#define		_10MB				(10*1024*1024)
#define		_100MB				(100*1024*1024)
#define		_1GB				(1024*1024*1024)
#define		_8_4GB				(1023*254*63)


/* resver 1MB from the start, it is bootrom required, don't modify it */
#define		PRELOADER_PART_OFFSET	(_1MB)
#define		PRELOADER_PART_SIZE		(_1MB)
#define		PRELOADER_PART_TYPE		(0xa2)
#define		PRELOADER_PART_NO			3

/* reserve 5MB from the preloader for save, it can be modified */
#define		ROOTFS_PART_OFFSET		(PRELOADER_PART_OFFSET + PRELOADER_PART_SIZE + 5*_1MB)
#define		ROOTFS_PART_SIZE			(3*_1GB + 3*_100MB)	/*3.3GB*/
#define		ROOTFS_PART_TYPE			(0x83)
#define		ROOTFS_PART_NO			2

/* reserve 5MB from the rootfs for save, it can be modified  */
#define		FAT_PART_OFFSET			(ROOTFS_PART_OFFSET + ROOTFS_PART_SIZE + 5*_1MB)	
#define		FAT_PART_SIZE				(2*_100MB)		/*200MB*/
#define		FAT_PART_TYPE				(0xb)
#define		FAT_PART_NO				1


u32 mbr_part_offset[] = {0x0, 0x1be, 0x1ce, 0x1de, 0x1ee};
static unsigned int fat_start =FAT_PART_OFFSET;
static unsigned int fat_size = FAT_PART_SIZE;
static unsigned int ext_size = ROOTFS_PART_SIZE;

#define		CHS_MODE			0
#define		LBA_MODE			!(CHS_MODE)

typedef struct
{
	int		C_start;
	int		H_start;
	int		S_start;

	int		C_end;
	int		H_end;
	int		S_end;

	int		available_block;
	int		unit;
	int		total_block_count;
	int		addr_mode;	// LBA_MODE or CHS_MODE
} SDInfo;

typedef struct
{
	unsigned char bootable;
	unsigned char partitionId;

	int		C_start;
	int		H_start;
	int		S_start;

	int		C_end;
	int		H_end;
	int		S_end;

	int		block_start;
	int		block_count;
	int		block_end;
} PartitionInfo;

/////////////////////////////////////////////////////////////////
int calc_unit(u32 length, SDInfo sdInfo)
{
	if (sdInfo.addr_mode == CHS_MODE)
		return ( (length / BLOCK_SIZE / sdInfo.unit + 1 ) * sdInfo.unit);
	else
		return ( (length / BLOCK_SIZE) );
}

/////////////////////////////////////////////////////////////////
void encode_chs(int C, int H, int S, unsigned char *result)
{
	*result++ = (unsigned char) H;
	*result++ = (unsigned char) ( S + ((C & 0x00000300) >> 2) );
	*result   = (unsigned char) (C & 0x000000FF); 
}

/////////////////////////////////////////////////////////////////
void encode_partitionInfo(PartitionInfo partInfo, unsigned char *result)
{
	*result++ = partInfo.bootable;

	encode_chs(partInfo.C_start, partInfo.H_start, partInfo.S_start, result);
	result +=3;
	*result++ = partInfo.partitionId;

	encode_chs(partInfo.C_end, partInfo.H_end, partInfo.S_end, result);
	result += 3;

	memcpy(result, (unsigned char *)&(partInfo.block_start), 4);
	result += 4;	
	
	memcpy(result, (unsigned char *)&(partInfo.block_count), 4);
}

/////////////////////////////////////////////////////////////////
void decode_partitionInfo(unsigned char *in, PartitionInfo *partInfo)
{
	partInfo->bootable	= *in;
	partInfo->partitionId	= *(in + 4); 

	memcpy((unsigned char *)&(partInfo->block_start), (in + 8), 4);
	memcpy((unsigned char *)&(partInfo->block_count), (in +12), 4);
}

/////////////////////////////////////////////////////////////////
void get_SDInfo(u32 block_count, SDInfo *sdInfo)
{
       int C, H, S;

        int C_max = 1023, H_max = 255, S_max = 63;
        int H_start = 1, S_start = 1;
        int diff_min = 0, diff = 0;

        if(block_count >= _8_4GB)
                sdInfo->addr_mode = LBA_MODE;
        else
                sdInfo->addr_mode = CHS_MODE;

//-----------------------------------------------------
        if (sdInfo->addr_mode == CHS_MODE)
        {
                diff_min = C_max;

                for (H = H_start; H <= H_max; H++)
                        for (S  = S_start; S <= S_max; S++)
                        {
                                C = block_count / (H * S);

                                if ( (C <= C_max) )
                                {
                                        diff = C_max - C;
                                        if (diff <= diff_min)
                                        {
                                                diff_min = diff;
                                                sdInfo->C_end = C;
                                                sdInfo->H_end = H;
                                                sdInfo->S_end = S;
                                        }
                                }
                        }
        }
//-----------------------------------------------------
        else
        {
                sdInfo->C_end = 1023;
                sdInfo->H_end = 254;
                sdInfo->S_end = 63;
        }

//-----------------------------------------------------
        sdInfo->C_start                 = 0;
        sdInfo->H_start                 = 0;
        sdInfo->S_start                 = 1;

        sdInfo->total_block_count       = block_count;
        sdInfo->available_block         = sdInfo->C_end * sdInfo->H_end * sdInfo->S_end;
        sdInfo->unit                    = sdInfo->H_end * sdInfo->S_end;
	printf("c_end = %d, h_end = %d, s_end=%d, sdInfo->unit = %d\n",  sdInfo->C_end,  sdInfo->H_end, sdInfo->S_end, sdInfo->unit);
}

/////////////////////////////////////////////////////////////////
void make_partitionInfo(int LBA_start, int count, SDInfo sdInfo, PartitionInfo *partInfo)
{
        int             temp = 0;
        int             _10MB_unit;

        partInfo->block_start   = LBA_start;

//-----------------------------------------------------
        if (sdInfo.addr_mode == CHS_MODE)
        {
                partInfo->C_start       = partInfo->block_start / (sdInfo.H_end * sdInfo.S_end);
                temp                    = partInfo->block_start % (sdInfo.H_end * sdInfo.S_end);
                partInfo->H_start       = temp / sdInfo.S_end;
                partInfo->S_start       = temp % sdInfo.S_end + 1;

                if (count == BLOCK_END)
                {
                        _10MB_unit = calc_unit(_10MB, sdInfo);
                        partInfo->block_end     = sdInfo.C_end * sdInfo.H_end * sdInfo.S_end - _10MB_unit - 1;
                        partInfo->block_count   = partInfo->block_end - partInfo->block_start + 1;

                        partInfo->C_end = partInfo->block_end / sdInfo.unit;
                        partInfo->H_end = sdInfo.H_end - 1;
                        partInfo->S_end = sdInfo.S_end;
                }
                else
                {
                        partInfo->block_count   = count;

                        partInfo->block_end     = partInfo->block_start + count - 1;
                        partInfo->C_end         = partInfo->block_end / sdInfo.unit;

                        temp                    = partInfo->block_end % sdInfo.unit;
                        partInfo->H_end         = temp / sdInfo.S_end;
                        partInfo->S_end         = temp % sdInfo.S_end + 1;
                }
        }
//-----------------------------------------------------
        else
        {
                partInfo->C_start       = 0;
                partInfo->H_start       = 1;
                partInfo->S_start       = 1;

                partInfo->C_end         = 1023;
                partInfo->H_end         = 254;
                partInfo->S_end         = 63;

                if (count == BLOCK_END)
                {
                        _10MB_unit = calc_unit(_10MB, sdInfo);
                        partInfo->block_end     = sdInfo.total_block_count - _10MB_unit - 1;
                        partInfo->block_count   = partInfo->block_end - partInfo->block_start + 1;

                }
                else
                {
                        partInfo->block_count   = count;
                        partInfo->block_end     = partInfo->block_start + count - 1;
                }
        }
}

/////////////////////////////////////////////////////////////////
int make_mmc_partition(u32 total_block_count, unsigned char *mbr, char * argv[])
{
	int		block_start = 0, block_offset;

	SDInfo		sdInfo;
	PartitionInfo	partInfo[4];
	unsigned int  capacity = total_block_count*512;
	
	if(strcmp(argv[3],"-fat") == 0)
	{
		fat_size = simple_strtoul(argv[4], NULL, 16);
		if(strcmp(argv[5], "-ext") == 0)
			ext_size = simple_strtoul(argv[6], NULL, 16);
		else
			goto err_usage;
	}
	else if(strcmp(argv[3], "-ext") == 0)
	{
		ext_size = simple_strtoul(argv[4], NULL, 16);
		if(strcmp(argv[5],"-fat") == 0)
			fat_size = simple_strtoul(argv[6], NULL, 16);
		else
			goto err_usage;
	}

///////////////////////////////////////////////////////////	
	memset((unsigned char *)&sdInfo, 0x00, sizeof(SDInfo));
	memset((unsigned char *)&partInfo, 0x0, sizeof(PartitionInfo)*4);

///////////////////////////////////////////////////////////	
	get_SDInfo(total_block_count, &sdInfo);

//for preloader, the postion must be accurate for bootrom
///////////////////////////////////////////////////////////
	block_start	= PRELOADER_PART_OFFSET/(BLOCK_SIZE); 
	block_offset	= PRELOADER_PART_SIZE/(BLOCK_SIZE);
	//printf("preloader_start = %d, preloader_size=%d\n", block_start, block_offset);
	partInfo[PRELOADER_PART_NO - 1].bootable	= 0x00;
	partInfo[PRELOADER_PART_NO - 1].partitionId	= PRELOADER_PART_TYPE;

	make_partitionInfo(block_start, block_offset, sdInfo, &partInfo[PRELOADER_PART_NO - 1]);

//for rootfs
///////////////////////////////////////////////////////////	
	if( ext_size >= capacity)
	{
		ext_size = capacity - 200*_1MB; // reserve 200MB for fat
		printf("the ext partition is too big!and ext is tuned to %u bytes\n", ext_size);
	}
	block_start  =  calc_unit(ROOTFS_PART_OFFSET, sdInfo);
	block_offset = calc_unit(ext_size, sdInfo);
	
	partInfo[ROOTFS_PART_NO - 1].bootable	= 0x00;
	partInfo[ROOTFS_PART_NO - 1].partitionId	= ROOTFS_PART_TYPE;

	make_partitionInfo(block_start, block_offset, sdInfo, &partInfo[ROOTFS_PART_NO - 1]);

//for kernel and dtb
///////////////////////////////////////////////////////////	
	fat_start = ROOTFS_PART_OFFSET + ext_size + 5*_1MB;
	if( (fat_start + fat_size ) >= capacity)
	{
		fat_size = capacity - fat_size;  // if ext is very large, fat max 200MB for fat.
	}
	block_start  = calc_unit(fat_start, sdInfo);
	block_offset = calc_unit(fat_size, sdInfo);
	
	partInfo[FAT_PART_NO - 1].bootable	= 0x00;
	partInfo[FAT_PART_NO - 1].partitionId	= FAT_PART_TYPE;

	make_partitionInfo(block_start, block_offset, sdInfo, &partInfo[FAT_PART_NO - 1]);


///////////////////////////////////////////////////////////	
	memset(mbr, 0x00, sizeof(mbr));
	mbr[510] = 0x55; mbr[511] = 0xAA;
	
	encode_partitionInfo(partInfo[FAT_PART_NO - 1],			mbr + mbr_part_offset[FAT_PART_NO]);
	encode_partitionInfo(partInfo[ROOTFS_PART_NO -1],		mbr + mbr_part_offset[ROOTFS_PART_NO]);
	encode_partitionInfo(partInfo[PRELOADER_PART_NO - 1],	mbr + mbr_part_offset[PRELOADER_PART_NO]);
	
	return 0;
	
err_usage:
	printf("Usage: fdisk <-c or -p> <device_num> [-fat size -ext size]\n");
	return -1;
}

/////////////////////////////////////////////////////////////////
int get_mmc_block_count(char *device_name)
{
	int rv;
	struct mmc *mmc;
	u32 block_count = 0;
	int dev_num;

	dev_num = simple_strtoul(device_name, NULL, 0);
	
	mmc = find_mmc_device(dev_num);
	if (!mmc)
	{
		printf("mmc/sd device is NOT founded.\n");
		return -1;
	}	
	
	rv = mmc_init(mmc);
	if (rv)
	{
		printf("mmc/sd device's initialization is failed.\n");
		return -1;
	}
	/* convert the capacity into block count, and not Byte */
	block_count = mmc->capacity / (BLOCK_SIZE);
		
	printf("block_count = %d, capacity = %lu\n", block_count, mmc->capacity);
	return block_count;
}

/////////////////////////////////////////////////////////////////
int get_mmc_mbr(char *device_name, unsigned char *mbr)
{
	int rv;
	struct mmc *mmc;
	int dev_num;

	dev_num = simple_strtoul(device_name, NULL, 0);
	
	mmc = find_mmc_device(dev_num);
	if (!mmc)
	{
		printf("mmc/sd device is NOT founded.\n");
		return -1;
	}	
	
	rv = mmc_init(mmc);
	if (rv)
	{
		printf("mmc/sd device's initialization is failed.\n");
		return -1;
	}

	rv = mmc->block_dev.block_read(dev_num, 0, 1, mbr);

	if(rv == 1)
		return 0;
	else
		return -1; 
}

/////////////////////////////////////////////////////////////////
int put_mmc_mbr(unsigned char *mbr, char *device_name)
{
	int rv;
	struct mmc *mmc;
	int dev_num;

	dev_num = simple_strtoul(device_name, NULL, 0);
	
	mmc = find_mmc_device(dev_num);
	if (!mmc)
	{
		printf("mmc/sd device is NOT founded.\n");
		return -1;
	}	
	
	rv = mmc_init(mmc);
	if (rv)
	{
		printf("mmc/sd device's initialization is failed.\n");
		return -1;
	}

	rv = mmc->block_dev.block_write(dev_num, 0, 1, mbr);

	if(rv == 1)
		return 0;
	else
		return -1; 
}

/////////////////////////////////////////////////////////////////
int get_mmc_part_info(char *device_name, int part_num, int *block_start, int *block_count, unsigned char *part_Id)
{
	int		rv;
	PartitionInfo	partInfo;
	unsigned char	mbr[512];
	
	rv = get_mmc_mbr(device_name, mbr);
	if(rv !=0)
		return -1;
				
	switch(part_num)
	{
		case 1:
			decode_partitionInfo(&mbr[0x1BE], &partInfo);
			*block_start	= partInfo.block_start;	
			*block_count	= partInfo.block_count;	
			*part_Id 	= partInfo.partitionId;	
			break;
		case 2:
			decode_partitionInfo(&mbr[0x1CE], &partInfo);
			*block_start	= partInfo.block_start;	
			*block_count	= partInfo.block_count;	
			*part_Id 	= partInfo.partitionId;	
			break;
		
		case 3:
			decode_partitionInfo(&mbr[0x1DE], &partInfo);
			*block_start	= partInfo.block_start;	
			*block_count	= partInfo.block_count;	
			*part_Id 	= partInfo.partitionId;	
			break;
		case 4:
			decode_partitionInfo(&mbr[0x1EE], &partInfo);
			*block_start	= partInfo.block_start;	
			*block_count	= partInfo.block_count;	
			*part_Id 	= partInfo.partitionId;	
			break;
		default:
			return -1;
	}	

	return 0;
}

/////////////////////////////////////////////////////////////////
int print_mmc_part_info(int argc, char *argv[])
{
	int		rv;

	PartitionInfo	partInfo[4];
	
	rv = get_mmc_part_info(argv[2], 1, &(partInfo[0].block_start), &(partInfo[0].block_count),
			&(partInfo[0].partitionId) );
	
	rv = get_mmc_part_info(argv[2], 2, &(partInfo[1].block_start), &(partInfo[1].block_count),
			&(partInfo[1].partitionId) );

	rv = get_mmc_part_info(argv[2], 3, &(partInfo[2].block_start), &(partInfo[2].block_count),
			&(partInfo[2].partitionId) );

	rv = get_mmc_part_info(argv[2], 4, &(partInfo[3].block_start), &(partInfo[3].block_count),
			&(partInfo[3].partitionId) );

	printf("\n");	
	printf("partion #    size(MB)     block start #    block count    partition_Id \n");

	if ( (partInfo[0].block_start !=0) && (partInfo[0].block_count != 0) ) 
		printf("   1        %6d         %8d        %8d          0x%.2X \n",
			(partInfo[0].block_count / 2048), partInfo[0].block_start,
			partInfo[0].block_count, partInfo[0].partitionId);
	
	if ( (partInfo[1].block_start !=0) && (partInfo[1].block_count != 0) ) 
		printf("   2        %6d         %8d        %8d          0x%.2X \n",
			(partInfo[1].block_count / 2048), partInfo[1].block_start,
			partInfo[1].block_count, partInfo[1].partitionId);
	
	if ( (partInfo[2].block_start !=0) && (partInfo[2].block_count != 0) ) 
		printf("   3        %6d         %8d        %8d          0x%.2X \n",
			(partInfo[2].block_count / 2048), partInfo[2].block_start,
			partInfo[2].block_count, partInfo[2].partitionId);

	if ( (partInfo[3].block_start !=0) && (partInfo[3].block_count != 0) ) 
		printf("   4        %6d         %8d        %8d          0x%.2X \n",
			(partInfo[3].block_count / 2048), partInfo[3].block_start,
			partInfo[3].block_count, partInfo[3].partitionId);

	return 1;
}

/////////////////////////////////////////////////////////////////
int create_mmc_fdisk(int argc, char *argv[])
{
	int		rv;
	u32		total_block_count;
	unsigned char	mbr[512];
	int ret = 0;

	memset(mbr, 0x00, 512);
	
	total_block_count = get_mmc_block_count(argv[2]);
	
	printf("begin make partion!\n");
	ret = make_mmc_partition(total_block_count, mbr, argv);
	if(ret < 0)
	{
		return -1;
	}

	printf("begin save partion to sd/mmc\n");
	rv = put_mmc_mbr(mbr, argv[2]);
	if (rv != 0)
		return -2;
		
	printf("fdisk is completed\n");

	argv[1][1] = 'p';
	print_mmc_part_info(argc, argv);
	return 0;
}

/////////////////////////////////////////////////////////////////
int do_fdisk(cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{	
	if ( argc < 3 )
	{
		printf("Usage: fdisk <-c or -p> <device_num> [-fat size -ext size]\n");
		return 0;
	}

	printf("do_fdisk 1\n");
	if ( strcmp(argv[1], "-c") == 0 )
		return create_mmc_fdisk(argc, argv);

	else if ( strcmp(argv[1], "-p") == 0 )
		return print_mmc_part_info(argc, argv);
	
	printf("Usage: fdisk <-c or -p> <device_num>\n");
	return 0;
}

U_BOOT_CMD(
        fdisk, 7, 0, do_fdisk,
        "fdisk\t- fdisk for sd/mmc.\n",
        "-c <device_num>\t- create partition.\n"
        "fdisk -p <device_num>\t- print partition information\n"
);

