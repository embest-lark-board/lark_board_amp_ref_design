#include <sys/mman.h>
#include <sys/types.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>

#define	BM_REGION_SIZE		(0x100000UL)
#define	AMP_REGION_START	(0x1E000000UL)
#define	AMP_REGION_END		(0x1FFFFFFFUL)
#define	AMP_REGION_SIZE		(0x2000000UL - BM_REGION_SIZE)

int main(int argc, char** argv) // map a normal file as shared mem:
{
	int fd, i, j;
	int *p, *pa;
	int phy_addr, size;

	if(argc != 4)
	{
		printf("usage: #amp_app /dev/amp phy_addr size\n");
		return -1;
	}

	phy_addr = atoi(argv[2]);
	size = atoi(argv[3]);

	if(phy_addr < (AMP_REGION_START + BM_REGION_SIZE) || phy_addr > AMP_REGION_END)
	{
		printf("phy_addr out of amp share region\n");
		return -1;
	}

	if((phy_addr&0x3) != 0)
	{
		printf("phy_addr should be dword align\n");
		return -1;
	}

	if( size > (AMP_REGION_END + 1 - phy_addr))
	{
		printf("size out of amp share region\n");
		return -1;
	}

	fd = open(argv[1], O_RDWR, 00666);
	if(fd < 0)
	{
		printf("open dev %s failed\n", argv[1]);
		return -1;
	}

	p = (unsigned int*) mmap(NULL, AMP_REGION_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0);
	if(p < 0)
	{
		printf("mmap faild\n");
		return -1;
	}

	pa = p + ((phy_addr - AMP_REGION_START - BM_REGION_SIZE) >> 2);
	for(i = 0; i < (size/4/8); i++)
	{
		for(j = 0; j < 8; j++)
		{
			printf("%08x ", *(pa + (i*8) + j));
		}
		printf("\n");
	}

	munmap(p, AMP_REGION_SIZE);
	close(fd);
	return 0;
}
