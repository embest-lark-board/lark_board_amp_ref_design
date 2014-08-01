#include <types.h>
#include <regs.h>
#include <iic_bitbang.h>

void Delay(void)
{
	volatile u32 i;
	for(i=0;i<DELAY;i++);
}

void SCLH_SDAH(void)
{
	IIC_ESCL_Hi;
	IIC_ESDA_Hi;
	Delay();
}

void SCLH_SDAL(void)
{
	IIC_ESCL_Hi;
	IIC_ESDA_Lo;
	Delay();
}

void SCLL_SDAH(void)
{
	IIC_ESCL_Lo;
	IIC_ESDA_Hi;
	Delay();
}

void SCLL_SDAL(void)
{
	IIC_ESCL_Lo;
	IIC_ESDA_Lo;
	Delay();
}

void IIC_ELow(void)
{
	SCLL_SDAL();
	SCLH_SDAL();
	SCLH_SDAL();
	SCLL_SDAL();
}

void IIC_EHigh(void)
{
	SCLL_SDAH();
	SCLH_SDAH();
	SCLH_SDAH();
	SCLL_SDAH();
}

void IIC_EStart(void)
{
	SCLH_SDAH();
	SCLH_SDAL();
	Delay();
	SCLL_SDAL();
}

void IIC_EEnd(void)
{
	SCLL_SDAL();
	SCLH_SDAL();
	Delay();
	SCLH_SDAH();
}

void IIC_EAck(void)
{
	unsigned long ack;

	IIC_ESDA_INP;			// Function <- Input

	IIC_ESCL_Lo;
	Delay();
	IIC_ESCL_Hi;
	Delay();
	ack = GPD1DAT;
	IIC_ESCL_Hi;
	Delay();
	IIC_ESCL_Hi;
	Delay();

	IIC_ESDA_OUTP;			// Function <- Output (SDA)

	ack = (ack>>4)&0x1;
	while(ack!=0);

	SCLL_SDAL();
}

void IIC_ESetport(void)
{
	//GPD1PUD &= ~(0xf<<28);	// Pull Up/Down Disable	SCL, SDA

	IIC_ESCL_Hi;
	IIC_ESDA_Hi;

	IIC_ESCL_OUTP;		// Function <- Output (SCL)
	IIC_ESDA_OUTP;		// Function <- Output (SDA)

	Delay();
}

void IIC_EWrite (unsigned char ChipId, unsigned char IicAddr, unsigned char IicData)
{
	unsigned long i;

	IIC_EStart();

////////////////// write chip id //////////////////
	for(i = 7; i>0; i--)
	{
		if((ChipId >> (i-1)) & 0x0001)
			IIC_EHigh();
		else
			IIC_ELow();
	}

	IIC_ELow();	// write 'W'

	IIC_EAck();	// ACK

////////////////// write reg. addr. //////////////////
	for(i = 8; i>0; i--)
	{
		if((IicAddr >> (i-1)) & 0x0001)
			IIC_EHigh();
		else
			IIC_ELow();
	}

	IIC_EAck();	// ACK

////////////////// write reg. data. //////////////////
	for(i = 8; i>0; i--)
	{
		if((IicData >> (i-1)) & 0x0001)
			IIC_EHigh();
		else
			IIC_ELow();
	}

	IIC_EAck();	// ACK

	IIC_EEnd();
}

void IIC_EXfer (unsigned char ChipId, char *cBuf, unsigned char cLen)
{
	unsigned long i, j;

	IIC_EStart();

////////////////// write chip id //////////////////
	for(i = 7; i>0; i--)
	{
		if((ChipId >> (i-1)) & 0x0001)
			IIC_EHigh();
		else
			IIC_ELow();
	}

	IIC_ELow();	// write 'W'

	IIC_EAck();	// ACK

	for(j = 0; j < cLen; j++)
	{
		for(i = 8; i>0; i--)
		{
			if((cBuf[j] >> (i-1)) & 0x0001)
				IIC_EHigh();
			else
				IIC_ELow();
		}

		IIC_EAck();	// ACK
	}

	IIC_EEnd();
}


struct tAddrVal
{
	unsigned char Addr;
	unsigned char Val;
};

void LCD_SetCursor(char line)
{
	char cBuf[3] ={LCD_COMMAND, LCD_SET_CURSOR, 0x40};
	cBuf[2] = 0x40*line;
	IIC_EXfer(LCD_ADDR, cBuf, 0x3);
}


void IIC_InitIp(void)
{
	IIC_ESetport();
	IIC_EWrite(LCD_ADDR, LCD_COMMAND, LCD_DISPLAY_ON);
	IIC_EWrite(LCD_ADDR, LCD_COMMAND, LCD_CLEAR_SCREEN);

}

