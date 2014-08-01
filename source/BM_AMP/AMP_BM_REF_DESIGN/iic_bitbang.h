#ifndef __IIC_BITBANG_H__
#define __IIC_BITBANG_H__

#define GPD1CON		*(volatile u32 *)(HPS_GPIO2_BASE_ADDR + HPS_GPIO_DIR_OFFSET)
#define GPD1DAT		*(volatile u32 *)HPS_GPIO2_BASE_ADDR


#define IIC_ESCL_Hi	GPD1DAT |= (0x1<<6)
#define IIC_ESCL_Lo	GPD1DAT &= ~(0x1<<6)
#define IIC_ESDA_Hi	GPD1DAT |= (0x1<<5)
#define IIC_ESDA_Lo	GPD1DAT &= ~(0x1<<5)

#define IIC_ESCL_INP	GPD1CON &= ~(0x1<<6)
#define IIC_ESCL_OUTP	GPD1CON |= (0x1<<6)

#define IIC_ESDA_INP	GPD1CON &= ~(0x1<<5)
#define IIC_ESDA_OUTP	GPD1CON |= (0x1<<5)

#define DELAY			0x120


#define LCD_ADDR	(0x28)	// when SRAD pin = 0, CC/CDh is selected

#define LCD_COMMAND             0xfe
#define LCD_DISPLAY_ON          0x41
#define LCD_DISPLAY_OFF         0x42
#define LCD_SET_CURSOR          0x45
#define LCD_BACKSPACE           0x4e
#define LCD_CLEAR_SCREEN        0x51

#define ASCII_BS                0x08
#define ASCII_LF                0x0a
#define ASCII_CR                0x0d
#define ASCII_ESC               0x1b
#define ASCII_SPACE             0x20


void IIC_InitIp(void);
void IIC_EWrite (unsigned char ChipId, unsigned char IicAddr, unsigned char IicData);
void IIC_EXfer (unsigned char ChipId, char *cBuf, unsigned char cLen);
void LCD_SetCursor(char line);



#endif /*__IIC_H__*/

