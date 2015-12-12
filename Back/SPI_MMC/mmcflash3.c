#include "c8051f120.h"
#include "mmcflash3.h"
#include <intrins.h>
#include <stdio.h>                     
#include <ctype.h> 


//KZA
//xdata unsigned char BufferInKZA[100];
xdata unsigned char MMC_CMD = 0xff;

xdata unsigned int i_CMD = 0x00;
xdata unsigned char i_RST = 0x00;
xdata unsigned long timeFlashOver=0;
xdata unsigned char b;

static xdata unsigned char Buffer[512];
xdata unsigned char flash=0;

xdata unsigned long address;// = 0x00000A00;


//SPI------------------------------------------------------------------
void SPI_Init (void)
{
	SPI0CFG = 0x40;
	SPI0CN = 0x0F;
	SPI0CKR = 0x08; //2.8 Mhz
}                                                                                                                                                                                                                                                                                                   

//----------------------------------------------------------------------
void SPI_isr(void) interrupt 6
{
	xdata unsigned int i;
	xdata unsigned char a_tmp;
	xdata char SFRPAGE_SAVE = SFRPAGE;
	SFRPAGE = SPI0_PAGE;
	
	SPIF = 0;
	if(MMC_CMD == 0xFE)
	{
		flash=0;
		CS = 1;
	}
	else if(MMC_CMD == 0xFF)
	{
		flash=1;
		CS = 1;
	}
	else if(MMC_CMD == 0x00) //init------------------------
	{
		flash=0;
		if(i_RST==0x00)
		{
			CS = 1;
			i_RST++;
			SPI0DAT = 0xFF;
		}
		else if((i_RST > 0x00)&&(i_RST < 0x09))
		{
			i_RST++;
			SPI0DAT = 0xFF;
		}
		else if(i_RST == 0x09)
		{
			CS = 0;
			i_RST++;
			SPI0DAT = 0xFF;
		}
		else
		{
//CMD0------------------------------------- 
			MMC_CMD = 0x01;
			i_RST = 0x00;
			SPI0DAT = 0x40;
		}
	}
	else if((MMC_CMD == 0x01) && (i_CMD < 0x04))
	{
		i_CMD++;			
		SPI0DAT = 0x00;
	}
	else if((MMC_CMD == 0x01) && (i_CMD >= 0x04))
	{
		i_CMD = 0x00;
		MMC_CMD = 0x02;
		SPI0DAT = 0x95;
	}
	else if(MMC_CMD == 0x02)
	{
		MMC_CMD = 0x03;
		SPI0DAT = 0xFF;
	}
	else if(MMC_CMD == 0x03)
	{
		b = SPI0DAT;
		timeFlashOver++;
		if(timeFlashOver > 0x00d00000)
		{
			timeFlashOver = 0;
			MMC_CMD = 0xFF;	
		}
		
		if(b != 0xFF);
		{
			MMC_CMD=0x04;
		}
		SPI0DAT = 0xFF;
	}
	else if(MMC_CMD == 0x04)
	{
		timeFlashOver++;
		if(timeFlashOver > 0x00d00000)
		{
			timeFlashOver=0;
			MMC_CMD = 0xFF;
		}
		if(SPI0DAT == 0x00)
		{
			CS = 1;
			flash = 1;
			MMC_CMD = 0xFF;
		}
		if(SPI0DAT == 0xFF)
		{
			MMC_CMD = 0x05;
			timeFlashOver = 0;
			CS = 1;
		}
		SPI0DAT = 0xFF;
//-----------------------------------------
	}
	else if(MMC_CMD == 0x05)
	{
		if(i_RST < 0x09)
		{
			i_RST++;
			SPI0DAT = 0xFF;
		}
		else
		{
//CMD1--------------------------------------  
			CS = 0;
			i_RST = 0x00;
			i_CMD = 0;
			MMC_CMD = 0x06;
			SPI0DAT = 0x41;
		}
	}
	else if((MMC_CMD == 0x06) && (i_CMD < 0x04))
	{
		i_CMD++;
		SPI0DAT = 0x00;
	}
	else if((MMC_CMD == 0x06) && (i_CMD >= 0x04))
	{
		i_CMD = 0x00;
		MMC_CMD=0x07;
		SPI0DAT = 0xFF;
	}
	else if(MMC_CMD == 0x07)
	{
		timeFlashOver++;
		if(timeFlashOver > 0x00d00000)
		{
			timeFlashOver = 0;
			flash = 0;
			CS = 1;
			MMC_CMD = 0xFE;
		}
		b = SPI0DAT;
		if(b != 0xFF)
			MMC_CMD=0x08;
		SPI0DAT = 0xFF;
	}
	else if(MMC_CMD == 0x08)
	{
		timeFlashOver++;
		if(timeFlashOver > 0x00d00000)
		{
			timeFlashOver=0;
			CS = 1;
			flash = 1;
			MMC_CMD = 0xFF;
		}
		if(SPI0DAT == 0xFF)
		{
			MMC_CMD = 0x09;
		}
		SPI0DAT = 0xFF;
	}
	else if(MMC_CMD == 0x09)
	{
		if(b & 0x01)
		{
			MMC_CMD = 0x05;
		}
		else
		{
			CS = 1;
			MMC_CMD = 0x0A;
		}
		SPI0DAT = 0xFF;
	}
	else if(MMC_CMD == 0x0A)
	{
		flash = 1;
		CS = 1;
		MMC_CMD = 0xFF;
	}
	else if(MMC_CMD == 0x10) //write------------------------
	{
		flash=0;
		CS = 0;
		MMC_CMD = 0x11;
		SPI0DAT = 0xFF;
	}
	else if(MMC_CMD == 0x11) 
	{
		MMC_CMD = 0x12;
		SPI0DAT = 0x58;
	}
	else if(MMC_CMD == 0x12) 
	{
		MMC_CMD = 0x13;
		a_tmp = ((address & 0xff000000) >> 24);
		Buffer[0] = a_tmp;
		SPI0DAT = a_tmp;
	}
	else if(MMC_CMD == 0x13) 
	{
		MMC_CMD = 0x14;
		a_tmp = ((address & 0x00ff0000) >> 16);
		Buffer[1] = a_tmp;
		SPI0DAT = a_tmp;
	}
	else if(MMC_CMD == 0x14) 
	{
		MMC_CMD = 0x15;
		a_tmp = ((address & 0x0000ff00) >> 8);
		Buffer[2] = a_tmp;
		SPI0DAT = a_tmp;
	}
	else if(MMC_CMD == 0x15) 
	{
		MMC_CMD = 0x16;
		a_tmp = address & 0x000000ff; 
		Buffer[3] = a_tmp;
		SPI0DAT = a_tmp;
	}
	else if(MMC_CMD == 0x16) 
	{
		address += 0x00000200;
		if(address > 0x001E3000) 
			address = 0x00000A00; //1Gb
		MMC_CMD = 0x17;
		SPI0DAT = 0xFF;
	}
	else if(MMC_CMD == 0x17)
	{
		MMC_CMD = 0x18;
		SPI0DAT = 0xFF;
	}
	else if(MMC_CMD == 0x18)
	{
		b = SPI0DAT;	// 00-ok, 20-Adress Err, 40,60-Param Err
		timeFlashOver++;
		if(timeFlashOver > 0x0000F000)
		{
			timeFlashOver=0;
			CS = 1;
			flash = 1;
			MMC_CMD = 0x00;
			address -= 0x00000200;
		}

		if(b == 0x00)
		{
			//OK
			MMC_CMD = 0x19;
		}
		else if(b == 0x20)
		{
			address = 0x00000A00;
			CS = 1;
			flash = 1;
			MMC_CMD = 0xFF;
		}
		else if(b == 0x40 || b == 0x60)
		{
			timeFlashOver=0;
			CS = 1;
			flash = 0;
			MMC_CMD = 0xFE;
		}
	
		SPI0DAT = 0xFF;
	}
	else if(MMC_CMD == 0x19)
	{
		timeFlashOver++;
		if(timeFlashOver > 0x0000F000)
		{
			timeFlashOver=0;
			CS = 1;
			flash = 1;
			MMC_CMD = 0xFF;
		}
		if(SPI0DAT==0xFF)
		{
			MMC_CMD = 0x1A;
		}
		SPI0DAT = 0xFF;
	}
	else if(MMC_CMD == 0x1A)
	{
		MMC_CMD = 0x1B;
		SPI0DAT = 0xFE;	
	}
	else if(MMC_CMD == 0x1B)
	{
		if(i_CMD >= 0x200)
		{
			i_CMD = 0;
			MMC_CMD = 0x1C;
			SPI0DAT = 0xFF;
		}
		else
		{
			SPI0DAT = Buffer[i_CMD++];
		}
	}
	else if(MMC_CMD == 0x1C)
	{
		MMC_CMD = 0x1D;
		SPI0DAT = 0xFF;
	}
	else if(MMC_CMD == 0x1D)
	{
		if(SPI0DAT == 0xFF)
		{
			MMC_CMD = 0x1E;
			CS = 1;
		}
		SPI0DAT = 0xFF;
	}
	else if(MMC_CMD == 0x1E)
	{
		MMC_CMD = 0xFF;
		CS = 1;
		flash = 1;
		SPI0DAT = 0xFF;
	}
//------------------------
	SFRPAGE = SFRPAGE_SAVE;
	return;
}

//-----------------------------------------------------------------------
void MMC_init(void)
{
	MMC_CMD = 0x00;
	SPIF = 1;
	return;
}

//-----------------------------------------------------------------------
void WriteMMC(unsigned char byte)
{
	static xdata unsigned int counterBuf = 4;
	if(MMC_CMD == 0xFE)
		return;
	Buffer[counterBuf++] = byte;	
	if((counterBuf >= 512)&&(flash == 1))
	{
		counterBuf = 4;
		MMC_CMD = 0x10;
		SPIF = 1;
	}
	else if(flash == 3)
	{
	//--------------- SPI MMC end ------------
		MMC_CMD = 0xfe;
	}

	return;
}

