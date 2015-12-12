#include "c8051f120.h"
#include "mmcflash.h"
#include <intrins.h>
#include <stdio.h>                     
#include <ctype.h> 

//SPI------------------------------------------------------------------
void SPI_Init (void)
{
	SPI0CFG = 0x40;
	SPI0CN = 0x0F;
	SPI0CKR = 0x08; //2.8 Mhz
}                                                                                                                                                                                                                                                                                                   

//-----------------------------------------------------------------------
char MMC_init(void)
{
	xdata unsigned int i, i2;
	xdata unsigned char b;
	xdata unsigned long flesh_init_timer=0;

	xdata char SFRPAGE_SAVE = SFRPAGE;
	SFRPAGE = SPI0_PAGE;

	CS = 1;
	for (i = 0; i < 10; i++)
	{
		SPI0DAT = 0xFF;	while(SPIF == 0) ;   SPIF = 0;
	}

	CS = 0;
//CMD0 
	SPI0DAT = 0x40;	while(SPIF == 0) ;   SPIF = 0;
	SPI0DAT = 0x00;   while(SPIF == 0) ;   SPIF = 0;
	SPI0DAT = 0x00;   while(SPIF == 0) ;   SPIF = 0;
	SPI0DAT = 0x00;   while(SPIF == 0) ;   SPIF = 0;
	SPI0DAT = 0x00;	while(SPIF == 0) ;   SPIF = 0;
	SPI0DAT = 0x95;   while(SPIF == 0) ;   SPIF = 0;
	
	do
	{
		SPI0DAT = 0xFF; while(SPIF == 0)	;	SPIF = 0;
		b = SPI0DAT;
		flesh_init_timer++;
		if (flesh_init_timer > 0x00dffff0)
			return 0;		
	} while(b == 0xFF);
	while(SPI0DAT != 0xFF)
	{
		SPI0DAT = 0xFF; while(SPIF == 0) ; 	SPIF = 0;
	}

	do
	{
		//CMD1 	
		CS = 1;
		for (i = 0; i < 10; i++)
		{
			SPI0DAT = 0xFF; while(SPIF == 0) ; 	SPIF = 0;
		}

		CS = 0;

		SPI0DAT = 0x41; while(SPIF == 0) ; 	SPIF = 0;
		SPI0DAT = 0x00; while(SPIF == 0) ; 	SPIF = 0;
		SPI0DAT = 0x00; while(SPIF == 0) ; 	SPIF = 0;
		SPI0DAT = 0x00; while(SPIF == 0) ; 	SPIF = 0;
		SPI0DAT = 0x00; while(SPIF == 0) ; 	SPIF = 0;
		SPI0DAT = 0xFF; while(SPIF == 0) ; 	SPIF = 0;
   	b = SPI0DAT;
		do
		{
			SPI0DAT = 0xFF; while(SPIF == 0) ; 	SPIF = 0;
			b = SPI0DAT;
		} while(b == 0xFF);
	
		while(SPI0DAT != 0xFF) 
		{
			SPI0DAT = 0xFF; while(SPIF == 0) ; 	SPIF = 0;
		}	
	
		for (i2 = 0; i2 < 15; i2++)
		{
			for (i = 0; i < 65534; i++)
			{
				_nop_();
			}
		}
	} while ((b & 0x01));
	CS = 1; 
	SPI0DAT = 0xFF; while(SPIF == 0) ; 	SPIF = 0;
	SFRPAGE = SFRPAGE_SAVE;
	return 1;
}

//-----------------------------------------------------------------------------------------
char WriteByteInKZA(unsigned char Byte)
{ 
	static xdata unsigned char Buffer[512];
	static xdata int nByte = 0;
	static xdata unsigned long address = 0;
	xdata unsigned char b;
	xdata unsigned int i = 0;
	xdata unsigned long flesh_init_timer=0;

	Buffer[nByte++] = Byte;
	if (nByte >= 512) 
	{
		SFRPAGE = SPI0_PAGE;

		SPI0DAT = 0xFF; while(SPIF == 0) ; 	SPIF = 0;
		CS = 0;
		SPI0DAT = 0x58; while(SPIF == 0) ; 	SPIF = 0;
		SPI0DAT = (address & 0xff); while(SPIF == 0) ; 	SPIF = 0;
		SPI0DAT = (address & 0xff00) >> 8; while(SPIF == 0) ; 	SPIF = 0;
		SPI0DAT = (address & 0xff0000) >> 16; while(SPIF == 0) ; 	SPIF = 0;
		SPI0DAT = (address & 0xff000000) >> 24; while(SPIF == 0) ; 	SPIF = 0;
		SPI0DAT = 0xFF; while(SPIF == 0) ; 	SPIF = 0;
		do
		{
			SPI0DAT = 0xFF; while(SPIF == 0) ; 	SPIF = 0;
			b = SPI0DAT;	// 00-ok, 20-Adress Err, 40,60-Param Err
			if(b!=0xFF && b!=0x00)
				return 0;
			flesh_init_timer++;
			if (flesh_init_timer > 0x00dffff0)
				return 0;
		} while(b == 0xFF);
	
		while(SPI0DAT != 0xFF) 
		{
			SPI0DAT = 0xFF; while(SPIF == 0) ; 	SPIF = 0;
		}
		SPI0DAT = 0xFE; while(SPIF == 0) ; 	SPIF = 0;
		for(i=0; i<512; i++)
		{
			SPI0DAT = Buffer[i]; while(SPIF == 0) ; 	SPIF = 0;
		}
		SPI0DAT = 0xFF; while(SPIF == 0) ; 	SPIF = 0; //crc1
		SPI0DAT = 0xFF; while(SPIF == 0) ; 	SPIF = 0; //crc0
		while(SPI0DAT != 0xFF) 
		{
			SPI0DAT = 0xFF; while(SPIF == 0) ; 	SPIF = 0;
		}
		CS = 1; 
		SPI0DAT = 0xFF; while(SPIF == 0) ; 	SPIF = 0;
		
		address += 512;
		nByte = 0;
	}
	return 1;
}											
