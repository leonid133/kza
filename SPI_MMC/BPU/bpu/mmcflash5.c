#include "c8051f120.h"
#include "mmcflash5.h"
#include <init.h>

xdata unsigned char flash = 0, MMC_CMD = 0xff, jump = 0, flageofrec = 0, BufferInKZA[512], BuffFromKza[512];
xdata unsigned long eofaddr=0x1200, address = 0x00000c00, szflash = 0, nzap = 0, lastaddr = 0, tmpaddr=0;
xdata unsigned int timeFlashOver, i_wr;

sbit CS = P3^2;

xdata unsigned char spifcount=0;

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
	static xdata unsigned long stoprec=0;
	static xdata unsigned char b=0xff;
	static xdata unsigned int i_CMD = 0x00;
	xdata unsigned char *pchar, dummy_CRC;
	xdata unsigned int size;
	xdata char SFRPAGE_SAVE = SFRPAGE;
	SFRPAGE = SPI0_PAGE;

	spifcount=0;
	
	SPIF = 0;
	flash = 0;
   if(MMC_CMD == 0xFF)
	{
		flash=1;
		CS = 1;
	}
	else if(MMC_CMD == 0x00) //init------------------------
	{
		i_CMD++;
		if(i_CMD == 1)
		{
			CS = 1;	
			SPI0DAT = 0xFF; 
		}
		else if(i_CMD < 10)
		{
			SPI0DAT = 0xFF; 
		}
		else if(i_CMD == 10)
		{
			CS = 0; 
			SPI0DAT = 0xFF; 
		}
		else		//CMD0------------------------------------- 
		{
			MMC_CMD = 0x01;
			i_CMD = 0x00;
			SPI0DAT = 0x40; 
		}
	}
	else if(MMC_CMD == 0x01)
	{
		if (i_CMD < 0x04)
		{
			i_CMD++;			
			SPI0DAT = 0x00; 
		}
		else 
		{
			i_CMD = 0x00;
			MMC_CMD = 0x02;
			SPI0DAT = 0x95;
		} 
	}
	else if(MMC_CMD == 0x02)
	{
		MMC_CMD = 0x03;
		SPI0DAT = 0xFF; 
	}
	else if(MMC_CMD == 0x03)
	{
		b = SPI0DAT;
		if(b != 0xFF);
		{
			MMC_CMD = 0x04;
			timeFlashOver = 0;
		}
		SPI0DAT = 0xFF; 
	}
	else if(MMC_CMD == 0x04)
	{
		b = SPI0DAT;
		if(b == 0x00)
		{
			timeFlashOver = 0;
			MMC_CMD = 0xFF;
		}
		else if(b == 0xFF)
		{
			timeFlashOver = 0;
			MMC_CMD = 0x05;
			CS = 1;
		}
		else
		{
			if(timeFlashOver > 10.0*FREQ)
			{
				MMC_CMD = 0; SPIF = 1;
				goto EndSpiIsr;
			}
		}
		SPI0DAT = 0xFF; 
	} //--end CMD0--
	else if(MMC_CMD == 0x05)
	{
		if(i_CMD < 0x09)
		{
			i_CMD++;
			SPI0DAT = 0xFF; 
		}
		else	//CMD1
		{
			CS = 0; 
			i_CMD = 0;
			MMC_CMD = 6;
			SPI0DAT = 0x41; 
		}
	}
	else if(MMC_CMD == 0x06) 
	{
		if (i_CMD < 0x04)
		{
			i_CMD++;
			SPI0DAT = 0x00; 
		}
		else
		{
			i_CMD = 0x00; 
			MMC_CMD = 0x07;
			SPI0DAT = 0xFF;
			timeFlashOver = 0;
		}
	}
	else if(MMC_CMD == 0x07)
	{
		b = SPI0DAT;
		if(b != 0xFF)
		{
			timeFlashOver = 0;
			MMC_CMD = 8;
		}
		else if(timeFlashOver > 10.0*FREQ)
		{
			MMC_CMD = 0; 
			SPIF = 1;
			goto EndSpiIsr;
		}
		SPI0DAT = 0xFF; 
	}
	else if(MMC_CMD == 0x08)
	{
		if(SPI0DAT == 0xFF)
		{
			MMC_CMD = 0x09;
		}
		else if(timeFlashOver > 10.0*FREQ)
		{
			MMC_CMD = 0; 
			SPIF = 1;
			goto EndSpiIsr;
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
		
		CS = 1;
		if(szflash > 0)
		{
			MMC_CMD = 0xFF;
		}
		else
		{
			MMC_CMD = 0x20; 
			SPIF = 1;
		}
	} //--end Init--
	else if(MMC_CMD == 0x10) //--write--
	{
		CS = 0;	
		MMC_CMD = 0x11; 
		stoprec++;
		if(stoprec>0xBB5D7)
		{
			CS = 1;
			MMC_CMD = 0xFE;
		}
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
		SPI0DAT = ((address & 0xff000000) >> 24);
	}
	else if(MMC_CMD == 0x13) 
	{
		MMC_CMD = 0x14;
		SPI0DAT = ((address & 0x00ff0000) >> 16);
	}
	else if(MMC_CMD == 0x14) 
	{
		MMC_CMD = 0x15;
		SPI0DAT = ((address & 0x0000ff00) >> 8);
	}
	else if(MMC_CMD == 0x15) 
	{
		MMC_CMD = 0x16;
		SPI0DAT = address & 0x000000ff;
	}
	else if(MMC_CMD == 0x16) 
	{
		address += 0x200;
		if(address > szflash) 
			address = 0xe00; 
		MMC_CMD = 0x17;
		SPI0DAT = 0xFF;
	}
	else if(MMC_CMD == 0x17)
	{
		timeFlashOver = 0;
		MMC_CMD = 0x18;
		SPI0DAT = 0xFF;
	}
	else if(MMC_CMD == 0x18)
	{
		b = SPI0DAT;	// 00-ok, 20-Adress Err, 40,60-Param Err
		if(b == 0x00)
		{
			timeFlashOver=0;
			MMC_CMD = 0x19;
		}
		else if(b == 0x20)
		{
			address = 0xe00;
			MMC_CMD = 0xFF;
		}
		else if(b == 0x40 || b == 0x60)
		{
			timeFlashOver=0;
			CS = 1;
			flash = 3;
			MMC_CMD = 0xFE;
		}
		else if(timeFlashOver > 10.0*FREQ)
		{
			timeFlashOver = 0;
			CS = 1; 
			flash = 0; 
			MMC_CMD = 0x00;
			address -= 0x200;
			SPIF = 1;
			goto EndSpiIsr;
		}
		SPI0DAT = 0xFF;
	}
	else if(MMC_CMD == 0x19)
	{
		b = SPI0DAT;
		if(b == 0xFF)
		{
			timeFlashOver=0;
			MMC_CMD = 0x1A;
		}
		else if(timeFlashOver > 10.0*FREQ)
		{
			address -= 0x200;
			CS = 1;
			flash = 0;
			MMC_CMD = 0x00;
			SPIF = 1;
			goto EndSpiIsr;
		}
		SPI0DAT = 0xFF;
	}
	else if(MMC_CMD == 0x1A)
	{
		i_CMD = 0;
		MMC_CMD = 0x1B; 
		SPI0DAT = 0xFE;
	}
	else if(MMC_CMD == 0x1B)
	{
		if(i_CMD >= 0x200)
		{
			i_CMD = 0;	
			MMC_CMD = 0x1C; 
			SPIF = 0; 
			SPI0DAT = 0xFF;
		}
		else
		{
			SPI0DAT = BufferInKZA[i_CMD++];
		}
	}
	else if(MMC_CMD == 0x1C)
	{
		timeFlashOver=0;
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
		else if(timeFlashOver > 10.0*FREQ)
		{
			address -= 0x200;
			CS = 1;
			flash = 0;
			MMC_CMD = 0x00;
			SPIF = 1;
			goto EndSpiIsr;
		}
		SPI0DAT = 0xFF;
	}
	else if(MMC_CMD == 0x1E)
	{
		if(jump > 0)
		{
			address = lastaddr;
			stoprec = 0;
			jump = 0;
		}

		if(i_CMD++ < 255)
		{
			SPI0DAT = 0xFF;
		}
		else
		{
			MMC_CMD = 0xFF;
			SPI0DAT = 0xFF;
		}
		
		SPI0DAT = 0xFF;
	}
	else if(MMC_CMD == 0x20)	//-- CMD 9 --
	{
		//-Ручной режим определения размера ММС-начало
		MMC_CMD = 0xFF;
		szflash = 0x3B600000; //950Mb
		SPIF = 1;
		goto EndSpiIsr;
		//-Ручной режим определения размера ММС-конец	
		CS = 1;
		flash = 0; 
		MMC_CMD = 0x21;	
		SPI0DAT = 0xFF;
	}
	else if(MMC_CMD == 0x21)
	{
		CS = 0; 
		MMC_CMD = 0x22; 
		SPI0DAT = 0xFF;
	}
	else if(MMC_CMD == 0x22)
	{
		MMC_CMD = 0x23; 
		SPI0DAT = (9 | 0x40);
	}
	else if(MMC_CMD == 0x23)
	{
		MMC_CMD = 0x24; 
		SPI0DAT = 0x00;
	}
	else if(MMC_CMD == 0x24)
	{
		MMC_CMD = 0x25; 
		SPI0DAT = 0x00;
	}
	else if(MMC_CMD == 0x25)
	{
		MMC_CMD = 0x26; 
		SPI0DAT = 0x00;
	}
	else if(MMC_CMD == 0x26)
	{
		MMC_CMD = 0x27; 
		SPI0DAT = 0xFF;
	}
	else if(MMC_CMD == 0x27)
	{
		b = SPI0DAT; 
		if(!(b & 0x80))
		{			
			timeFlashOver=0;
			MMC_CMD = 0x28;
		}
		SPI0DAT = 0xFF; 
	}
	else if(MMC_CMD == 0x28)
	{
		b = SPI0DAT;
		if(b == 0xFE)
		{
			timeFlashOver=0;
			i_CMD = 0;	
			MMC_CMD =0x2A;	
			SPI0DAT = 0x00; 
			goto EndSpiIsr;
		}
		else if(timeFlashOver > FREQ)
		{
			timeFlashOver=0;
			CS = 1;	
			flash = 0;	
			MMC_CMD = 0x00; 
			SPIF = 1;
			goto EndSpiIsr;
		}
		SPI0DAT = 0xFF; 
	}
	else if(MMC_CMD == 0x2A)
	{
		*pchar++ = SPI0DAT;
		if(i_CMD == 16)
		{
			i_CMD = 0;
			MMC_CMD = 0x2B;
			SPI0DAT = 0x00;
		}
		else
		{
			i_CMD++;
			SPI0DAT = 0x00;
		}
	}
	else if(MMC_CMD == 0x2B)
	{
		dummy_CRC = SPI0DAT;  
		MMC_CMD = 0x2C; 
		SPI0DAT = 0x00;
	}
	else if(MMC_CMD == 0x2C)
	{
		dummy_CRC = SPI0DAT;  
		CS = 1; 
		MMC_CMD = 0x2D; 
		SPIF = 1; 
	}
	else if(MMC_CMD == 0x2D)
	{
		MMC_CMD = 0xFF;
		pchar += 9;
		size = (unsigned int)((((*pchar) & 0x03) << 1) | (((*(pchar+1)) & 0x80) >> 7));
   	switch(size)                        
   	{                                   
      	case 1: szflash = 0x800000; break;
      	case 2: szflash = 0x1000000; break;
      	case 3: szflash = 0x2000000; break;
      	case 4: szflash = 0x4000000; break;
      	case 5: szflash = 0x8000000; break;
			case 6: szflash = 0x10000000; break;
			case 7: szflash = 0x20000000; break;
			case 8: szflash = 0x40000000; break; //1Gb
			case 9: szflash = 0x80000000; break;
      	default: szflash = 0xf0000000; break;
   	}		
		SPI0DAT = 0xFF;
		flash = 1;
	}//--end CMD9--
	else if(MMC_CMD == 0x30) //Read
	{
		flash = 4; 
		CS = 0; 
		MMC_CMD = 0x31;
		SPI0DAT = 0xFF;
	}
	else if(MMC_CMD == 0x31) 
	{
		MMC_CMD = 0x32; 
		SPI0DAT = 0x51;
	}
	else if(MMC_CMD == 0x32) 
	{
		MMC_CMD = 0x33; 
		SPI0DAT = (address & 0xff000000) >> 24;
	}
	else if(MMC_CMD == 0x33) 
	{
		MMC_CMD = 0x34; 
		SPI0DAT = (address & 0xff0000) >> 16;
	}
	else if(MMC_CMD == 0x34) 
	{
		MMC_CMD = 0x35; 
		SPI0DAT = (address & 0xff00) >> 8;
	}
	else if(MMC_CMD == 0x35) 
	{
		MMC_CMD = 0x36; 
		SPI0DAT = (address & 0xff);
	}
	else if(MMC_CMD == 0x36) 
	{
		timeFlashOver=0;
		MMC_CMD = 0x37; 
		SPI0DAT = 0xFF;
	}
	else if(MMC_CMD == 0x37) 
	{
		b = SPI0DAT;	// 00-ok, 20-Adress Err, 60-Param Err
		if(b == 0x00)
		{
			MMC_CMD = 0x38;
			timeFlashOver=0;
		}
		else if(b == 0x20)
		{
			address = 0x00000c00;
			MMC_CMD = 0x00;
		}
		else if(b == 0x60)
		{
			MMC_CMD = 0xFE;
		}
		else if(timeFlashOver > 10.0*FREQ)
		{
			CS = 1;
			flash = 0;
			SPIF = 1;
			MMC_CMD = 0x00;
			goto EndSpiIsr;
		}
		SPI0DAT = 0xFF;
	}
	else if(MMC_CMD == 0x38) 
	{
		b = SPI0DAT;	
		if(b == 0xFF)
		{
			timeFlashOver = 0;
			MMC_CMD = 0x39;
		}
		else if(timeFlashOver > 10.0*FREQ)
		{
			CS = 1;
			flash = 0;
			SPIF = 1;
			MMC_CMD = 0x00;
			goto EndSpiIsr;
		}
		SPI0DAT = 0xFF;
	}
	else if(MMC_CMD == 0x39) 
	{
		b = SPI0DAT;  // FE - DataToken, 08-Err Token Out Of Range
		if(b != 0xFF)
		{
			MMC_CMD = 0x3A;
			i_CMD = 0;
		}
		else if(timeFlashOver > 10.0*FREQ)
		{
			CS = 1;
			flash = 0;
			SPIF = 1;
			MMC_CMD = 0x00;
			goto EndSpiIsr;
		}
		SPI0DAT = 0xFF;
	}
	else if(MMC_CMD == 0x3A) 
	{
		b = SPI0DAT; 
		if(i_CMD<512)
		{
			BuffFromKza[i_CMD++] = b;
			SPI0DAT = 0xFF;
		}
		else
		{
			i_CMD++;
			SPIF = 1;
		}

		if(i_CMD>=514)
		{
			i_CMD = 0;
			timeFlashOver = 0;
			MMC_CMD = 0x3B;
		}
		
	}
	else if(MMC_CMD == 0x3B)
	{
		if(SPI0DAT == 0xFF)
		{
			CS = 1;
			MMC_CMD = 0x3C;
			flash = 5;
			goto EndSpiIsr;
		}
		else if(timeFlashOver > 10.0*FREQ)
		{
			CS = 1;
			flash = 0;
			SPIF = 1;
			MMC_CMD = 0x00;
			goto EndSpiIsr;
		}
		SPI0DAT = 0xFF;
	} 
 //-end read
	else if(MMC_CMD == 0x40) //--eof--
	{
		CS = 0;	
		MMC_CMD = 0x41; 
		SPI0DAT = 0xFF;
	}
	else if(MMC_CMD == 0x41) 
	{
		MMC_CMD = 0x42; 
		SPI0DAT = 0x58;
	}
	else if(MMC_CMD == 0x42) 
	{
		MMC_CMD = 0x43;
		SPI0DAT = ((eofaddr & 0xff000000) >> 24);
	}
	else if(MMC_CMD == 0x43) 
	{
		MMC_CMD = 0x44;
		SPI0DAT = ((eofaddr & 0x00ff0000) >> 16);
	}
	else if(MMC_CMD == 0x44) 
	{
		MMC_CMD = 0x45;
		SPI0DAT = ((eofaddr & 0x0000ff00) >> 8);
	}
	else if(MMC_CMD == 0x45) 
	{
		MMC_CMD = 0x46;
		SPI0DAT = eofaddr & 0x000000ff;
	}
	else if(MMC_CMD == 0x46) 
	{
		MMC_CMD = 0x47;
		SPI0DAT = 0xFF;
	}
	else if(MMC_CMD == 0x47)
	{
		timeFlashOver = 0;
		MMC_CMD = 0x48;
		SPI0DAT = 0xFF;
	}
	else if(MMC_CMD == 0x48)
	{
		b = SPI0DAT;	// 00-ok, 20-Adress Err, 40,60-Param Err
		if(b == 0x00)
		{
			timeFlashOver=0;
			MMC_CMD = 0x49;
		}
		else if(b == 0x20)
		{
			address = 0x00000e00;
			MMC_CMD = 0xFF;
		}
		else if(b == 0x40 || b == 0x60)
		{
			timeFlashOver=0;
			CS = 1;
			flash = 3;
			MMC_CMD = 0xFE;
		}
		else if(timeFlashOver > FREQ)
		{
			timeFlashOver = 0;
			CS = 1; 
			flash = 0; 
			flageofrec = 1;
			MMC_CMD = 0x00;
			SPIF = 1;
			goto EndSpiIsr;
		}
		SPI0DAT = 0xFF;
	}
	else if(MMC_CMD == 0x49)
	{
		b = SPI0DAT;
		if(b == 0xFF)
		{
			timeFlashOver=0;
			MMC_CMD = 0x4A;
		}
		else if(timeFlashOver > FREQ)
		{
			CS = 1;
			flash = 0;
			flageofrec = 1;
			MMC_CMD = 0x00;
			SPIF = 1;
			goto EndSpiIsr;
		}
		SPI0DAT = 0xFF;
	}
	else if(MMC_CMD == 0x4A)
	{
		i_CMD = 0;
		MMC_CMD = 0x4B; 
		SPI0DAT = 0xFE;
	}
	else if(MMC_CMD == 0x4B)
	{
		if(i_CMD >= 0x200)
		{
			i_CMD = 0;	
			MMC_CMD = 0x4C; 
			SPIF = 0; 
			SPI0DAT = 0xFF;
		}
		else
		{
			SPI0DAT = BufferInKZA[i_CMD++];
		}
	}
	else if(MMC_CMD == 0x4C)
	{
		timeFlashOver=0;
		MMC_CMD = 0x4D; 
		SPI0DAT = 0xFF;
	}
	else if(MMC_CMD == 0x4D)
	{
		if(SPI0DAT == 0xFF)
		{
			i_CMD = 0;
			MMC_CMD = 0x4E;
			CS = 1;
		}
		else if(timeFlashOver > FREQ)
		{
			flageofrec = 1;
			CS = 1;
			flash = 0;
			MMC_CMD = 0x00;
			SPIF = 1;
			goto EndSpiIsr;
		}
		SPI0DAT = 0xFF;
	}
	else if(MMC_CMD == 0x4E)
	{
		if(i_CMD++ < 255)
		{
			SPI0DAT = 0xFF;
		}
		else
		{
			MMC_CMD = 0x50;
			SPI0DAT = 0xFF;
		}
	}
	else if(MMC_CMD == 0x50)
	{
		flash = 4; 
		CS = 0; 
		MMC_CMD = 0x51;
		SPI0DAT = 0xFF;
	}
	else if(MMC_CMD == 0x51) 
	{
		MMC_CMD = 0x52; 
		SPI0DAT = 0x51;
	}
	else if(MMC_CMD == 0x52) 
	{
		MMC_CMD = 0x53; 
		SPI0DAT = (eofaddr & 0xff000000) >> 24;
	}
	else if(MMC_CMD == 0x53) 
	{
		MMC_CMD = 0x54; 
		SPI0DAT = (eofaddr & 0xff0000) >> 16;
	}
	else if(MMC_CMD == 0x54) 
	{
		MMC_CMD = 0x55; 
		SPI0DAT = (eofaddr & 0xff00) >> 8;
	}
	else if(MMC_CMD == 0x55) 
	{
		MMC_CMD = 0x56; 
		SPI0DAT = (eofaddr & 0xff);
	}
	else if(MMC_CMD == 0x56) 
	{
		timeFlashOver=0;
		MMC_CMD = 0x57; 
		SPI0DAT = 0xFF;
	}
	else if(MMC_CMD == 0x57) 
	{
		b = SPI0DAT;	// 00-ok, 20-Adress Err, 60-Param Err
		if(b == 0x00)
		{
			MMC_CMD = 0x58;
			timeFlashOver=0;
		}
		else if(b == 0x20)
		{
			flageofrec = 1;
			MMC_CMD = 0x00;
		}
		else if(b == 0x60)
		{
			MMC_CMD = 0xFE;
		}
		else if(timeFlashOver > FREQ)
		{
			flageofrec = 1;
			CS = 1;
			flash = 0;
			SPIF = 1;
			MMC_CMD = 0x00;
			goto EndSpiIsr;
		}
		SPI0DAT = 0xFF;
	}
	else if(MMC_CMD == 0x58) 
	{
		b = SPI0DAT;	
		if(b == 0xFF)
		{
			timeFlashOver = 0;
			MMC_CMD = 0x59;
		}
		else if(timeFlashOver > FREQ)
		{
			CS = 1;
			flash = 0;
			SPIF = 1;
			flageofrec = 1;
			MMC_CMD = 0x00;
			goto EndSpiIsr;
		}
		SPI0DAT = 0xFF;
	}
	else if(MMC_CMD == 0x59) 
	{
		b = SPI0DAT;  // FE - DataToken, 08-Err Token Out Of Range
		if(b != 0xFF)
		{
			MMC_CMD = 0x5A;
			i_CMD = 0;
		}
		else if(timeFlashOver > FREQ)
		{
			CS = 1;
			flash = 0;
			SPIF = 1;
			flageofrec = 1;
			MMC_CMD = 0x00;
			goto EndSpiIsr;
		}
		SPI0DAT = 0xFF;
	}
	else if(MMC_CMD == 0x5A) 
	{
		b = SPI0DAT; 
		if(i_CMD<512)
		{
			BuffFromKza[i_CMD++] = b;
			SPI0DAT = 0xFF;
		}
		else
		{
			i_CMD++;
			SPIF = 1;
		}

		if(i_CMD>=514)
		{
			i_CMD = 0;
			timeFlashOver = 0;
			MMC_CMD = 0x5B;
		}
		
	}
	else if(MMC_CMD == 0x5B)
	{
		if(SPI0DAT == 0xFF)
		{
			CS = 1;
			MMC_CMD = 0x5C;
		}
		else if(timeFlashOver > FREQ)
		{
			CS = 1;
			flash = 0;
			SPIF = 1;
			flageofrec = 1;
			MMC_CMD = 0x00;
			goto EndSpiIsr;
		}
		SPI0DAT = 0xFF;
	} 
	else if(MMC_CMD == 0x5C)
	{
		if(BuffFromKza[0]==0x65 && BuffFromKza[1]==0x6f && BuffFromKza[2] == 0x66 && BuffFromKza[3]==0x65 && BuffFromKza[4]==0x6f && BuffFromKza[5] ==0x66 && BuffFromKza[6]==0x65 && BuffFromKza[7]==0x6f && BuffFromKza[8] ==0x66)
		{
			flash = 1;
			flageofrec = 0;
			MMC_CMD = 0xFF;
		}
		else
		{
			MMC_CMD = 0x40;
			eofaddr += 0x200;
		}
	}
//------------------------
	EndSpiIsr:
	SFRPAGE = SFRPAGE_SAVE;
	return;
}

//-----------------------------------------------------------------------
void MMC_init1(void)
{
	address = 0x00000A00;
	nzap = 0;
	szflash = 0;
	MMC_CMD = 0x00;
	SPIF = 1;
	return;
}

//-----------------------------------------------------------------------
char WriteInKZA(unsigned char byte)
{
	static xdata unsigned int counterBuf = 9;
	xdata unsigned char CRC;
	xdata unsigned long tmpNzap;
	static xdata unsigned long eof = 0;
	
	
	if(MMC_CMD == 0xFF)
	{
		flash = 1;
	}

	if(MMC_CMD == 0xFE)
	{
		return 0;
	}

	if((flageofrec == 1)&&(flash == 1))
	{
		flash = 0;
		MMC_CMD = 0x40;
	}	
	if(MMC_CMD == 0x40)
	{
		SPIF = 1;
	}
	if((flash == 1)&&(nzap == 0)&&(szflash > 0))
	{
		address = 0x00000c00;
		flash = 4;
		MMC_CMD = 0x30;
		SPIF = 1;
	}
	else if((flash == 5)&&(nzap == 1))
	{
		eofaddr = 0x1000;
		address = 0xc00;
		lastaddr = 0xe00;
		jump = 1;
		nzap++;
		for(i_wr = 0; i_wr < 512; i_wr++)
		{
			BufferInKZA[i_wr++]=0x00;
		}
		BufferInKZA[0] = (nzap & 0xff000000) >> 24;
		BufferInKZA[1] = (nzap & 0x00ff0000) >> 16;
		BufferInKZA[2] = (nzap & 0x0000ff00) >> 8;
		BufferInKZA[3] =  nzap & 0x000000ff;  
		BufferInKZA[4] = (lastaddr & 0xff000000) >> 24;
		BufferInKZA[5] = (lastaddr & 0x00ff0000) >> 16;
		BufferInKZA[6] = (lastaddr & 0x0000ff00) >> 8;
		BufferInKZA[7] =  lastaddr & 0x000000ff;  
		CRC = BufferInKZA[0];		
		for(i_wr = 1; i_wr < 8; i_wr++)
		{
				CRC = CRC^BufferInKZA[i_wr];
		}
		BufferInKZA[8]  = CRC;
		MMC_CMD = 0x10;
		SPIF = 1;	
	}
	else if((flash == 5)&&(nzap > 1))
	{
		tmpNzap = BuffFromKza[0];
		for(i_wr = 1; i_wr < 4; i_wr++)
		{
			tmpNzap = (tmpNzap<<8)+BuffFromKza[i_wr];
		}
				
		if(nzap != tmpNzap)
		{
			if(BuffFromKza[0]==0x65 && BuffFromKza[1]==0x6f && BuffFromKza[2] == 0x66 && BuffFromKza[3]==0x65 && BuffFromKza[4]==0x6f && BuffFromKza[5] ==0x66 && BuffFromKza[6]==0x65 && BuffFromKza[7]==0x6f && BuffFromKza[8] ==0x66)
			{
					lastaddr = address;
					address = 0xc00;
					jump = 1;
					nzap++;
					for(i_wr = 0; i_wr < 512; i_wr++)
					{
						BufferInKZA[i_wr++]=0x00;
					}
					BufferInKZA[0] = (nzap & 0xff000000) >> 24;
					BufferInKZA[1] = (nzap & 0x00ff0000) >> 16;
			   	BufferInKZA[2] = (nzap & 0x0000ff00) >> 8;
					BufferInKZA[3] =  nzap & 0x000000ff;  
					BufferInKZA[4] = (lastaddr & 0xff000000) >> 24;
					BufferInKZA[5] = (lastaddr & 0x00ff0000) >> 16;
					BufferInKZA[6] = (lastaddr & 0x0000ff00) >> 8;
					BufferInKZA[7] =  lastaddr & 0x000000ff;  
					CRC = BufferInKZA[0];		
					for(i_wr = 1; i_wr < 8; i_wr++)
					{
						CRC = CRC^BufferInKZA[i_wr];
					}
					BufferInKZA[8]  = CRC;
					MMC_CMD = 0x10;
					SPIF = 1;
			}
			else
			{
				address += 0x200;	
				if(address > szflash) 
					address = 0x1000;
				
				eof++;
				if(eof>0x3e8)	
				{
					lastaddr = eofaddr;
					address = 0x00000c00;
					jump = 1;
					nzap++;
					for(i_wr = 0; i_wr < 512; i_wr++)
					{
						BufferInKZA[i_wr++]=0x00;
					}
					BufferInKZA[0] = (nzap & 0xff000000) >> 24;
					BufferInKZA[1] = (nzap & 0x00ff0000) >> 16;
			   	BufferInKZA[2] = (nzap & 0x0000ff00) >> 8;
					BufferInKZA[3] =  nzap & 0x000000ff;  
					BufferInKZA[4] = (lastaddr & 0xff000000) >> 24;
					BufferInKZA[5] = (lastaddr & 0x00ff0000) >> 16;
					BufferInKZA[6] = (lastaddr & 0x0000ff00) >> 8;
					BufferInKZA[7] =  lastaddr & 0x000000ff;  
					CRC = BufferInKZA[0];		
					for(i_wr = 1; i_wr < 8; i_wr++)
					{
						CRC = CRC^BufferInKZA[i_wr];
					}
					BufferInKZA[8]  = CRC;
					MMC_CMD = 0x10;
					SPIF = 1;
				}
				else
				{
					flash = 4;
					MMC_CMD = 0x30;
					SPIF = 1;
				}
			}
		}
		else  //читать дальше
		{
			CRC = BuffFromKza[0];
			for(i_wr = 1; i_wr < 9; i_wr++)
			{
				CRC = CRC^BuffFromKza[i_wr];;
			}
			if(!CRC) 
			{
				address = BuffFromKza[4];
				address = (address<<8)+BuffFromKza[5];
				address = (address<<8)+BuffFromKza[6];
				address = (address<<8)+BuffFromKza[7];
			}
			else
			{
				address += 0x200;	
			}
			flash = 4;
			MMC_CMD = 0x30;
			SPIF = 1;
		}
	}
	else if((flash == 5)&&(nzap == 0))
	{
		CRC = BuffFromKza[0];
		for(i_wr = 1; i_wr < 9; i_wr++)
		{
			CRC = CRC^BuffFromKza[i_wr];;
		}
		if(!CRC && (BuffFromKza[0]!=0 || BuffFromKza[1]!=0 || BuffFromKza[2]!=0 || BuffFromKza[3]!=0 || BuffFromKza[4]!=0 || BuffFromKza[5]!=0 || BuffFromKza[6]!=0 || BuffFromKza[7]!=0 || BuffFromKza[8]!=0)) 
		{
				nzap = BuffFromKza[0];
				for(i_wr = 1; i_wr < 4; i_wr++)
				{
					nzap = (nzap<<8)+BuffFromKza[i_wr];
				}
				lastaddr = BuffFromKza[4];
				for(i_wr = 5; i_wr < 8; i_wr++)
				{
					lastaddr = (lastaddr<<8)+BuffFromKza[i_wr];
				}
				
				address = lastaddr;
		}
		else
		{
				nzap = 1;
				address = 0xe00;
				lastaddr = address;
		}
		flash = 4;
		MMC_CMD = 0x30;
		SPIF = 1;
	}

	if((flash == 1)&&(counterBuf < 512))
	{
		BufferInKZA[counterBuf++] = byte;	
		return 1;
	}
	else if((counterBuf >= 512)&&(szflash > 0)&&(flash == 1))
	{
		if((eofaddr-address) == 0x00 || (eofaddr-address) > szflash)
		{
			flageofrec = 1;
			eofaddr = address + 0x7d000;
			if(eofaddr > szflash) 
				eofaddr = 0x1200; 
			
			for(i_wr = 0; i_wr < 9; i_wr++)
			{
				BufferInKZA[i_wr++]=0x65;
				BufferInKZA[i_wr++]=0x6f;
				BufferInKZA[i_wr]=0x66;
			}
			MMC_CMD = 0x40;
		}
		else
		{
			BufferInKZA[0] = (nzap & 0xff000000) >> 24;
			BufferInKZA[1] = (nzap & 0x00ff0000) >> 16;
   		BufferInKZA[2] = (nzap & 0x0000ff00) >> 8;
			BufferInKZA[3] =  nzap & 0x000000ff;  
			BufferInKZA[4] = (eofaddr & 0xff000000) >> 24;
			BufferInKZA[5] = (eofaddr & 0x00ff0000) >> 16;
			BufferInKZA[6] = (eofaddr & 0x0000ff00) >> 8;
			BufferInKZA[7] =  eofaddr & 0x000000ff;  
			CRC = BufferInKZA[0];		
			for(i_wr = 1; i_wr < 8; i_wr++)
			{
				CRC = CRC^BufferInKZA[i_wr];
			}
			BufferInKZA[8]  = CRC;
			counterBuf = 9;
			MMC_CMD = 0x10;
			SPIF = 1;
		}
	}
	else if((flash == 1)&&(szflash == 0))
	{
		MMC_CMD = 0x20;
		SPIF = 1;
	}
	
	if(spifcount>200)
	{
		spifcount=0;
		SPIF=1;
	}
	else
	{
		spifcount++;
	}

	return 0;
}