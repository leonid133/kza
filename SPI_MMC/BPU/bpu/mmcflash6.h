#ifndef MMCFLASH6H
#define MMCFLASH6H

//sbit CS = P3^2;
/*extern unsigned char xdata BufferInKZA[], flash;*/
//extern xdata unsigned long address;
extern xdata unsigned int timeFlashOver;

void SPI_Init(void);
void MMC_init1(void);
bit WriteInKZA(unsigned char Byte);
#endif