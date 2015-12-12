#ifndef MMCFLASH5
#define MMCFLASH5

//sbit CS = P3^2;
/*extern unsigned char xdata BufferInKZA[], flash;*/
//extern xdata unsigned long address;
extern xdata unsigned int timeFlashOver;


void SPI_Init(void);
void MMC_init1(void);
char WriteInKZA(unsigned char Byte);
#endif