#ifndef MMCFLASH2
#define MMCFLASH2

//sbit CS = P3^2;
extern unsigned char xdata /*BufferInKZA[], */flash;
//extern xdata unsigned long address;

void SPI_Init(void);
void MMC_init1(void);
void WriteInKZA(unsigned char Byte);
#endif