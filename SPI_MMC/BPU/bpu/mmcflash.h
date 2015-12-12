#ifndef MMCFLASH
#define MMCFLASH

char WriteByteInKZA(unsigned char Byte);
sbit CS = P3^2;

void SPI_Init(void);
char MMC_init(void);

//void WriteMMC(unsigned long address);

#endif