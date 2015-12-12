#ifndef MMCFLASH2
#define MMCFLASH2

sbit CS = P3^2;
extern unsigned char xdata Buffer[];
extern xdata unsigned long address;

void SPI_Init(void);
void MMC_init(void);
void WriteMMC(unsigned long address);
#endif