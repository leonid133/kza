#ifndef MMCFLASH
#define MMCFLASH

sbit P3_2 = P3^2;
extern unsigned char xdata Buffer[];

void SPI_Init(void);
void MMC_init(void);
void SD_init(void);
void ReadMMC(unsigned long address);
void WriteMMC(unsigned long address);
#endif