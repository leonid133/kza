#ifndef  INTERFACE1H
#define INTERFACE1H

#define SIZE_BUFFER0	52
extern xdata char BufferInModem[]; // Для отправки в последовательный порт
extern xdata int r0, rk;
extern bit flTransmiter;	

void OutModem20(void);
void OutModem21(void);
void OutModem1(unsigned char Data, char i);
void OutModem2(unsigned int Data, char i);
void OutModem4(unsigned long int Data, char i);

#endif
