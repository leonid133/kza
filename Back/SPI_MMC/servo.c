//Функции для работы с 485м интерфейсом
#include "c8051f120.h"
#include "servo.h"
#include <intrins.h>

//--------------------------------
//	P6 = addr;
//	P4 = data_lsb write to fpga
//	P5 = data_msb write to fpga
//	P7 = data_lsb read from fpga
//	P2 = data_msb read from fpga
sbit we = P3^3;	//write enabled
sbit re = P3^4;	//read enabled
sbit wr = P3^5;	//write push
sbit rd = P3^6;	//read pull

//-----------------------------------------------------------------------------
unsigned int read(unsigned char adress)
{
	unsigned int dat;
	SFRPAGE = 0x00;
	wr = we = 0;
	re = rd = 1;
	for (dat = 0; dat < 100; dat++)
		_nop_();

	SFRPAGE = 0x0f;
	P6 = adress;
	dat = P2;
	dat = (dat << 8)+P7;
	
	SFRPAGE = 0x00;
	rd = 0;

	return dat;
}

//-----------------------------------------------------------------------------
void write(unsigned  char adress, unsigned int dat)
{
	SFRPAGE = 0x00;
	re = rd = 0;
	we = 1;

	SFRPAGE = 0x0f;
	P6 = adress;
	P4 = dat & 0x00ff;
	P5 = (dat & 0xff00) >> 8;

	SFRPAGE = 0x00;
	wr = 1;
	for (adress = 0; adress < 0xf; adress++)
		_nop_();

	wr = 0;
	return;
}