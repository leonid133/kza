#include "interface1.h"
#include "bpu1.h" 
#include "c8051f120.h"


xdata char BufferInModem[SIZE_BUFFER0]; // Для отправки в последовательный порт
xdata int r0, rk;
bit flTransmiter;	

//----------------------------------------------------------------------------
void OutModem20(void)
{
	xdata char i;
	while (flTransmiter)
		;			
   BufferInModem[0] = 20 | 0x40;
	if(flNoKoord)
   {
   	BufferInModem[1] = 0x80;	   
      BufferInModem[2] = 0x80;	   
      BufferInModem[3] = 0x80;	   
      BufferInModem[4] = 0xff;	   
      BufferInModem[5] = 0x80;	   
      BufferInModem[6] = 0x80;	   
      BufferInModem[7] = 0x80;	   
      BufferInModem[8] = 0x80;	   
      BufferInModem[9] = 0x80;	   
      BufferInModem[10] = 0x80;	   
      BufferInModem[11] = 0x80;	   
	}
   else
   {
     	OutModem4(LatFly, 1);
      OutModem4(LonFly, 5);
      OutModem2(koors, 9);	//курс
      BufferInModem[11] = 0x80 | (unsigned char)Vz;	   //Vзем
   }
 	 
   OutModem2(H_filtr+5000, 12);
	OutModem2(V_dat*10, 14);
   OutModem2((Vy_filtr+800)*10, 16);
   OutModem2(kren_dat/ToGrad*2500+8000, 18);
   OutModem1(delta_z+10, 20);
   OutModem1(delta_g, 21);
   BufferInModem[22] = 0x80 | dataRSTSRC;        
   BufferInModem[23] = 0x80 | SteckPoint;        
   BufferInModem[24] = (kren_zad+60) | 0x80;
   OutModem2(Vy_zad_ogr_nab*100, 25);
   OutModem2(KrenKam_zad/ToGrad*2500+8000, 27); //Временно KrenKam_zad = KrenKam
   OutModem2(UgolKam_zad/ToGrad*2500+8000, 29); //Временно UgolKam_zad = UgolKam

   BufferInModem[31] = 0x80;
   BufferInModem[32] = 0x80;
   BufferInModem[33] = 0x80;
   OutModem2(nSU, 34);
   OutModem1(10*Vtop, 36);

   BufferInModem[37] = 0x80 | Dat37;
   BufferInModem[38] = 0x80 | Dat38;
   BufferInModem[39] = 0x80;
   BufferInModem[40] = 0x80;
	BufferInModem[41] = 0x80;

	BufferInModem[42] = 0;
   for (i = 0; i < 42; i++ )
   	BufferInModem[42] = BufferInModem[42] ^ BufferInModem[i];
   BufferInModem[42] =  0x80 | BufferInModem[42];

	r0 = 0;
  	rk = 43;

	flTransmiter = 1;
	SFRPAGE = 0x00;
	TI0 = 1;
	return;
}

//----------------------------------------------------------------------------
void OutModem21(void)
{
   xdata char i;
	while (flTransmiter)
		;			

   flCommand = 0;
   BufferInModem[0] = 21 | 0x40;

   if (RegimeVy == 0)
      OutModem1((Vy_zad_buf+6)*10, 1);
   else if (RegimeVy)
      OutModem1((H_zad_buf+1000)/50, 1);
   if (RegimeV == 0)
      OutModem1(delta_g, 2);
   else if (RegimeV == 1)
      BufferInModem[2] = 0x80 | Vz_zad;
   else if (RegimeV == 2)
      BufferInModem[2] = 0x80 | (unsigned char)V_zad;
   OutModem1(delta_z_zad+10, 3);
   if (RegimeKren)
	   BufferInModem[4] = 0x80 | n_;
   else
	   BufferInModem[4] = 0x80 | (unsigned char)(kren_zad+60);
   BufferInModem[5] = 0x80;
   BufferInModem[6] = 0x80 | (unsigned char)(KrenKam_zad+60);
   BufferInModem[7] = 0x80 | (unsigned char)UgolKam_zad;
   BufferInModem[8] = 0x80;

   BufferInModem[9] = 0x80 | (RegimeKren & 0x03) | ((RegimeSU & 0x03) << 5);
   BufferInModem[10] = 0x80 | (RegimeStart & 0x03) | ((RegimeVy & 0x03) << 2) | ((RegimeV & 0x03) << 4);

   BufferInModem[11] = 0;
   for (i = 0; i < 11; i++ )
      BufferInModem[11] = BufferInModem[11] ^ BufferInModem[i];
   BufferInModem[11] = BufferInModem[11] | 0x80;

	r0 = 0;
	rk = 12;
	flTransmiter = 1;
	SFRPAGE = 0x00;
	TI0 = 1;
}
//------------------------------------------------------------------------------
void OutModem1(unsigned char Data, char i)
{
	BufferInModem[i] = Data | 0x80;
}

//------------------------------------------------------------------------------
void OutModem2(unsigned int Data, char i)
{
	BufferInModem[i] = (Data & 0x007f)| 0x80;
	BufferInModem[i+1] = ((Data & 0x3f80) >> 7)| 0x80;
}

//------------------------------------------------------------------------------
void OutModem4(unsigned long int Data, char i)
{
	BufferInModem[i] = (Data & 0x0000007f)| 0x80;
	BufferInModem[i+1] = ((Data & 0x3f80) >> 7) | 0x80;
	BufferInModem[i+2] = ((Data & 0x1fc000) >> 14) | 0x80;
   BufferInModem[i+3] = ((Data & 0xfe00000)>> 21) | 0x80;
}

