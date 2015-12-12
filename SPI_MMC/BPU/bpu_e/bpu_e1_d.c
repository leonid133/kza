#include "c8051f120.h"
#include <math.h>
#include <stdlib.h>
#include <intrins.h>
#include <absacc.h>

sfr16 DP       = 0x82;                 // data pointer
sfr16 ADC0     = 0xbe;                 // ADC0 data
sfr16 ADC0GT   = 0xc4;                 // ADC0 greater than window
sfr16 ADC0LT   = 0xc6;                 // ADC0 less than window
sfr16 RCAP2    = 0xca;                 // Timer2 capture/reload
sfr16 RCAP3    = 0xca;                 // Timer3 capture/reload
sfr16 RCAP4    = 0xca;                 // Timer4 capture/reload
sfr16 TMR2     = 0xcc;                 // Timer2
sfr16 TMR3     = 0xcc;                 // Timer3
sfr16 TMR4     = 0xcc;                 // Timer4
sfr16 DAC0     = 0xd2;                 // DAC0 data
sfr16 DAC1     = 0xd5;	//???          // DAC1 data

//SYSCLK = 49766400 - тактовая частота

//-------------------------------------------------------------------
//sbit led = P3^0;
//// int0 = P1^6;
sbit bitStart = P2^6; //int0
sbit int1 = P2^7;
sbit pow = P6^6;  

//-------------------------------------------------------------------
void config(void);
void sysclk(void);
void UART0_Init(void);
void UART1_Init(void);
void port_init(void);
void UART0_isr(void);
void UART1_isr(void);
void DAC0_init(void);
void ADC0_init(void);
void Timer3_init(void);
void ADC0_isr(void);

void Timer0_init(void);
void PCA_init(void);
void PCA_ISR (void);

//-------------------------------------------------------------------------------------
#define FREQ         50.625      		// Частота прерываний таймера Гц

#define ToGrad 57.2957795130823
#define M_PI   3.14159265358979323846
#define D_PI 	6.28318530717958647692
#define M_PI_2 1,57079632679489661

#define NBFM 		25
//#define NBFM_1 	14
xdata char BuferFromModem [NBFM]; // Для анализа с последовательного порта
xdata char wBFM, rBFM, marBFM;	 

void OutModem(void);
//void PackageInModem1(unsigned char Data, char i);
void PackageInModem2(unsigned int Data, char i);
void PackageInModem4(unsigned long int Data, char i);
#define SIZE_BUFFER0	528
	xdata char BufferInModem[SIZE_BUFFER0]; // Для отправки в последовательный порт
	xdata int r0, rk;
   bit flTransmiter;	

char ContrlSumma(char *Array, char NByteContrlSumma);

xdata int koors;
unsigned long DecodeLatOrLon(char *Array, char nomer);
xdata unsigned long lLatFly, lLonFly, lLatZad, lLonZad, lLatZad_pr, lLonZad_pr;
	xdata unsigned long int LatMar[128], LonMar[128];
	xdata unsigned char n_, i_mar;
	xdata float cos_Lat0;

xdata unsigned int timer1_tick1; // время(сек) = timer1_tick*FREQ
xdata long int liTimer_tick = 0; //???
xdata long int liTimer_tick_GPS; //???
xdata unsigned int i_Vzlet;              

#define  H_avar   	250
//#define N_MIN 500
//#define N_MAX 7000
#define EPSILON 400
#define V_MIN 20
#define V_BUM 15

//#define HH 50
xdata float H0, H_tmp, H, s_Vy_filtr, s_H_filtr, s_H_filtr_pr, V, H_filtr, H_filtr_pr, Vy;
xdata float Wx, Wy, Kren_dat, int_dKren, int_dV = 0, Delta_E, Delta_V, integral_delta_V; 
xdata unsigned char deltaH;
xdata int  H_zad;
xdata char Kren_zad, Kren_zad_buf, Vz_zad, KrenKam_zad;
xdata float Delta_KrenKam, Delta_UgolKam;
xdata unsigned char Vz, UgolKam_zad;
xdata unsigned int NoCommand;
xdata float int_delta_ax, Vy_zad_ogr_nab, int_delta_Vy_zad_ogr_nab, Vy_zad_ogr_sn;
xdata char Vy_zad_max, Vy_zad_min;
#define NS 	75
xdata char mess [NS], r, w, mar;		// Для анализа посылки GPS
//idata char mess[NS]="$GPRMC,064600,A,5551.8573,N,04906.4012,E,251.315,312.7,200500,11.5,E*40"; 
//idata char mess[NS]="$GPRMC,100508,A,5550.9399,N,04906.4464,E,1.640,343.1,170703,11.6,E*4E"; 

void ReadKZA(void);
void WriteByteInKZA(unsigned char Byte);
void EraseKZA(void);
sbit CS = P3^0;
sbit WP = P3^1;
sbit RESET_FLASH = P3^2;
sbit RDY = P3^3; 

bdata unsigned char  Dat1 = 0x80; /*	В битах 0:1 содержится информация о режиме наведения:
                                 0 - ручное управление
									      1 - автомат
									      2 - автоном	
                                 4 - Возврат*/
sbit flStart = Dat1^2;
sbit flSleep = Dat1^3;
sbit flOtkazRK = Dat1^4;
sbit flStopSU = Dat1^5;
sbit flAvtomatV = Dat1^6;

bdata unsigned char  Dat2 = 0x80; 
sbit flAvtomatK = Dat2^0;


bit flSet_H0 = 0, flRun;// , flWDTRun = 0;
bit flNoKoord, flAnswer, flGPS, flCommand;

void Timer4_isr(void);
bit flTimer4Interrupt;
xdata unsigned int TMR4_pr; 

xdata int H_max, iV;
bit   flOk, flNew, flPriv, fln_;


xdata float Delta_G = 0, Delta_Z;
xdata float Delta_G_pr, int_Delta_G;


xdata float lLatFly_gir, lLonFly_gir, Vx_gir, Vy_gir, Vz_gir, ax_gir, ay_gir, az_gir;
xdata float Wx_r, Wy_r, Wz_r, Tx, Ty, Tz, kren_gir, tang_gir, koors_gir, Wz, ex_gir, ey_gir, ez_gir;
xdata char Wak, KrenKam, UgolKam;
//--------------------------------------------------------------------------------------
/*float code a0_wx, a1_wx, a2_wx, 
			  a0_wy, a1_wy, a2_wy, 
			  a0_H,  a1_H,  a2_H, 
			  a0_q,  a1_q,  a2_q, N_OfWorks;*/
/*float code a[256] _at_ 0x1000;

void GetValue(char NPackage);
//------------------------------------------------------------------------------------
void GetValue(char NPackage)
{
	unsigned long int Buf = *(unsigned long int *) &a[NPackage];

   BufferInModem[0] = 0x40 | NPackage;
   BufferInModem[1] = 0x80 | (Buf&0x7f);
   BufferInModem[2] = 0x80 | ((Buf &     0x3f80   ) >> 7 );
	BufferInModem[3] = 0x80 | ((Buf &   0x1fc000 ) >> 14);
   BufferInModem[4] = 0x80 | ((Buf &  0xfe00000) >> 21);
   BufferInModem[5] = 0x80 | ((Buf & 0xf0000000) >> 28);
   BufferInModem[6] = (BufferInModem[0]^BufferInModem[1]^BufferInModem[2]^BufferInModem[3]
		^BufferInModem[4]^BufferInModem[5]) | 0x80;
	r0 = 0;
	rk = 7;
	flTransmiter = 1;

	SFRPAGE = 0x00;
	TI0 = 1;
	return;
}*/


xdata float ax = 0;

xdata unsigned char SteckPoint;
unsigned char code RLB _at_ 0xfbff;
unsigned char code WELB _at_ 0xfbfe;
unsigned char xdata *pwrite;

xdata float Wx0, Wy0, Wx0_pr, Wy0_pr, Wz0, Wz0_pr;
//-----------------------------------------------------------------------
void main(void)
{
	//Для работы с последовательным портом "Модем"
	xdata char RK_code[26], nByte = 0, KontrSumma = 0, temp1,/* i_stop,*/ NPackage;	

   xdata float angle, tmp, /*tmp1,*/ napr_vetv_mar, otkl_ot_mar, dz, dx, dz_pr, dx_pr; //Для автоуправления

   bit ValidGPS, flPoint; 	
	xdata unsigned char i, i_comma, tmpGPS[6], nLetter = 7;
	xdata unsigned long temp_koord;

//   char xdata * idata pwrite;

	xdata float delta_ro, V_pr, delta_Vy;
	xdata unsigned char j;

//   nByte = CBYTE [0x1fbff];
//   char *p;
//   p = 0x1fbff;
//   nByte = *p;

//-----------	

	WDTCN = 0xde;
	WDTCN = 0xad;

	SteckPoint = SP;	
   Wx0 = Wy0 = Wz0 = 0;
   Wx0_pr = Wy0_pr = Wz0_pr = 0;

	for (r0 = 0; r0 < SIZE_BUFFER0; r0++)
		BufferInModem[r0] = 0x80;	
	r0 = rk = 0;
 	rBFM = wBFM = marBFM = 0;

	H_zad = 300;
	s_Vy_filtr = s_H_filtr = s_H_filtr_pr = 0;
//-----------
//	SFRPAGE = 0;
//	i = RSTSRC;
//-------------

	H0 = 0;	
	Vz_zad = 30;
	KrenKam_zad = 0;
	UgolKam_zad = 90;

	flNoKoord = 1;

	liTimer_tick = liTimer_tick_GPS = 0;
	H0 = 0;
	r = w = mar = 0;
	V = 0;
	int_delta_Vy_zad_ogr_nab = Vy_zad_ogr_nab = 3;
	Vy_zad_ogr_sn = -4;

   Wak = 90;
//---------
	flStart = 1;
   FLSCL = FLSCL | 1;	

/*	SFRPAGE = 0;   //Управление источниками сброса
	if((RSTSRC & 0x80) && bitStart)
	{
		flAvarStop = 1;
		timer1_tick1 = 0;
	}
*/
	port_init();
	sysclk();
	UART0_Init();
	UART1_Init();
	DAC0_init();
	ADC0_init();
	Timer3_init();
	Timer0_init();

	//Инициализация счетчика оборотов СУ--------------
	SFRPAGE = 2;
	TMR4CN = 0x06;
	TMR4CF = 0;

	RCAP4L = 0;
	RCAP4H = 0;
	TMR4L = 0;
	TMR4H = 0;

	PCA_init();

	//SPI Configuration----------------------------------------------
	SFRPAGE = 0x00;	
	SPI0CFG = 0x60;	// SPI Configuration Register
	SPI0CKR = 0x03;	// SPI Clock Rate Register--- частота 12441600 Гц  Fsck=0.5*Fsysclk/(SPI0CKR+1)
	SPI0CN = 0x09;//0x01;	// SPI Control Register

   WP = 1;
   RESET_FLASH = 1; 

	config();
	DAC0 = 0x00;

//   DeviceWork = 1;
	//---------------------------------------------------------------
            
            i = PSBANK;       
		      EA = 0;           // Disable interrupts                 
			   
			   SFRPAGE = 0x0f;	// Enable FLASH block writes
            PSBANK &= 0xcf;
            PSBANK |= 0x3f;
			   CCH0CN &= 0xfe;                   

			   SFRPAGE = 0;

		      FLSCL |= 0x01;    // Enable FLASH writes/erases
		      PSCTL = 0x01;
            pwrite = (char xdata *)&WELB;
		      *pwrite = 0;      // Write Data to FLASH                   
		      PSCTL = 0;        // MOVX targets XRAM

		      PSCTL = 0x01;     // MOVX writes write FLASH byte
            pwrite = (char xdata *)&RLB;
		      *pwrite = 0;      // Write Data to FLASH                   
   	      PSCTL = 0;        // MOVX targets XRAM
		      FLSCL &= ~0x01;   // Disable FLASH writes/erases
   
			   SFRPAGE = CONFIG_PAGE;
			   CCH0CN &= ~0x01;           // Clear the CHBLKW bit
		      EA = 1;                    // Restore interrupt state      
            PSBANK = i;
   
	Delta_E = 0;
	Delta_V = 0;
   Delta_KrenKam = 0;
	Delta_UgolKam = 90;

	//-----------------------------------------------------------------------------------------
//	if (bitStart == 0) //если есть отрывной разъем
  	{ 
	   Delta_E=25;   //элероны 
	   timer1_tick1 = 0;
		while(timer1_tick1 < FREQ)
			;
    
	   Delta_E=-25;
   	timer1_tick1 = 0;
		while(timer1_tick1 < FREQ)
			;

	   Delta_E=0;
	   timer1_tick1 = 0;
		while(timer1_tick1 < FREQ)
			;
		
	   Delta_V=20;		//руль высоты
	   timer1_tick1 = 0;
		while(timer1_tick1 < FREQ)
			;

   	Delta_V=-20;
	   timer1_tick1 = 0;
		while(timer1_tick1 < FREQ)
			;

	   Delta_V=0;
	   timer1_tick1 = 0;
		while(timer1_tick1 < FREQ)
			;

	   Delta_KrenKam = -45;	//крен камеры
	   timer1_tick1 = 0;
		while(timer1_tick1 < FREQ)
			;

	   Delta_KrenKam = 45;
	   timer1_tick1 = 0;
		while(timer1_tick1 < FREQ)
			;

	   Delta_KrenKam = 0;
	   timer1_tick1 = 0;
		while(timer1_tick1 < FREQ)
			;

	   Delta_UgolKam = 20;		//Камера по тангажу
	   timer1_tick1 = 0;
		while(timer1_tick1 < FREQ)
			;

	   Delta_UgolKam = 90;
	   timer1_tick1 = 0;
		while(timer1_tick1 < FREQ)
			;

/*	   Delta_G = 0;			//Газ
	   timer1_tick1 = 0;
		while(timer1_tick1 < FREQ)
			;

	   Delta_G = 50;
	   timer1_tick1 = 0;
		while(timer1_tick1 < FREQ)
			;

	   Delta_G = 100;
	   timer1_tick1 = 0;
		while(timer1_tick1 < FREQ)
			;
*/
		liTimer_tick_GPS = liTimer_tick = 0;
	}
/*	else
	{
//		flAvarStop = 1;
		timer1_tick1 = 0;
	}*/
	EA = 0;
   H0 = s_H_filtr;    //установим H0
   Wx0 = Wx0_pr;
   Wy0 = Wy0_pr;
   Wz0 = Wz0_pr;
   kren_gir = 0;
   tang_gir = 0;
   koors_gir = 0;

	H_filtr_pr = H_filtr = 0;
	s_H_filtr_pr = s_H_filtr = 0;
	flSet_H0 = 1;
	EA = 1;

//-------------------
/*lLatFly = 54000000UL+55UL*60*10000+50UL*10000+8680;
lLonFly = 108000000UL+49UL*60*10000+6UL*10000+3760;
koors = 30;*/
//----------------------------------

	//Выбор режима работы БУП -----------------------------------------------------------------
	while(1)
	{
		flStart = !bitStart;
//		bitSysReady = !flNoKoord;

   	if(rBFM < wBFM+marBFM*NBFM)
   	{
			if (BuferFromModem[rBFM]  == 0x42)	
			{
				nByte = 0;
				KontrSumma = 0;
				i = rBFM;
				temp1 = marBFM;
			}
			else if (nByte > 25)
				nByte = 25;
			RK_code[nByte] = BuferFromModem[rBFM] & 0x7f;
			KontrSumma = KontrSumma^RK_code[nByte++];
         rBFM++;
			if(rBFM >= NBFM)
			{
   			rBFM = 0;
				marBFM = 0;	
			}
			if ( (nByte == 3) && (KontrSumma == 0) )
			{
				if(RK_code[1] == 1)	//Управление движением МКМ 
				{
					rBFM = i;
					marBFM = temp1;

//					SFRPAGE = 0;
//					WDTCN = 0x07;	//1048576/Sysclc=0,02107
//					WDTCN = 0xA5;	//Перезапустить охранный таймер
					goto first;
				}
			   else if(RK_code[1] == 5)			//Первоначальная инициализация БУП 
				{
					rBFM = i;
					marBFM = temp1;
					goto zero;
				}
				else if(RK_code[1] == 6)	//Сброс данных бортовой КЗА через COM - порт
				{
					SFRPAGE = 0;

//---------------------
/*	   TI0=1;
      while(1)
         ;*/
//-----------------------
//					EA = 0;
					ReadKZA();
//					EA = 1;
				}
				else if(RK_code[1] == 7)	//Стереть КЗА
				{
					SFRPAGE = 0;
					EA = 0;
					EraseKZA();					
					EA = 1;
//						OutModem(25);		    //Подтверждение очистки КЗА
	  			   	BufferInModem[0] = 0x40 | 25;
						BufferInModem[1] = 0x80;
    					BufferInModem[2] = (BufferInModem[0]^BufferInModem[1]) | 0x80;
						SFRPAGE = 0x00;
						TI0 = 1;
				}
				else
					;
			}
		}
	}
zero:		//Первоначальная инициализация БУП---------------------------------------------------
	while(1)
	{
/*   	if(rBFM < wBFM+marBFM*NBFM)
   	{
			if ( (BuferFromModem[rBFM] & 0xc0) == 0x40)	
	      {
   	      NPackage = BuferFromModem[rBFM] & 0x3f;
      	   KontrSumma = 0;
         	nByte = 0;
	      }
   	   RK_code[nByte] = BuferFromModem[rBFM] & 0x7f;
      	KontrSumma = KontrSumma^RK_code[nByte++];
	      if (nByte > 9)
   	      nByte = 9;
         rBFM++;
			if(rBFM > NBFM_1)
			{
   			rBFM = 0;
				marBFM = 0;	
			}

			if (NPackage == 0x3e) //Режим прошивки
			{
/*				if(nByte == 3 && KontrSumma == 0)
				{
				   pwrite = (char xdata *)a;          // FLASH write/erase pointer

			      EA = 0;                          // Disable interrupts                 
				   SFRPAGE = 0x0f;
				   CCH0CN &= 0xfe;                     // Set the CHBLKW bit to disable

				   SFRPAGE = 0;
			      FLSCL |= 0x01;                   // Enable FLASH writes/erases
			      PSCTL = 0x03;                   // e FLASH byte

			      *pwrite = 0;                 // Write Data to FLASH                   
      
			      PSCTL = 0;                    // MOVX targets XRAM
			      FLSCL &= ~0x01;                  // Disable FLASH writes/erases
			      EA = 1;                    // Restore interrupt state      

					OutModem(24);		    
			   	BufferInModem[0] = 0x42;
					BufferInModem[1] = 0x82;
					BufferInModem[2] = (BufferInModem[0]^BufferInModem[1]) | 0x80;

					SFRPAGE = 0x00;
					TI0 = 1;
				}
			}*/
/*			else if (NPackage == 0x3f) //Считать поле
			{
				if(nByte == 3 && KontrSumma == 0)
	            GetValue(NPackage);
			}
         else if (nByte == 7 && KontrSumma == 0 && NPackage <= 13)
         {
/*				temp_koord = RK_code[5] & 0x0f;
				temp_koord = (temp_koord << 7) + RK_code[4];
				temp_koord = (temp_koord << 7) + RK_code[3];
				temp_koord = (temp_koord << 7) + RK_code[2];
				temp_koord = (temp_koord << 7) + RK_code[1];

				Arr = (char xdata *) &temp_koord;
			   pwrite = (char xdata *) &a[NPackage];          // FLASH write/erase pointer
				  
		      EA = 0;                          // Disable interrupts                 
			   
			   SFRPAGE = 0x0f;	// Enable FLASH block writes
			   CCH0CN &= 0xfe;                   

			   SFRPAGE = 0;
			   for( i = 0; i < 4; i++ )
				{
			      FLSCL |= 0x01;                   // Enable FLASH writes/erases
			      PSCTL = 0x01;                    // MOVX writes write FLASH byte
		
			      pwrite[i] = Arr[i];                 // Write Data to FLASH                   

			      PSCTL = 0;                    // MOVX targets XRAM
			      FLSCL &= ~0x01;                  // Disable FLASH writes/erases
			   }
   
			   SFRPAGE = CONFIG_PAGE;
			   CCH0CN &= ~0x01;                    // Clear the CHBLKW bit
		      EA = 1;                    // Restore interrupt state      

            GetValue(NPackage);
			}
		}*/
	}
	return;

first:	//Управление движением-----------------------------------------------------------
	while(1)
	{
//		   WDTCN = 0xA5;	//Перезапустить охранный таймер
		flStart = !bitStart;
//		bitSysReady = !flNoKoord;

   	if(rBFM < wBFM+marBFM*NBFM)
   	{
			if ((BuferFromModem[rBFM] & 0xC0) == 0x40)	
			{
				nByte = 0;
				KontrSumma = 0;
				NPackage = BuferFromModem[rBFM] & 0x3f;
				NoCommand = 0;

  				WriteByteInKZA (0x40|21);	
				WriteByteInKZA( (liTimer_tick & 0x007f) | 0x80 );
				WriteByteInKZA( ((liTimer_tick & 0x3f80) >> 7) | 0x80 );
				WriteByteInKZA( ((liTimer_tick & 0x1fc000) >> 14) | 0x80 );
				WriteByteInKZA( ((liTimer_tick & 0xfe00000) >> 21) | 0x80 );
			}
   		WriteByteInKZA( BuferFromModem[rBFM] );

			if (nByte > 11)
				nByte = 11;
			RK_code[nByte] = BuferFromModem[rBFM] & 0x7f;
			KontrSumma = KontrSumma^RK_code[nByte++];

			if ( (nByte == 3) && (KontrSumma == 0) )
			{
            if ( NPackage == 1 )	//H_zad
				{
					if (flStopSU != 1)
					{
                  flCommand = 1;
						H_zad = RK_code[1];
						H_zad = 50*H_zad - 1000;
						if (H_zad < 100)
							H_zad = 100;
						else if (H_zad > 3000)
							H_zad = 3000;

						if(s_H_filtr < H_zad)	//летим снизу
							H_max = s_H_filtr;
						else
							H_max = H_zad;
					}
				}
			   else if (NPackage == 2)
			   {
	   		   if(RK_code[1] == 1 && flOk == 0)  //Телеметрия
               {
                  flGPS = 1;
                  OutModem();
               }
	   		   else if(RK_code[1] == 2)    //Стоп двигатель Режим планирования
					{
                  flCommand = 1;
   					flStopSU = 1;
						H_zad = 0;
						H_max = H_zad;	
					}
	   		   else if(RK_code[1] == 3)    //Стоп двигатель Посадка
					{
                  flCommand = 1;
   					flStopSU = 1;
						H_zad = 0;
						H_max = H_zad;	
					}
		      	else if (RK_code[1] == 4)  //Инициализация Ок
      			{
	  			   	BufferInModem[0] = 0x40 | 25;
						BufferInModem[1] = 0x80;
    					BufferInModem[2] = (BufferInModem[0]^BufferInModem[1]) | 0x80;
                 	while (flTransmiter)
	   	            ;			
                  flTransmiter = 1;
	               r0 = 0;
	               rk = 3;
					   SFRPAGE = 0x00;
   					TI0 = 1;
						flOk = 1;

						n_ = 1;
				      lLatZad = LatMar[n_];
  					   lLonZad = LonMar[n_];
	      			lLatZad_pr = LatMar[n_-1];
		   		   lLonZad_pr = LonMar[n_-1];
     				}
		      	else if (RK_code[1] == 9)  //Возврат
				   {
                  flCommand = 1;
				      n_ = 0;
				      Dat1 = Dat1 | 0x03;
				      lLatZad = LatMar[n_];
				      lLonZad = LonMar[n_];
			         lLatZad_pr = lLatFly;	//либо ReadKoord(LatMar, 1); - наведение на первую ветвь 
		   	      lLonZad_pr = lLonFly;	//либо ReadKoord(LonMar, 1);
					}
				}
				else if ( NPackage == 3 )//Kren_zad
				{
					if ((RK_code[1] > 17) && (RK_code[1] < 103))
					{
						tmp = RK_code[1];
						Kren_zad_buf = -(tmp-60);
               }
			      Dat1 = Dat1 & 0xfc;
				}
			   else if(NPackage == 4)	         //автономный полет
			   {
               flCommand = 1;
			      n_ = RK_code[1];
					if (n_ <= i_mar)
					{
  				      Dat1 = (Dat1 & 0xfc) | 0x02;
				      lLatZad = LatMar[n_];
				      lLonZad = LonMar[n_];
			         lLatZad_pr = LatMar[n_-1];
			         lLonZad_pr = LonMar[n_-1];
					}
			   }
			   else if(NPackage == 5)	//Vz_zad
			   {
               flCommand = 1;
/*			      if ((Dat & 0x0060) != 0x40)
         			int_delta_ax = nSU;*/
//			      Dat1 = Dat1 | 0x40;
			      Vz_zad = RK_code[1];
			   }
			   else if(NPackage == 8)	//Крен камеры зад.
			   {
               flCommand = 1;
			      KrenKam_zad = RK_code[1];
					KrenKam_zad = KrenKam_zad-60;
			   }
			   else if(NPackage == 9)	//Угол камеры к горизонту зад.UgolKam_zad
			   {
               flCommand = 1;
			      UgolKam_zad = RK_code[1];
			   }
			   else if(NPackage == 10) //Номер точки во время неавтономного полета
			   {
               flCommand = 1;
			      n_ = RK_code[1];
				}
			   else if(NPackage == 12)	//Газ зад.
			   {
               flCommand = 1;
					flStopSU = 0;
               flAvtomatV = 0;
			      Delta_G = RK_code[1];
					if(Delta_G > 100)
						Delta_G = 100;
					else if(Delta_G < 0)
						Delta_G = 0;
			   }
			   else if(NPackage == 13) //Vy_zad_max
			   {
               flCommand = 1;
			      Vy_zad_max = RK_code[1];
				}
			   else if(NPackage == 14) //Vy_zad_min
			   {
               flCommand = 1;
			      Vy_zad_min = RK_code[1];
				}
 			   else if(NPackage == 15) //Delta_Zak
   		   {
               flCommand = 1;
   	         Delta_Z = -10+RK_code[1];
			   }
			}	//if ( nByte == 3 )
		   else if((NPackage == 6) && (nByte == 10))    //наводиться на точку
			{
				if ( KontrSumma == 0)
				{
			      lLatZad_pr = lLatFly;
			      lLonZad_pr = lLonFly;

			      Dat1 = (Dat1 & 0xfc) | 0x01;
					lLatZad = DecodeLatOrLon(RK_code, 1);
					lLonZad = DecodeLatOrLon(RK_code, 5);
				}
			}
         //Координаты ППМ
			else if ((NPackage == 11) && (nByte == 11) && (flOk == 0) && (KontrSumma == 0))
			{
				temp1 = RK_code[9];	//temp1 = n_point
				if(temp1 < 127)
				{
					if(temp1 > i_mar)
						i_mar = temp1;
					LatMar[temp1] = DecodeLatOrLon(RK_code, 1);
					if (temp1 == 0)
					{
				  		cos_Lat0 = LatMar[0];
				  		cos_Lat0 = cos((cos_Lat0/60/10000-90)/ToGrad);
						flPriv = 1;
					}

					LonMar[temp1] = DecodeLatOrLon(RK_code, 5);

				  	BufferInModem[0] = 24 | 0x40;
				   PackageInModem4(LatMar[temp1], 1);	
					PackageInModem4(LonMar[temp1], 5);	
			      BufferInModem[9] = temp1 | 0x80;  
      			BufferInModem[10] = ContrlSumma(BufferInModem, 10) | 0x80;
	
              	while (flTransmiter)
		            ;			
               flTransmiter = 1;
	            r0 = 0;
	            rk = 11;
					SFRPAGE = 0x00;
					TI0 = 1;
				}
         }
         rBFM++;
			if(rBFM >= NBFM)
			{
   			rBFM = 0;
				marBFM = 0;	
			}
      }

		//-----------------------------------------------------------------------------------
		if(flPriv && (Dat1 & 0x03) && flNew && flOk)	//не ручное управление
		{
			flNew = 0;
start:
			dz = lLonZad;
			dz = 0.1856*(dz - lLonFly)*cos_Lat0;
		   dx = lLatZad;
		   dx = 0.1856*(dx-lLatFly);

		  	if( (Dat1 & 0xfc) != 1)//автономный полет
			{
	   		dz_pr = lLonZad_pr;
			   dz_pr = 0.1856*(dz_pr-lLonFly)*cos_Lat0;
   			dx_pr = lLatZad_pr;
		   	dx_pr = 0.1856*(dx_pr-lLatFly);

			   tmp =sqrt(dz*dz+dx*dx);		//tmp = rasst_toch_mar
   			if ((EPSILON > tmp) && (n_ != i_mar) && (n_ != 0))
			   {
					n_++;
			      lLatZad = LatMar[n_];
   			   lLonZad = LonMar[n_];
		      	lLatZad_pr = LatMar[n_-1];
	   		   lLonZad_pr = LonMar[n_-1];
					
//       			OutModem(-1);		//Уведомление Земли

		   	   goto start;
   			}

			   if ( fabs(dx-dx_pr) < 1e-3 )
				{
					if(dz-dz_pr > 0)	napr_vetv_mar = M_PI_2;
					else					napr_vetv_mar = -M_PI_2;
				}
			  	else
				{
//					tmp1 = (dz-dz_pr)/(dx-dx_pr);
// 					napr_vetv_mar = atan(tmp1);
 					napr_vetv_mar = atan2(dz-dz_pr, dx-dx_pr);
				}
				if ( fabs(dx) < 1e-3 )
				{
					if(dz > 0)	angle = M_PI_2;
					else			angle = -M_PI_2;
				}
   			else
				{
//					tmp1 = dz/dx;
					angle = atan2(dz,dx);	//napr_toch_mar = atan2(dz, dx),
				}
				angle = angle-napr_vetv_mar;

			   otkl_ot_mar = tmp*sin(angle);
				if (fabs(otkl_ot_mar) > 500) tmp = 200;	//tmp = l_km
				else                         tmp = 500;

			   while (angle > M_PI)
		  			angle -= D_PI;
				while (angle < -M_PI)
			  		angle += D_PI;
			   if(fabs(angle) > M_PI_2)
			      tmp = -tmp;

			   dz = otkl_ot_mar*cos(napr_vetv_mar)+tmp*sin(napr_vetv_mar);	//dz = z_toch_pricel
   			dx = -otkl_ot_mar*sin(napr_vetv_mar)+tmp*cos(napr_vetv_mar);	//dx = x_toch_pricel
			}
			tmp = koors;
			tmp =  -tmp/ToGrad;
			if ( fabs(dx) > 1e-3 ) 
			{
//				tmp1 = dz/dx;
//				tmp =  tmp + atan(tmp1);
				tmp =  tmp + atan2(dz, dx);
			}
		   while (tmp > M_PI)
      		tmp -= D_PI;
		   while (tmp < -M_PI)
      		tmp += D_PI;
		   tmp = -ToGrad*tmp;		//tmp = kren_zad_auto
			if(tmp > 42)
				tmp = 42;
		   else if(tmp < -42)
				tmp = -42;
/*			if(flAvarStop)
				tmp = 0;
*/
			Kren_zad_buf = tmp;
		}

		//Расшифровка посылки GPS
		if (r < w+mar*NS) 
		{
			if(mess[r] == '$')
			{
            nLetter = 0;
			}
         else if ((nLetter == 0) && (mess[r] == 'G'))
            nLetter++;
         else if ((nLetter == 1) && (mess[r] == 'P'))
            nLetter++;
         else if ((nLetter == 2) && (mess[r] == 'R'))
            nLetter++;
         else if ((nLetter == 3) && (mess[r] == 'M'))
            nLetter++;
         else if ((nLetter == 4) && (mess[r] == 'C'))
         {
            nLetter++;
				i_comma = 0;
				ValidGPS = 0;
         } 
		   else if(mess[r] == ',')
			{
				i_comma++;
				i = 0;
				flPoint = 0;
			}
		   else if(i_comma == 2)
			{
				if(mess[r] == 'A') 
					ValidGPS = 1;
				else
				{
					ValidGPS = 0;
					flNoKoord = 1;
					liTimer_tick_GPS = liTimer_tick;
				}
			}
         else if (ValidGPS)
			{
				if (i_comma == 3)                //Latitude
				{
					if(mess[r] == '.')
					{
						flPoint = 1;
						i = 0;
					}
					else if (flPoint == 0)			//Целая часть
					{
						tmpGPS[i++] = mess[r];
						if(i == 2)
						{
		      		   tmpGPS[i] = 0;
							temp_koord = atoi(tmpGPS);
   			      	temp_koord = 60UL*10000*temp_koord;
						}	
						else if(i == 4)
						{
							tmpGPS[0] = tmpGPS[1] = '0';
			      	   tmpGPS[i] = 0;
   				      temp_koord = temp_koord+10000UL*atoi(tmpGPS);
						}	
					}
					else					//Дробная часть
					{
						tmpGPS[i++] = mess[r];
	      		   tmpGPS[i] = 0;
					}
				}
				else if (i_comma == 4)
				{
		      	temp_koord = temp_koord+atoi(tmpGPS);
//-----------
//lLatFly = 55UL*60*10000+50UL*10000+8680;
//--------
					if (mess[r] == 'S')   				//знак Latitude
						lLatFly = 54000000UL-temp_koord;		//90UL*60*10000-koord;
					else		  
						lLatFly = 54000000UL+temp_koord;	//90UL*60*10000+koord;
					liTimer_tick_GPS = liTimer_tick;
				}
				else if (i_comma == 5)                //Longitude
				{
					if(mess[r] == '.')
					{
						flPoint = 1;
						i = 0;
					}
					else if (flPoint == 0)			//Целая часть
					{
						tmpGPS[i++] = mess[r];
						if(i == 3)
						{
		      		   tmpGPS[i] = 0;
   			      	temp_koord = atoi(tmpGPS);
   			      	temp_koord = 60UL*10000*temp_koord;
						}	
						else if(i == 5)
						{
							tmpGPS[0] = tmpGPS[1] = tmpGPS[2] = '0';
			      	   tmpGPS[i] = 0;
   				      temp_koord = temp_koord+10000UL*atoi(tmpGPS);
						}	
					}
					else					//Дробная часть
					{
						tmpGPS[i++] = mess[r];
	      		   tmpGPS[i] = 0;
					}
				}
				else if (i_comma == 6)
				{
		      	temp_koord = temp_koord+atoi(tmpGPS);
//----------------
//lLonFly = 49UL*60*10000+6UL*10000+3760;
//----------------
					if (mess[r] == 'W')   //знак Longitude
						lLonFly = 108000000UL-temp_koord;		//180UL*60*10000-koord;
					else       
						lLonFly = temp_koord+108000000UL;	//180UL*60*10000;
				}
				else if (i_comma == 7)	//скорость в узлах
				{
					if(mess[r] == '.')
					{
						flPoint = 1;
   		      	Vz = 1.852*atoi(tmpGPS)/3.6;   //??? Преобразовать из узлов в м/с
//---------
//Vz = 20;
//-----------
					}
					else if(flPoint == 0)
					{
						tmpGPS[i++] = mess[r];
	      	   	tmpGPS[i] = 0;
					}
            }
				else if (i_comma == 8)	//курс в градусах
				{
					if(mess[r] == '.')
					{
						flPoint = 1;
   		      	koors = atoi(tmpGPS);
						flNoKoord = 0;
						flGPS =1;
						flNew = 1;
						liTimer_tick_GPS = liTimer_tick;	
//-----------						   
//koors = 30;
//-----------
					}
					else if(flPoint == 0)
					{
						tmpGPS[i++] = mess[r];
	      	   	tmpGPS[i] = 0;
					}
            }
			}
			r++;
	   	if(r >= NS)
			{
   	   	r = 0;
				mar = 0;	
			}      
		}

		//-------------------------------------------------------------------------------------
		if(flRun)	
		{
			flRun = 0;

			//аппроксимация скорости --------------------------------
        	delta_ro=1-0.000095*s_H_filtr+3.05E-09*s_H_filtr*s_H_filtr;
			if (delta_ro < 0.001)
				delta_ro = 0.001;

//			tmp = 2.*(-172.1943+1.37597*iV)/0.125/delta_ro;	
			tmp = 2.*(-255.+1.37597*iV)/0.125/delta_ro;	//bort N1
//			tmp = 2.*(-255.+1.37597*iV)/0.125/delta_ro;	//bort N2
			if ( tmp > 0 )
				tmp = sqrt(tmp);
			else
				tmp = 0;
			V = V+(tmp-V)/FREQ/0.4;		//q_filtr+(q-q_filtr)*dt/0.4	
     	   ax = ax+((V-V_pr)*FREQ-ax)/FREQ/0.4;
         V_pr = V;

			//Ограничение набора высоты---------------------------------------------------
         if ((H_zad-s_H_filtr) > Vy_zad_ogr_nab*3)
         {
            if (flAvtomatV)
            {
               tmp = Vz_zad-(Vz-V);	//tmp = V_zad
               if (tmp < V_MIN)
                  tmp = V_MIN;
                delta_Vy = -0.15*(tmp-V);
            }
            else
               delta_Vy = -0.15*(100/3.6-V);
					
				if (flStart == 0)
				{
	        	   int_delta_Vy_zad_ogr_nab = 3;
        	      Vy_zad_ogr_nab = int_delta_Vy_zad_ogr_nab;
				}
				else
				{
	        	   int_delta_Vy_zad_ogr_nab += 0.08*delta_Vy/FREQ;
	            if (int_delta_Vy_zad_ogr_nab > 7)
                  int_delta_Vy_zad_ogr_nab = 7;
     	         else if (int_delta_Vy_zad_ogr_nab < 0.5)
        	         int_delta_Vy_zad_ogr_nab = 0.5;
					
	            Vy_zad_ogr_nab = 0.3*delta_Vy+int_delta_Vy_zad_ogr_nab;
               if (Vy_zad_ogr_nab > 7)
     	            Vy_zad_ogr_nab = 7;
        	      else if (Vy_zad_ogr_nab < 0.5)
           	      Vy_zad_ogr_nab = 0.5;
				}
         }
/*         else if(flNabor)
			{
            flNabor = 0;
				if (RegimeSU == 2)
      	      int_delta_ax = n_maxSU+200;
				else if (RegimeSU == 1)
				{
					int_Delta_G = 100;
					Delta_G = 100;
				}
			}*/

			//Управление СУ----------------------------------------------------------
			if (flStopSU == 0)
			{
	         if (flAvtomatV /*&& flNabor == 0*/ )		//стабилизация скорости
  		      {
     		      tmp = Vz_zad-(Vz-V);	//tmp = V_zad
        		   if (tmp < V_MIN)
           		   tmp = V_MIN;

	            if (V < V_MIN)		//tmp = ax_zad
		            tmp = -0.8*(V-tmp);
     		      else
  	   		      tmp = -0.5*(V-tmp);

     	   		tmp = 0.15*V*V*(-ax+tmp);	//tmp = delta_ax
            	if (tmp > 500)
  	            	tmp = 500;
      	      else if (tmp < -500)
  	      	      tmp = -500;

/*     	      	int_delta_ax += 0.1*tmp/FREQ;
      	      if (int_delta_ax > n_maxSU+200)
  	      	      int_delta_ax = n_maxSU+200;
     	      	else if (int_delta_ax < n_minSU-200)
        	      	int_delta_ax = n_minSU-200;
*/	
//  	         	n_zad = tmp + int_delta_ax;
//   		      tmp = n_zad-nSU;		//tmp = delta_n
	
   	   	   j = Delta_G/20;
      	      if (j > 4)
         	   	   j = 4;
					else if (j < 0)
							j = 0;
//					tmp = tmp/FREQ/dn_Delta_G[j];

	         	int_Delta_G = int_Delta_G+0.08*tmp;
					if(int_Delta_G > 100)
						int_Delta_G = 100;
					else if(int_Delta_G < 0)
						int_Delta_G = 0;

   	         Delta_G = int_Delta_G + 0.22*tmp;
				}
			}	//if(flStopSU == 0)

         if (flOk)
            OutModem();
         
         //Отсутствие координат----------------------------------------------
			if (liTimer_tick-liTimer_tick_GPS > 2*FREQ)
			{
				flNoKoord = 1;
				liTimer_tick_GPS = liTimer_tick;	
			}
		}	//if (flRun)
	}	//while (1)
	return;
}

//----------------------------------------------------------------------------
void OutModem(void)
{
   xdata char i;
   xdata unsigned long tmp;
   static bit fl;

   if (flOk)
   {
      tmp = (liTimer_tick/50)*50;
      if ((liTimer_tick - tmp) == 0)
         return;

      fl = !fl;
      if (fl)
         return;
   }
	while (flTransmiter)
		;			

	r0 = 0;
   if (flAnswer == 0)
   {
      BufferInModem[0] = 21 | 0x40;
      PackageInModem4(liTimer_tick, 1);	

      BufferInModem[13] = 0x80;
      PackageInModem2((Vx_gir+800)*10, 14);
      PackageInModem2((Vy_gir+800)*10, 16);
      PackageInModem2((Vz_gir+800)*10, 18);
      PackageInModem2((ax_gir+800)*10, 20);
      PackageInModem2((ay_gir+800)*10, 22);
      PackageInModem2((az_gir+800)*10, 24);
      PackageInModem2(Kren_dat/ToGrad*2500+8000, 26);
      PackageInModem2(tang_gir*2500+8000, 28);
      PackageInModem2((koors_gir-M_PI)*2500+8000, 30);
      PackageInModem2(Wx_r*2500+8000, 32);
      PackageInModem2(Wy_r*2500+8000, 34);
      PackageInModem2(Wz_r*2500+8000, 36);
      PackageInModem2(ex_gir*2500+8000, 38);
      PackageInModem2(ey_gir*2500+8000, 40);
      PackageInModem2(ez_gir*2500+8000, 42);
      PackageInModem2(H+5000, 44);
	   PackageInModem2(V*10, 46);
      BufferInModem[48] = SteckPoint | 0x80;   

      BufferInModem[49] = 0;
      for (i = 0; i < 49; i++ )
         BufferInModem[49] = BufferInModem[49] ^ BufferInModem[i];
      BufferInModem[49] = BufferInModem[49] | 0x80;
  		rk = 50;
   }
   else
   {
      BufferInModem[0] = 22 | 0x40;
      BufferInModem[1] = (-Kren_zad_buf+60) | 0x80;

     	PackageInModem2((integral_delta_V+30)*100, 2);
     	PackageInModem2((int_dKren+30)*100, 4);
     	PackageInModem2((Delta_V+30)*100, 6);
     	PackageInModem2((Delta_E+30)*100, 8);
     	PackageInModem2(Vy_zad_ogr_nab*100, 10);
     	BufferInModem[12] = (char)Delta_G | 0x80;
     	BufferInModem[13] = Wak | 0x80;   //Wak

      PackageInModem2(KrenKam/ToGrad*2500+8000, 14); 
      PackageInModem2(UgolKam/ToGrad*2500+8000, 16); 
      BufferInModem[18] = 0x80 | Dat1;
      BufferInModem[19] = 0x80 | Dat2;
      BufferInModem[20] = 0x80;
      BufferInModem[21] = 0x80;
      BufferInModem[22] = 0x80;
      BufferInModem[23] = 0x80;
//      PackageInModem2((Vy+800)*10, 22);s_Vy_filtr
      PackageInModem2((s_Vy_filtr+800)*10, 22);
      PackageInModem2(Kren_dat/ToGrad*2500+8000, 24);
      BufferInModem[26] = (char)(Delta_Z+20) | 0x80;
      BufferInModem[27] = 0x80;

      BufferInModem[28] = 0;
      for (i = 0; i < 28; i++ )
         BufferInModem[28] = BufferInModem[28] ^ BufferInModem[i];
      BufferInModem[28] = BufferInModem[28] | 0x80;
  		rk = 29;

      if(flGPS)
      {
         flGPS = 0;
         BufferInModem[rk] = 20 | 0x40;
			if(flNoKoord)
   		{
            BufferInModem[rk+1] = 0x80;	   
            BufferInModem[rk+2] = 0x80;	   
            BufferInModem[rk+3] = 0x80;	   
            BufferInModem[rk+4] = 0xff;	   
            BufferInModem[rk+5] = 0x80;	   
            BufferInModem[rk+6] = 0x80;	   
            BufferInModem[rk+7] = 0x80;	   
            BufferInModem[rk+8] = 0x80;	   
            BufferInModem[rk+9] = 0x80;	   
            BufferInModem[rk+10] = 0x80;	   
            BufferInModem[rk+11] = 0x80;	   
			}
         else
         {
            PackageInModem4(lLatFly, rk+1);
         	PackageInModem4(lLonFly, rk+5);
         	PackageInModem2(koors, rk+9);	//курс
            BufferInModem[rk+11] = Vz | 0x80;	   //Vзем
            PackageInModem4(lLatFly, rk+12);	
    	      PackageInModem4(lLonFly, rk+16);	
         }
         BufferInModem[rk+20] = 0;
         for (i = rk; i < rk+20; i++ )
	         BufferInModem[rk+20] = BufferInModem[rk+20] ^ BufferInModem[i];
     	   BufferInModem[rk+20] = BufferInModem[rk+20] | 0x80;
     		rk = 50;
      }
      else if(flCommand)
      {
         flCommand = 0;
         BufferInModem[rk] = 23 | 0x40;

  	      BufferInModem[rk+1] = (H_zad+1000)/50 | 0x80;
  	      BufferInModem[rk+2] = (char)Delta_G | 0x80;
    	   BufferInModem[rk+3] = n_ | 0x80;
  	      BufferInModem[rk+4] = Vz_zad | 0x80;
  	      BufferInModem[rk+5] = Vy_zad_max | 0x80;
     	   BufferInModem[rk+6] = Vy_zad_min | 0x80;
         BufferInModem[rk+7] = KrenKam_zad+60 | 0x80;
         BufferInModem[rk+8] = UgolKam_zad | 0x80;
         BufferInModem[rk+9] = (char)(Delta_Z+10) | 0x80;

         BufferInModem[rk+10] = 0;
         for (i = rk; i < rk+10; i++ )
	         BufferInModem[rk+10] = BufferInModem[rk+10] ^ BufferInModem[i];
     	   BufferInModem[rk+10] = BufferInModem[rk+10] | 0x80;
     		rk = 48;
      }
   }
   flAnswer = !flAnswer;
	flTransmiter = 1;
	SFRPAGE = 0x00;
	TI0 = 1;
	return;
}

//------------------------------------------------------------------------------
void PackageInModem2(unsigned int Data, char i)
{
	BufferInModem[i] = (Data & 0x007f)| 0x80;
	BufferInModem[i+1] = ((Data & 0x3f80) >> 7)| 0x80;
}

//------------------------------------------------------------------------------
void PackageInModem4(unsigned long int Data, char i)
{
	BufferInModem[i] = (Data & 0x0000007f)| 0x80;
	BufferInModem[i+1] = ((Data & 0x3f80) >> 7) | 0x80;
	BufferInModem[i+2] = ((Data & 0x1fc000) >> 14) | 0x80;
   BufferInModem[i+3] = ((Data & 0xfe00000)>> 21) | 0x80;
}

//---------------------------------------------------------------------------
char ContrlSumma(char Array[], char NByteContrlSumma)
{
	xdata char i, KontrSumma = 0;
	for (i = 0; i < NByteContrlSumma; i++)
		KontrSumma = KontrSumma^Array[i];

	return KontrSumma;
}

//----------------------------------------------------------------------------
unsigned long DecodeLatOrLon(char Array[], char n)
{
	xdata unsigned long koord, tmp;
	   koord = Array[n] & 0x7f;
		tmp = Array[n+1] & 0x7f;
		koord = koord+(tmp << 7);
		tmp = Array[n+2] & 0x7f;
		koord = koord+(tmp << 14);
		tmp = Array[n+3] & 0x7f;
		koord = koord+(tmp << 21);
	return koord;
}

// Config Routine---------------------------------------------------------------------
void SYSCLK(void)
{
	xdata int n = 0;

	SFRPAGE = 0x00;
	FLSCL = 0x10;   // FLASH Memory Control

	SFRPAGE = 0x0F;
	OSCXCN = 0x67;	// EXTERNAL Oscillator Control Register	
	for (n = 0; n < 256; n++)             // wait for osc to start
		;
	while ( (OSCXCN & 0x80) == 0 )        // wait for xtal to stabilize
		;
	CLKSEL = 0x01;  // Oscillator Clock Selector
	OSCICN = 0x00;	// Internal Oscillator Control Register
	PLL0CN = 0x05;  // PLL Control Register 

	SFRPAGE = 0x00;
	FLSCL = 0x10;

	SFRPAGE = 0x0F;
	PLL0CN |= 0x02;
	PLL0DIV = 0x04; // PLL pre-divide Register 
	PLL0MUL = 0x09; // PLL Clock scaler Register
	PLL0FLT = 0x01; // PLL Filter Register
	for(n = 0; n < 60; n++) // wait at least 5us
		;               
	PLL0CN |= 0x02;                        // enable PLL
	while ( (PLL0CN & 0x10) == 0 )        // wait for PLL to lock
		;
	CLKSEL = 0x02;  // Oscillator Clock Selector 
}

//------------------------------------------------------------------------------------------
void port_init(void)
{
   SFRPAGE = 0x0F;
	XBR0 = 0x36;	// XBAR0: Initial Reset Value
	XBR1 = 0x14;	// XBAR1: Initial Reset Value
   XBR2 = 0x44;	// XBAR2: Initial Reset Value

// Select Pin I/0
// NOTE: Some peripheral I/O pins can function as either inputs or 
// outputs, depending on the configuration of the peripheral. By default,
// the configuration utility will configure these I/O pins as push-pull 
// outputs.
                    // Port configuration (1 = Push Pull Output)
    SFRPAGE = 0x0F;
    P0MDOUT = 0x75; // Output configuration for P0 
//    P0MDOUT = 0x49; // Output configuration for P0 
    P1MDOUT = 0x00; // Output configuration for P1 
//    P2MDOUT = 0x18; // Output configuration for P2 
//    P3MDOUT = 0x3F; // Output configuration for P3 
    P3MDOUT = 0x30; // Output configuration for P3 

    P1MDIN = 0x00;  // Input configuration for P1
}

//------------------------------------------------------------------------------------------
void config(void)
{
	SFRPAGE = 0x00;
	RSTSRC = 0x00;	// Reset Source Register

	IE = 0x10;          //Interrupt Enable
	EIE1 = 0x08;        //Extended Interrupt Enable 1
   EIE2 = 0x44;        //Extended Interrupt Enable 2

	IP = 0x10;          //Interrupt Priority
	EIP1 = 0x00;        //Extended Interrupt Priority 1
	EIP2 = 0x40;        //Extended Interrupt Priority 2

	EA = 1;
}

//------------------------------------------------------------------------------------------
void UART0_Init(void)
{
	SFRPAGE = 0x00;

	TMR2CF = 0x08;  // Timer 2 Configuration
//----------------------------для спутника
//   RCAP2L = 0x5e;  // Timer 2 Reload Register Low Byte BaudRate = 9600
//   RCAP2H = 0xff;  // Timer 2 Reload Register High Byte
//----------------------------

   RCAP2L = 0xCA;  // Timer 2 Reload Register Low Byte   BaudRate = 57
   RCAP2H = 0xFF;  // Timer 2 Reload Register High Byte

   TMR2L = 0x00;   // Timer 2 Low Byte	
   TMR2H = 0x00;   // Timer 2 High Byte	
   TMR2CN = 0x04;  // Timer 2 CONTROL
	TR2 = 1;	
	SFRPAGE = UART0_PAGE;
	
	SCON0 = 0x50;
	SSTA0 = 0x15;
}

//-----------------------------------------------------------------
void UART1_Init(void)
{
	SFRPAGE = UART1_PAGE;
	SCON1 = 0x10;       // Serial Port 1 Control Register 
	
	SFRPAGE =0x00;

/*	CKCON = CKCON & 0x08;
	TMOD = 0x22;*/
	
	CKCON = CKCON & 0x08;
	TMOD = TMOD | 0x20;
	TH1 = 0x28;
	TL1 = TH1;
	TR1 = 1;  
}

//-------------------------------------------------------------------
void Timer4_isr(void) interrupt 16 using 1
{
	xdata char SFRPAGE_SAVE = SFRPAGE;
	SFRPAGE = 2;
	TF4 = 0;
	flTimer4Interrupt = 1;
	SFRPAGE = SFRPAGE_SAVE;
	return;
}

//-------------------------------------------------------------------
void UART0_isr(void) interrupt 4 using 1
{
	xdata char SFRPAGE_SAVE = SFRPAGE;
	SFRPAGE = UART0_PAGE;
	if (SteckPoint < SP)
		SteckPoint = SP;	
	
	if (RI0)  
  	{ 
		BuferFromModem [wBFM++] = SBUF0;  // read character
		if(wBFM >= NBFM)
		{
     		wBFM = 0;
			marBFM = 1;	
		}      
	  	RI0 = 0;		
	}
	if (TI0)
	{
  		if(r0 < rk)
			SBUF0 = BufferInModem[r0++];
		else						
			flTransmiter = 0;			//Окончание передачи
		TI0 = 0;
  	}
	SFRPAGE = SFRPAGE_SAVE;
	return;
}

//---------------------------------------------------------------------------------
void UART1_isr(void) interrupt 20  using 2
{
	xdata char SFRPAGE_SAVE = SFRPAGE;
	SFRPAGE = UART1_PAGE;
	if (SteckPoint < SP)
		SteckPoint = SP;	

	if (RI1)  
  	{  
		mess [w++] = SBUF1;  // read character	                     
		if(w >= NS)
		{
      	w = 0;
			mar = 1;	
		}      
		RI1 = 0;
  	}
	TI1 = 0;
	SFRPAGE = SFRPAGE_SAVE;
	return;
}

//-----------------------------------------------------------------------------------
void DAC0_init(void)
{
	SFRPAGE = 0x00;
	REF0CN = 0x03;	// Reference Control Register

	DAC0L = 0xff;	// DAC0 Low Byte Register
	DAC0H = 0x0f;	// DAC0 High Byte Register
	DAC0CN = 0x80;	// DAC0 Control Register

	SFRPAGE = 0x01;	
	DAC1L = 0xff;	// DAC1 Low Byte Register
	DAC1H = 0x0f;	// DAC1 High Byte Register
	DAC1CN = 0x80;	// DAC1 Control Register
}

//-------------------------------------------------------------------
void ADC0_init(void)
{
	SFRPAGE = 0x00;
	REF0CN |= 0x03;	// Reference Control Register
//	REF0CN = 0x03;	// Reference Control Register
	AMX0CF = 0x00;	// AMUX Configuration Register
	AMX0SL = 0x00;
	ADC0CF = 0x80;
	ADC0CN = 0x80;//0xc0;//0x80;

//	ADC0CN = 0xc0;//0x80;

	ADC0LTH = 0x00;	// ADC Less-Than High Byte Register
	ADC0LTL = 0x00;	// ADC Less-Than Low Byte Register
	ADC0GTH = 0xFF;	// ADC Greater-Than High Byte Register
	ADC0GTL = 0xFF;	// ADC Greater-Than Low Byte Register

}

//-------------------------------------------------------------------
void Timer3_init(void)
{
	SFRPAGE = 0x01;
	TMR3CN = 0;
	TMR3CF = 0x08;
	RCAP3 = 0x00;
	TMR3 = RCAP3;
	TR3 = 1;
	return;
}

//-------------------------------------------------------------------
void Timer0_init(void)
{
	SFRPAGE = 0x00;
	CKCON = CKCON | 0x08;   // Clock Control Register	Источник - Sysclk
	TH0 = 0xF1;     // Timer 0 High Byte
	TL0 = TH0;     // Timer 0 Low Byte
	TMOD = TMOD | 0x02;    // Timer Mode Register
	TCON = TCON | 0x50;    // Timer Control Register 
	return;
}

//--------------------------------------------------------------------
void PCA_init(void)
{
	SFRPAGE = 0x00;
	PCA0MD = 0x05;       // PCA Mode Register
	PCA0CN = 0x40;      // PCA Control Register
	PCA0L = 0x00;       // PCA Counter/Timer Low Byte
	PCA0H = 0x00;       // PCA Counter/Timer High Byte	
	
		//Module 0
	PCA0CPM0 = 0xC2;    // PCA Capture/Compare Register 0
	PCA0CPL0 = 0x2B;    // PCA Counter/Timer Low Byte
	PCA0CPH0 = 0xEB;    // PCA Counter/Timer High Byte		

		//Module 1
	PCA0CPM1 = 0xC2;    // PCA Capture/Compare Register 1
	PCA0CPL1 = 0x2B;    // PCA Counter/Timer Low Byte
	PCA0CPH1 = 0xEB;    // PCA Counter/Timer High Byte		

		//Module 2
	PCA0CPM2 = 0xC2;    // PCA Capture/Compare Register 2
	PCA0CPL2 = 0x2B;    // PCA Counter/Timer Low Byte
	PCA0CPH2 = 0xEB;    // PCA Counter/Timer High Byte		

		//Module 3
	PCA0CPM3 = 0xC2;    // PCA Capture/Compare Register 3  
	PCA0CPL3 = 0x2B;    // PCA Counter/Timer Low Byte
	PCA0CPH3 = 0xEB;    // PCA Counter/Timer High Byte		

		//Module 4
	PCA0CPM4 = 0xC2;    // PCA Capture/Compare Register 4	
	PCA0CPL4 = 0x2B;    // PCA Counter/Timer Low Byte
	PCA0CPH4 = 0xEB;    // PCA Counter/Timer High Byte		

		//Module 5
	PCA0CPM5 = 0xC2;    // PCA Capture/Compare Register 5
	PCA0CPL5 = 0x2B;    // PCA Counter/Timer Low Byte
	PCA0CPH5 = 0xEB;    // PCA Counter/Timer High Byte
	return;
}

//----------------------------------------------------------------------
void PCA_ISR (void) interrupt 9 using 3
{
  	xdata unsigned int Count;
	xdata unsigned char j;
	xdata float tmp;
	xdata char SFRPAGE_SAVE = SFRPAGE;
	SFRPAGE = 0;

	if (SteckPoint < SP)
		SteckPoint = SP;	
//   WDTCN = 0xA5;	//Перезапустить охранный таймер
	
	
	//Обороты двигателя---------------------------------------------------
/*	SFRPAGE = 2;
	TR4 = 0;
	Count = TMR4;
	TR4 = 1;
	tmp = Count;

	if(flTimer4Interrupt)
	{
		flTimer4Interrupt = 0;
		tmp = (tmp+65536-TMR4_pr)*FREQ*60;
	}
	else
	{
		tmp = (tmp-TMR4_pr)*FREQ*60;
	}
	nSU = nSU+(tmp-nSU)/0.8/FREQ;
	TMR4_pr = Count;
*/
	//-------------------------------------------------------------------------
	flRun = 1;
   timer1_tick1++;
	liTimer_tick++;


   //Опрос датчиков----------------------------------------------------------------------
	H_tmp = 0;			
	SFRPAGE = 0x00;
	for(j = 0; j < 5; j++)
	{
		AMX0SL = 0x06;	//Выбор канала
		for (Count = 0; Count < 10; Count++)
			;
     	AD0INT = 0;
		AD0BUSY = 1;
     	AD0INT = 0;
		while(AD0BUSY)
			;
  		H_tmp = H_tmp+(ADC0 & 0x0fff);
	}
  	H_tmp = H_tmp/5;
	H_tmp = 10825.6-4.2149*H_tmp+0.00030126*H_tmp*H_tmp-H0;

//---------------
   s_H_filtr = s_H_filtr+(H_tmp-s_H_filtr)/FREQ;

	tmp =  (s_H_filtr-s_H_filtr_pr)*FREQ;
   s_H_filtr_pr = s_H_filtr;
 	s_Vy_filtr = s_Vy_filtr+(tmp-s_Vy_filtr)/FREQ;
   H_filtr = H_filtr+(H_tmp-H_filtr)/FREQ/0.3;

	tmp =  (H_filtr-H_filtr_pr)*FREQ;
   H_filtr_pr = H_filtr;
 	Vy = Vy+(tmp-Vy)/FREQ/0.3;

	//Wx		
/*	AMX0SL = 0x02;	 	
	for (Count = 0; Count < 10; Count++)
		;
  	AD0INT = 0;
	AD0BUSY = 1;
  	AD0INT = 0;
	while(AD0BUSY)
		;
	Wx = ADC0 & 0x0fff;
	Wx = -587.1568 + 0.2848*Wx-Wx0; //Борт №1
//	Wx = -154.39 + 0.083*Wx; //Борт №2
//	Wx = -558.44 + 0.25455*Wx + 0.000010395*Wx*Wx;

	AMX0SL = 0x00;	 	//Wy	
	for (Count = 0; Count < 10; Count++)
		;
  	AD0INT = 0;
	AD0BUSY = 1;
  	AD0INT = 0;
	while(AD0BUSY)
		;
	Wy = ADC0 & 0x0fff;
	Wy = 630.0329-2.5 - 0.33505*Wy-Wy0;  //Борт №1
//	Wy = 173.62 - 0.0848*Wy;         //Борт №2
//	Wy = -755.54 + 0.3486*Wy + 0.000011427*Wy*Wy;

	AMX0SL = 0x01;	 	//Wz	
	for (Count = 0; Count < 10; Count++)
		;
  	AD0INT = 0;
	AD0BUSY = 1;
  	AD0INT = 0;
	while(AD0BUSY)
		;
	Wz = ADC0 & 0x0fff;
	Wz = 630.0329-2.5 - 0.33505*Wz-Wz0;  //Борт №1
	if(flSet_H0 == 0)
   {
      Wx0_pr = Wx0_pr+(Wx-Wx0_pr)/FREQ/0.3;
      Wy0_pr = Wy0_pr+(Wy-Wy0_pr)/FREQ/0.3;
      Wz0_pr = Wz0_pr+(Wz-Wz0_pr)/FREQ/0.3;
   }
*/
	//Wx--------------------------------------------------------------------
   SFRPAGE = 0x02;
   AMX2SL = 2;	
	for (Count = 0; Count < 10; Count++)
		;
  	AD2INT = 0;
	AD2BUSY = 1;
  	AD2INT = 0;
	while(AD2BUSY)
		;
   Tx = Tx+(ADC2-Tx)/FREQ;

	SFRPAGE = 0;
	AMX0SL = 0x02;	 	
	for (Count = 0; Count < 10; Count++)
		;
  	AD0INT = 0;
	AD0BUSY = 1;
  	AD0INT = 0;
	while(AD0BUSY)
		;
/*	Wx = -453.+3.*Tx+(ADC0 & 0x0fff);    //Борт №1 
	Wx = 562.363647 - 0.272727*Wx-Wx0; 
*/
	Wx = -453.+3.*Tx+(ADC0 & 0x0fff);    //Борт №2 
	Wx = 151.656113 - 0.081448*Wx-Wx0; 
   Wx_r = Wx/ToGrad;

   //Wy--------------------------------------------------------------------
   SFRPAGE = 0x02;
   AMX2SL = 7;	
	for (Count = 0; Count < 10; Count++)
		;
  	AD2INT = 0;
	AD2BUSY = 1;
  	AD2INT = 0;
	while(AD2BUSY)
		;
   Ty = Ty+(ADC2-Ty)/FREQ;

	SFRPAGE = 0;
	AMX0SL = 0x00;	 		
	for (Count = 0; Count < 10; Count++)
		;
  	AD0INT = 0;
	AD0BUSY = 1;
  	AD0INT = 0;
	while(AD0BUSY)
		;
/*   Wy = -187.5+1.25*Ty+(ADC0 & 0x0fff);
	Wy = 598.300903-0.318584*Wy-Wy0;  //Борт №1 */
   Wy = -187.5+1.25*Ty+(ADC0 & 0x0fff);
	Wy = 174.070587-0.084706*Wy-Wy0;  //Борт №2 
   Wy_r = Wy/ToGrad;
   
   //Wz--------------------------------------------------------------------
   SFRPAGE = 0x02;
   AMX2SL = 1;	
	for (Count = 0; Count < 10; Count++)
		;
  	AD2INT = 0;
	AD2BUSY = 1;
  	AD2INT = 0;
	while(AD2BUSY)
		;
   Tz = Tz+(ADC2-Tz)/FREQ;

	SFRPAGE = 0;
	AMX0SL = 0x01;	 	
	for (Count = 0; Count < 10; Count++)
		;
  	AD0INT = 0;
	AD0BUSY = 1;
  	AD0INT = 0;
	while(AD0BUSY)
		;
/*   Wz = 150.-1.*Tz+(ADC0 & 0x0fff);
	Wz = -159.214783+0.083141*Wz-Wz0;  //Борт №1*/
   Wz = 150.-1.*Tz+(ADC0 & 0x0fff);
	Wz = -193.718658+0.092072*Wz-Wz0;  //Борт №2
   Wz_r = Wz/ToGrad;

	if(flSet_H0 == 0)
   {
      Wx0_pr = Wx0_pr+(Wx-Wx0_pr)/FREQ/0.3;
      Wy0_pr = Wy0_pr+(Wy-Wy0_pr)/FREQ/0.3;
      Wz0_pr = Wz0_pr+(Wz-Wz0_pr)/FREQ/0.3;
   }

	AMX0SL = 0x07;	 	//Скорость	
	for (Count = 0; Count < 10; Count++)
		;
  	AD0INT = 0;
	AD0BUSY = 1;
  	AD0INT = 0;
	while(AD0BUSY)
		;
	iV = ADC0 & 0x0fff;

	//продольный канал----------------------------------------------------------
	if(flSet_H0)
	{
		Kren_zad = Kren_zad_buf;
		if (Kren_zad > 42)
			Kren_zad = 42;
		else if (Kren_zad < -42)
			Kren_zad = -42;

		if (flStart == 0) //если стоим на катапульте
		{
			H_max = 0;
			i_Vzlet = 0;
     		H_zad = 300;
       	Kren_zad = 0;
         tang_gir = 0;
		}
/*  		else if (i_Vzlet < 30*FREQ)  //взлетный режим Time = 30 c
		{
       	Kren_zad = 0;
     	 	i_Vzlet++;
		} 
  		else if (s_H_filtr < 50)  //Около земли
		{
       	Kren_zad = 0;
		}
      if (V < V_MIN+5)
      {
         tmp = V;
         if (tmp < V_BUM)
            tmp = V_BUM;
         tmp = (tmp-V_BUM)*(42.-6.)/(V_MIN+5.-V_BUM)+6; //tmp = kren_zad_max
         if (Kren_zad > tmp)
            Kren_zad = tmp;
         else if (Kren_zad < -tmp)
            Kren_zad = -tmp;
      }
*/
      if (flStopSU == 0)
      {
	 		tmp = -0.25*(H_filtr - H_zad);    //tmp = Vy_zad
 			if (tmp > Vy_zad_ogr_nab) tmp = Vy_zad_ogr_nab;
			else if (tmp < Vy_zad_ogr_sn) tmp = Vy_zad_ogr_sn;

			tmp = Vy - tmp;   		//tmp = delta_Vy
			if (tmp > 15)  tmp = 15; 
 			else if (tmp < -15) tmp = -15; 
	
	  		int_dV = int_dV + 0.075*tmp/FREQ;
		}
		else
		{
   	   tmp = -0.25*(V-18);	//tmp = delta_V_dat
   	   int_dV += 0.08*tmp/FREQ;

      	tmp = Vy;	//tmp = delta_Vy
			if (tmp > 15)  tmp = 15; 
 			else if (tmp < -15) tmp = -15; 

		}
		if (int_dV	> 11)  int_dV = 11; 
		else if (int_dV	< -20) int_dV = -20; 

		if (flStart == 0) //если стоим на катапульте
			int_dV = -4.5;	//для борта №2 

		Delta_V = 0.3*tmp + int_dV;
		if (Delta_V > 14)  Delta_V = 14;
		else if (Delta_V < -23) Delta_V = -23;

  		//боковой канал
		if (flStart == 0) //если стоим на катапульте
      {
  		 	Kren_dat=0;
         kren_gir = 0;
         tang_gir = 0;
         koors_gir = 0;
      }
		else
		{							
			Kren_dat = Kren_dat+(Wx-1.2*Wy-0.1*Kren_dat)/FREQ;
			if (Kren_dat > 50)	Kren_dat = 50;
			else if (Kren_dat < -50)	Kren_dat = -50;
		}
		tmp = Kren_dat - Kren_zad;   //В данном случае tmp = dKren
	
		if (flStart == 0) //если стоим на катапульте
		{
			int_dKren=0;
		}
		else
		{							  //0.025*tmp*dt;
	  		int_dKren = int_dKren + 0.025*tmp/FREQ;
			if (int_dKren > 8) int_dKren = 8;
			else if (int_dKren < -8) int_dKren =-8;
      }
		Delta_E = 0.2*tmp + int_dKren;
		if (Delta_E > 15)  Delta_E = 15;
		else if (Delta_E < -15) Delta_E = -15;

      //гироскоп--------------------------------------------------------------
      tang_gir += Wz/FREQ;

	 	//управление камерой----------------------------------------------------
		Delta_UgolKam = UgolKam_zad-tang_gir;
		Delta_KrenKam = -0.8*Kren_dat+KrenKam_zad; 

	 	//оценим текущую ситуацию-----------------------------------------------
/*		if (flStart == 0)        //Стоим на катапульте
		{
			flStopSU = 0;
			NoCommand = 0;
		}
		else 
		{
			NoCommand++;
			if (s_H_filtr >= H_avar)	deltaH = 100;
			else							deltaH = 50;

			if (s_H_filtr > H_max && s_H_filtr < H_zad)  //Если летим снизу
				H_max = s_H_filtr;
			if( s_H_filtr < (H_max-deltaH ))	
			{
				flAvarStop = 1;         
				timer1_tick1 = 0;
			}
			if (NoCommand > FREQ*20)   //По отказу РК,  Time = 20 c
			{
				flOtkazRK = 1;
            //возврат
			}
      }*/
   }

	//Шимы---------------------------------------------------------------
	SFRPAGE = 0;

	PCA0CN = 0x40;
//	Count = 60493+Delta_E/30*25/22*1659;	
	Count = 60493+Delta_E*62.84;
	PCA0CPL0 = Count & 0x00ff;		    // PCA Counter/Timer Low Byte
	PCA0CPH0 = (Count & 0xff00) >> 8;  // PCA Counter/Timer High Byte		

//	Count = 60493+Delta_V/30*1659;		//1.52 +-0.5 мс
	Count = 60493+Delta_V*55.3;		//1.52 +-0.5 мс
	PCA0CPL1 = Count & 0x00ff;		    // PCA Counter/Timer Low Byte
	PCA0CPH1 = (Count & 0xff00) >> 8;  // PCA Counter/Timer High Byte		

	if (Delta_UgolKam < 20)
		Delta_UgolKam = 20;
	else if (Delta_UgolKam > 90)
		Delta_UgolKam = 90;
	Count = 60493+(Delta_UgolKam-34)/35*1659;
	PCA0CPL3 = Count & 0x00ff;		    // PCA Counter/Timer Low Byte
	PCA0CPH3 = (Count & 0xff00) >> 8;  // PCA Counter/Timer High Byte		

	if (Delta_KrenKam < -45)
		Delta_KrenKam = -45;
	else if (Delta_KrenKam > 45)
		Delta_KrenKam = 45;
	Count = 60493+(Delta_KrenKam+45)*1659/45;
	PCA0CPL2 = Count & 0x00ff;		    // PCA Counter/Timer Low Byte
	PCA0CPH2 = (Count & 0xff00) >> 8;  // PCA Counter/Timer High Byte		

/*	Count = 60493+(Delta_G-50)/50*1659*0.8;//Борт №2
	PCA0CPL5 = Count & 0x00ff;		    // PCA Counter/Timer Low Byte
	PCA0CPH5 = (Count & 0xff00) >> 8;  // PCA Counter/Timer High Byte		
*/	
	Count = 60493+(Delta_G-50)/50*1659*0.8;//Борт №2
	PCA0CPL4 = Count & 0x00ff;		    // PCA Counter/Timer Low Byte
	PCA0CPH4 = (Count & 0xff00) >> 8;  // PCA Counter/Timer High Byte		


	SFRPAGE = SFRPAGE_SAVE;
	return;
}

//------------------------------------------------------------------------------------
void EraseKZA(void)
{
	xdata int nBlock;
	xdata unsigned char i,j;

	SFRPAGE = 0x00;
	RI0=0;
	TI0=1;

	for (nBlock = 0; nBlock <= 1024; nBlock++)	 	  		
	{
	   CS=0;		  								
	   for(i=0;i<10;i++)						
			;

		SPIF=0;
	   SPI0DAT=0x50;          						//block erase
	   while(!SPIF)
			;

	   SPIF=0;
		SPI0DAT=(nBlock & 0x3F8) >> 3;			// High 7 bits of Block Adress
	   while(!SPIF)
			;
	   
		SPIF=0;
      SPI0DAT=(nBlock & 0x7) << 5;   			// Low 3 bits of Block Adress
	   while(!SPIF)
			;

	   SPIF=0;
      SPI0DAT=0xFF;                				// Don't care byte
	   while(!SPIF)
			;

	   for(i=0;i<10;i++); CS=1; for(i=0;i<10;i++);
   	if (j < 50) 
			j++;
    	else
      {
//        	led = !led;
        	j=0;
      } 

    	while (!RDY)  								// Loop untill flash is busy      
			;
   }
// 	for (k=0; k<300000; k++)
//		;
//	led = 0;
	return;        
}                 

//-----------------------------------------------------------------------------------------
void WriteByteInKZA(unsigned char Byte)
{ 
	static xdata int nByte = 0, nPage = 0;
	static bit buf_num = 1, flNewPage = 1;
	xdata unsigned int i = 0;

	SFRPAGE = 0x00;

	if (flNewPage)
	{
		flNewPage = 0;
		CS=0;               				// Select flash
		for(i=0;i<50;i++)					// Wait ??? nS
			;									
		SPIF=0;
		if (buf_num)       				// Buffer 2 Write
			SPI0DAT=0x87;
		else             					// Buffer 1 Write
			SPI0DAT=0x84;
		while(!SPIF)
			;

		SPIF=0;
		SPI0DAT=0xFF;		  				// Don't care byte
		while(!SPIF)
			;
		
		SPIF=0;
		SPI0DAT=0xfc; 			               // Начинаем запись с нулевого байта
		while(!SPIF)
			;                                         
	
		SPIF=0;
		SPI0DAT = 0;           		
		while(!SPIF)
			;
	}
	SPIF=0;
	SPI0DAT = Byte;//nPage+1;//0x33;		
	while(!SPIF)
		;

	if (nByte < 527) 
		nByte++;
	else 
	{
		nByte = 0;
		flNewPage = 1;
		CS = 1;
		for(i=0;i<10;i++)								// Wait ??? nS
			;
		while (!RDY)
			;

		CS=0;		  
		for(i=0;i<10;i++)							// Wait ??? nS
			;
		SPIF=0;	
		if (buf_num)       			// Oрсоde for Buffer 2 writing
			SPI0DAT=0x86;
		else             				// Oрсоde for Buffer 1 writing
			SPI0DAT=0x83;
		while(!SPIF)
			;
		SPIF=0;
		SPI0DAT=(nPage & 0x1FC0) >> 6;			// High 7 bits of 13-bit Page Adress
		while(!SPIF)
			;
		SPIF=0;
		SPI0DAT=(nPage & 0x003F) << 2;   		// Low 6 bits of 13-bit page adress
		while(!SPIF)
			;
		SPIF=0;
		SPI0DAT=0xFF;                				// Don't care byte
		while(!SPIF)
			;
		for(i=0;i<10;i++)							// Wait 250 nS 
			;
		CS=1;        

		buf_num=!buf_num;
	
		if (nPage < 8192) 
			nPage++;
/*		else 
			nPage = 0;*/	//пока
		for(i=0;i<50000;i++)								// Wait 250 nS
			;
		CS = 1;
	}
}												// End Routine


//--------------------------------------------------------------------------------------------
void ReadKZA(void)
{
	xdata int nPage, i;
	static xdata char RK_code[3], nByte, KontrSumma;

	SFRPAGE = 0x00;

	for (nPage = 0; nPage < 8192; nPage++)	 	  		
   {
		CS=0;            							// Select flash
		for(i=0;i<10;i++)
			;

		SPIF=0;
		SPI0DAT=0x52;         						// Opcode for Main Memory Page Read
		while(!SPIF);

		SPIF=0;	
		SPI0DAT=(nPage & 0x1FC0) >> 6;			// High 7 bits of 13-bit Page Adress
		while(!SPIF);

		SPIF=0;  
		SPI0DAT=((nPage & 0x003F) <<2); 			// Low 6 bits of 13-bit Page Adress and 2
		while(!SPIF)
			;								

		SPIF=0;
		SPI0DAT=0x00; 								// Low byte of Starting byte Adress
		while(!SPIF)
			;

		SPIF=0;
		SPI0DAT=0xFF;  								// Don't care byte
		while(!SPIF)
			;

		SPIF=0;
		SPI0DAT=0xFF;  								// Don't care byte
		while(!SPIF)
			;

		SPIF=0;
		SPI0DAT=0xFF;  								// Don't care byte
		while(!SPIF)
			;

		SPIF=0;
		SPI0DAT=0xFF;  								// Don't care byte
		while(!SPIF)
			;

		for (i = 0;i < 528; i++)
		{
			SPIF=0;
			SPI0DAT=0xFF;
			while(!SPIF)
				;
			BufferInModem[i] = SPI0DAT;
		}	//end read page
		for(i = 0; i < 10; i++)
			;
		CS=1;     

      flTransmiter = 1;
	   r0 = 0;
	   rk = 528;
		TI0 = 1;

   	while(rBFM < wBFM+marBFM*NBFM)
   	{
			if (BuferFromModem[rBFM] == 0x88)	
			{
				nByte = 0;
				KontrSumma = 0;
			}
			if (nByte < 3)
         {
   			RK_code[nByte] = BuferFromModem[rBFM] & 0x7f;
	   		KontrSumma = KontrSumma^RK_code[nByte++];
            rBFM++;
	   		if(rBFM >= NBFM)
		   	{
   		   	rBFM = 0;
				   marBFM = 0;	
   			}

         }
	   	else if ( (nByte == 3) && (KontrSumma == 0) )
            return;
      }


     	while (flTransmiter)
         ;			
	}

   //Окончание чтения
  	BufferInModem[0] = 0x40 | 25;
	BufferInModem[1] = 0x80;
	BufferInModem[2] = (BufferInModem[0]^BufferInModem[1]) | 0x80;

   flTransmiter = 1;
   r0 = 0;
   rk = 3;
	TI0 = 1;
  	while (flTransmiter)
      ;			
	return;
}

/*
//--------------------------------------------------------------------------------------------
void ReadKZA(void)
{
	xdata int nPage, i;
	xdata char tmp, nByte, KontrSumma;

	SFRPAGE = 0x00;

	TI0 = 1;
	for (nPage = 0; nPage < 8192; nPage++)	 	  		
	{
		CS=0;            							// Select flash
		for(i=0;i<10;i++)
			;

		SPIF=0;
		SPI0DAT=0x52;         						// Opcode for Main Memory Page Read
		while(!SPIF);

		SPIF=0;	
		SPI0DAT=(nPage & 0x1FC0) >> 6;			// High 7 bits of 13-bit Page Adress
		while(!SPIF);

		SPIF=0;  
		SPI0DAT=((nPage & 0x003F) <<2); 			// Low 6 bits of 13-bit Page Adress and 2
		while(!SPIF)
			;								

		SPIF=0;
		SPI0DAT=0x00; 								// Low byte of Starting byte Adress
		while(!SPIF)
			;

		SPIF=0;
		SPI0DAT=0xFF;  								// Don't care byte
		while(!SPIF)
			;

		SPIF=0;
		SPI0DAT=0xFF;  								// Don't care byte
		while(!SPIF)
			;

		SPIF=0;
		SPI0DAT=0xFF;  								// Don't care byte
		while(!SPIF)
			;

		SPIF=0;
		SPI0DAT=0xFF;  								// Don't care byte
		while(!SPIF)
			;

		for (i = 0;i < 528; i++)
		{
			SPIF=0;
			SPI0DAT=0xFF;
			while(!SPIF)
				;
			while(!TI0)
				;
			TI0=0;
			SBUF0=SPI0DAT;

			//---------------------------------------------------------------------------------
			if(RI0)
			{
				RI0 = 0;
				tmp = SBUF0;
				if (tmp  == 0x42)	
				{
					nByte = 0;
					KontrSumma = 0;
				}
				else if (nByte > 11)
					nByte = 11;
				BuferFromModem[nByte] = tmp & 0x7f;
				KontrSumma = KontrSumma^BuferFromModem[nByte++];

				if ( (nByte == 3) && (KontrSumma == 0) )
				{
				   if(BuferFromModem[1] == 8)			//Стоп чтение КЗА 
					{
						for(i = 0; i < 10; i++)
							;
						CS=1;     
						for(i=0;i<10;i++)
							;
						return;
					}
				}
			}	//if(RI0)
		}	//end read page
		for(i = 0; i < 10; i++)
			;
		CS=1;     
		for(i=0;i<10;i++)
			;
	}
	return;
}
*/