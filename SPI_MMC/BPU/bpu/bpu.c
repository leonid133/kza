#include "c8051f120.h"
#include <math.h>
#include <stdlib.h>
#include <intrins.h>
#include <float.h>

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

//-------------------------------------------------------------------
sbit bitStart = P2^6; //int0
sbit int1 = P2^7;
//sbit pow = P6^6;  

//-------------------------------------------------------------------
void config(void);
void sysclk(void);
void UART0_Init(void);
void UART1_Init(void);
void port_init(void);
void UART0_isr(void);
void UART1_isr(void);
void DAC0_init(void);
void ADC_init(void);

void Timer0_init(void);
void Timer0_isr(void);

//-------------------------------------------------------------------------------------
//SYSCLK = 49766400 - тактовая частота
#define FREQ         50      		// Частота прерываний таймера Гц
#define Rz  6380.

#define ToGrad 57.2957795130823
#define M_PI   3.14159265358979323846
#define D_PI 	6.28318530717958647692
#define M_PI_2 1,57079632679489661

#define delta_e_max 25
#define delta_v_max 50
#define delta_v_min -30
#define delta_n_max 50
#define delta_k_max 10
#define delta_z_max 50
#define delta_z_min -10
#define delta_g_max 100
#define delta_t_max 100

#define NBFM 		25
xdata char BuferFromModem [NBFM]; // Для анализа с последовательного порта
xdata char wBFM, rBFM, marBFM;	 

void OutModem(void);
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

//время(сек) = timer_tick*FREQ
xdata unsigned int timer_tick;   //относительное (счётчик) 
xdata long int liTimer_tick;     //абсолютное
xdata long int liTimer_tick_GPS; //прихода последней GPS посылки
xdata unsigned int i_Vzlet;             

#define EPSILON 400
#define G 9.81
#define V_MIN 60/3.6
#define V_BUM 50/3.6

xdata float H0, H_tmp, H, s_Vy_filtr, s_H_filtr, s_H_filtr_pr, V, H_filtr, H_filtr_pr, Vy;
xdata float Kren_dat, int_dKren, int_dV, delta_e, delta_v; 
xdata int  H_zad;
xdata float V_zad, V_vetra;
xdata char Kren_zad, Kren_zad_buf, Vz_zad, KrenKam_zad;
xdata unsigned char Vz;
xdata unsigned int NoCommand;
xdata float int_delta_ax, Vy_zad_ogr_nab, int_delta_Vy_zad_ogr_nab, Vy_zad_ogr_sn;
xdata char Vy_zad_max, Vy_zad_min;
#define NS 	75
xdata char mess [NS], r, w, mar;		// Для анализа посылки GPS
//idata char mess[NS]="$GPRMC,064600,A,5551.8573,N,04906.4012,E,251.315,312.7,200500,11.5,E*40"; 
//idata char mess[NS]="$GPRMC,100508,A,5550.9399,N,04906.4464,E,1.640,343.1,170703,11.6,E*4E"; 

/*void ReadKZA(void);*/
void WriteByteInKZA(unsigned char Byte);
//void EraseKZA(void);
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

xdata char flOk, CountRun, dataRSTSRC;
bit flRun   //сработал таймер основного цикла
//, flWDTRun
, flNoKoord
, flAnswer
, flGPS
, flCommand
, flNew
, flPriv
, fln_
, flFromStart;

xdata float q, delta_g, delta_n, delta_k, delta_z, delta_tl, delta_tp;//, Delta_G_pr, int_Delta_G;

#define xx_gir -0.05
#define yy_gir 0.5
#define zz_gir 0.14
xdata unsigned long lLatFly0, lLonFly0;
xdata char Lat0;
xdata float ax_gir, ay_gir, az_gir;
xdata float Wx, Wy, Wz, ex_gir, ey_gir, ez_gir, cos_tang_gir;
xdata float Tx, Ty, Tz, Wx_pr, Wy_pr, Wz_pr;

xdata char KrenKam, UgolKam;

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

unsigned int read(unsigned char adress);
void write(unsigned  char adress, unsigned int PWM);


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

xdata unsigned char SteckPoint;
unsigned char code RLB _at_ 0xfbff;
unsigned char code WELB _at_ 0xfbfe;
unsigned char xdata *pwrite;

xdata float delta_ro, V_pr, ax;

xdata char flInit;
//--------------------------------------------------------------
//xdata char BufferInGPS[25]="PSRF103,02,00,00,01*21";
xdata char BufferInGPS1[25]="$PSRF100,1,4800,8,1,0*0C";
xdata char r01 = 0, rk1 = 25, flTransmiter1 = 1;
xdata char BufferInGPS2[23]="$PSRF103,02,00,00,01*21";
xdata char r02 = 0, rk2 = 23;
int Mycount=0; 
//-----------------------------------------------------------------------
void main(void)
{
	//Для работы с последовательным портом "Модем"
	xdata char RK_code[26], nByte = 0, KontrSumma = 0, temp1, NPackage;	

   xdata float angle, tmp, napr_vetv_mar, otkl_ot_mar, dz, dx, dz_pr, dx_pr; //Для автоуправления

   bit ValidGPS, flPoint; 	
	xdata unsigned char i, i_comma, tmpGPS[6], nLetter = 7;
	xdata unsigned long temp_koord;
	xdata long int tt;

	xdata float delta_Vy;

   SFRPAGE = 0;
   dataRSTSRC = RSTSRC; //Управление источниками сброса
	WDTCN = 0xde;			//Стоп сторожевой таймер
	WDTCN = 0xad;
   //WDTCN = 0x07;	   // Макс время = 0,021 с

   FLSCL = FLSCL | 1;	//Разрешение стирания/записи FLASH памяти

	port_init(); 
	sysclk();	
	UART0_Init();	
	UART1_Init(); 
	DAC0_init();	
	ADC_init();
	Timer0_init(); 

	//SPI Configuration----------------------------------------------
/*	SFRPAGE = 0x00;	
	SPI0CFG = 0x60;	// SPI Configuration Register
	SPI0CKR = 0x03;	// SPI Clock Rate Register--- частота 12441600 Гц  Fsck=0.5*Fsysclk/(SPI0CKR+1)
	SPI0CN = 0x09;//0x01;	// SPI Control Register

   WP = 1;
   RESET_FLASH = 1; 
*/
	config();
//	DAC0 = 0x00;
	SteckPoint = SP;	

	//---------------------------------------------------------------
            
/*            i = PSBANK;       
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
*/
	//Управление источниками сброса-------------------------------------------------
   //если сброс от собаки
	if(dataRSTSRC == 0x08 && flOk && flInit) 
	{
		;
	}
	else
	{
   	//Инициализация переменных------------------------------------------------------
		for (r0 = 0; r0 < SIZE_BUFFER0; r0++)
			BufferInModem[r0] = 0x80;	
		r0 = rk = 0;
		r = w = mar = 0;
 		rBFM = wBFM = marBFM = 0;

		H_zad = 300;
		H0 = 0;
		Vz_zad = 30;

   	flOk = 0;

  		int_delta_Vy_zad_ogr_nab = Vy_zad_ogr_nab = 0.5;
   	Vy_zad_ogr_sn = -4;
   	Vy_zad_max = 40; Vy_zad_min = 20;	
   	int_dV = -4.5; 
   	Kren_dat = int_dKren = 0;

   	liTimer_tick = liTimer_tick_GPS = timer_tick = 0;

   	flInit = 0;
		timer_tick = 0;
		H0 = 0;
   	delta_k = delta_n = delta_e = delta_g = delta_z = delta_v = delta_tl = delta_tp = 0;
	}
	SFRPAGE = 0;
	TI0 = 0;
//	TR0 = 0;
SFRPAGE = UART1_PAGE;
	TI1 = 1;
for ( Mycount= 0; Mycount < 10000; Mycount++)
			_nop_();			
	while(1)	//Управление движением -----------------------------------------------------------
	{ 

	
      CountRun = 0;
//		flStart = !bitStart;
		if (flTransmiter1 == 2)
		{
			 for(tt = 0; tt < 10000; tt++)
			 	_nop_();
			SFRPAGE = UART1_PAGE;
			flTransmiter1 = 3;
			TI1 = 1;
		}
		

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
						H_zad = 10*H_zad - 200;
						if (H_zad < -200)
							H_zad = -200;
						else if (H_zad > 1000)
							H_zad = 1000;
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
					}
	   		   else if(RK_code[1] == 3)    //Стоп двигатель Посадка
					{
                  flCommand = 1;
   					flStopSU = 1;
						H_zad = 0;
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
		      	else if (RK_code[1] == 7)  //Тест рулей + Инициализация
					{
                  flCommand = 1;
						flInit = 0;
						timer_tick = H0 = 0;
				   	delta_k = delta_n = delta_e = delta_g = delta_z = delta_v = delta_tl = delta_tp = 0;
					}
		      	else if (RK_code[1] == 10)  //Возврат
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
						Kren_zad_buf = tmp-60;
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
//			      UgolKam_zad = RK_code[1];
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
			      delta_g = RK_code[1];
					if(delta_g > 100)
						delta_g = 100;
					else if(delta_g < 0)
						delta_g = 0;
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
			      delta_z = -10+RK_code[1];
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

        	   if ( (fabs(dx-dx_pr) <= FLT_EPSILON) && (fabs(dz-dz_pr) <= FLT_EPSILON) )
  		      	napr_vetv_mar = 0;
        	  	else
  		      	napr_vetv_mar = atan2(dz-dz_pr, dx-dx_pr);

      		if ( (fabs(dx) <= FLT_EPSILON) && (fabs(dz) <= FLT_EPSILON) )
		      	angle = 0;
      		else
		      	angle = atan2(dz, dx);	//napr_toch_mar = atan2(dz, dx),
      		angle = angle-napr_vetv_mar;

			   otkl_ot_mar = tmp*sin(angle);
				if (fabs(otkl_ot_mar) > 500) tmp = 200;	//tmp = l_km
				else                         tmp = 500;
			   dz = otkl_ot_mar*cos(napr_vetv_mar)+tmp*sin(napr_vetv_mar);	//dz = z_toch_pricel
   			dx = -otkl_ot_mar*sin(napr_vetv_mar)+tmp*cos(napr_vetv_mar);	//dx = x_toch_pricel
			}
			tmp = koors;
			tmp =  -tmp/ToGrad;
    		if ( (fabs(dx) > FLT_EPSILON) && (fabs(dz) > FLT_EPSILON) )
				tmp =  tmp + atan2(dz, dx);

		   while (tmp > M_PI)
      		tmp -= D_PI;
		   while (tmp < -M_PI)
      		tmp += D_PI;

		   tmp = ToGrad*tmp;		
			if(tmp > 42)
				tmp = 42;
		   else if(tmp < -42)
				tmp = -42;
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
               flGPS = 1;
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
                  if (koors < 0)
                     koors = 360+koors;

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

		//Инициализация ----------------------------------------------------------------------------
      if (flInit == 0 && flRun)
      {
			flRun = 0;
   		if (timer_tick < FREQ)    	//элероны
   			delta_e = 0;     
   		else if (timer_tick < 2.0*FREQ)    	//элероны
   			delta_e = delta_e_max;     
   		else if (timer_tick < 3.0*FREQ)
   			delta_e = -delta_e_max;
   		else if (timer_tick < 4.0*FREQ) 	//руль высоты
			{
	   		delta_e = 0;			
   			delta_v = delta_v_max;		
			}
   		else if (timer_tick < 5.0*FREQ)
   			delta_v = delta_v_min;
   		else if (timer_tick < 6.0*FREQ)  //закрылок
			{
		   	delta_v = 0;			
   			delta_z = delta_z_max;		
			}
   		else if (timer_tick < 7.0*FREQ)
   			delta_z = delta_z_min;
		   else if (timer_tick < 8.0*FREQ)  //рули направления
   		{
	   		delta_z = 0;			
   			delta_n = delta_n_max;		
   		}
   		else if (timer_tick < 9.0*FREQ)
	   		delta_n = -delta_n_max;
   		else if (timer_tick < 10.0*FREQ) 	//колесо
			{
   			delta_n = 0;			
   			delta_k = delta_k_max;		
			}
   		else if (timer_tick < 11.0*FREQ)
	   		delta_k = -delta_k_max;
   		else if (timer_tick < 12.0*FREQ)	//газ
			{
   			delta_k = 0;			
   			delta_g = delta_g_max;		
			}
			else if (timer_tick < 13.0*FREQ)
			{
   			delta_g = 0;
				delta_tl = delta_t_max;
			}
			else if (timer_tick < 14.0*FREQ)
			{
   			delta_tl = 0;
				delta_tp = delta_t_max;
			}
			else
			{
				delta_tp = 0;
   			H0 = s_H_filtr;    //установим H0
   			s_H_filtr = s_H_filtr_pr = 0;

      		lLatFly0 = lLatFly;
      		lLonFly0 = lLatFly;
      		Lat0 = lLatFly0*ToGrad;                     //Установить опорную широту

				flInit = 1;
			}
      }
		
		//Работа---------------------------------------------------------------------
		if(flInit && flRun)		
		{
			flRun = 0;

         //Находим скорость-----------------------------------------------
         tmp = q*2./0.125/delta_ro;
      	if ( tmp > 0 )
		      V = sqrt(tmp);
      	else
		      V = FLT_EPSILON;
         if (V < FLT_EPSILON)
            V = FLT_EPSILON;
         ax = ax+((V-V_pr)*FREQ-ax)/FREQ/0.4;
         V_pr = V;

         if(flGPS && flNoKoord == 0)
         {

            //посадка-----------------------------------------------------
/*            float sVx_dat = V_dat*V_dat-Vy_dat*Vy_dat;
            if (sVx_dat > 0)
               V_vetra = Vz-sqrt(sVx_dat);
            else
               V_vetra = 0;
            if (n_ == i_mar)
            {
            	float dz = lLonMar[n_];
              	dz = 0.1856*(dz - lLonFly)*cos_Lat0;
               float dx = lLatMar[n_];
               dx = 0.1856*(dx-lLatFly);
          	   float r = sqrt(dz*dz+dx*dx);
               if (r < min_float)
                  r = min_float;
               teta_zad = atan(H_dat/r)*ToGrad;
            }*/
         }

			//Ограничение набора высоты--------------------------------------------
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
        	      int_delta_Vy_zad_ogr_nab = Vy_zad_ogr_nab = 0.5;
				}
				else
				{
	        	   int_delta_Vy_zad_ogr_nab += 0.075*delta_Vy/FREQ;
	            if (int_delta_Vy_zad_ogr_nab > 0.1*Vy_zad_max) int_delta_Vy_zad_ogr_nab = 0.1*Vy_zad_max;
     	         else if (int_delta_Vy_zad_ogr_nab < 0.5)       int_delta_Vy_zad_ogr_nab = 0.5;
					
	            Vy_zad_ogr_nab = 0.3*delta_Vy+int_delta_Vy_zad_ogr_nab;
               if (Vy_zad_ogr_nab > 0.1*Vy_zad_max) Vy_zad_ogr_nab = 0.1*Vy_zad_max;
        	      else if (Vy_zad_ogr_nab < 0.5)       Vy_zad_ogr_nab = 0.5;
				}
         }

			//Управление СУ----------------------------------------------------------
/*			if (flStopSU == 0)
			{
	         if (flAvtomatV /*&& flNabor == 0*/// )		//стабилизация скорости
/*  		      {
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
	
/*   	   	   j = Delta_G/20;
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
*/
         if (flOk)
            OutModem();
         
         //Отсутствие координат----------------------------------------------
			if (liTimer_tick-liTimer_tick_GPS > 2*FREQ)
			{
				flNoKoord = 1;
				liTimer_tick_GPS = liTimer_tick;	
            flGPS = 1;
			}
         if (SteckPoint > 100)
            tmp = 1;
		}	//if (flRun)
	
	if (TI1)
	{
		if (flTransmiter1 == 1)
		{
  	  		if(r01 < rk1)
				SBUF1 = BufferInGPS1[r01++];
			else						
				flTransmiter1 = 2;	
							//Окончание передачи
		}
		else if (flTransmiter1 == 3)
		{
			for(Mycount = 0; Mycount < 100; Mycount++)
			 	_nop_();
  	  		if(r02 < rk2)
				SBUF1 = BufferInGPS2[r02++];
			else						
				flTransmiter1 = 0;			//Окончание передачи
	
		}

	TI1 = 0;
		
  	}
	
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

      BufferInModem[5] = dataRSTSRC | 0x80;   
//      dataRSTSRC = 0;
      BufferInModem[6] = 0x80;
      BufferInModem[7] = 0x80;
      BufferInModem[8] = 0x80;
      BufferInModem[9] = 0x80;
      BufferInModem[10] = 0x80;
      BufferInModem[11] = 0x80;
      BufferInModem[12] = 0x80;
      BufferInModem[13] = 0x80;
/*      PackageInModem2((vV.x+800)*10, 14);
      PackageInModem2((vV.y+800)*10, 16);
      PackageInModem2((vV.z+800)*10, 18);*/
      PackageInModem2((ax_gir+800)*10, 20);
      PackageInModem2((ay_gir+800)*10, 22);
      PackageInModem2((az_gir+800)*10, 24);
/*      PackageInModem2(kren_gir*2500+8000, 26);
      PackageInModem2(tang_gir*2500+8000, 28);
      PackageInModem2((rsk_gir-M_PI)*2500+8000, 30);
  */    PackageInModem2(Wx/ToGrad*2500+8000, 32);
      PackageInModem2(Wy/ToGrad*2500+8000, 34);
      PackageInModem2(Wz/ToGrad*2500+8000, 36);
      PackageInModem2(ex_gir/ToGrad*2500+8000, 38);
      PackageInModem2(ey_gir/ToGrad*2500+8000, 40);
      PackageInModem2(ez_gir/ToGrad*2500+8000, 42);
      PackageInModem2(s_H_filtr+5000, 44);
	   PackageInModem2(V*10, 46);
      
      BufferInModem[48] = SteckPoint/2 | 0x80;   

      BufferInModem[49] = 0;
      for (i = 0; i < 49; i++ )
         BufferInModem[49] = BufferInModem[49] ^ BufferInModem[i];
      BufferInModem[49] = BufferInModem[49] | 0x80;
  		rk = 50;
   }
   else
   {
      BufferInModem[0] = 22 | 0x40;
      BufferInModem[1] = (Kren_zad_buf+60) | 0x80;

     	PackageInModem2((int_dV+30)*100, 2);
     	PackageInModem2((int_dKren+30)*100, 4);
     	PackageInModem2((delta_v+30)*100, 6);
     	PackageInModem2((delta_e+30)*100, 8);
     	PackageInModem2(Vy_zad_ogr_nab*100, 10);
     	BufferInModem[12] = (char)delta_g | 0x80;
     	BufferInModem[13] = 0x80;   //Wak

      PackageInModem2(KrenKam/ToGrad*2500+8000, 14); 
      PackageInModem2(UgolKam/ToGrad*2500+8000, 16); 
      BufferInModem[18] = 0x80 | Dat1;
      BufferInModem[19] = 0x80 | Dat2;
      PackageInModem2((ax+800)*10, 20);
      PackageInModem2((s_Vy_filtr+800)*10, 22);
      PackageInModem2(Kren_dat/ToGrad*2500+8000, 24);
      BufferInModem[26] = (char)(delta_z+10) | 0x80;
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

  	      BufferInModem[rk+1] = (H_zad+200)/10 | 0x80;
  	      BufferInModem[rk+2] = (char)delta_g | 0x80;
    	   BufferInModem[rk+3] = n_ | 0x80;
  	      BufferInModem[rk+4] = Vz_zad | 0x80;
  	      BufferInModem[rk+5] = Vy_zad_max | 0x80;
     	   BufferInModem[rk+6] = Vy_zad_min | 0x80;
         BufferInModem[rk+7] = (KrenKam_zad+60) | 0x80;
//         BufferInModem[rk+8] = UgolKam_zad | 0x80;
         BufferInModem[rk+9] = (char)(delta_z+10) | 0x80;

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
	//while ( (PLL0CN & 0x10) == 0 )        // wait for PLL to lock
		;
	CLKSEL = 0x02;  // Oscillator Clock Selector 
}

//------------------------------------------------------------------------------------------
void port_init(void)
{
   SFRPAGE = CONFIG_PAGE;
	XBR0 = 0x06;	// XBAR0: Initial Reset Value
	XBR1 = 0x00;	// XBAR1: Initial Reset Value
   XBR2 = 0x44;	// XBAR2: Initial Reset Value
                    
	// Port configuration (1 = Push Pull Output)
	SFRPAGE = 0x0F;
   P0MDOUT = 0x75; // Output configuration for P0 
   P1MDOUT = 0x00; // Output configuration for P1 
   P2MDOUT = 0x00; // Output configuration for P2 
   P3MDOUT = 0x3F; // Output configuration for P3
	P4MDOUT = 0x00; // Output configuration for P4
   P5MDOUT = 0x00; // Output configuration for P5
   P6MDOUT = 0x00; // Output configuration for P6
	P7MDOUT = 0x00; // Output configuration for P7 

	P1MDIN = 0xFF;  // Input configuration for P1
}

//------------------------------------------------------------------------------------------
void config(void)
{
	SFRPAGE = 0x00;
	RSTSRC = 0x00;	// Reset Source Register

	IE = 0x92;          //Interrupt Enable
	EIE1 = 0x00;        //Extended Interrupt Enable 1
   EIE2 = 0x40;        //Extended Interrupt Enable 2

	IP = 0x10;          //Interrupt Priority
	EIP1 = 0x00;        //Extended Interrupt Priority 1           Приоритет SPI = ?
	EIP2 = 0x40;        //Extended Interrupt Priority 2

//	EA = 1;
}

//Скорость 57600 бод. Тактируется Timer2------------------------------------------------------
void UART0_Init(void)
{
	SFRPAGE = 0x00;

	TMR2CF = 0x08;  // Timer 2 Configuration
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

//-------------------------------------------------------------------
void UART0_isr(void) interrupt 4
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
//SBUF0  = 11;
		
/*  		if(r0 < rk)
			SBUF0 = BufferInModem[r0++];
		else						
			flTransmiter = 0;			//Окончание передачи*/
		TI0 = 0;
  	}
	SFRPAGE = SFRPAGE_SAVE;
	return;
}

//-----------------------------------------------------------------
//GPS, скорость 4800 бод, тактируется Timer1 
void UART1_Init(void)
{
	SFRPAGE = UART1_PAGE;
	SCON1 = 0x10;       		/* Регистр управления COM1
									SCON1.7 = S1MODE= 0: 8-ми разрядный COM-порт с изменяемой скоростью передачи данных
									SCON1.5 = MCE1  = 0: (при S1MODE = SCON1.7 = 0) логический уровень стопового бита игнорируется
									SCON1.4 = REN1 = 1: Разрешение приема 	*/
	
	SFRPAGE =0x00;
	CKCON = CKCON | 0x02;	/*регистр управления тактированием
									CKCON.3 = T0M = 1:	Timer 0 тактируется системным тактовым сигналом 
									(т.е. биты CKCON.1 и CKCON.0 не влияют)
									CKCON.1 = SCA1 = 1:	Биты выбора делителя частоты тактирования Timer 0, 1 
									CKCON.0 = SCA0 = 0: 	SYSCLK/48*/

	TMOD = TMOD | 0x20;		/*Регистр режима Таймеров 0 и 1
									TMOD.7 = GATE1 = 0: Таймер 1 включен, если TR1 = 1, независимо от логического уровня на входе /INT1.
									TMOD.6 = C/T1 = 0:  Т/С1  работает  как  таймер:  Таймер 1  инкрементируется  от  внутреннего  сигнала   
                 										тактирования, который задается битом T1M (CKCON.4).  
									TMOD.5 = T1M1 = 1,  	
									TMOD.4 = T1M0 = 0: 	Выбор режима работы Таймера 1: 8-разрядный таймер/счетчик с автоперезагрузкой*/
	TH1 = 148;		
	TL1 = TH1;
	TR1 = 1;  
}

//---------------------------------------------------------------------------------
void UART1_isr(void) interrupt 20
{
	xdata char SFRPAGE_SAVE = SFRPAGE, tmp;
	//xdata long int tt;
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

		tmp = SBUF1;
		SFRPAGE = UART0_PAGE;
		SBUF0 = tmp;
		TI0 = 1;
		SFRPAGE = UART1_PAGE;			  
  	}	
	
	 
//	TI1 = 0;
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

//--------------------------------------------------------------------
void ADC_init(void)
{
	SFRPAGE = 0x00;
	REF0CN = 0x03;	// Reference Control Register
	AMX0CF = 0x00;	// AMUX Configuration Register
	ADC0CF = 0x80;
	ADC0CN = 0x80;

	ADC0LT = 0x0000;	// ADC Less-Than High Byte Register
	ADC0GT = 0xFFFF;	// ADC Greater-Than High Byte Register

	SFRPAGE = 0x02;
	AMX2CF = 0x00;	// AMUX Configuration Register
	ADC2CF = 0x81;
	ADC2CN = 0x80;

	ADC2LT = 0x00;	// ADC Less-Than High Byte Register
	ADC2GT = 0xFF;	// ADC Greater-Than High Byte Register
}

//-------------------------------------------------------------------
void Timer0_init(void)
{
	SFRPAGE = 0x00;
	CKCON = CKCON | 0x02;   // Clock Control Register	Источник - Sysclk/48
	TH0 = 0xAE;     // 0xFFFF-49766400/48/FREQ = 0xAEFF
	TL0 = 0xFF;     
	TMOD = TMOD | 0x01;    // Timer Mode Register 1 режим - 16 разрядный таймер счетчик

	TR0 = 1; 
	return;
}

//----------------------------------------------------------------------
void TIMER0_ISR (void) interrupt 1
{
  	xdata unsigned int uitmp;
	xdata unsigned char j;
	xdata float tmp;
	xdata char SFRPAGE_SAVE = SFRPAGE;
	
	SFRPAGE = 0;

	TH0 = 0xAE;     // 0xFFFF-49766400/48/FREQ = 0xAEFF
	TL0 = 0xFF;     

	if (SteckPoint < SP)
		SteckPoint = SP;	
   //if (CountRun++ < 5)
    //  WDTCN = 0xA5;	//Перезапустить охранный таймер
   CountRun++;

	flRun = 1;
   timer_tick++;
	liTimer_tick++;
   
   //Опрос датчиков----------------------------------------------------------------------
	H_tmp = 0;			
	SFRPAGE = 0x00;
	for(j = 0; j < 5; j++)
	{
		AMX0SL = 0x06;	//Выбор канала
		for (uitmp = 0; uitmp < 10; uitmp++)
			_nop_();
     	AD0INT = 0;
		AD0BUSY = 1;
     	AD0INT = 0;
		while(AD0BUSY)
			;
  		H_tmp = H_tmp+(ADC0 & 0x0fff);
	}
  	H_tmp = H_tmp/5;
//	H_tmp = 11168.6-4.578*H_tmp+0.000362245*H_tmp*H_tmp-H0;  //№1MKM
//	H_tmp = 10825.6-4.2149*H_tmp+0.00030126*H_tmp*H_tmp-H0;  //№2MKM

	H_tmp = 11635.8-4.7889*H_tmp+0.000393*H_tmp*H_tmp-H0;  //№3

   s_H_filtr = s_H_filtr+(H_tmp-s_H_filtr)/FREQ;
   delta_ro=1-0.000095*s_H_filtr+3.05E-09*s_H_filtr*s_H_filtr;
	if (delta_ro < 0.001)
		delta_ro = 0.001;

	tmp =  (s_H_filtr-s_H_filtr_pr)*FREQ;
   s_H_filtr_pr = s_H_filtr;
 	s_Vy_filtr = s_Vy_filtr+(tmp-s_Vy_filtr)/FREQ;
   H_filtr = H_filtr+(H_tmp-H_filtr)/FREQ/0.3;

	tmp =  (H_filtr-H_filtr_pr)*FREQ;
   H_filtr_pr = H_filtr;
 	Vy = Vy+(tmp-Vy)/FREQ/0.3;

   //Скорость--------------------------------------------------------------
	AMX0SL = 0x07;	 		
	for (uitmp = 0; uitmp < 10; uitmp++)
		_nop_();
  	AD0INT = 0;
	AD0BUSY = 1;
  	AD0INT = 0;
	while(AD0BUSY)
		;
	tmp = ADC0 & 0x0fff;
   tmp = -172.953751+0.524876*tmp+0.000231*tmp*tmp;  //№3
   q = q+(tmp-q)/FREQ/0.3;

	//Wx--------------------------------------------------------------------
   SFRPAGE = 0x02;
   AMX2SL = 2;	
	for (uitmp = 0; uitmp < 10; uitmp++)
		_nop_();
  	AD2INT = 0;
	AD2BUSY = 1;
  	AD2INT = 0;
	while(AD2BUSY)
		;
   tmp = ADC2;
   Tx = Tx+(tmp-Tx)/FREQ*0.1;

	SFRPAGE = 0;
	AMX0SL = 0x02;	 	
	for (uitmp = 0; uitmp < 10; uitmp++)
		_nop_();
  	AD0INT = 0;
	AD0BUSY = 1;
  	AD0INT = 0;
	while(AD0BUSY)
		;
	Wx = -170.285721+1.142857*Tx+(ADC0 & 0x0fff);    //Борт №3 
	Wx = -178.237106 + 0.092784*Wx; 

   //Wy--------------------------------------------------------------------
   SFRPAGE = 0x02;
   AMX2SL = 7;	
	for (uitmp = 0; uitmp < 10; uitmp++)
		_nop_();
  	AD2INT = 0;
	AD2BUSY = 1;
  	AD2INT = 0;
	while(AD2BUSY)
		;
   tmp = ADC2;
   Ty = Ty+(tmp-Ty)/FREQ*0.1;

	SFRPAGE = 0;
	AMX0SL = 0x00;	 		
	for (uitmp = 0; uitmp < 10; uitmp++)
		_nop_();
  	AD0INT = 0;
	AD0BUSY = 1;
  	AD0INT = 0;
	while(AD0BUSY)
		;
   Wy = -383.142853+2.571429*Ty+(ADC0 & 0x0fff);
	Wy = 179.269043-0.091371*Wy;  //Борт №3    

   //Wz--------------------------------------------------------------------
   SFRPAGE = 0x02;
   AMX2SL = 1;	
	for (uitmp = 0; uitmp < 10; uitmp++)
		_nop_();
  	AD2INT = 0;
	AD2BUSY = 1;
  	AD2INT = 0;
	while(AD2BUSY)
		;
   tmp = ADC2;
   Tz = Tz+(tmp-Tz)/FREQ*0.1;

	SFRPAGE = 0;
	AMX0SL = 0x01;	 	
	for (uitmp = 0; uitmp < 10; uitmp++)
		_nop_();
  	AD0INT = 0;
	AD0BUSY = 1;
  	AD0INT = 0;
	while(AD0BUSY)
		;

   Wz = 392.-2.666667*Tz+(ADC0 & 0x0fff);
	Wz = 204.274811-0.091603*Wz;  //Борт №3

   //ax--------------------------------------------------------------------
	SFRPAGE = 0;
	AMX0SL = 0x03;	 	
	for (uitmp = 0; uitmp < 10; uitmp++)
		_nop_();
  	AD0INT = 0;
	AD0BUSY = 1;
  	AD0INT = 0;
	while(AD0BUSY)
		;
//	ax_gir = G*(25.97156-0.009488*(ADC0 & 0x0fff)-cos_nx_vj);  //Борт №1
/*	ax_gir = ADC0 & 0x0fff;
	ax_gir = (25.206301-0.009186*ax_gir);  //Борт №2*/

//	ax_gir = G*(25.285999-0.009225*(ADC0 & 0x0fff)+cos_nx_vj);  //Борт №3

   //ay--------------------------------------------------------------------
	SFRPAGE = 0;
	AMX0SL = 0x05;	 	
	for (uitmp = 0; uitmp < 10; uitmp++)
		_nop_();
  	AD0INT = 0;
	AD0BUSY = 1;
  	AD0INT = 0;
	while(AD0BUSY)
		;
//	ay_gir = G*(25.739033-0.009355*(ADC0 & 0x0fff)+cos_ny_vj);  //Борт №1
/*	ay_gir = ADC0 & 0x0fff;
	ay_gir = (25.9517-0.009324*ay_gir);  //Борт №2*/
//	ay_gir = G*(28.56-0.01*(ADC0 & 0x0fff)+cos_ny_vj);  //Борт №3

   //az--------------------------------------------------------------------
	SFRPAGE = 0;
	AMX0SL = 0x04;	 	
	for (uitmp = 0; uitmp < 10; uitmp++)
		_nop_();
  	AD0INT = 0;
	AD0BUSY = 1;
  	AD0INT = 0;
	while(AD0BUSY)
		;
//	az_gir = G*(-26.35598+0.009569*(ADC0 & 0x0fff)-cos_nz_vj);  //Борт №1
/*	az_gir = ADC0 & 0x0fff;
	az_gir = (-26.7495+0.00946*az_gir);  //Борт №2*/
//	az_gir = G*(-26.956522+0.009662*(ADC0 & 0x0fff)+cos_nz_vj);  //Борт №3
//--------------------------------------------------------------------------------------------------



	//продольный канал----------------------------------------------------------

		if (flStart == 0) 
      {
         int_dV = -5; //если зажата кнопка старта
         tmp = 0;
         i_Vzlet = 0;
      }
      else if (i_Vzlet < 3*FREQ)   //Со старта поддерживаем начальное пространственное положение ЛА
      {
//  	      tmp = ToGrad*tang_gir-(15.+(3.-(float)i_Vzlet/FREQ)*15.);
         if (tmp > 60) tmp = 60;
  	      else if (tmp < -60) tmp = -60;
 	      int_dV += 0.15*tmp/FREQ;
         flFromStart = 1;
      }
      else if (flStopSU == 0)   //моторный полет
      {
	 		tmp = -0.25*(H_filtr - H_zad);    //tmp = Vy_zad
 			if (tmp > Vy_zad_ogr_nab) tmp = Vy_zad_ogr_nab;
			else if (tmp < Vy_zad_ogr_sn) tmp = Vy_zad_ogr_sn;

			tmp = Vy - tmp;   		//tmp = delta_Vy
			if (tmp > 60)  tmp = 60; 
 			else if (tmp < -60) tmp = -60; 
         if (flFromStart)
         {
            flFromStart = 0;
    	      int_dV = delta_v-0.3*tmp;
         }
         else	
	  	   	int_dV = int_dV + 0.075*tmp/FREQ;
		}
		else        //планирование cо скоростью 13 м/с
		{
   	   tmp = -0.25*(V-13);	//tmp = delta_V_dat
			if (tmp > 60)  tmp = 60; 
 			else if (tmp < -60) tmp = -60; 

	  		int_dV = int_dV + 0.075*tmp/FREQ;

      	tmp = Vy;	//tmp = delta_Vy
			if (tmp > 60)  tmp = 60; 
 			else if (tmp < -60) tmp = -60; 
		}
		if (int_dV	> 20)  int_dV = 20; 
		else if (int_dV	< -20) int_dV = -20; 

		delta_v = 0.3*tmp + int_dV;
		if (delta_v > 20)  delta_v = 20;
		else if (delta_v < -20) delta_v = -20;

  		//боковой канал-------------------------------------------------------
		if (flStart == 0) //если зажата кнопка старта
		{
  		 	tmp = Kren_dat = 0;
         Kren_zad = 0;
			int_dKren = -15;
		}
		else
		{	
         if (i_Vzlet < 3*FREQ)
         {
        	 	i_Vzlet++;
          	Kren_zad = 0;
            Kren_dat = Kren_dat+(Wx-Wy-0.1*Kren_dat)/FREQ;
      		tmp = 2.*(Kren_dat - Kren_zad);   //В данном случае tmp = dKren
         }
         else
         {						
      		Kren_zad = Kren_zad_buf;
		      if (Kren_zad > 42)
			      Kren_zad = 42;
      		else if (Kren_zad < -42)
		      	Kren_zad = -42;
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
/*          tmp = 0.9441365-0.015913937*tang_gir*ToGrad; //tmp = K__
            Kren_dat = Kren_dat+(Wx-Wy-Kren_dat/0.11/V*cos_tang_gir*tmp)/FREQ;
            Kren_dat = Kren_dat+(Wx-Wy-Kren_dat/0.11/15)/FREQ;*/

            Kren_dat = Kren_dat+(Wx-Wy-0.1*Kren_dat)/FREQ;
      		tmp = Kren_dat - Kren_zad;   //В данном случае tmp = dKren
         }
	  		int_dKren = int_dKren + 0.05*tmp/FREQ;
			if (int_dKren > delta_e_max) int_dKren = delta_e_max;
			else if (int_dKren < -delta_e_max) int_dKren =-delta_e_max;
      }
//		delta_e = 0.1*tmp + int_dKren;
		if (delta_e > delta_e_max)  delta_e = delta_e_max;          //сломалась машинка это пока
		else if (delta_e < -delta_e_max) delta_e = -delta_e_max;

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
				timer_tick = 0;
			}
			if (NoCommand > FREQ*20)   //По отказу РК,  Time = 20 c
			{
				flOtkazRK = 1;
            //возврат
			}
      }*/
	//Шимы---------------------------------------------------------------
//  delta_e, delta_z, delta_v, delta_n, delta_k, delta_g, delta_tl, delta_tp

   //правый элерон
	uitmp = 25000*(1.5+0.55/delta_e_max*delta_e);
	write(113, uitmp);
	
   //левый элерон
	uitmp = 25000*(1.5-0.55/delta_e_max*delta_e);
	write(114, uitmp);
	
	//delta_z
//	Count = 25000*(0.95+1.1/(delta_z_max-delta_z_min));//*(delta_z-delta_z_min));
	uitmp = 25000*(0.95+1.1/(delta_z_max-(delta_z_min))*(delta_z-(delta_z_min)));
	write(115, uitmp);
	
	//delta_v
	uitmp = 25000*(0.95+1.1/(delta_v_max-(delta_v_min))*(delta_v-(delta_v_min)));
	write(116, uitmp);
	
	//левый delta_n
	uitmp = 25000*(1.5+0.55/delta_n_max*delta_n);
	write(117, uitmp);

	//правый delta_n
	uitmp = 25000*(1.5-0.55/delta_n_max*delta_n);
	write(118, uitmp);

	//delta_k
	uitmp = 25000*(1.5+0.55/delta_k_max*delta_k);
	write(119, uitmp);
	
	//delta_g	
	uitmp = 25000*(0.95+1.1/delta_g_max*delta_g);
	write(120, uitmp);

	//левый delta_tl
	uitmp = 25000*(0.95+1.1/delta_t_max*delta_tl);
	write(121, uitmp);

	//delta_tp
	uitmp = 25000*(0.95+1.1/delta_t_max*delta_tp);
	write(122, uitmp);

	SFRPAGE = SFRPAGE_SAVE;
	return;
}

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
//-----------------------------------------------------------------------------------------
void WriteByteInKZA(unsigned char Byte)
{ 
	Byte++;
	return;
}
