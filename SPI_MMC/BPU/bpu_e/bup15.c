#include <c8051f120.h>
#include <math.h>
#include <stdlib.h>
#include <intrins.h>

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
sbit led = P3^0;
sbit int0 = P1^6;

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

#define NBFM 		15
#define NBFM_1 	14
xdata char BuferFromModem [NBFM]; // Для анализа с последовательного порта
xdata char wBFM, rBFM, marBFM;	 

void OutModem(char NAnswer);
void PackageInModem1(unsigned char Data, char i);
void PackageInModem2(unsigned int Data, char i);
void PackageInModem4(unsigned long int Data, char i);
#define SIZE_BUFFER0	27
	xdata char BufferInModem[SIZE_BUFFER0]; // Для отправки в последовательный порт
//	bit flTransmiter;
	xdata char r0, rk;
	
char ContrlSumma(char *Array, char NByteContrlSumma);

xdata int koors;
unsigned long DecodeLatOrLon(char *Array, char nomer);
xdata unsigned long lLatFly, lLonFly, lLatZad, lLonZad, lLatZad_pr, lLonZad_pr;
	xdata unsigned long int LatMar[128], LonMar[128];
	xdata unsigned char n_, i_mar;
	xdata float cos_Lat0;

xdata unsigned int timer1_tick1; // время(сек) = timer1_tick*FREQ
//-----------------
//xdata unsigned int tttt = 0;
//------------------
xdata long int liTimer_tick = 0;
xdata long int liTimer_tick_GPS;
xdata unsigned int i_Vzlet;              


#define  H_avar   	250
#define N_MIN 500
#define N_MAX 7000
#define EPSILON 400
#define V_MIN 20
#define V_BUM 15

#define HH 50
xdata float H0, s_Vy_filtr, s_H_filtr, s_H_filtr_pr, V, H_filtr, H_filtr_pr, Vy, Wx, Wy, Kren_dat/*, kurs_dat*/, int_dKren, int_dV = 0, Delta_E, Delta_V; 
xdata float H_tmp, H, his_H[HH], his_H0;
xdata unsigned char i_his_H, deltaH;
xdata int  H_zad;
bit flNewH_zad, flTransmiter_pr;
xdata char Kren_zad, Kren_zad_buf, Vz_zad, KrenKam_zad;
xdata float Delta_KrenKam, Delta_UgolKam;
xdata unsigned char Vz, UgolKam_zad;
xdata unsigned int NoCommand, n_zad;
xdata float nSU, int_delta_ax;
xdata float Vy_zad_ogr_nab, int_delta_Vy_zad_ogr_nab, Vy_zad_ogr_sn;
#define NS 	75
#define NS_1 74
xdata char mess [NS], r, w, mar;		// Для анализа посылки GPS
//idata char mess[NS]="$GPRMC,064600,A,5551.8573,N,04906.4012,E,251.315,312.7,200500,11.5,E*40"; 
//idata char mess[NS]="$GPRMC,100508,A,5550.9399,N,04906.4464,E,1.640,343.1,170703,11.6,E*4E"; 

void PackageInKZA1(unsigned char CodByte, unsigned char Cod);
void PackageInKZA2(unsigned char CodByte, unsigned int Cod);
void ReadKZA(void);
void WriteByteInKZA(unsigned char Byte);
void EraseKZA(void);
sbit RDY = P2^2; 
sbit CS = P2^1;

bdata char  Dat = 0;
sbit flStart = Dat^0;
sbit flParashut = Dat^1;
sbit flStopSU = Dat^2;
sbit flTestSU = Dat^3;
sbit flOtkazRK = Dat^4;
sbit flNabor = Dat^5;
sbit flTransmiter = Dat^6;

bdata char  Dat1 = 0;
sbit flKorH = Dat^0;

sbit bitStart = P1^7;
sbit bitStopSU = P2^3;		// 1 - СУ выключена
sbit bitParashut = P2^4;
sbit bitSysReady = P3^1;	// Системная готовность 

bit flAvarStop = 0, flSet_H0 = 0, flRun;// , flWDTRun = 0;
bit flLatNew, flLonNew, flKoorsNew, flVzNew, flNoKoord;

void Timer4_isr(void);
bit flTimer4Interrupt;
xdata unsigned int TMR4_pr; 

xdata int H_max, iV;
bit   flOk, flNew, flPriv, fln_;


	xdata float Delta_G = 100, n_maxSU, n_minSU;
	xdata char Delta_A = 100, RegimeSU;				/* 0 - управление газовой заслонкой
																	1 - управление оборотами двигателя
																	2 - стабилизация скорости полета
																	3 - тест двигателя*/			
	xdata unsigned long t_TestSU;
	xdata float dn_Delta_G[5]={62, 275, 68, 68, 16};
	xdata char count_dn_Delta_G;
	xdata float Delta_G_pr, int_Delta_G;
	xdata int nSU_pr;

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
//-----------------------------------------------------------------------
void main(void)
{
	//Для работы с последовательным портом "Модем"
	xdata char RK_code[12], nByte = 0, KontrSumma = 0, temp1, i_stop, NPackage;	

	xdata char flRegime; /*	0 - ручное управление
									1 - автомат
									2 - автоном	
                           4 - Возврат*/
   xdata float angle, tmp, /*tmp1,*/ napr_vetv_mar, otkl_ot_mar, dz, dx, dz_pr, dx_pr; //Для автоуправления

   bit ValidGPS, flPoint; 	
	xdata unsigned char i, i_comma, tmpGPS[6];
	xdata unsigned long temp_koord;
	xdata unsigned int uiTmp;

//   char xdata * idata pwrite;

	xdata float delta_ro, V_pr, delta_Vy;
	xdata unsigned char j;
//-----------	

	WDTCN = 0xde;
	WDTCN = 0xad;

	SteckPoint = SP;	

	for (r0 = 0; r0 < SIZE_BUFFER0; r0++)
		BufferInModem[r0] = 0x80;	
	r0 = rk = 0;
 	rBFM = wBFM = marBFM = 0;

	H_zad = 300;
	flNewH_zad = 1;

	s_Vy_filtr = s_H_filtr = s_H_filtr_pr = 0;
//-----------
//	SFRPAGE = 0;
//	i = RSTSRC;
//-------------

	for (i_his_H = 0; i_his_H < HH; i_his_H++)
		his_H[i_his_H] = 0;
	H0 = 0;	
	Vz_zad = 30;
	KrenKam_zad = 0;
	UgolKam_zad = 90;
	n_zad = (N_MAX-N_MIN)/2;

	flNoKoord = 1;

	liTimer_tick = liTimer_tick_GPS = 0;
	nSU = 0;
	H0 = 0;
//	H_pr = 0;
	r = w = mar = 0;
	V = 0;
	int_delta_Vy_zad_ogr_nab = Vy_zad_ogr_nab = 3;
	Vy_zad_ogr_sn = -4;


//---------
	bitParashut = 0;
	bitSysReady = 0;
	bitStopSU = 0;
	flStart = 0;	
	n_maxSU = N_MAX;
	n_minSU = N_MIN;
	RegimeSU = 0;

	SFRPAGE = 0;
	if((RSTSRC & 0x80) && bitStart)
	{
		flAvarStop = 1;
		timer1_tick1 = 0;
	}

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

	config();
	DAC0 = 0x00;

	//---------------------------------------------------------------
	Delta_E = 0;
	Delta_V = 0;
   Delta_KrenKam = 0;
	Delta_UgolKam = 90;

	//-----------------------------------------------------------------------------------------
	if (!bitStart) //если есть отрывной разъем
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

	   Delta_G = 0;			//Газ
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

	   Delta_A=0;			//Дроссель
	   timer1_tick1 = 0;
		while(timer1_tick1 < FREQ)
			;

	   Delta_A=100;
	   timer1_tick1 = 0;
		while(timer1_tick1 < FREQ)
			;

		liTimer_tick_GPS = liTimer_tick = 0;
	}
	else
	{
		flAvarStop = 1;
		timer1_tick1 = 0;
	}
	EA = 0;
   H0 = s_H_filtr;    //установим H0
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
		led = int0;
		bitSysReady = !flNoKoord;

   	if(rBFM < wBFM+marBFM*NBFM)
   	{
			if (BuferFromModem[rBFM]  == 0x42)	
			{
				NPackage = 0;
				nByte = 0;
				KontrSumma = 0;
				i = rBFM;
				temp1 = marBFM;
			}
			else if (BuferFromModem[rBFM]  == 0x7e)	
			{
				i = rBFM;
				temp1 = marBFM;
				NPackage = 5;
				nByte = 0;
				KontrSumma = 0;
			}
			else if (nByte > 11)
				nByte = 11;
			RK_code[nByte] = BuferFromModem[rBFM] & 0x7f;
			KontrSumma = KontrSumma^RK_code[nByte++];
         rBFM++;
			if(rBFM > NBFM_1)
			{
   			rBFM = 0;
				marBFM = 0;	
			}
			if ( (nByte == 3) && (KontrSumma == 0) )
			{
			   if(NPackage == 5)			//Первоначальная инициализация БУП 
				{
					rBFM = i;
					marBFM = temp1;
					goto zero;
				}
				else if(RK_code[1] == 1)	//Управление движением МКМ 
				{
					rBFM = i;
					marBFM = temp1;

					SFRPAGE = 0;
//					WDTCN = 0x07;	//1048576/Sysclc=0,02107
//					WDTCN = 0xA5;	//Перезапустить охранный таймер
					goto first;
				}
				else if(RK_code[1] == 6)	//Сброс данных бортовой КЗА через COM - порт
				{
					SFRPAGE = 0;
					EA = 0;
					ReadKZA();
					EA = 1;
				}
				else if(RK_code[1] == 7)	//Стереть КЗА
				{
					SFRPAGE = 0;
					EA = 0;
					EraseKZA();					
					EA = 1;
						OutModem(25);		    //Подтверждение очистки КЗА
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

first:	//Управление движением МКМ-----------------------------------------------------------
	while(1)
	{
//		   WDTCN = 0xA5;	//Перезапустить охранный таймер
		led = int0;
		bitSysReady = !flNoKoord;

   	if(rBFM < wBFM+marBFM*NBFM)
   	{
			if ((BuferFromModem[rBFM] & 0xC0) == 0x40)	
			{
				nByte = 0;
				KontrSumma = 0;
				NPackage = BuferFromModem[rBFM] & 0x3f;
				if (NPackage != 0)
					i_stop = 0;

				NoCommand = 0;
			}
			if (nByte > 11)
				nByte = 11;
			RK_code[nByte] = BuferFromModem[rBFM] & 0x7f;
			KontrSumma = KontrSumma^RK_code[nByte++];

			if ( (nByte == 3) && (KontrSumma == 0) )
			{
			   if(NPackage == 0)							 //Потенциальные команды
   			{
			  		temp1  = (RK_code[1] & 0x78) >> 3;	//temp = command

					if (temp1 == 2 && flParashut == 0)	//аварийный стоп
					{
						i_stop++;
 					   if (i_stop > 1)
						{
							timer1_tick1 = 0;
							flAvarStop = 1;
						}
					}
					else if (temp1 == 3)	//Стоп двигатель Режим планирования
					{
						i_stop++;
 					   if (i_stop > 1)
						{
							flStopSU = 1;
							bitStopSU = flStopSU;
							H_zad = 0;
							H_max = H_zad;	
							flNewH_zad = 1;
						}
					}
					else if (temp1 == 4)	//Коррекция H
					{
						i_his_H = -1;
					}
					else if (temp1 == 5)	//Без коррекции H
					{
						flKorH = 0;
					}
					PackageInKZA1(0x4c, RK_code[1]);
				}
				else if ( NPackage == 1 )	//H_zad
				{
					if (flStopSU != 1)
					{
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
						flNewH_zad = 1;
					}
				}
			   else if (NPackage == 2)
			   {
	   		   if(RK_code[1] == 1 || RK_code[1] == 2)
     				{
						if (flNoKoord)
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
					      PackageInModem4(lLatFly, 1);	
							PackageInModem4(lLonFly, 5);	
							PackageInModem2(koors, 9);	
							PackageInModem1(Vz, 11);	   
						}
				   	PackageInModem2(s_H_filtr+5000, 12);
						PackageInModem1(V, 14);

				      if(RK_code[1] == 1)
				      {
							BufferInModem[0] = 20 | 0x40;
				      	PackageInModem2((-Kren_dat+90)*10, 15);
					      PackageInModem2((s_Vy_filtr+40)*100, 17);
							PackageInModem1(Delta_G, 19);
				     	   PackageInModem1(Delta_A, 20);
				   	   PackageInModem2(nSU, 21);
				      }
				      else
				      {
							BufferInModem[0] = 21 | 0x40;
				     	   PackageInModem1((H_zad+1000)/50, 15);
				   	   PackageInModem1(Delta_G, 16);
				   	   PackageInModem1(Delta_A, 17);
				   	   PackageInModem1(n_, 18);
				   	   PackageInModem1(Vz_zad, 19);
							PackageInModem1(n_zad/250, 20);
					      PackageInModem2(SteckPoint, 21);
				      }
					   PackageInModem1(KrenKam_zad+60, 23);
					   PackageInModem1(UgolKam_zad, 24);
					   PackageInModem1(Dat, 25);

				      BufferInModem[26] = ContrlSumma(BufferInModem, 26) | 0x80;
	         		OutModem(20);	//Телеметрия
						SFRPAGE = 0x00;
						TI0 = 1;
			      }
		      	else if (RK_code[1] == 4)  //Инициализация Ок
      			{
						OutModem(24);		    
	  			   	BufferInModem[0] = 0x40 | 24;
						BufferInModem[1] = 0x80;
    					BufferInModem[2] = (BufferInModem[0]^BufferInModem[1]) | 0x80;
						SFRPAGE = 0x00;
						TI0 = 1;
						flOk = 1;

						n_ = 1;
						PackageInKZA1(0x40 | 34, n_);
				      lLatZad = LatMar[n_];
  					   lLonZad = LonMar[n_];
	      			lLatZad_pr = LatMar[n_-1];
		   		   lLonZad_pr = LonMar[n_-1];
     				}
		      	else if (RK_code[1] == 9 && flStart == 0)  //Тест СУ
      			{
			         RegimeSU = 3;
         			t_TestSU = liTimer_tick + 5*FREQ;
						Delta_G = 0;
						count_dn_Delta_G = -1;
					}
		      	else if (RK_code[1] == 10)  //Возврат
				   {
				      n_ = 0;
						PackageInKZA1(0x40 | 34, n_);
				      flRegime = 4;
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
					flRegime = 0;
				}
			   else if(NPackage == 4)	         //автономный полет
			   {
			      n_ = RK_code[1];
					PackageInKZA1(0x40 | 34, RK_code[1]);
					if (n_ <= i_mar)
					{
				      flRegime = 2;
				      lLatZad = LatMar[n_];
				      lLonZad = LonMar[n_];
			         lLatZad_pr = LatMar[n_-1];
			         lLonZad_pr = LonMar[n_-1];
					}
			   }
			   else if(NPackage == 5)	//Vz_zad
			   {
			      if (RegimeSU != 2)
         			int_delta_ax = nSU;
			      if (RegimeSU != 1)
						int_Delta_G = Delta_G; 

			      RegimeSU = 2;
			      Vz_zad = RK_code[1];
					PackageInKZA1(0x58, RK_code[1]);
			   }
			   else if(NPackage == 7)	//n_zad
			   {
			      if (RegimeSU != 1)
						int_Delta_G = Delta_G; 
			      RegimeSU = 1;
			      uiTmp = RK_code[1];
					n_zad = uiTmp*250;
					if (n_zad < N_MIN)
						n_zad = N_MIN;
					else if (n_zad > N_MAX)
						n_zad = N_MAX;
					PackageInKZA1(0x59, RK_code[1]);
			   }
			   else if(NPackage == 8)	//Крен камеры зад.
			   {
			      KrenKam_zad = RK_code[1];
					KrenKam_zad = KrenKam_zad-60;
					PackageInKZA1(0x5a, RK_code[1]);
			   }
			   else if(NPackage == 9)	//Угол камеры к горизонту зад.UgolKam_zad
			   {
			      UgolKam_zad = RK_code[1];
					PackageInKZA1(0x5b, RK_code[1]);
			   }
			   else if(NPackage == 10) //Номер точки во время неавтономного полета
			   {
			      n_ = RK_code[1];
					PackageInKZA1(0x40 | 34, n_);
				}
			   else if(NPackage == 12)	//Газ зад.
			   {
					RegimeSU = 0;
			      Delta_G = RK_code[1];
					if(Delta_G > 100)
						Delta_G = 100;
					else if(Delta_G < 0)
						Delta_G = 0;
					PackageInKZA1(0x5e, RK_code[1]);
			   }
			   else if(NPackage == 13)	//Воздушная заслонка
			   {
			      Delta_A = RK_code[1];
					if(Delta_A > 100)
						Delta_A = 100;
					else if(Delta_A < 0)
						Delta_A = 0;
					PackageInKZA1(0x5f, RK_code[1]);
			   }
			}	//if ( nByte == 3 )
		   else if((NPackage == 6) && (nByte == 10))    //наводиться на точку
			{
				if ( KontrSumma == 0)
				{
			      lLatZad_pr = lLatFly;
			      lLonZad_pr = lLonFly;

			     	flRegime = 1;
					lLatZad = DecodeLatOrLon(RK_code, 1);
					lLonZad = DecodeLatOrLon(RK_code, 5);
				}
			}
			else if ((NPackage == 11) && (nByte == 11) && (flOk == 0) )	//точка маршрута
			{
				if ( KontrSumma == 0 )
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

				   	BufferInModem[0] = 22 | 0x40;
				      PackageInModem4(LatMar[temp1], 1);	
						PackageInModem4(LonMar[temp1], 5);	

			   	   BufferInModem[9] = temp1 | 0x80;  
   
						BufferInModem[10] = ContrlSumma(BufferInModem, 10) | 0x80;
	
						OutModem(22);
						SFRPAGE = 0x00;
						TI0 = 1;
					}
				}
         }
         rBFM++;
			if(rBFM > NBFM_1)
			{
   			rBFM = 0;
				marBFM = 0;	
			}
      }

		//-----------------------------------------------------------------------------------
		if(flPriv && flRegime && /*flNew &&*/ flOk)	//не ручное управление
		{
			flNew = 0;
start:
			dz = lLonZad;
			dz = 0.1856*(dz - lLonFly)*cos_Lat0;
		   dx = lLatZad;
		   dx = 0.1856*(dx-lLatFly);

		  	if(flRegime != 1)//автономный полет
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
					
       			OutModem(-1);		//Уведомление Земли
					PackageInKZA1(0x40 | 34, n_);

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
			if(flAvarStop)
				tmp = 0;

			Kren_zad_buf = tmp;
		}

		//Расшифровка посылки GPS
		if (r < w+mar*NS) 
		{
			if(mess[r] == '$')
			{
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
				{
					ValidGPS = 1;
//					flNoKoord = 0;
				}
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

					flLatNew = 1;
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

					flLonNew = 1;
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
						flVzNew = 1;
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
						flNew =1;
						flKoorsNew = 1;
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
	   	if(r > NS_1)
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

//			tmp = 2.*(-175.14229+1.39533*iV)/0.125/delta_ro;		//bort N1
			tmp = 2.*(-172.1943+1.37597*iV)/0.125/delta_ro;	//bort N2
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
            flNabor = 1;
            if (RegimeSU == 2)
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
         else if(flNabor)
			{
            flNabor = 0;
				if (RegimeSU == 2)
      	      int_delta_ax = n_maxSU+200;
				else if (RegimeSU == 1)
				{
					int_Delta_G = 100;
					Delta_G = 100;
				}
			}

			//Управление СУ----------------------------------------------------------
			if (flStopSU == 0)
			{
  				if (Delta_G > 90)
   				n_maxSU = n_maxSU + 0.05*((nSU+dn_Delta_G[4]*(100-Delta_G))-n_maxSU)/FREQ;
		   	else if (Delta_G < 10)
   				n_minSU = n_minSU + 0.05*((nSU+dn_Delta_G[0]*(0-Delta_G))-n_minSU)/FREQ;

	         if (RegimeSU == 2 && flNabor == 0 && flStart)		//стабилизация скорости
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

     	      	int_delta_ax += 0.1*tmp/FREQ;
      	      if (int_delta_ax > n_maxSU+200)
  	      	      int_delta_ax = n_maxSU+200;
     	      	else if (int_delta_ax < n_minSU-200)
        	      	int_delta_ax = n_minSU-200;
	
  	         	n_zad = tmp + int_delta_ax;
				}
	  		   if (RegimeSU == 2 && flNabor)
					;
	  		   else if (RegimeSU == 1 || RegimeSU == 2) //стабилизация оборотов
		      {                       
   		      tmp = n_zad-nSU;		//tmp = delta_n
	
   	   	   j = Delta_G/20;
      	      if (j > 4)
         	   	   j = 4;
					else if (j < 0)
							j = 0;
					tmp = tmp/FREQ/dn_Delta_G[j];

	         	int_Delta_G = int_Delta_G+0.08*tmp;
					if(int_Delta_G > 100)
						int_Delta_G = 100;
					else if(int_Delta_G < 0)
						int_Delta_G = 0;

   	         Delta_G = int_Delta_G + 0.22*tmp;
	   	   }
		   	else if (RegimeSU == 3 && flStart == 0 && (liTimer_tick > t_TestSU))
  		   	{
					if (count_dn_Delta_G == -1)
					{
					   Delta_G_pr = 0;
					   nSU_pr = nSU;
						n_minSU = nSU;
	  					t_TestSU = liTimer_tick + 3*FREQ;
						count_dn_Delta_G++;
						Delta_G = Delta_G + 20;
					}
					else if (count_dn_Delta_G < 5)
					{
				      dn_Delta_G[count_dn_Delta_G] = nSU-nSU_pr;
				      dn_Delta_G[count_dn_Delta_G] = dn_Delta_G[count_dn_Delta_G]/20.;
				      if (dn_Delta_G[count_dn_Delta_G] < 4)
     					dn_Delta_G[count_dn_Delta_G] = 4;
			   	   Delta_G_pr = Delta_G;
		   	   	nSU_pr = nSU;
						if (Delta_G > 99)
							n_maxSU = nSU;
  						t_TestSU = liTimer_tick + 3*FREQ;
						count_dn_Delta_G++;
						Delta_G = Delta_G + 20;
					}
					else
					{
						RegimeSU = 0;
						Delta_G = 100;
					}
				}
			}	//if(flStopSU == 0)

			if (flStart)
			{
				//Отсутствие координат-------------------------------------------------------------------------
				if(liTimer_tick-liTimer_tick_GPS > 2*FREQ)
				{
					flNoKoord = 1;
					liTimer_tick_GPS = liTimer_tick;	
				}
					
				//Передача в КЗА-----------------------------------------------------------------------------
				if(flNoKoord)
				{
					WriteByteInKZA( 0x80 );
					WriteByteInKZA( 0x80 );
					WriteByteInKZA( 0x80 );
					WriteByteInKZA( 0xff );
				}
				else
				{
					if(flLatNew && flLonNew)
					{
						flLatNew = 0;
						flLonNew = 0;
						WriteByteInKZA (0x41);	

						WriteByteInKZA( (lLatFly & 0x007f) | 0x80 );
						WriteByteInKZA( ((lLatFly & 0x3f80) >> 7) | 0x80 );
						WriteByteInKZA( ((lLatFly & 0x1fc000) >> 14) | 0x80 );
						WriteByteInKZA( ((lLatFly & 0xfe00000) >> 21) | 0x80 );

						WriteByteInKZA( (lLonFly & 0x007f) | 0x80 );
						WriteByteInKZA( ((lLonFly & 0x3f80) >> 7) | 0x80 );
						WriteByteInKZA( ((lLonFly & 0x1fc000) >> 14) | 0x80 );
						WriteByteInKZA( ((lLonFly & 0xfe00000) >> 21) | 0x80 );
					}
					if(flKoorsNew)
					{
						flKoorsNew = 0;
						PackageInKZA2(0x44, (unsigned int)koors);	
					}
					if(flVzNew)
					{
						flVzNew = 0;
						PackageInKZA1(0x45, Vz);	
					}
					if (flNewH_zad)
					{
						PackageInKZA1(0x4d, (H_zad+1000)/50);
						flNewH_zad = 0;
					}
				}
				WriteByteInKZA (0x4f);	
				WriteByteInKZA( (liTimer_tick & 0x007f) | 0x80 );
				WriteByteInKZA( ((liTimer_tick & 0x3f80) >> 7) | 0x80 );
				WriteByteInKZA( ((liTimer_tick & 0x1fc000) >> 14) | 0x80 );
				WriteByteInKZA( ((liTimer_tick & 0xfe00000) >> 21) | 0x80 );

				PackageInKZA2(0x46, s_H_filtr+5000);				
				PackageInKZA1(0x47, V);

		 		PackageInKZA2(0x48, (Kren_dat+90)*10);
				PackageInKZA2(0x49, (s_Vy_filtr+40)*100);
				PackageInKZA2(0x4a, (Wx+100)*80);
				PackageInKZA2(0x42, (Wy+100)*80);
		  		PackageInKZA1(0x4b, Dat);	 //Потенциальные датчики
				PackageInKZA1(0x4e, Kren_zad+60);

				PackageInKZA2(0x50, (int_dV+30)*100);
	  			PackageInKZA2(0x51, (int_dKren+30)*100);	 
				PackageInKZA2(0x52, (Delta_V+30)*100);

				PackageInKZA2(0x53, (Delta_E+30)*100);
				PackageInKZA2(0x54, Vy_zad_ogr_nab*100);

				PackageInKZA2(0x55, nSU);
				PackageInKZA2(0x5c, (ax+80)*100);
				PackageInKZA1(0x5e, Delta_G);
				PackageInKZA1(0x5f, Delta_A);
				PackageInKZA2(0x61, int_delta_ax);
				PackageInKZA2(0x40+35, H_tmp+5000);
				PackageInKZA2(0x40+36, H+5000);
			}
		}	//if (flRun)
	}	//while (1)
	return;
}

//------------------------------------------------------------------------------
void PackageInModem1(unsigned char Data, char i)
{
	BufferInModem[i] = Data | 0x80;
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

//-----------------------------------------------------------------------------------------------
void PackageInKZA1(unsigned char CodByte, unsigned char Cod)
{
	WriteByteInKZA (CodByte);	
	WriteByteInKZA (Cod | 0x80);
	return;
}

//-----------------------------------------------------------------------------------------------
void PackageInKZA2(unsigned char CodByte, unsigned int Cod)
{
	WriteByteInKZA (CodByte);	
	WriteByteInKZA( (Cod & 0x007f) | 0x80 );
	WriteByteInKZA( ((Cod & 0x3f80) >> 7) | 0x80 );
	return;
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

//--------------------------------------------------------------------------
void OutModem(char NAnswer)
{
  	if (NAnswer == -1)//Посылка №6 при автономном переключениии - уведомление Земли
	{
		fln_ = 1;
		return;
	}

	while (flTransmiter)
		;			

	switch (NAnswer)
	{
	case 20:			//Посылка №0
		if (fln_)	//Посылка №6 при автономном переключениии n_ - уведомление Земли
		{
			fln_ = 0;
			rk = 3;
			BufferInModem[0] = 0x40 | 26 ;
		   BufferInModem[1] = n_ | 0x80;
	      BufferInModem[2] = BufferInModem[0]^BufferInModem[1] | 0x80;
		}
		else
		{
			rk = 27;
		}
		break;
	case 21:			//Посылка №1
 		rk = 27;
		break;
	case 22:				//Посылка №2 
		rk = 11;
		break;
	case 24:			//Посылка №4 
		rk = 3;
		break;
	case 25:			//Посылка №5 
		rk = 3;
		break;
	default:			
		rk = 0;
	}
	r0 = 0;
	flTransmiter = 1;
	return;
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
	XBR1 = 0x00;	// XBAR1: Initial Reset Value
	XBR2 = 0x4c;	// XBAR2: Initial Reset Value
// Select Pin I/0

// NOTE: Some peripheral I/O pins can function as either inputs or 
// outputs, depending on the configuration of the peripheral. By default,
// the configuration utility will configure these I/O pins as push-pull 
// outputs.
                    // Port configuration (1 = Push Pull Output)
    SFRPAGE = 0x0F;
    P0MDOUT = 0x75; // Output configuration for P0 
    P1MDOUT = 0x00; // Output configuration for P1 
    P2MDOUT = 0x18; // Output configuration for P2 
    P3MDOUT = 0x3F; // Output configuration for P3 

    P1MDIN = 0xFF;  // Input configuration for P1
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
//	char SFRPAGE_SAVE = SFRPAGE;

	SFRPAGE = 0x00;

	TMR2CF = 0x08;  // Timer 2 Configuration
//----------------------------для спутника
//   RCAP2L = 0x5e;  // Timer 2 Reload Register Low Byte
//   RCAP2H = 0xff;  // Timer 2 Reload Register High Byte
//----------------------------

   RCAP2L = 0xBC;  // Timer 2 Reload Register Low Byte
   RCAP2H = 0xFE;  // Timer 2 Reload Register High Byte

   TMR2L = 0x00;   // Timer 2 Low Byte	
   TMR2H = 0x00;   // Timer 2 High Byte	
   TMR2CN = 0x04;  // Timer 2 CONTROL
	TR2 = 1;	
	SFRPAGE = UART0_PAGE;
	
	SCON0 = 0x50;
	SSTA0 = 0x15;

//	SFRPAGE = SFRPAGE_SAVE;
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
		if(wBFM > NBFM_1)
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
		if(w > NS_1)
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
//	static xdata unsigned char i_non_nSU;
  	xdata unsigned int Count;
	xdata unsigned char j;
	xdata float tmp;
	xdata char SFRPAGE_SAVE = SFRPAGE;
	SFRPAGE = 0;

	if (SteckPoint < SP)
		SteckPoint = SP;	
//   WDTCN = 0xA5;	//Перезапустить охранный таймер
	
	
	//Обороты двигателя---------------------------------------------------
	SFRPAGE = 2;
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
/*	if (nSU < 10 && flStart)
	{
		i_non_nSU++;
 	   if (i_non_nSU > 3)
		{
			flStopSU = 1;
			bitStopSU = flStopSU;
			H_zad = 0;
			H_max = H_zad;	
			flNewH_zad = 1;
		}
	}
	else
		i_non_nSU = 0;
*/
	//-------------------------------------------------------------------------
	flRun = 1;
   timer1_tick1++;
	liTimer_tick++;

   //Опрос датчиков----------------------------------------------------------------------
/*	H_tmp = 0;			//bort N1
	SFRPAGE = 0x00;

	for(j = 0; j < 5; j++)
	{
		AMX0SL = 0x00;	//Выбор канала
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
	H_tmp = 10795.9-4.1594*H_tmp+0.0002888*H_tmp*H_tmp-H0;	
/*	if (flSet_H0)
	{
		tmp = H_tmp-s_H_filtr;
		if (tmp > 30)
		{
		   dettaH = dettaH+(tmp-dettaH)/5.;
		}
		else if (tmp < -30)
		{
		   dettaH2 = dettaH2+(tmp-dettaH2)/5.;
		}
	   H = H+(H_tmp+dettaH+dettaH2-H)/FREQ/0.3;
		tmp =  (H-H_pr)*FREQ;
   	H_pr = H;
	 	Vy_k = Vy_k+(tmp-Vy_k)/FREQ/0.3;
	}*/
		
/*   H_filtr = H_filtr+(H_tmp-H_filtr)/FREQ/0.3;
   s_H_filtr = s_H_filtr+(H_tmp-s_H_filtr)/FREQ;

	tmp =  (H_filtr-H_filtr_pr)*FREQ;
   H_filtr_pr = H_filtr;
 	Vy = Vy+(tmp-Vy)/FREQ/0.3;

	tmp =  (s_H_filtr-s_H_filtr_pr)*FREQ;
   s_H_filtr_pr = s_H_filtr;
 	s_Vy_filtr = s_Vy_filtr+(tmp-s_Vy_filtr)/FREQ;

	AMX0SL = 0x03;	 		//Wx		
	for (Count = 0; Count < 10; Count++)
		;
  	AD0INT = 0;
	AD0BUSY = 1;
  	AD0INT = 0;
	while(AD0BUSY)
		;
	Wx = ADC0 & 0x0fff;
	Wx = -623.17 + 0.308*Wx - 0.000000276*Wx*Wx;
//	Wx = -623.47 + 0.308*Wx - 0.000000276*Wx*Wx;

	AMX0SL = 0x04;	 	//Wy	
	for (Count = 0; Count < 10; Count++)
		;
  	AD0INT = 0;
	AD0BUSY = 1;
  	AD0INT = 0;
	while(AD0BUSY)
		;
	Wy = ADC0 & 0x0fff;
//	Wy = -767.35 + 0.3644*Wy + 0.000006446*Wy*Wy;
	Wy = -767.5 + 0.3644*Wy + 0.000006446*Wy*Wy;

	AMX0SL = 0x02;	 	//Скорость	
	for (Count = 0; Count < 10; Count++)
		;
  	AD0INT = 0;
	AD0BUSY = 1;
  	AD0INT = 0;
	while(AD0BUSY)
		;
	iV = ADC0 & 0x0fff;

*/
   //Опрос датчиков----------------------------------------------------------------------
	H_tmp = 0;			//bort N2
	SFRPAGE = 0x00;
	for(j = 0; j < 5; j++)
	{
		AMX0SL = 0x00;	//Выбор канала
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
//------------

	if (i_his_H == -1)
	{
		if (flTransmiter == 1 && flTransmiter_pr ==0)
			flKorH = 1;
	}	
	if (flKorH)
	{
		if (flTransmiter == 1 && flTransmiter_pr ==0)
		{
			his_H0 = s_H_filtr;
			i_his_H = 0;
		}
		if (i_his_H < HH)
		{
			tmp = H_tmp - his_H0 + (s_H_filtr-his_H0);
		   his_H[i_his_H] = his_H[i_his_H]+(tmp-his_H[i_his_H])/10;
		   H = H_tmp+his_H[i_his_H];
		   H_filtr = H_filtr+(H-H_filtr)/FREQ/0.3;
		}		
	}
	else
	   H_filtr = H_filtr+(H_tmp-H_filtr)/FREQ/0.3;

	tmp =  (H_filtr-H_filtr_pr)*FREQ;
   H_filtr_pr = H_filtr;
 	Vy = Vy+(tmp-Vy)/FREQ/0.3;

	//Wx		
	AMX0SL = 0x03;	 	
	for (Count = 0; Count < 10; Count++)
		;
  	AD0INT = 0;
	AD0BUSY = 1;
  	AD0INT = 0;
	while(AD0BUSY)
		;
	Wx = ADC0 & 0x0fff;
//	Wx = -561.66 + 0.25455*Wx + 0.000010395*Wx*Wx;
	Wx = -558.44 + 0.25455*Wx + 0.000010395*Wx*Wx;

	AMX0SL = 0x04;	 	//Wy	
	for (Count = 0; Count < 10; Count++)
		;
  	AD0INT = 0;
	AD0BUSY = 1;
  	AD0INT = 0;
	while(AD0BUSY)
		;
	Wy = ADC0 & 0x0fff;
//	Wy = -754.51 + 0.3486*Wy + 0.000011427*Wy*Wy;
	Wy = -755.54 + 0.3486*Wy + 0.000011427*Wy*Wy;

	AMX0SL = 0x02;	 	//Скорость	
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

		if (bitStart == 0 && flStart == 0) //если стоим на катапульте
		{
			H_max = 0;
			i_Vzlet = 0;
     		H_zad = 300;
       	Kren_zad = 0;
//			flStart = 0;
		}
  		else if (i_Vzlet < 30*FREQ)  //взлетный режим Time = 30 c
		{
       	Kren_zad = 0;
			flStart = 1;
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
//			int_dV = -5.5;	//для борта №1 
			int_dV = -4.5;	//для борта №2 

		Delta_V = 0.3*tmp + int_dV;
		if (Delta_V > 14)  Delta_V = 14;
		else if (Delta_V < -23) Delta_V = -23;

  		//боковой канал
		if (flStart == 0) //если стоим на катапульте
  		 	Kren_dat=0;
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

	 	//управление камерой----------------------------------------------------
		Delta_UgolKam = UgolKam_zad;
		Delta_KrenKam = -0.8*Kren_dat+KrenKam_zad; 

	 	//оценим текущую ситуацию-----------------------------------------------
		if (flStart == 0)        //Стоим на катапульте
		{
			flStopSU = 0;
			flParashut = 0;
			flAvarStop = 0;
			NoCommand = 0;
		}
		else 
		{
			NoCommand++;
			if(flAvarStop == 1)
			{
				if (timer1_tick1 < FREQ*2)
				{
					flStopSU = 1;
					bitStopSU = flStopSU;
     				H_zad = 0;  
					if (timer1_tick1 < 2)
						flNewH_zad = 1;
				}
			   else if (timer1_tick1 > 30*FREQ)  //остановим машинку ввода ПС
	   			bitParashut = 0;
				else if (timer1_tick1 > 2*FREQ)
				{
					flParashut = 1;
	   			bitParashut = flParashut;
				}
			}
			else 
			{
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
					flAvarStop = 1;         
					timer1_tick1 = 0;
					Kren_zad = 0;
				}
			}    
      }
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
	PCA0CPL2 = Count & 0x00ff;		    // PCA Counter/Timer Low Byte
	PCA0CPH2 = (Count & 0xff00) >> 8;  // PCA Counter/Timer High Byte		

	if (Delta_KrenKam < -45)
		Delta_KrenKam = -45;
	else if (Delta_KrenKam > 45)
		Delta_KrenKam = 45;
	Count = 60493+(Delta_KrenKam+45)*1659/45;
	PCA0CPL3 = Count & 0x00ff;		    // PCA Counter/Timer Low Byte
	PCA0CPH3 = (Count & 0xff00) >> 8;  // PCA Counter/Timer High Byte		

	if (flStart && flNabor)	//tmp = Delta_G
		tmp = 100;
	else
		tmp = Delta_G;
	if (tmp > 100)
		tmp = 100;
	else if (tmp < 0)
		tmp = 0;
	Count = 60493+(tmp-50)/50*1659*0.8;//Борт №2
//	Count = 60493+(tmp-50)/50*1659*0.7;//Борт №1
	PCA0CPL5 = Count & 0x00ff;		    // PCA Counter/Timer Low Byte
	PCA0CPH5 = (Count & 0xff00) >> 8;  // PCA Counter/Timer High Byte		
	
	if (Delta_A > 100)
		Delta_A = 100;
	else if (Delta_A < 0)
		Delta_A = 0;
	if (flStart)
		Delta_A = 100;
	Count = 60493+(Delta_A-50)/50*0.7*1659;//Борт №2
//	Count = 60493+(Delta_A-50)/50*0.55*1659;//Борт №1
	PCA0CPL4 = Count & 0x00ff;		    // PCA Counter/Timer Low Byte
	PCA0CPH4 = (Count & 0xff00) >> 8;  // PCA Counter/Timer High Byte		


	flTransmiter_pr = flTransmiter;

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
        	led = !led;
        	j=0;
      } 

    	while (!RDY)  								// Loop untill flash is busy      
			;
   }
// 	for (k=0; k<300000; k++)
//		;
	led = 0;
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

	if (nByte<527) 
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
	
		if (nPage < 8191) 
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

