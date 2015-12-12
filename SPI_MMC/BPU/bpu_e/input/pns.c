// Autopilot program for Electroplane
// Written by Sergey V. Pobezhimov

//-----------------------------------------------------------------------------
// Includes
//-----------------------------------------------------------------------------

#include <C:\Keil\C51\INC\Cygnal\C8051F120.h>		// Register definition file.
#include <C:\Keil\C51\INC\stdio.h>
#include <C:\Keil\C51\INC\stdlib.h>
#include <C:\Keil\C51\INC\string.h>
#include <C:\Keil\C51\INC\math.h>

//-----------------------------------------------------------------------------
// 16-bit SFR Definitions for 'F12x
//-----------------------------------------------------------------------------

sfr16 ADC  = 0xbe;                 					// ADC data
sfr16 DAC  = 0xd2;                 					// DAC data
sfr16 TMR4 = 0xCC;									// Timer 4 data
sfr16 PCA0 = 0xF9;				   					// PCA0 counter

//-----------------------------------------------------------------------------
// Global CONSTANTS
//-----------------------------------------------------------------------------

#define Version 1.1
#define dt 0.0197
#define H_alarm 30
#define H_set_alarm 50
#define Pi 3.141592

sbit LED = P3^0;                        			// LED='1' means ON
sbit STRT_SW = P2^7;                       			// STRT_SW='1' means on FLIGHT
sbit PAR = P2^6;									// PAR='1' means parachute goes ON
sbit S3 = P2^5;										// S3='0' means 12V S3P out
sbit S2 = P2^4;										// S2='0' means 12V S2P out
sbit S1 = P2^3;										// S1='0' means 12V S1P out
sbit CS = P3^1;										// CS='0' means chip selected
sbit RDY = P3^2;									// RDY='1' means chip is ready

//-----------------------------------------------------------------------------
// Function PROTOTYPES
//-----------------------------------------------------------------------------

void config (void);
void UART0_ISR (void);  							// Modem
void UART1_ISR (void);  							// GPS
void PCA0_ISR (void);
void SET_H0 (void);
void PWM (void);
void Test (void);
void Service(void);
void Erase_flash(void);
void Read_flash(void);
void Sens_test(void);
void Write_byte(unsigned char byte);
void GPS_Access (void);
void Delay(void);
void auto_control(void);
bit  GPS_tlm(void);
float diap_Pi(float angle);
float napr_vector (float x1, float y1, float x2, float y2);
int fsgn(float val);

//-----------------------------------------------------------------------------
// Global VARIABLES
//-----------------------------------------------------------------------------

data unsigned int U,T;
data long H;
data float W,H_filt;

unsigned char Mess_OUT[32],Mess_GPS[80],STAT=0;

unsigned char TRT=0,Command=0x81;

bdata bit GPSRDY=0,SENSRDY=0,CH_bit=0,Stop_bit=1,P_bit=0,R_bit=0,St_bit,TM_bit,RDY_bit=0;

float Delta_left,Delta_right,Cam_LR;

unsigned int RC_fault=0;

int Bank_auto=0,tmp;

char Cam_UD=0;

float coord_X, coord_Z,  coord_X0, coord_Z0, ground_speed, course;
float Z_izm, X_izm, koef_dolg=1;

//-----------------------------------------------------------------------------
// MAIN Routine
//-----------------------------------------------------------------------------

void main (void)
{
float Vy_predv=0, dVy, Delta_V, Delta_E, Vy_zad=0, int_dVy=0,Kren_dat=0;
float Vy=0, dKren=0, int_dKren=0;					
unsigned int Fl_cnt=0;
unsigned char Par_delay=0,i=0;
char Bank_given=0,Cam_step=0;
bit H_trigger=0;
int H_zad;

 PAR=0;

 LED=0;

 S1=0;												// Modem On

 config();

 TI0=1;												// Need to use 'printf'

 Test();											// Rudders test

 Service();									

 SET_H0();  										// Zero altitude setting

 LED=1;

 S3=0;												// Onboard LED On	

 S2=0;												// TV On

 SFRPAGE=0x00;

 RI0=0;
 TI0=0;												// 'printf' can't be used in Main cycle
 
 IE = 0x90; 										// Enable interrupts and set UART0 interrupt


while (1)											// Main cycle
    {
     while (!SENSRDY) PCON |=0x01;					// Idle core untill Interrupt

// Vy calculations

     Vy_predv=(H-H_filt)/0.3;		    			// LPF T=0.3
	 H_filt=H_filt+Vy_predv*dt;

	 if(Vy_predv>10) Vy_predv=10;
	 if(Vy_predv<-10) Vy_predv=-10;

	 Vy=Vy+(Vy_predv-Vy)*dt/0.3;					// LPF T=0.3

// U (Voltage) calculations

     U=3.58+U*0.1232;								// Calculating voltage

//-----------------------------------------------------------------------------
// Automatic

	 if(STRT_SW) 									// If on the Air
	  {
	   St_bit=1;									// Bit 4 (START) of stat byte = 1

	   if(!Stop_bit)								// If engine is ready
		{
		 if (Fl_cnt<6000) Fl_cnt++;					// Increment flight counter
		 if (RC_fault<200) RC_fault++;  			// Increment Radio Channel fault counter If RC fault
		 else CH_bit=1;								// less than 2s, else RC fault bit (0) of STAT byte = 1 
		}
	  }
     
     else
		{
		 Fl_cnt=0;									// Reset Flight counter 
		 St_bit=0;									// Bit 4 (START) of STAT byte = 0

		 if(Stop_bit&&!P_bit)
		  {
		   PCA0CPM4 = 0xC2;	    					// Enable PCM on motor
		   Stop_bit=0;
		  } 
		}

	 if (CH_bit) Stop_bit=1;						// If RC fault, stop bit (1) of STAT byte = 1
      
//-----------------------------------------------------------------------------
// RC Commands

	 switch (Command&0x0F)
		  {
		   case 1 : H_zad=100;   break;
		   case 2 : H_zad=150;   break;
		   case 3 : H_zad=200;   break;
		   case 4 : H_zad=250;   break;
		   case 5 : H_zad=300;   break;
		   case 6 : Cam_step=1;  break;
		   case 7 : Cam_step=-1; break;
		   case 8 : Cam_step=0;  break;
		   case 15 : Stop_bit=1; break;
		  }

//-----------------------------------------------------------------------------
// Auto tracking

	 if (GPSRDY)
	  {
	   GPSRDY=0;

	   if (GPS_tlm ())
        {
         if (RDY_bit)								// If starting position is already set
          {
           Z_izm=koef_dolg*(coord_Z-coord_Z0);
		   X_izm=1.85*(coord_X-coord_X0);
           auto_control();
		   S3=!S3;
          }

         else										// Else setting starting coordinates
            {
			 coord_X0=coord_X; 						
			 coord_Z0=coord_Z;

			 koef_dolg=1.85*cos(coord_X0/3438);
			 i++;
			 if (i>9) RDY_bit=1;					// If position defined, bit 6 of STAT = 1
            }
        }
	  }

// Bank commands

	 tmp=Command&0xF0;

	 if (tmp)										// If high nibble of Command >0
	  {
       Bank_given=(tmp>>4)*6-48;					// High nibble of Command = Bank_given
	   R_bit=0;										// Reset return bit (bit3) of STAT
	  }

	 else
		{
		 Bank_given=Bank_auto;						
		 R_bit=1;									// Set return bit (bit3)
		} 

// H Alarm

	 if (H_filt>H_set_alarm) H_trigger=1;			// If H < H_alarm and H already been more than
	 if ((H_filt<H_alarm)&&H_trigger) Stop_bit=1;	// H_set_alarm, stop the engine (stop bit = 1)

// Launch mode

	 if (Fl_cnt<250) Bank_given=0;

// Parachute control
	  							
	 if (Stop_bit&&(Fl_cnt>0))	 					// If engine stop during flight
	  {
	   H_zad=0;
	   Cam_step=0;
	   if(Par_delay<250)							// If parashute still is't open
		{
		 Bank_given=0;
		 Par_delay++;		

		 if (Par_delay==50)							// Open parashute in 1 s after the engine stop
		  {
		   P_bit=1;									// Set Parashute bit (bit2) = 1
		   PAR=1;									// Enable power to parachute
		   Cam_UD=100;								// Set parachute lock to opened position
		  }
		
	     if (Par_delay==150) Cam_UD=0;				// Set parachute lock to closed position
        }
	   else PAR=0;									// Power to lock disabled in 4 s
	  }
 

// Engine control

	 if (Fl_cnt<50 || Stop_bit) TRT=0;				// Engine stop if Stop bit or on the Ground
 	 else if (Fl_cnt<500) TRT=10;
		  else TRT=9;

	 if (TRT==10) TM_bit=1;							// If Trottle maximum and engine PCM enabled, bit5 (TM) of STAT = 1
	 else TM_bit=0;									// Else bit5 (TM) of STAT = 0

// Calculating STAT byte

	 STAT=0;
	 STAT=STAT|RDY_bit;
	 STAT=(STAT<<1)|TM_bit;
	 STAT=(STAT<<1)|St_bit;
	 STAT=(STAT<<1)|R_bit;
	 STAT=(STAT<<1)|P_bit;
	 STAT=(STAT<<1)|Stop_bit;
	 STAT=(STAT<<1)|CH_bit;

//-----------------------------------------------------------------------------
// Flight Control Laws 

// Longitudal channel

	 Vy_zad=-0.1*(H_filt - H_zad);
     if (Vy_zad>2) Vy_zad=2;
     if (Vy_zad<-1.5) Vy_zad=-1.5;

 	 dVy=Vy - Vy_zad;
 	 if (dVy>15)  dVy=15; 
 	 if (dVy<-15) dVy=-15; 

 	 int_dVy=int_dVy + 0.4*dVy*dt;
 	 if (int_dVy>20)  int_dVy=20; 
 	 if (int_dVy<-20) int_dVy=-20; 

 	 if (Fl_cnt<2) int_dVy=-13;						// If On the Ground

 	 Delta_V=dVy + int_dVy;
 	 if (Delta_V>20)  Delta_V=20;
 	 if (Delta_V<-20) Delta_V=-20;

// Lateral channel

  	 Kren_dat=Kren_dat + (1.5*W - Kren_dat/5)*dt;

	 if (Fl_cnt<2) Kren_dat=0;						// If on the Ground      

 	 dKren=Kren_dat - Bank_given;

 	 int_dKren=int_dKren + 0.025*dKren*dt;
 	 if (int_dKren>5)  int_dKren=5;
 	 if (int_dKren<-5) int_dKren=-5;

	 if (Fl_cnt<2) int_dKren=0;						// If on the Ground

 	 Delta_E=0.3*dKren + int_dKren;
 	 if (Delta_E>10)  Delta_E=10;
     if (Delta_E<-10) Delta_E=-10;

  	 Delta_right=(Delta_V+Delta_E);
	 Delta_left=(Delta_V-Delta_E);

//-----------------------------------------------------------------------------
// Camera control

	 Cam_LR=0.8*Kren_dat;
	 if (Cam_LR>30) Cam_LR=30;
	 if (Cam_LR<-30) Cam_LR=-30;

	 Cam_UD+=Cam_step;
	 if(Cam_UD>100) Cam_UD=100;
	 if(Cam_UD<0) Cam_UD=0;

//-----------------------------------------------------------------------------
   	 PWM();

	 Write_byte(STAT);	 							// Writing stat byte				#1 

	 tmp=H*10;
	 Write_byte((tmp&0xFF00)>>8);					// Writing high byte H				#2
	 Write_byte(tmp&0x00FF);						// Writing low byte H				#3

	 Write_byte(Vy*10);								// Writing Vy*10					#4

	 tmp=W*10;
	 Write_byte((tmp&0xFF00)>>8);					// Writing high byte W*10			#5
	 Write_byte(tmp&0x00FF);						// Writing low byte W*10			#6

	 Write_byte(int_dKren*10);						// Writing int_dKren				#7

	 tmp=Kren_dat*10;
	 Write_byte((tmp&0xFF00)>>8);					// Writing high byte Kren_dat*10	#8
	 Write_byte(tmp&0x00FF);						// Writing low byte Kren_dat*10		#9	 

	 tmp=Delta_V*10;
	 Write_byte((tmp&0xFF00)>>8);					// Writing high byte Delta_V*10		#10
	 Write_byte(tmp&0x00FF);						// Writing low byte Delta_V*10		#11

	 Write_byte(Delta_E*10);						// Writing Delta_V*10				#12

	 tmp=int_dVy;
	 Write_byte((tmp&0xFF00)>>8);					// Writing high byte int_dVy_V*10	#13
	 Write_byte(tmp&0x00FF);						// Writing low byte int_dVy*10		#14

	 Write_byte(Command);							// Writing Command					#15

	 Write_byte(U);									// Writing Voltage					#16

	 SENSRDY=0;										// Means Cycle ended
    }    											// End Main Cycle
}                        							// END MAIN       

//-----------------------------------------------------------------------------
// UART0 Interrupt Service Routine
//-----------------------------------------------------------------------------

void UART0_ISR (void) interrupt 4
   {
	static unsigned char i;
	
	if (TI0)
	 {
	  TI0=0;
	  if(i<32) SBUF0=Mess_OUT[i++];					// Transmit bytes on each TI interrupt
													// till the end of the buffer
	  else i=0;										// Reset buffer pointer if entire buffer
	 }												// transmitted

	if (RI0)
	 {
	  RI0=0;
	  Command=SBUF0;
	  RC_fault=0;
	 }
   }

//-----------------------------------------------------------------------------
// UART1 Interrupt Service Routine
//-----------------------------------------------------------------------------

void UART1_ISR (void) interrupt 20
   {
	static unsigned char i=0;
	idata unsigned char BUF;

    if (RI0)
     {
      BUF=SBUF1;
      RI0=0;

      if(BUF=='$')
       {
        GPSRDY=0;
        i=0;
       }         

	  Mess_GPS[i++]=BUF; 

	  if(BUF==0x0A) GPSRDY=1;
     }
	else TI0=0;
   }                     							//END ISR

//-----------------------------------------------------------------------------
// Setting H0 Routine
//-----------------------------------------------------------------------------

void SET_H0 (void)
   {
	unsigned int i,d0,d1=0;
	int n;

 	printf ("\nSetting zero altitude \n");

    do 
     {
      if (d1>0) d1--; else d1=4095;    				// DAC1 forms offset voltage

      SFRPAGE = 0x01;
      DAC=d1;
      DAC=d1;

//       d0=1376+0.1288595*d1-0.0000617276*d1*d1;
	  d0=2850;


      SFRPAGE = 0x00;
      DAC=d0;                           			// DAC0 forms reference voltage for
      DAC=d0;                           			// ADC0, which depends upon altitude

       AD0INT=0;
       AD0BUSY=1;                       			// Start ADC0 conversion
       AD0INT=0;
                       
         while (AD0BUSY);

       H_filt=-ADC;                 

        if (H_filt<1000)               				// When ADC0 enters the band needed 
         {
          for(i=0; i<1000; i++)          			// Start averaging procedure on each
    	    {                            			// DAC1 tick. 
             AD0INT=0;
             AD0BUSY=1;                 			// Start ADC0 conversion
             AD0INT=0;

               while (AD0BUSY);

			 n=-ADC;
             H_filt+=(n-H_filt)*0.001;				// Low-pass filtering
            }
         }
     }
     while (H_filt>100);
													// H_shift=2047/6-H_filt/6
     H_filt=0;
   }

//-----------------------------------------------------------------------------
// PCA Interrupt Service Routine
//-----------------------------------------------------------------------------

void PCA_ISR (void) interrupt 9
   {
    data int i,n;

    CF=0; 											// reset interrupt flag

    SFRPAGE = 0x02;						
    U=TMR4;											// reading U (Supply voltage)
    TMR4=0;

    SFRPAGE = 0x00;

// H (altitude) reading

    H=0;

    for (i=0; i<20; i++)
	  { 
       AD0INT=0;
       AD0BUSY=1;
       AD0INT=0;

         while (AD0BUSY);

        n=ADC;
        H+=n;
      }

// W reading

	  REF0CN = 0x07;								// Select Vref as reference of ADC0
	  AMX0SL = 0x02;								// Select W sensor
      ADC0CF = 0x50;								// Select PGA gain = 1

      H=-H/120;										// Calculating H while settling ADC0
													// H=H_shift-H/20/6
    W=0;

    for (i=0; i<20; i++)
	  { 
       AD0INT=0;
       AD0BUSY=1;
       AD0INT=0;

         while (AD0BUSY);

        n=ADC;
        W+=n;
      }
         
// T reading

	  AMX0SL = 0x08;								// Select T sensor
      ADC0CF = 0x51;								// Select PGA gain = 2

      W=W*0.00291-121.9;							// Calculating W while settling ADC0

       AD0INT=0;
       AD0BUSY=1;
       AD0INT=0;

         while (AD0BUSY);

      T=ADC;
        
// Prepare to reading H on next cycle 

      REF0CN = 0x17;								// Select DAC0 as reference of ADC0
	  AMX0SL = 0x06;								// Select H sensor
	  ADC0CF = 0x54;								// Select PGA gain=16	     

    SENSRDY=1;										// Means Sensors Data is Ready

   }												// End ISR  

//-----------------------------------------------------------------------------
// PWM Routine
//-----------------------------------------------------------------------------

void PWM(void)
   {
    unsigned int n;

    SFRPAGE = 0x00;

    //Module 0 Left rudder

    n=60558-55.3*Delta_left;						// PCA0CP0=60558 +- 1659 
 	
    PCA0CPL0 = n&0xFF;    							// PCA Counter/Timer Low Byte
    PCA0CPH0 = (n >> 8) & 0xFF ;    				// PCA Counter/Timer High Byte

    //Module 1 Right rudder

    n=60558+55.3*Delta_right;						// PCA0CP1=60558 +- 1659
 	
    PCA0CPL1 = n&0xFF;    							// PCA Counter/Timer Low Byte
    PCA0CPH1 = (n >> 8) & 0xFF ;  	    			// PCA Counter/Timer High Byte

    //Module 2 Camera LR

    n=60558+55.3*Cam_LR;							// PCA0CP2=60558 +- 1659
 	
    PCA0CPL2 = n&0xFF;    							// PCA Counter/Timer Low Byte
    PCA0CPH2 = (n >> 8) & 0xFF ;  					// PCA Counter/Timer High Byte

    //Module 3 Camera UD
  
    n=62217-33.18*Cam_UD;							// PCA0CP3=62217 - 3318
 	
    PCA0CPL3 = n&0xFF;    							// PCA Counter/Timer Low Byte
    PCA0CPH3 = (n >> 8) & 0xFF ;    				// PCA Counter/Timer High Byte

    //Module 4 Engine throttle			

    n=62217-331.8*TRT;								// PCA0CP4=62217 - 3318
 	
    PCA0CPL4 = n&0xFF;    							// PCA Counter/Timer Low Byte
    PCA0CPH4 = (n >> 8) & 0xFF ;    				// PCA Counter/Timer High Byte		
   }

//------------------------------------------------------------------------------------
// Rudders Test Routine
//------------------------------------------------------------------------------------

void Test (void)
   {
 	Delta_left=-30;
	Delta_right=30;
 	PWM();
 	Delay();

 	Delta_left=30;
 	Delta_right=-30;
 	PWM(); 
 	Delay();

 	Delta_left=0;
 	Delta_right=0;
 	PWM();
 	Delay();

 	Delta_left=30;
 	Delta_right=30;
 	PWM();
	Delay();

 	Delta_left=-30;
 	Delta_right=-30;
 	PWM(); 
 	Delay();

 	Delta_left=0;
 	Delta_right=0;
 	PWM();
	Delay();

 	printf ("\n$ SW Version %1.1f\n",Version);

   } // End test

//------------------------------------------------------------------------------------
// Service Routine
//------------------------------------------------------------------------------------

void Service(void)
   {
	unsigned long j;
	static unsigned char n=0;

    SFRPAGE = 0x00;

	for (j=0; j<1000000; j++)			
	  {
	   if (RI0)
		{
		 if (SBUF0==115) n++;						// Increment n if received byte 's'
		 else n=0;
		 RI0=0;
		}
	  }

	if (n>4) printf ("\nEntering Service");



	while (n>4)
	 	{
      	 LED=1;
	   	 Delay();
     	 printf ("\n\n'g'-GPS Access\n'r'-Read\n'e'-Erase\n's'-S.Test\n'q'-Quit\n");
	 	 RI0=0;
	 	 while(!RI0);

	 	 switch(SBUF0)
			  {
			   case 103 : GPS_Access(); break;		// Received byte 'g'
			   case 114 : Read_flash(); break;		// Received byte 'r'
			   case 101 : Erase_flash();break;		// Received byte 'e'
			   case 115 : Sens_test();	break;		// Received byte 's'
			   case 113 : n=0;			break;		// Received byte 'q' - Quit
			  }
	 	}
   } // End Service

//------------------------------------------------------------------------------------
// GPS Access routine
//------------------------------------------------------------------------------------

void GPS_Access (void)
   {
	unsigned long j=0;
	unsigned char n=0;

	printf ("\nGPS Access\n");
	SFRPAGE = 0x00;
	RI0=0;
	TI0=1;

	SFRPAGE = 0x01;
	RI0=0;
	TI0=1;

	while(j<10)
		{
		 SFRPAGE = 0x00;
		 if(RI0)
		  {
		   n=SBUF0;
		   RI0=0;
    	   SFRPAGE = 0x01;
		   while(!TI0);
		   TI0=0;
		   SBUF1=n;
		   if(n==113) j++; else j=0;				// Received byte 'q' - Quit GPS Access
		  }

		 SFRPAGE = 0x01;
		 if(RI0)
		  {
		   n=SBUF1;
		   RI0=0;
    	   SFRPAGE = 0x00;
		   while(!TI0);
		   TI0=0;
		   SBUF0=n;
		  }
		}
    SFRPAGE = 0x00;
	TI0=1;
	printf ("\nEnd GPS Access\n");
   }

//------------------------------------------------------------------------------------
// Flash Reading routine
//------------------------------------------------------------------------------------

void Read_flash(void)
   {
	int Page_Adr;
	unsigned char i=0,j=0;

	SFRPAGE = 0x00;

	RI0=0;
	TI0=1;
	printf ("\nFlash Reading Ready\n");

 	for (Page_Adr=0;Page_Adr<8192;Page_Adr++)	 	  		
   	  {
       CS=0;            							// Select flash

	   for(i=0;i<10;i++);							// Wait 250 nS

	   SPI0DAT=0x52;         						// Opcode for Main Memory Page Read
	   while(!SPIF);
	   SPIF=0;
	
       SPI0DAT=(Page_Adr & 0x1FC0) >> 6;			// High 7 bits of 13-bit Page Adress
	   while(!SPIF);
	   SPIF=0;
  
       SPI0DAT=((Page_Adr & 0x003F) <<2); 			// Low 6 bits of 13-bit Page Adress and 2
	   while(!SPIF);								// High bits of Starting byte Adress on Page
	   SPIF=0;


       SPI0DAT=0x00; 								// Low byte of Starting byte Adress
	   while(!SPIF);
	   SPIF=0;

       SPI0DAT=0xFF;  								// Don't care byte
	   while(!SPIF);
	   SPIF=0;

       SPI0DAT=0xFF;  								// Don't care byte
	   while(!SPIF);
	   SPIF=0;

       SPI0DAT=0xFF;  								// Don't care byte
	   while(!SPIF);
	   SPIF=0;

       SPI0DAT=0xFF;  								// Don't care byte
	   while(!SPIF);
	   SPIF=0;

       LED=!LED;

       for (i=0;i<33;i++)
         {
		  while(!RI0);
	      RI0=0;

		  for(j=0;j<16;j++)
			{
			 SPI0DAT=0xFF;
			 while(!SPIF);
			 SPIF=0;

			 while(!TI0);
			 TI0=0;

			 SBUF0=SPI0DAT;
			}
		 }
       CS=1;     
   	  }
    LED=0;       
   }

//------------------------------------------------------------------------------------
// Sensors test routine
//------------------------------------------------------------------------------------

void Sens_test(void)
   {
	int a,b,c,d;
	unsigned char n;

	SFRPAGE = 0x00;

	printf ("\nSensors Test\n");

	SET_H0();

	RI0=0;

 	IE = 0x80; 										// Enable interrupts without UART0 interrupt

	while(SBUF0!=113)								// Loop untill received byte 'q' - Quit
		{
     	 while (n<7) 
			 {
              PCON |=0x01;							// Idle core untill Interrupt
			  if (SENSRDY) n++;
			  SENSRDY=0;
			 }
		 n=0;

		 a=H;
		 b=W;
		 c=DAC;

		 SFRPAGE = 0x01;

		 d=DAC;

		 SFRPAGE = 0x00;

		 printf("$%04.0d %04.0d %04.0d %04.0d\n",a,b,c,d);
		}
 	IE = 0x00;
   }
//------------------------------------------------------------------------------------
// Flash Erasing routine
//------------------------------------------------------------------------------------

void Erase_flash(void)
   {
	int Block_Adr;
	unsigned char i,j;

	SFRPAGE = 0x00;

	RI0=0;
	TI0=1;
	printf ("\nErasing in Progress>\n");

	for (Block_Adr=0;Block_Adr<=1024;Block_Adr++)	 	  		
	  {
	   CS=0;		  								// Select flash

	   for(i=0;i<10;i++);							// Wait 250 nS

	   SPI0DAT=0x50;          						// орсоde for block erase

	   while(!SPIF);
	   SPIF=0;
       SPI0DAT=(Block_Adr & 0x3F8) >> 3;			// High 7 bits of Block Adress

	   while(!SPIF);
	   SPIF=0;
       SPI0DAT=(Block_Adr & 0x7) << 5;   			// Low 3 bits of Block Adress

	   while(!SPIF);
	   SPIF=0;
       SPI0DAT=0xFF;                				// Don't care byte

	   while(!SPIF);
	   SPIF=0;

       CS=1;        								
 
   		if (j<50) j++;
    	else
       	   {
        	LED=!LED;
        	j=0;
			printf ("*");
       	   } 

    	while (!RDY);  								// Loop untill flash is busy      
   	  }
	Delay();
	printf ("\nDone\n");
	LED=0;        
   }                 

//------------------------------------------------------------------------------------
// Write Byte to Flash Routine
//------------------------------------------------------------------------------------

void Write_byte(unsigned char byte)
   { 
	static int Buf_Pointer=0,Page_Adr=0;
	static bit buf_num=1;
	idata unsigned char i=0;

	SFRPAGE = 0x00;

	CS=0;               							// Select flash
	for(i=0;i<10;i++);								// Wait 250 nS

	if (buf_num) SPI0DAT=0x87;      				// Opcode for Buffer 2 Write
	else SPI0DAT=0x84;             					// Opcode for Buffer 1 Write

	while(!SPIF);
	SPIF=0;

	SPI0DAT=0xFF;		        					// Don't care byte

	while(!SPIF);
	SPIF=0;
	SPI0DAT=(Buf_Pointer & 0x0300) >> 8; 			// 2 High bits of starting byte Adress

	while(!SPIF);                                         
	SPIF=0;
	SPI0DAT=Buf_Pointer & 0x00FF;           		// Low byte of starting byte Adress

	while(!SPIF);
	SPIF=0;
	SPI0DAT=byte;

	while(!SPIF);

	for(i=0;i<10;i++);								// Wait 250 nS

	CS=1;              								// Select flash

	if (Buf_Pointer<527) Buf_Pointer++;
	else 
	   {
		while (!RDY);          						// Loop if previous Page still writing

		CS=0;		  
		for(i=0;i<10;i++);							// Wait 250 nS

		if (buf_num) SPI0DAT=0x86;      			// Oрсоde for Buffer 2 writing
		else SPI0DAT=0x83;             				// Oрсоde for Buffer 1 writing

		while(!SPIF);
		SPIF=0;
		SPI0DAT=(Page_Adr & 0x1FC0) >> 6;			// High 7 bits of 13-bit Page Adress

		while(!SPIF);
		SPIF=0;
		SPI0DAT=(Page_Adr & 0x003F) << 2;   		// Low 6 bits of 13-bit page adress
		while(!SPIF);
		SPIF=0;
		SPI0DAT=0xFF;                				// Don't care byte

		while(!SPIF);
		SPIF=0;

		for(i=0;i<10;i++);							// Wait 250 nS 
		CS=1;        

		buf_num=!buf_num;

		Buf_Pointer=0;

		if (Page_Adr<8191) Page_Adr++;
		else Page_Adr=0;
	   }
   }												// End Routine

//------------------------------------------------------------------------------------
// Delay Routine
//------------------------------------------------------------------------------------

void Delay(void)
   {
	unsigned long j;
 	for (j=0; j<300000; j++);
   }
 
//------------------------------------------------------------------------------------
// GPS & Telemetry Routine
//------------------------------------------------------------------------------------

bit GPS_tlm (void)
  {
   idata unsigned char zpt_pos[12]; 
   char i=0, j=0, k=0; // unsigned

//определ€ю позиции зап€тых
   for (i=0;i<=strlen(Mess_GPS);i++)
   if (Mess_GPS[i]==',')
			   zpt_pos[j++]=i;
   j=0;

	//если координаты не определены - выход
   if (Mess_GPS[zpt_pos[1]+1]!='A')
    {
	 	//заполн€ем буфер дл€ передачи на землю
		Mess_OUT[k++]='$';
		Mess_OUT[k++]='V';
		Mess_OUT[k++]='\r';
		Mess_OUT[k++]=(int)H_filt>>1;
		Mess_OUT[k++]=(unsigned char)U;
		Mess_OUT[k++]=STAT;
		Mess_OUT[k++]='\0';

		SFRPAGE=0x00;
		TI0=1;

		return 0;
    }

	//заполн€ем буфер дл€ передачи на землю
	Mess_OUT[k++]='$';

	//широта†
	for (j=zpt_pos[2]+1;j<zpt_pos[3];j++)
	{
		if (Mess_GPS[j]!='.')
			Mess_OUT[k++]=Mess_GPS[j];
	}


	//добиваем 0-и сзади до стандартного формата
	for(j=0;j<10-(zpt_pos[3]-zpt_pos[2]);j++)
		Mess_OUT[k++]='0';


	//долгота
	for (j=zpt_pos[4]+1;j<zpt_pos[5];j++)
	{
		if (Mess_GPS[j]!='.')
			Mess_OUT[k++]=Mess_GPS[j];
	}

	//добиваем 0-и сзади до стандартного формата
	for(j=0;j<11-(zpt_pos[5]-zpt_pos[4]);j++)
		Mess_OUT[k++]='0';


	//скорость

	//определ€ем позицию дес€тичной точки
	for (j=0;j<=(zpt_pos[7]-zpt_pos[6]);j++)
		if (Mess_GPS[zpt_pos[6]+j]=='.') break;

	//добиваем 0-и спереди до стандартного формата
	for(i=0;i<=3-j;i++)
//	for(i=0;i<=j-3;i++)
			Mess_OUT[k++]='0';

	//устанавливаем значащие символы до зап€той
	for(i=0;i<j-1;i++)
			Mess_OUT[k++]=Mess_GPS[zpt_pos[6]+1+i];

	//устанавливаем 1 знак после зап€той
	Mess_OUT[k++]=Mess_GPS[zpt_pos[6]+2+i];


	//курс

	//определ€ем позицию дес€тичной точки
	for (j=0;j<=(zpt_pos[8]-zpt_pos[7]);j++)
		if (Mess_GPS[zpt_pos[7]+j]=='.') break;

	//добиваем 0-и спереди до стандартного формата
	for(i=0;i<=3-j;i++)
//	for(i=0;i<=j-3;i++)
			Mess_OUT[k++]='0';

	//устанавливаем значащие символы до зап€той
	for(i=0;i<j-1;i++)
			Mess_OUT[k++]=Mess_GPS[zpt_pos[7]+1+i];

	//устанавливаем 1 знак после зап€той
	Mess_OUT[k++]=Mess_GPS[zpt_pos[7]+2+i];

	Mess_OUT[k++]='\r';




	Mess_OUT[k++]=(int)H_filt>>1;
	Mess_OUT[k++]=(unsigned char)U;
	Mess_OUT[k++]=STAT;
	Mess_OUT[k++]='\0';

	SFRPAGE=0x00;
	TI0=1;


	//ƒЋя Ѕќ–“ќ¬ќ… Ќј¬»√ј÷»»


	//X - градусы
	Mess_GPS[0]=Mess_GPS[zpt_pos[2]+1];
	Mess_GPS[1]=Mess_GPS[zpt_pos[2]+2];
	Mess_GPS[2]='\0';

	coord_X=60*atof(Mess_GPS);

	//X - минуты
	for (i=zpt_pos[2]+3;i<zpt_pos[3];i++)
		Mess_GPS[j++]=Mess_GPS[i];
	Mess_GPS[j++]='\0';
	j=0;

	coord_X=coord_X+atof(Mess_GPS);

	//Z - градусы
	Mess_GPS[0]=Mess_GPS[zpt_pos[4]+1];
	Mess_GPS[1]=Mess_GPS[zpt_pos[4]+2];
	Mess_GPS[2]=Mess_GPS[zpt_pos[4]+3];
	Mess_GPS[3]='\0';

	coord_Z=60*atof(Mess_GPS);

	//Z - минуты
	for (i=zpt_pos[4]+4;i<zpt_pos[5];i++)
		Mess_GPS[j++]=Mess_GPS[i];
	Mess_GPS[j++]='\0';
	j=0;

	coord_Z=coord_Z+atof(Mess_GPS);


	//скорость путева€
	for (i=zpt_pos[6]+1;i<zpt_pos[7];i++)
		Mess_GPS[j++]=Mess_GPS[i];
	Mess_GPS[j++]='\0';
	j=0;
	ground_speed=atof(Mess_GPS);

	//курс
	for (i=zpt_pos[7]+1;i<zpt_pos[8];i++)
		Mess_GPS[j++]=Mess_GPS[i];
	Mess_GPS[j++]='\0';

	course=atof(Mess_GPS);

	return 1;
}
 
//------------------------------------------------------------------------------------
// Auto Control Routine
//------------------------------------------------------------------------------------

void auto_control(void)
{

  idata float course_tek_r=0;
  
  idata float rasst_toch_mar=0,pr_rasst_toch_mar=0,angle;
  idata float napr_vetv_mar=0,napr_toch_mar=0,otkl_ot_mar=0,l_km=0.8;
  float z_toch_pricel=0,x_toch_pricel=0, napr_toch_pricel=0;
  static float Z_mem=0, X_mem=0; 

    if(!R_bit)
    {
        Z_mem=Z_izm;
        X_mem=X_izm;   
		  return;	 
    }

    course_tek_r=-course/57.3;

    rasst_toch_mar=sqrt(Z_izm*Z_izm+X_izm*X_izm);

    napr_vetv_mar=napr_vector(Z_mem, X_mem, 0, 0);

    napr_toch_mar=napr_vector(Z_izm, X_izm, 0, 0);

    angle=diap_Pi(napr_toch_mar-napr_vetv_mar);

    pr_rasst_toch_mar=rasst_toch_mar*cos(angle);

    otkl_ot_mar=rasst_toch_mar*sin(angle);

    l_km=(fabs(otkl_ot_mar)>0.2)?0.1:0.5;

    z_toch_pricel=(pr_rasst_toch_mar-l_km*fsgn(pr_rasst_toch_mar))*sin(napr_vetv_mar);
    x_toch_pricel=-(pr_rasst_toch_mar-l_km*fsgn(pr_rasst_toch_mar))*cos(napr_vetv_mar);

    napr_toch_pricel=napr_vector(Z_izm, X_izm, z_toch_pricel, x_toch_pricel);

    Bank_auto=57.3*diap_Pi(course_tek_r-napr_toch_pricel);
  
    if(Bank_auto>42) Bank_auto=42;
    if(Bank_auto<-42) Bank_auto=-42;  
}

//------------------------------------------------------------------------------------
// Angle Band Routine
//------------------------------------------------------------------------------------

float diap_Pi(float angle)
{
   int nkr;
   float ugolp;

   nkr=(angle>0)?floor(angle/(2*Pi)):ceil(angle/(2*Pi));

   ugolp=angle-nkr*2*Pi;
   angle=ugolp;

   if (ugolp>Pi) angle=ugolp-2*Pi;
   if (ugolp<=-Pi) angle=ugolp+2*Pi;

   return angle;

}

//------------------------------------------------------------------------------------
// Direction Vector Routine
//------------------------------------------------------------------------------------ 

float napr_vector (float x1, float y1, float x2, float y2)
	{
     if (y1==y2)
     return (x1<x2)?-Pi/2:Pi/2;
     return atan2(-x2+x1, y2-y1);
	}

//------------------------------------------------------------------------------------
// Signature Routine
//------------------------------------------------------------------------------------

int fsgn(float val)
  {
   if(val>0) return 1;
   if(val<0) return -1;
   if(val==0) return 0;
  }

//------------------------------------------------------------------------------------
// Config Routine
//------------------------------------------------------------------------------------

void config (void) {

//Local Variable Definitions
    int b = 0;
    int n = 0;

	

//----------------------------------------------------------------
// Watchdog Timer Configuration
//
// WDTCN.[7:0]: WDT Control
//   Writing 0xA5 enables and reloads the WDT.
//   Writing 0xDE followed within 4 clocks by 0xAD disables the WDT
//   Writing 0xFF locks out disable feature.
//
// WDTCN.[2:0]: WDT timer interval bits
//   NOTE! When writing interval bits, bit 7 must be a 0.
//
//  Bit 2 | Bit 1 | Bit 0
//------------------------     
//    1   |   1   |   1      Timeout interval = 1048576 x Tsysclk
//    1   |   1   |   0      Timeout interval =  262144 x Tsysclk
//    1   |   0   |   1      Timeout interval =   65636 x Tsysclk
//    1   |   0   |   0      Timeout interval =   16384 x Tsysclk
//    0   |   1   |   1      Timeout interval =    4096 x Tsysclk
//    0   |   1   |   0      Timeout interval =    1024 x Tsysclk
//    0   |   0   |   1      Timeout interval =     256 x Tsysclk
//    0   |   0   |   0      Timeout interval = 	 64 x Tsysclk
// 
//------------------------

	WDTCN = 0x07;	// Watchdog Timer Control Register
    WDTCN = 0xDE;   // Disable WDT
    WDTCN = 0xAD;

//----------------------------------------------------------------
// CROSSBAR REGISTER CONFIGURATION
//
// NOTE: The crossbar register should be configured before any  
// of the digital peripherals are enabled. The pinout of the 
// device is dependent on the crossbar configuration so caution 
// must be exercised when modifying the contents of the XBR0, 
// XBR1, and XBR2 registers. For detailed information on 
// Crossbar Decoder Configuration, refer to Application Note 
// AN001, "Configuring the Port I/O Crossbar Decoder". 
//----------------------------------------------------------------

// Configure the XBRn Registers

    SFRPAGE = 0x0F;
	XBR0 = 0x2E;	// XBAR0: Initial Reset Value
	XBR1 = 0x20;	// XBAR1: Initial Reset Value
	XBR2 = 0x4C;	// XBAR2: Initial Reset Value
// Select Pin I/0

// NOTE: Some peripheral I/O pins can function as either inputs or 
// outputs, depending on the configuration of the peripheral. By default,
// the configuration utility will configure these I/O pins as push-pull 
// outputs.
                    // Port configuration (1 = Push Pull Output)
    SFRPAGE = 0x0F;
    P0MDOUT = 0x74; // Output configuration for P0 
    P1MDOUT = 0x00; // Output configuration for P1 
    P2MDOUT = 0x00; // Output configuration for P2 
    P3MDOUT = 0x03; // Output configuration for P3 

    P1MDIN = 0xFF;  // Input configuration for P1

// View port pinout

		// The current Crossbar configuration results in the 
		// following port pinout assignment:
		// Port 0
		// P0.0 = UART0 TX        (Open-Drain Output/Input)(Digital)
		// P0.1 = UART0 RX        (Open-Drain Output/Input)(Digital)
		// P0.2 = SPI Bus SCK     (Push-Pull Output)(Digital)
		// P0.3 = SPI Bus MISO    (Open-Drain Output/Input)(Digital)
		// P0.4 = SPI Bus MOSI    (Push-Pull Output)(Digital)
		// P0.5 = SPI Bus NSS     (Push-Pull Output)(Digital)
		// P0.6 = UART1 TX        (Push-Pull Output)(Digital)
		// P0.7 = UART1 RX        (Open-Drain Output/Input)(Digital)

		// Port 1
		// P1.0 = PCA CEX0        (Open-Drain Output/Input)(Digital)
		// P1.1 = PCA CEX1        (Open-Drain Output/Input)(Digital)
		// P1.2 = PCA CEX2        (Open-Drain Output/Input)(Digital)
		// P1.3 = PCA CEX3        (Open-Drain Output/Input)(Digital)
		// P1.4 = PCA CEX4        (Open-Drain Output/Input)(Digital)
		// P1.5 = T2              (Open-Drain Output/Input)(Digital)
		// P1.6 = T4              (Open-Drain Output/Input)(Digital)
		// P1.7 = GP I/O          (Open-Drain Output/Input)(Digital)
					
		// Port 2		
		// P2.0 = GP I/O          (Open-Drain Output/Input)(Digital)
		// P2.1 = GP I/O          (Open-Drain Output/Input)(Digital)
		// P2.2 = GP I/O          (Open-Drain Output/Input)(Digital)
		// P2.3 = GP I/O          (Open-Drain Output/Input)(Digital)
		// P2.4 = GP I/O          (Open-Drain Output/Input)(Digital)
		// P2.5 = GP I/O          (Open-Drain Output/Input)(Digital)
		// P2.6 = GP I/O          (Open-Drain Output/Input)(Digital)
		// P2.7 = GP I/O          (Open-Drain Output/Input)(Digital)

		// Port 3		
		// P3.0 = GP I/O          (Push-Pull Output)(Digital)
		// P3.1 = GP I/O          (Push-Pull Output)(Digital)
		// P3.2 = GP I/O          (Open-Drain Output/Input)(Digital)
		// P3.3 = GP I/O          (Open-Drain Output/Input)(Digital)
		// P3.4 = GP I/O          (Open-Drain Output/Input)(Digital)
		// P3.5 = GP I/O          (Open-Drain Output/Input)(Digital)
		// P3.6 = GP I/O          (Open-Drain Output/Input)(Digital)
		// P3.7 = GP I/O          (Open-Drain Output/Input)(Digital)

    SFRPAGE = 0x00;
    EMI0CF = 0x03;    // External Memory Configuration Register

//----------------------------------------------------------------
// Comparators Register Configuration
//
// Bit 7  | Bit 6  | Bit 5  | Bit 4  | Bit 3 | Bit 2 | Bit 1 | Bit 0
//------------------------------------------------------------------     
//  R/W	  |    R   |  R/W   |  R/W   |  R/W  |  R/W  |  R/W  |  R/W
//------------------------------------------------------------------
// Enable | Output | Rising | Falling|  Positive     |  Negative    
//        | State  | Edge   | Edge   |  Hysterisis   |  Hysterisis    
//        | Flag   | Int.   | Int.   |  00: Disable  |  00: Disable
//        |        | Flag   | Flag   |  01:  5mV     |  01:  5mV  
//        |        |        |        |  10: 10mV     |  10: 10mV
//        |        |        |        |  11: 20mV     |  11: 20mV 
// ----------------------------------------------------------------

    SFRPAGE = 0x01;
    CPT0MD = 0x30;   // Comparator 0 Configuration Register
    CPT0CN = 0x00;   // Comparator 0 Control Register
	
    SFRPAGE = 0x02;
    CPT1MD = 0x30;   // Comparator 1 Configuration Register
    CPT1CN = 0x00;   // Comparator 1 Control Register
					
//----------------------------------------------------------------
// Oscillator Configuration
//----------------------------------------------------------------

    SFRPAGE = 0x00;
    FLSCL = 0x10;   // FLASH Memory Control
    SFRPAGE = 0x0F;
	OSCXCN = 0x67;	// EXTERNAL Oscillator Control Register	
    for (n = 0; n < 383; n++) ;            // wait for osc to start
    while ( (OSCXCN & 0x80) == 0 );        // wait for xtal to stabilize
    PLL0CN = 0x5;  // External clock to PLL

    PLL0DIV = 0x04; // PLL pre-divide Register
    PLL0FLT = 0x3F; // PLL Filter Register
    PLL0MUL = 0x09; // PLL Clock scaler Register

    for(b = 0; b < 15; b++);               // wait at least 5us
    PLL0CN |= 0x02;                        // enable PLL
    while ( (PLL0CN & 0x10) == 0 );        // wait for PLL to lock
    CLKSEL = 0x02
;  // Oscillator Clock Selector
	OSCICN = 0x00;	// Internal Oscillator Control Register
	
//----------------------------------------------------------------
// Reference Control Register Configuration
//----------------------------------------------------------------

    SFRPAGE = 0x00;
	REF0CN = 0x17;	// Reference Control Register

//----------------------------------------------------------------
// ADC0 Configuration
//----------------------------------------------------------------

    SFRPAGE = 0x00;
	AMX0CF = 0x08;	// AMUX Configuration Register
	AMX0SL = 0x06;	// AMUX Channel Select Register
	ADC0CF = 0x54;	// ADC Configuration Register
	ADC0CN = 0x80;	// ADC Control Register
	
	ADC0LTH = 0x00;	// ADC Less-Than High Byte Register
	ADC0LTL = 0x00;	// ADC Less-Than Low Byte Register
	ADC0GTH = 0xFF;	// ADC Greater-Than High Byte Register
	ADC0GTL = 0xFF;	// ADC Greater-Than Low Byte Register

//----------------------------------------------------------------
// ADC2 Configuration
//----------------------------------------------------------------   
	SFRPAGE = 0x01;
    AMX2SL = 0x00;  // AMUX Channel Select Register
    AMX2CF = 0x00;  // AMUX Configuration Register
    ADC2CF = 0xF8;  // ADC Configuration Register
    ADC2LT = 0xFF;  // ADC Less-Than Register
    ADC2GT = 0xFF;  // ADC Greater-Than Register
    ADC2CN = 0x00;  // ADC Control Register

//----------------------------------------------------------------
// DAC Configuration
//----------------------------------------------------------------

    SFRPAGE = 0x00;
	DAC0L = 0x55;	// DAC0 Low Byte Register
	DAC0H = 0x10;	// DAC0 High Byte Register
    DAC0CN = 0x80;	// DAC0 Control Register

    SFRPAGE = 0x01;	
	DAC1L = 0x55;	// DAC1 Low Byte Register
	DAC1H = 0x10;	// DAC1 High Byte Register
    DAC1CN = 0x80;	// DAC1 Control Register

//----------------------------------------------------------------
// SPI Configuration
//----------------------------------------------------------------

    SFRPAGE = 0x00;	
	SPI0CFG = 0x60;	// SPI Configuration Register
	SPI0CKR = 0x01;	// SPI Clock Rate Register
    SPI0CN |= 0x01;
    SPI0CN = 0x09;	// SPI Control Register

//----------------------------------------------------------------
// UART0 Configuration
//----------------------------------------------------------------

    SFRPAGE = 0x00;
    SADEN0 = 0x00;      // Serial 0 Slave Address Enable
    SADDR0 = 0x00;      // Serial 0 Slave Address Register
    SSTA0 = 0x15;       // UART0 Status and Clock Selection Register
    SCON0 = 0x50;       // Serial Port Control Register

    PCON = 0x00;        // Power Control Register

//----------------------------------------------------------------
// UART1 Configuration
//----------------------------------------------------------------

    SFRPAGE = 0x01;    
    SCON1 = 0x10;       // Serial Port 1 Control Register   

//----------------------------------------------------------------
// SMBus Configuration
//----------------------------------------------------------------	
    
	SFRPAGE = 0x00; 
	SMB0CN = 0x00;	// SMBus Control Register
	SMB0ADR = 0x00;	// SMBus Address Register
	SMB0CR = 0x00;	// SMBus Clock Rate Register


//----------------------------------------------------------------
// PCA Configuration
//----------------------------------------------------------------

    SFRPAGE = 0x00;
    PCA0MD = 0x05;       // PCA Mode Register
    PCA0CN = 0x40;      // PCA Control Register
    PCA0L = 0x00;       // PCA Counter/Timer Low Byte
    PCA0H = 0x00;       // PCA Counter/Timer High Byte	
	

    //Module 0
    PCA0CPM0 = 0xC2;    // PCA Capture/Compare Register 0
    PCA0CPL0 = 0x8D;    // PCA Counter/Timer Low Byte
    PCA0CPH0 = 0xEC;    // PCA Counter/Timer High Byte		1.5 mS

    //Module 1
    PCA0CPM1 = 0xC2;    // PCA Capture/Compare Register 1
    PCA0CPL1 = 0x8D;    // PCA Counter/Timer Low Byte
    PCA0CPH1 = 0xEC;    // PCA Counter/Timer High Byte		1.5 mS

    //Module 2
    PCA0CPM2 = 0xC2;    // PCA Capture/Compare Register 2
    PCA0CPL2 = 0x8D;    // PCA Counter/Timer Low Byte
    PCA0CPH2 = 0xEC;    // PCA Counter/Timer High Byte		1.5 mS

    //Module 3
    PCA0CPM3 = 0xC2;    // PCA Capture/Compare Register 3  
    PCA0CPL3 = 0x8D;    // PCA Counter/Timer Low Byte
    PCA0CPH3 = 0xEC;    // PCA Counter/Timer High Byte		1.5 mS

    //Module 4
    PCA0CPM4 = 0x00;    // PCA Capture/Compare Register 4	
    PCA0CPL4 = 0x09;    // PCA Counter/Timer Low Byte
    PCA0CPH4 = 0xF3;    // PCA Counter/Timer High Byte		1 mS

    //Module 5
    PCA0CPM5 = 0x00;    // PCA Capture/Compare Register 5
    PCA0CPL5 = 0x00;    // PCA Counter/Timer Low Byte
    PCA0CPH5 = 0x00;    // PCA Counter/Timer High Byte

//----------------------------------------------------------------
// Timers Configuration
//----------------------------------------------------------------

    SFRPAGE = 0x00;
    CKCON = 0x08;   // Clock Control Register
    TL0 = 0x00;     // Timer 0 Low Byte
    TL1 = 0xFF;     // Timer 1 Low Byte
    TH0 = 0xF1;     // Timer 0 High Byte
    TH1 = 0x28;     // Timer 1 High Byte    
    TMOD = 0x22;    // Timer Mode Register
    TCON = 0x50;    // Timer Control Register 

    TMR2CF = 0x08;  // Timer 2 Configuration
    RCAP2L = 0xBC;  // Timer 2 Reload Register Low Byte
    RCAP2H = 0xFE;  // Timer 2 Reload Register High Byte
    TMR2L = 0x00;   // Timer 2 Low Byte	
    TMR2H = 0x00;   // Timer 2 High Byte	
    TMR2CN = 0x04;  // Timer 2 CONTROL	
		
    SFRPAGE = 0x01;
    TMR3CF = 0x00;  // Timer 3 Configuration
    RCAP3L = 0x00;  // Timer 3 Reload Register Low Byte
    RCAP3H = 0x00;  // Timer 3 Reload Register High Byte
    TMR3H = 0x00;   // Timer 3 High Byte
    TMR3L = 0x00;   // Timer 3 Low Byte
    TMR3CN = 0x00;  // Timer 3 Control Register

    SFRPAGE = 0x02;
    TMR4CF = 0x00;  // Timer 4 Configuration
    RCAP4L = 0x00;  // Timer 4 Reload Register Low Byte
    RCAP4H = 0x00;  // Timer 4 Reload Register High Byte
    TMR4H = 0x00;   // Timer 4 High Byte
    TMR4L = 0x00;   // Timer 4 Low Byte
    TMR4CN = 0x06;  // Timer 4 Control Register

//----------------------------------------------------------------
// Reset Source Configuration
//
// Bit 7  | Bit 6  | Bit 5  | Bit 4  | Bit 3 | Bit 2 | Bit 1 | Bit 0
//------------------------------------------------------------------     
//    R	 |   R/W  |  R/W   |  R/W   |   R   |   R   |  R/W  |  R
//------------------------------------------------------------------
//  JTAG  |Convert | Comp.0 | S/W    | WDT   | Miss. | POR   | HW
// Reset  |Start   | Reset/ | Reset  | Reset | Clock | Force | Pin
// Flag   |Reset/  | Enable | Force  | Flag  | Detect| &     | Reset
//        |Enable  | Flag   | &      |       | Flag  | Flag  | Flag
//        |Flag    |        | Flag   |       |       |       |
//------------------------------------------------------------------
// NOTE! : Comparator 0 must be enabled before it is enabled as a 
// reset source.
//
// NOTE! : External CNVSTR must be enalbed through the crossbar, and
// the crossbar enabled prior to enabling CNVSTR as a reset source 
//------------------------------------------------------------------

    SFRPAGE = 0x00;
	RSTSRC = 0x00;	// Reset Source Register


//----------------------------------------------------------------
// Interrupt Configuration
//----------------------------------------------------------------

    IE = 0x00;          //Interrupt Enable
    IP = 0x00;          //Interrupt Priority
    EIE1 = 0x08;        //Extended Interrupt Enable 1
    EIE2 = 0x40;        //Extended Interrupt Enable 2
    EIP1 = 0x08;        //Extended Interrupt Priority 1
    EIP2 = 0x40;        //Extended Interrupt Priority 2

	

// other initialization code here...

}   //End of config
