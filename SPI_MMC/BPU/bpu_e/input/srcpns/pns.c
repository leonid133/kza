#include "c8051f120.h"

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
sfr16 DAC1     = 0xd2;                 // DAC1 data
sfr16 PCA0CP5  = 0xe1;                 // PCA0 Module 5 capture
sfr16 PCA0CP2  = 0xe9;                 // PCA0 Module 2 capture
sfr16 PCA0CP3  = 0xeb;                 // PCA0 Module 3 capture
sfr16 PCA0CP4  = 0xed;                 // PCA0 Module 4 capture
sfr16 PCA0     = 0xf9;                 // PCA0 counter
sfr16 PCA0CP0  = 0xfb;                 // PCA0 Module 0 capture
sfr16 PCA0CP1  = 0xfd;                 // PCA0 Module 1 capture


void config(void);
void sysclk(void);
void UART0_Init(void);
void UART1_Init(void);
void port_init(void);
void UART0_isr(void);
void UART1_isr(void);
void ADC0_init(void);
void Timer3_init(void);
void ADC0_isr(void);

void Timer0_init(void);
void PCA_init(void);
void PCA_ISR (void);
/*
void PWM(void);
void Test (void);
*/
void Read_flash(void);
void Write_byte(void);

int tmpl, tmph, tmp;
char sw1 = 1, sw2, sw;
int adcrdy;
int i = 0, j = 0;
sbit pow = P6^6;

void main(void)
{
	WDTCN = 0xde;
	WDTCN = 0xad;

	port_init();
	sysclk();
	UART0_Init();
	UART1_Init();
	config();
	ADC0_init();
	Timer3_init();

	Timer0_init();
	PCA_init();

	while(1)
	{

		if (sw == 1)
		{
			SFRPAGE = 0x00;
			SBUF0 = tmp;
			TI0 = 1;
		}
//		else
//		{
//			SFRPAGE = 0x00;
//			TI0 = 0;
//		}
		if (sw2 == 1)
		{
			SFRPAGE = UART1_PAGE;
			SBUF1 = j;
			TI1 = 1;
		}
//		else
//		{
//			SFRPAGE = UART1_PAGE;
//			TI1 = 0;
//		}
	}
}

void SYSCLK(void)
{
	int n = 0;

	SFRPAGE = 0x00;
	FLSCL = 0x10;   // FLASH Memory Control
	SFRPAGE = 0x0F;
	OSCXCN = 0x67;	// EXTERNAL Oscillator Control Register	
	for (n = 0; n < 256; n++) ;            // wait for osc to start
	while ( (OSCXCN & 0x80) == 0 );        // wait for xtal to stabilize

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
	for(n = 0; n < 60; n++);               // wait at least 5us
	PLL0CN |= 0x02;                        // enable PLL
	while ( (PLL0CN & 0x10) == 0 );        // wait for PLL to lock

	CLKSEL = 0x02;  // Oscillator Clock Selector 
}

void port_init(void)
{
    SFRPAGE = 0x0F;
	XBR0 = 0x36;	// XBAR0: Initial Reset Value
	XBR1 = 0x00;	// XBAR1: Initial Reset Value
	XBR2 = 0x44;	// XBAR2: Initial Reset Value
// Select Pin I/0

// NOTE: Some peripheral I/O pins can function as either inputs or 
// outputs, depending on the configuration of the peripheral. By default,
// the configuration utility will configure these I/O pins as push-pull 
// outputs.
                    // Port configuration (1 = Push Pull Output)
    SFRPAGE = 0x0F;
    P0MDOUT = 0x49; // Output configuration for P0 
    P1MDOUT = 0x00; // Output configuration for P1 
    P2MDOUT = 0x00; // Output configuration for P2 
    P3MDOUT = 0x00; // Output configuration for P3 
    P4MDOUT = 0x00; // Output configuration for P4
    P5MDOUT = 0x00; // Output configuration for P5
    P6MDOUT = 0x40; // Output configuration for P6
    P7MDOUT = 0x00; // Output configuration for P7

    P1MDIN = 0x00;  // Input configuration for P1
}

void config(void)
{
	SFRPAGE = 0x00;
	RSTSRC = 0x00;	// Reset Source Register

	IE = 0x90;          //Interrupt Enable
	IP = 0x10;          //Interrupt Priority
	EIE1 = 0x08;        //Extended Interrupt Enable 1
	EIE2 = 0x40;        //Extended Interrupt Enable 2
	EIP1 = 0x00;        //Extended Interrupt Priority 1
	EIP2 = 0x40;        //Extended Interrupt Priority 2
}


void UART0_Init(void)
{
	char SFRPAGE_SAVE = SFRPAGE;

	SFRPAGE = 0x00;

	TMR2CF = 0x08;  // Timer 2 Configuration
	RCAP2L = 0xBC;  // Timer 2 Reload Register Low Byte
	RCAP2H = 0xFE;  // Timer 2 Reload Register High Byte
	TMR2L = 0x00;   // Timer 2 Low Byte	
	TMR2H = 0x00;   // Timer 2 High Byte	
	TMR2CN = 0x04;  // Timer 2 CONTROL
	TR2 = 1;	
	SFRPAGE = UART0_PAGE;
	
	SCON0 = 0x50;
	SSTA0 = 0x15;

	SFRPAGE = SFRPAGE_SAVE;
}

void UART1_Init(void)
{
	char SFRPAGE_SAVE = SFRPAGE;

	SFRPAGE = UART1_PAGE;

	SCON1 = 0x10;       // Serial Port 1 Control Register 
	
	SFRPAGE =0x00;

	CKCON = CKCON & 0x08;
	TMOD = 0x22;
	TH1 = 0x28;
	TL1 = TH1;
	TR1 = 1;  

	SFRPAGE = SFRPAGE_SAVE;
}


void UART0_isr(void) interrupt 4
{
	char SFRPAGE_SAVE = SFRPAGE;
	SFRPAGE = UART0_PAGE;

	if(TI0)
	{
//		SBUF0 = tmp;
		TI0 = 0;
		sw = 0;
	}
	if(RI0)
	{
		RI0 = 0;
		j = SBUF0;
		sw2 = 1;
	}
	SFRPAGE = SFRPAGE_SAVE;
}

void UART1_isr(void) interrupt 20
{
	char SFRPAGE_SAVE = SFRPAGE;
	SFRPAGE = UART1_PAGE;

	if(TI1)
	{
		TI1 = 0;
		sw2 = 0;
	}
	if(RI1)
	{
		tmp = SBUF1;
		RI1 = 0;
		sw = 1;
	}
	SFRPAGE = SFRPAGE_SAVE;
}


void ADC0_init(void)
{
	SFRPAGE = 0x00;
	REF0CN |= 0x03;	// Reference Control Register
	AMX0CF = 0x00;	// AMUX Configuration Register
	ADC0CN = 0x80;
	AMX0SL = 0x00;
	ADC0CF = 0x50;
}

void Timer3_init(void)
{
	SFRPAGE = 0x01;
	TMR3CN = 0;
	TMR3CF = 0x08;
	RCAP3 = 0x00;
	TMR3 = RCAP3;
	TR3 = 1;
}



void Timer0_init(void)
{
	SFRPAGE = 0x00;
	CKCON = CKCON | 0x08;   // Clock Control Register
	TL0 = 0x00;     // Timer 0 Low Byte
	TH0 = 0xF1;     // Timer 0 High Byte
	TMOD = TMOD | 0x22;    // Timer Mode Register
	TCON = TCON | 0x50;    // Timer Control Register 
}

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
}

void PCA_ISR (void) interrupt 9
{
	SFRPAGE = 0x00;
	PCA0CN &= 0xFF;
/*	AMX0SL = 0x00;
	AD0BUSY = 1;
	while(AD0BUSY == 0)
	{
		tmph = (ADC0&0x0fc0)>>6 | 0x80;
		tmpl = (ADC0&0x3f) | 0xc0;
		sw = 1;
	}
*/
}

/*
void PWM(void)
{
	unsigned int n;

	SFRPAGE = 0x00;

		//Module 0 Left rudder
	n=60203-21.61*Delta_left;						// PCA0CP0=60558 +- 1659 
	PCA0CPL0 = n&0xFF;    							// PCA Counter/Timer Low Byte
	PCA0CPH0 = (n >> 8) & 0xFF ;    				// PCA Counter/Timer High Byte

		//Module 1 Right rudder
	n=60203+21.61*Delta_right;						// PCA0CP1=60558 +- 1659
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



void Delay(void)
{
	unsigned long j;
	for (j=0; j<900000; j++);
}



void Test (void)
{
	Delta_left=0;
	Delta_right=0;
//	PWM();
	Delay();

	Delta_left=0;
	Delta_right=0;
//	PWM(); 
	Delay();

	Delta_left=0;
	Delta_right=0;
//	PWM();
	Delay();

	Delta_left=0;
	Delta_right=0;
//	PWM();
	Delay();

	Delta_left=0;
	Delta_right=0;
//	PWM(); 
	Delay();

	Delta_left=0;
	Delta_right=0;
//	PWM();
	Delay();

} 



void ADC0_isr(void) interrupt 15
{
	SFRPAGE = 0x00;
	AD0INT = 0;
	adcrdy = 1;
	tmp = (ADC0&0x0fc0)>>6 | 0x80;
	SBUF0 = 0x33;
	TI0 = 1;
	tmp = (ADC0&0x3f) | 0xc0;
	SBUF0 = tmp;
	TI0 = 1;
	DAC0 = 0x0000;
	SFRPAGE = 0x01;	
	DAC1 = 0;
}


void Write_byte(void)
{ 
	static int Buf_Pointer=0,Page_Adr=0;
	static bit buf_num=1;
	idata unsigned int i=0;

	SFRPAGE = 0x00;

	for(tmpl = 0; tmpl< 200;tmpl ++)
	{
		CS=0;               							// Select flash
		for(i=0;i<50;i++);
									// Wait 250 nS
		SPIF=0;
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

		for(tmp = 0; tmp< 528;tmp ++)
		{
			SPIF=0;
			SPI0DAT=tmpl;
			while(!SPIF);

			if (Buf_Pointer<527) 
				Buf_Pointer++;
			else 
			{
				CS = 1;
				for(i=0;i<10;i++);								// Wait 250 nS

				while (RDY == 0);

				CS=0;		  
				for(i=0;i<10;i++);							// Wait 250 nS

				SPIF=0;	
				if (buf_num) SPI0DAT=0x86;      			// Oðñîde for Buffer 2 writing
				else SPI0DAT=0x83;             				// Oðñîde for Buffer 1 writing
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

				for(i=0;i<10;i++);							// Wait 250 nS 
				CS=1;        

				buf_num=!buf_num;
	
				Buf_Pointer=0;
	
				if (Page_Adr<8191) Page_Adr++;
				else Page_Adr=0;
			}
		}
		for(i=0;i<40000;i++);								// Wait 250 nS
	}
}												// End Routine

void Read_flash(void)
{
	int Page_Adr;
	unsigned char i=0,j=0;

	SFRPAGE = 0x00;
	
	RI0=0;
	TI0=1;

	for (Page_Adr=0;Page_Adr<8192;Page_Adr++)	 	  		
	{
		CS=0;            							// Select flash

		for(i=0;i<10;i++);							// Wait 250 nS

		SPIF=0;
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

		for (i=0;i<33;i++)
		{
			while(!RI0);
			RI0=0;

			for(j=0;j<16;j++)
			{
				SPIF=0;
				SPI0DAT=0xFF;
				while(!SPIF);

				while(!TI0);
				TI0=0;
				SBUF0=SPI0DAT;
			}
		}
		for(i=0;i<10;i++);							// Wait 250 nS
		CS=1;     
	}
}
*/