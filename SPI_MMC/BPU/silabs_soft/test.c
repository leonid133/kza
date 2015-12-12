#include "c8051f120.h"
#include <intrins.h>

//Программа теста, данные из FPGA передаются в RS232
//12 битное АЦП - 8 датчиков - нет.
//8 битное АЦП - 3 датчикa - нет.
// sysclk = 49766400 Hz.

sfr16 DP       = 0x82;                 // data pointer
sfr16 ADC0     = 0xbe;                 // ADC0 data
sfr16 ADC0GT   = 0xc4;                 // ADC0 greater than window
sfr16 ADC0LT   = 0xc6;                 // ADC0 less than window
sfr16 RCAP2    = 0xca;                 // Timer2 capture/reload
sfr16 RCAP3    = 0xca;                 // Timer3 capture/reload
sfr16 RCAP4    = 0xca;                 // Timer4 capture/reload
sfr16 TMR2     = 0xcc;                 // Timer2


void config(void);
void sysclk(void);
void port_init(void);
void UART0_Init(void);
void UART0_isr(void);
void timer3_init(void);
void Timer3_ISR (void);
//void ADC_init(void);

char i, tt, lsb, msb, addr_rd = 0, tmp;
bit flTransmit = 0;
bit flcounter = 0;

int  discret = 1;
int  addr_wr = 0;
int  asd;
int data_pwm, data_pwm_i;
sbit we = P3^3;
sbit re = P3^4;
sbit wr = P3^5;
sbit rd = P3^6;
bit sw = 0;


//---------------------------------------------------------------------
void main(void)
{
	WDTCN = 0xde;
	WDTCN = 0xad;

	port_init();
	sysclk();
	UART0_Init();
	timer3_init();
//	ADC_init();
	config();




	while(1)
	{

//------------------write to FPGA--------------------------------
/*
		we = 1;
		re = 0;
		wr = 0;
			


		for (addr_wr = 112; addr_wr < 127; addr_wr++)
		{
			data_pwm = data_pwm_i + 22500;
			SFRPAGE = 0x0f;
			P6 = addr_wr;
			P4 = (data_pwm & 0x00ff);
			P5 = (data_pwm & 0xff00)>>8;
			for (tmp = 0; tmp < 0x8; tmp++)
				wr = 1;
			wr = 0;
		}
*/
//---------------------------------------------------------------
//------------------read from FPGA-------------------------------

/*
		wr = 0;
		we = 0;
		re = 1;
		SFRPAGE = 0x0f;

		P6 = addr_rd;
		lsb = P7;
		msb = P2;
*/
		if (data_pwm_i == 27500)//delta 1.1ms // delta 1ms =25000
			flcounter = 1;
		else 
			if (data_pwm_i == 0)
				flcounter = 0;

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
	XBR0 = 0x06;	// XBAR0: Initial Reset Value
	XBR1 = 0x00;	// XBAR1: Initial Reset Value
	XBR2 = 0x44;	// XBAR2: Initial Reset Value
// Select Pin I/0

// NOTE: Some peripheral I/O pins can function as either inputs or 
// outputs, depending on the configuration of the peripheral. By default,
// the configuration utility will configure these I/O pins as push-pull 
// outputs.
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

void config(void)
{

	SFRPAGE = 0x00;
	RSTSRC = 0x00;	// Reset Source Register

	IE = 0x90;          //Interrupt Enable
	IP = 0x10;          //Interrupt Priority
	EIE1 = 0x00;        //Extended Interrupt Enable 1
	EIE2 = 0x01;        //Extended Interrupt Enable 2
	EIP1 = 0x00;        //Extended Interrupt Priority 1
	EIP2 = 0x01;        //Extended Interrupt Priority 2

}

void UART0_Init(void)
{
	char SFRPAGE_SAVE = SFRPAGE;

	SFRPAGE = 0x00;

	TMR2CF = 0x08;  // Timer 2 Configuration
	RCAP2L = 0xaf;//0xBC;  // Timer 2 Reload Register Low Byte
	RCAP2H = 0xff;//0xFE;  // Timer 2 Reload Register High Byte
	TMR2L = 0x00;   // Timer 2 Low Byte	
	TMR2H = 0x00;   // Timer 2 High Byte	
	TMR2CN = 0x04;  // Timer 2 CONTROL
	TR2 = 1;	
	SFRPAGE = UART0_PAGE;
	
	SCON0 = 0x50;
	SSTA0 = 0x15;

	SFRPAGE = SFRPAGE_SAVE;
}

//----------------------------------------------------------------------
void UART0_isr(void) interrupt 4
{
	char SFRPAGE_SAVE = SFRPAGE;
	SFRPAGE = UART0_PAGE;

	if(TI0)
	{
		TI0 = 0;
	}
	if(RI0)
	{
		RI0 = 0;
	}
	SFRPAGE = SFRPAGE_SAVE;
}

//--------------------------------------------------------------------
void timer3_init(void)
{
	SFRPAGE = 0x01;
	TMR3CF = 0x00;  // Timer 3 Configuration
	RCAP3L = 0x00;  // Timer 3 Reload Register Low Byte
	RCAP3H = 0xa2;  // Timer 3 Reload Register High Byte
	TMR3H = 0x00;   // Timer 3 High Byte
	TMR3L = 0x00;   // Timer 3 Low Byte
	TMR3CN = 0x04;  // Timer 3 Control Register
}
//--------------------------------------------------------------------
void Timer3_ISR (void) interrupt 14
{
	char SFRPAGE_SAVE = SFRPAGE;
	TF3 = 0;                               // clear TF3

	sw = ~sw;

	if (sw == 0)
	{
		SFRPAGE = 0x00;


		wr = 0;
		we = 0;
		re = 1;
		SFRPAGE = 0x0f;


		P6 = addr_rd;
		lsb = P7;
		msb = P2;


		if (flTransmit == 0)
		{
			SFRPAGE = UART0_PAGE;
			SBUF0 = msb;
			TI0 = 1;
			flTransmit = 1;
		}
		else
		{
			for (asd = 0; asd < 0xf; asd++)
			{ 
				if (asd > 3)
				{
					rd = 1;
				}
			}
			rd = 0;
	
			SFRPAGE = UART0_PAGE;
			SBUF0 = lsb;
			TI0 = 1;
			flTransmit = 0;

			if (addr_rd == 127)
				addr_rd = 0;
			else
				addr_rd = addr_rd + 1;

			if (flcounter == 1) 
				data_pwm_i = data_pwm_i - 500;
			else
				data_pwm_i = data_pwm_i + 500;

		}

		SFRPAGE = 0x00;
		we = 1;
		re = 0;
		wr = 0;
		for (addr_wr = 112; addr_wr <= 128; addr_wr++)
		{
			data_pwm = data_pwm_i + 23750;//-0.95ms //22500;-0.9ms
			SFRPAGE = 0x0f;
			P6 = addr_wr;
			if (addr_wr == 128)
			{
				P4 = (discret & 0x00ff);
				P5 = (discret & 0xff00)>>8;
				if (discret == 0x200)
					discret = 1;
				else
					discret = (discret) * 2;
			}
			else
			{
				P4 = (data_pwm & 0x00ff);
				P5 = (data_pwm & 0xff00)>>8;
			}
			for (tmp = 0; tmp < 0xf; tmp++)
			{
				if (tmp > 1)
				{
					wr = 1;
				}
			}
			wr = 0;
		}
	}
	SFRPAGE = SFRPAGE_SAVE;
}

/*
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
*/