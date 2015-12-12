#include "c8051f120.h"
#include "init.h"


// Config Routine---------------------------------------------------------------------
void SYSCLK(void)
{
	xdata int n = 0;

	SFRPAGE = 0x00;
	FLSCL = 0x10;   // FLASH Memory Control

	SFRPAGE = CONFIG_PAGE;
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
	//P7MDOUT = 0x00; // Output configuration for P7 
	P7MDOUT = 0x02; // Output configuration for P7 

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
}

//Скорость 57600 бод. Тактируется Timer2------------------------------------------------------
void UART0_Init(void)
{
	SFRPAGE = UART0_PAGE;

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

//-----------------------------------------------------------------------------------
void DAC0_init(void)
{
	SFRPAGE = DAC0_PAGE;
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
	SFRPAGE = ADC0_PAGE;
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
	SFRPAGE = TIMER01_PAGE;
	CKCON = CKCON | 0x02;   // Clock Control Register	Источник - Sysclk/48
	TH0 = 0xAE;     // 0xFFFF-49766400/48/FREQ = 0xAEFF
	TL0 = 0xFF;     
	TMOD = TMOD | 0x01;    // Timer Mode Register 1 режим - 16 разрядный таймер счетчик

	TR0 = 1; 
	return;
}
//-----------------------------------------------------------------------
