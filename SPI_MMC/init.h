#ifndef INITH
#define INITH

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

void config(void);
void sysclk(void);
void UART0_Init(void);
void UART1_Init(void);
void port_init(void);
void DAC0_init(void);
void ADC_init(void);

void Timer0_init(void);



//-------------------------------------------------------------------------------------
//SYSCLK = 49766400 - тактовая частота
#define FREQ         50      		// Частота прерываний таймер0 = 50Гц

#endif