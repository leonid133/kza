//-----------------------------------------------------------------------------
// F41x_Watchdog.c
//-----------------------------------------------------------------------------
// Copyright 2006 Silicon Laboratories, Inc.
// http://www.silabs.com  
//
// Program Description:
//
// This program helps the user to learn about operating the Watch Dog Timer.
// The WDT is used to generate resets if the times between writes to the WDT 
// update register (PCA0CPH5) exceed a specified limit. The WDT can be disabled
// and enabled in the software as needed. When enabled the PCA Module 5 acts as
// the WDT. This program resets the MCU when P1.4 switch is pressed for 3 
// seconds.  Also upon reset the LED blinks approximately five times faster 
// when compared to before. The reset is caused due to a WDT overflow and can 
// be confirmed by checking the value of the RSTSRC register where bit 3 is 
// set to indicate a reset caused by WDT.
// 
// How to Test:
// 1) Compile and download code to a 'F410 device
// 2) Place shorting blocks on P2.1/D3 and P1.4/SW2 pin pairs on jumper J5.
// 3) Run the code:
//        - The test will blink the LED at a rate of 8Hz until the switch SW2
//          (P0.7) is pressed for at least 3 secs.
//        - Once the the switch is pressed and held for a long enough time, 
//          it will prevent the WDT from being touched and the WDT will
//          cause a reset.
//        - Upon reset the code checks for a WDT reset and blinks the LED five
//          times faster than before to indicate the same.      
//
//
// FID:            41X000031
// Target:         C8051F410
// Tool chain:     Keil C51 7.50 / Keil EVAL C51
// Command Line:   None
//
// Release 1.0
//    -Initial Revision SM
//    -20 JULY 2006	

//-----------------------------------------------------------------------------
// Includes
//-----------------------------------------------------------------------------

#include <c8051f410.h>                 // SFR declarations

//-----------------------------------------------------------------------------
// 16-bit SFR Definitions for 'F41x
//-----------------------------------------------------------------------------

sfr16 TMR2RL   = 0xca;                 // Timer2 reload value
sfr16 TMR2     = 0xcc;                 // Timer2 counter

//-----------------------------------------------------------------------------
// Global CONSTANTS
//-----------------------------------------------------------------------------

#define SYSCLK       24500000 / 8      // SYSCLK frequency in Hz

sbit LED = P2^1;                       // LED='1' means ON
sbit SW2 = P1^4;                       // SW2='0' means switch pressed

//-----------------------------------------------------------------------------
// Function PROTOTYPES
//-----------------------------------------------------------------------------

void OSCILLATOR_Init (void);
void PORT_Init (void);
void PCA_Init (void);
void Timer2_Init (int counts);
void Timer2_ISR (void);

//-----------------------------------------------------------------------------
// main() Routine
//-----------------------------------------------------------------------------
//
// The MAIN routine performs all the intialization, and then loops until the 
// switch is pressed. When SW2 (P1.4) is pressed the code checks the RSTSRC 
// register to make sure if the last reset is because of WDT.
//-----------------------------------------------------------------------------
void main (void) 
{
   PCA0MD &= ~0x40;                    // WDTE = 0 (clear watchdog timer 
                                       // enable)
   
   OSCILLATOR_Init ();                 // Initialize system clock to 24.5/8 MHz      
   PCA_Init();                         // Intialize the PCA
   PORT_Init();                        // Initialize crossbar and GPIO
      
   if ((RSTSRC & 0x02) == 0x00)        // First check the PORSF bit. if PORSF
   {                                   // is set, all other RSTSRC flags are
                                       // invalid
      // Check if the last reset was due to the Watch Dog Timer
      if (RSTSRC == 0x08)                    	
      { 
         // Make LED blink at 12Hz
         Timer2_Init (SYSCLK / 12 / 6); 
         // Enable global interrupts
         EA = 1;                         
         while(1);                   // wait forever
      }
      else 
      {
         // Init Timer2 to generate interrupts at a 4Hz rate.
         Timer2_Init (SYSCLK / 12 / 4); 
      }                                   
   
   }
   // Calculate Watchdog Timer Timeout
   // Offset calculated in PCA clocks
   // Offset = ( 256 x PCA0CPL5 ) + 256 - PCA0L
   //        = ( 256 x 255(0xFF)) + 256 - 0
   // Time   = Offset * (12/SYSCLK)   
   //        = ~255 ms ( PCA uses SYSCLK/12 as its clock source)

   PCA0MD  &= ~0x40;                   // WDTE = 0 (clear watchdog timer 
                                       // enable)
   PCA0L    = 0x00;                    // Set lower byte of PCA counter to 0  
   PCA0H    = 0x00;                    // Set higher byte of PCA counter to 0
   PCA0CPL5 = 0xFF;                    // Write offset for the WDT 
   PCA0MD  |= 0x40;                    // Enable the WDT
   EA = 1;                             // Enable global interrupts	


   //--------------------------------------------------------------------------
   // Main Application Loop/Task Scheduler
   //--------------------------------------------------------------------------

   while (1) 
   {   
      //----------------------------------------------------------------------
      // Task #1 - Check Port I/O
      //----------------------------------------------------------------------                              
      while(!SW2);                     // Force the MCU to stay in this task as 
                                       // long as SW2 is pressed. This task
                                       // must finish before the watchdog timer 
                                       // timeout expires.

      //-----------------------------------------------------------------------
      // Task #2 - Reset Watchdog Timer
      //-----------------------------------------------------------------------
      PCA0CPH5 = 0x00;                 // Write a 'dummy' value to the PCA0CPH5 
                                       // register to reset the watchdog timer 
                                       // timeout. If a delay longer than the 
                                       // watchdog timer delay occurs between 
                                       // successive writes to this register,
                                       // the device will be reset by the watch
                                       // dog timer.                         
    }
}

//-----------------------------------------------------------------------------
// Initialization Subroutines
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// OSCILLATOR_Init
//-----------------------------------------------------------------------------
// Return Value : None
// Parameters   : None
// This routine initializes the system clock to use the internal 24.5MHz / 8 
// oscillator as its clock source.  Also enables missing clock detector reset.
//-----------------------------------------------------------------------------
void OSCILLATOR_Init (void)
{ 

   OSCICN = 0x84;                      // Configure internal oscillator
   RSTSRC = 0x04;                      // Enable missing clock detector
}

//-----------------------------------------------------------------------------
// PCA_Init
//-----------------------------------------------------------------------------
// Return Value : None
// Parameters   : None
// This routine initializes the PCA to use the SYSCLK / 12 
// as its clock source.  It also sets the offset value by writing to PCA0CPL2.
//-----------------------------------------------------------------------------
void PCA_Init()
{
    PCA0CN     =  0x40;        		// PCA counter enable
    PCA0MD    &= ~0x40 ;       		// Watchdog timer disabled-clearing bit 6
    PCA0MD    &=  0xF1;        		// Timebase selected - System clock / 12
    PCA0CPL5   =  0xFF;        		// Offset value
}

//-----------------------------------------------------------------------------
// PORT_Init
//-----------------------------------------------------------------------------
// Return Value : None
// Parameters   : None
//
// This function configures the Crossbar and GPIO ports.
// P2.1   digital   push-pull     LED
//-----------------------------------------------------------------------------
void PORT_Init (void)
{

   XBR0     = 0x00;                    // No digital peripherals selected
   XBR1     = 0x40;                    // Enable crossbar and weak pull-ups
   P2MDOUT |= 0x02;                    // Enable LED as a push-pull output
}

//-----------------------------------------------------------------------------
// Timer2_Init
//-----------------------------------------------------------------------------
// Return Value : None
// Parameters   :
//   1)  int counts - calculated Timer overflow rate
//                    range is positive range of integer: 0 to 32767 
//
// Configure Timer2 to 16-bit auto-reload and generate an interrupt at 
// interval specified by <counts> using SYSCLK/48 as its time base.
//-----------------------------------------------------------------------------
void Timer2_Init (int counts)
{
   TMR2CN  = 0x00;                     // Stop Timer2; Clear TF2;
                                       // use SYSCLK/12 as timebase
   
   CKCON   = 0x30;                     // Timer2 clocked based on SYSCLK

   TMR2RL  = -counts;                  // Init reload values
   TMR2    = 0xffff;                   // Set to reload immediately
   ET2     = 1;                        // Enable Timer2 interrupts
   TR2     = 1;                        // Start Timer2
}

//-----------------------------------------------------------------------------
// Interrupt Service Routines
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Timer2_ISR
//-----------------------------------------------------------------------------
// This routine changes the state of the LED whenever Timer2 overflows.
//-----------------------------------------------------------------------------
void Timer2_ISR (void) interrupt 5
{
   TF2H = 0;                           // Clear Timer2 interrupt flag
   LED = ~LED;                         // Change state of LED
}

//-----------------------------------------------------------------------------
// End Of File
//-----------------------------------------------------------------------------
