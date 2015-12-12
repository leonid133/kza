//-----------------------------------------------------------------------------
// T60x_Watchdog.c
//-----------------------------------------------------------------------------
// Copyright 2008 Silicon Laboratories, Inc.
// http://www.silabs.com  
//
// Program Description:
//
// This program helps the user to learn about operating the Watch Dog Timer.
// The WDT is used to generate resets if the times between writes to the WDT 
// update register (PCA0CPH2) exceed a specified limit. The WDT can be disabled 
// and enabled in the software as needed. When enabled the PCA Module 2 acts as
// the WDT. This program resets the MCU when P0.3 switch is pressed. Also upon 
// reset the LED blinks approximately five times faster when compared to before. 
// The reset is caused due to a WDT overflow and can be confirmed by checking 
// the value of the RSTSRC register where bit 3 is set to indicate a reset
// caused by WDT.
// 
// How to Test:
// 1) Define the chip number being used under CHIPNUM
// 2) Compile and download code to a 'T60x device.
// 3) Place shorting blocks on jumper J3.
// 4) Run the code:
//        - The test will blink the LED at a rate of 10Hz until the switch SW2
//          (P0.3) is pressed long enough to trip the WDT.
//        - Once the the switch is pressed it will cause the WDT to trip and 
//          cause a reset.
//        - Upon reset the code checks for a WDT reset and blinks the LED five
//          times faster than before to indicate the same.      
//
//
// Target:         C8051T60x
// Tool chain:     Keil C51 8.0 / Keil EVAL C51
// Command Line:   None
//
// Release 1.1 / 01 AUG 2008 (ADT)
//    - Updated to use compiler_defs.h
// 
// Release 1.0
//    -Initial Revision (BD)
//    -17 JAN 2007

//-----------------------------------------------------------------------------
// Includes
//-----------------------------------------------------------------------------
#include <compiler_defs.h>
#include <C8051T600_defs.h>            // SFR declarations

//-----------------------------------------------------------------------------
// Global CONSTANTS
//-----------------------------------------------------------------------------

#define CHIPNUM   600                  // Define the chip number being used
                                       // Options: 600, 601, 602, 603,
                                       //          604, 605, 606

#define SYSCLK       24500000 / 8      // SYSCLK frequency in Hz


#if (CHIPNUM == 606)                   // 1 = on
 SBIT (LED, SFR_P0, 2);
#else
 SBIT (LED,  SFR_P0, 6);               
#endif

SBIT (SW1, SFR_P0, 3);                 // SW1='0' means switch pressed

//-----------------------------------------------------------------------------
// Function PROTOTYPES
//-----------------------------------------------------------------------------
void OSCILLATOR_Init (void);
void PORT_Init (void);
void PCA_Init (void);
void Timer2_Init (U16 counts);

INTERRUPT_PROTO (Timer2_ISR, INTERRUPT_TIMER2);

//-----------------------------------------------------------------------------
// main() Routine
//-----------------------------------------------------------------------------
//
// The MAIN routine performs all the intialization, and then loops until the 
// switch is pressed. When SW2 (P0.3) is pressed the code checks the RSTSRC 
// register to make sure if the last reset is because of WDT.
//-----------------------------------------------------------------------------
void main (void) 
{
   
   // disable watchdog timer
   PCA0MD &= ~0x40;                    // WDTE = 0 (clear watchdog timer 
                                       // enable)
   REG0CN |= 0x01;                     // OTP Power Controller Enable
   // Initialize system clock to 24.5/8 MHz
   OSCILLATOR_Init ();                        
   PCA_Init();                         // Intialize the PCA
   PORT_Init();                        // Initialize crossbar and GPIO
      
   if ((RSTSRC & 0x02) == 0x00)        // First check the PORSF bit. if PORSF
   {                                   // is set, all other RSTSRC flags are
                                       // invalid
      // Check if the last reset was due to the Watch Dog Timer
      if (RSTSRC == 0x08)                    	
      { 
         // Make LED blink at 50Hz
         Timer2_Init (SYSCLK / 12 / 50); 
         // Enable global interrupts
         EA = 1;                         
         while(1);                     // wait forever
      }
      else 
      {
         // Init Timer2 to generate interrupts at a 10Hz rate.
         Timer2_Init (SYSCLK / 12 / 10); 
      }                                   
   
   }
   // Calculate Watchdog Timer Timeout
   // Offset calculated in PCA clocks
   // Offset = ( 256 x PCA0CPL2 ) + 256 - PCA0L
   //        = ( 256 x 255(0xFF)) + 256 - 0
   // Time   = Offset * (12/SYSCLK)   
   //        = 255 ms ( PCA uses SYSCLK/12 as its clock source)
     
   PCA0MD  &= ~0x40;                   // WDTE = 0 (clear watchdog timer 
                                       // enable)
   PCA0L    = 0x00;                    // Set lower byte of PCA counter to 0  
   PCA0H    = 0x00;                    // Set higher byte of PCA counter to 0
   PCA0CPL2 = 0xFF;                    // Write offset for the WDT 
   PCA0MD  |= 0x40;                    // enable the WDT
   PCA0CPH2 = 0x00;                    // reloads the WDT
   EA = 1;                             // enable global interrupts

  
   //--------------------------------------------------------------------------
   // Main Application Loop/Task Scheduler
   //--------------------------------------------------------------------------

   while (1) 
   {   
      //-----------------------------------------------------------------------
      // Task #1 - Check Port I/O
      //-----------------------------------------------------------------------
      while(!SW1);                     // Force the MCU to stay in this task as
                                       // long as SW1 is pressed. This task 
                                       // must finish before the watchdog timer
                                       // timeout expires.

      //-----------------------------------------------------------------------
      // Task #2 - Reset Watchdog Timer
      //-----------------------------------------------------------------------
      PCA0CPH2 = 0x00;                 // Write a 'dummy' value to the PCA0CPH2 
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
   OSCICN = 0x04;                         // Configure internal oscillator for
                                          // its lowest frequency
   RSTSRC = 0x04;                         // Enable missing clock detector
}

//-----------------------------------------------------------------------------
// PCA_Init
//-----------------------------------------------------------------------------
// Return Value : None
// Parameters   : None
// This routine initializes the PCA to use the SYSCLK / 12 
// as its clock source.  It also sets the offset value by writing to PCA0CPL2.
//-----------------------------------------------------------------------------
void PCA_Init (void)
{
    PCA0CN     =  0x40;                // PCA counter enable
    PCA0MD    &= ~0x40 ;               // Watchdog timer disabled
    PCA0MD    &=  0xF1;                // timebase selected - System clock / 12
    PCA0CPL2   =  0xFF;                // Offset value
}

//-----------------------------------------------------------------------------
// PORT_Init
//-----------------------------------------------------------------------------
// Return Value : None
// Parameters   : None
//
// This function configures the Crossbar and GPIO ports.
// P0.2   digital   push-pull     LED (T606)
// P0.3   digital   open-drain    Switch 1
// P0.6   digital   push-pull     LED (T600/1/2/3/4/5)
//-----------------------------------------------------------------------------
void PORT_Init (void)
{
   XBR1     = 0x00;                       // No digital peripherals selected
   XBR2     = 0x40;                       // Enable crossbar and weak pull-ups

#if (CHIPNUM == 606)
   P0MDOUT |= 0x04;
#else
   P0MDOUT |= 0x40;                       // Enable LED as a push-pull output
#endif
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
void Timer2_Init (U16 counts)
{
   TMR2CN  = 0x00;                        // Stop Timer2; Clear TF2
                                          // use SYSCLK/12 as timebase
   CKCON  &= ~0x60;                       // Timer2 clocked based on T2XCLK

   TMR2RL  = -counts;                     // Init reload values
   TMR2    = 0xffff;                      // set to reload immediately
   ET2     = 1;                           // enable Timer2 interrupts
   TR2     = 1;                           // start Timer2
}

//-----------------------------------------------------------------------------
// Interrupt Service Routines
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Timer2_ISR
//-----------------------------------------------------------------------------
// This routine changes the state of the LED whenever Timer2 overflows.
//-----------------------------------------------------------------------------
INTERRUPT (Timer2_ISR, INTERRUPT_TIMER2)
{
   TF2H = 0;                           // Clear Timer2 interrupt flag
   LED = ~LED;                         // Change state of LED
}

//-----------------------------------------------------------------------------
// End Of File
//-----------------------------------------------------------------------------
