//-----------------------------------------------------------------------------
// F12x_PCA0_8Bit_PWM_Output.c
//-----------------------------------------------------------------------------
// Copyright 2006 Silicon Laboratories, Inc.
// http://www.silabs.com
//
// Program Description:
//
// This program sends a PWM waveform out of an I/O pin, using the PCA's
// 8-bit PWM Output Mode.  The duty cycle of the waveform is modified
// periodically within the main loop.
//
// In this example, PCA Module 0 is used to generate the waveform, and the
// crossbar is configured to send the CEX0 pin out on P0.0.
//
// How To Test:
//
// 1) Download code to a 'F12x device which has an oscilloscope monitoring P0.0
// 2) Run the program - the waveform should be visible on the oscilloscope.
// 3) Verify that the duty cycle of the waveform varies smoothly between
//    minimum and maximum values.
//
//
// FID:            12X000037
// Target:         C8051F12x / C8051F13x
// Tool chain:     Keil C51 7.50 / Keil EVAL C51
// Command Line:   None
//
//
// Release 1.0
//    -Initial Revision (BD)
//    -10 MAR 2006
//

//-----------------------------------------------------------------------------
// Includes
//-----------------------------------------------------------------------------

#include <c8051f120.h>                 // SFR declarations


//-----------------------------------------------------------------------------
// Global Constants
//-----------------------------------------------------------------------------

#define INTCLK       24500000          // Internal oscillator frequency in Hz
#define SYSCLK       49000000          // Output of PLL derived from (INTCLK*2)

//-----------------------------------------------------------------------------
// Function Prototypes
//-----------------------------------------------------------------------------

void OSCILLATOR_Init (void);
void PORT_Init (void);
void PCA0_Init (void);

//-----------------------------------------------------------------------------
// main() Routine
//-----------------------------------------------------------------------------

void main (void) 
{
   unsigned int delay_count;           // Used to implement a delay
   bit duty_direction = 0;             // 0 = Decrease; 1 = Increase

   // Disable watchdog timer
   WDTCN = 0xde;
   WDTCN = 0xad;

   PORT_Init ();                       // Initialize crossbar and GPIO
   OSCILLATOR_Init ();                 // Initialize oscillator
   PCA0_Init ();                       // Initialize PCA0

   SFRPAGE = PCA0_PAGE;                // Change to PCA0 page

   while (1)
   {
      // Wait a little while
      for (delay_count = 30000; delay_count > 0; delay_count--);

      if (duty_direction == 1)         // Direction = Increase
      {
         // First, check the ECOM0 bit
         if ((PCA0CPM0 & 0x40) == 0x00)
         {
            PCA0CPM0 |= 0x40;          // Set ECOM0 if it is '0'
         }
         else                          // Increase duty cycle otherwise
         {
            PCA0CPH0--;                // Increase duty cycle

            if (PCA0CPH0 == 0x00)
            {
               duty_direction = 0;     // Change direction for next time
            }
         }
      }
      else                             // Direction = Decrease
      {
         if (PCA0CPH0 == 0xFF)
         {
            PCA0CPM0 &= ~0x40;         // Clear ECOM0
            duty_direction = 1;        // Change direction for next time
         }
         else
         {
            PCA0CPH0++;                // Decrease duty cycle
         }
      }

   };
}


//-----------------------------------------------------------------------------
// Initialization Subroutines
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Initialization Subroutines
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// OSCILLATOR_Init
//-----------------------------------------------------------------------------
//
// Return Value : None
// Parameters   : None
//
// This function initializes the system clock to use the internal oscillator
// at 24.5 MHz multiplied by two using the PLL.
//
//-----------------------------------------------------------------------------
void OSCILLATOR_Init (void)
{
   int loop;                           // Software timer

   char SFRPAGE_save = SFRPAGE;        // Save Current SFR page

   SFRPAGE = CONFIG_PAGE;              // Set SFR page

   OSCICN = 0x83;                      // Set internal oscillator to run
                                       // at its maximum frequency

   CLKSEL = 0x00;                      // Select the internal osc. as
                                       // the SYSCLK source

   //Turn on the PLL and increase the system clock by a factor of M/N = 2
   SFRPAGE = CONFIG_PAGE;

   PLL0CN  = 0x00;                     // Set internal osc. as PLL source
   SFRPAGE = LEGACY_PAGE;
   FLSCL   = 0x10;                     // Set FLASH read time for 50MHz clk
                                       // or less
   SFRPAGE = CONFIG_PAGE;
   PLL0CN |= 0x01;                     // Enable Power to PLL
   PLL0DIV = 0x01;                     // Set Pre-divide value to N (N = 1)
   PLL0FLT = 0x01;                     // Set the PLL filter register for
                                       // a reference clock from 19 - 30 MHz
                                       // and an output clock from 45 - 80 MHz
   PLL0MUL = 0x02;                     // Multiply SYSCLK by M (M = 2)

   for (loop=0; loop < 256; loop++);   // Wait at least 5us
   PLL0CN  |= 0x02;                    // Enable the PLL
   while(!(PLL0CN & 0x10));            // Wait until PLL frequency is locked
   CLKSEL  = 0x02;                     // Select PLL as SYSCLK source

   SFRPAGE = SFRPAGE_save;             // Restore SFR page
}

//-----------------------------------------------------------------------------
// PORT_Init
//-----------------------------------------------------------------------------
//
// Return Value : None
// Parameters   : None
//
// This function configures the crossbar and GPIO ports.
//
// P0.0   digital   push-pull     PCA0 CEX0
//
//-----------------------------------------------------------------------------
void PORT_Init (void)
{
   char SFRPAGE_save = SFRPAGE;        // Save Current SFR page

   SFRPAGE = CONFIG_PAGE;              // Set SFR Page

   XBR0    = 0x08;                     // Route CEX0 to P0.0
   XBR1    = 0x00;
   XBR2    = 0x40;                     // Enable crossbar and weak pull-ups

   P0MDOUT |= 0x01;                    // Set CEX0 (P0.0) to push-pull

   SFRPAGE = SFRPAGE_save;             // Restore SFR Page
}


//-----------------------------------------------------------------------------
// PCA0_Init
//-----------------------------------------------------------------------------
//
// Return Value : None
// Parameters   : None
//
// This function configures the PCA time base, and sets up 8-bit PWM output
// mode for Module 0 (CEX0 pin).
//
// The frequency of the PWM signal generated at the CEX0 pin is equal to the
// PCA main timebase frequency divided by 256.
//
// The PCA time base in this example is configured to use SYSCLK, and SYSCLK
// is set up to use the internal PLL running at 49 MHz.  Therefore,
// the frequency of the PWM signal will be 49 MHz / 256 = 191.4 kHz.
// Using different PCA clock sources or a different processor clock will
// result in a different frequency for the PWM signal.
//
//    -------------------------------------------------------------------------
//    How "8-Bit PWM Mode" Works:
//
//       The PCA's 8-bit PWM Mode works by setting an output pin low every
//    time the main PCA counter low byte (PCA0L) overflows, and then setting
//    the pin high whenever a specific match condition is met.
//
//    Upon a PCA0L overflow (PCA0L incrementing from 0xFF to 0x00), two things
//    happen:
//
//    1) The CEXn pin will be set low.
//    2) The contents of the PCA0CPHn register for the module are copied into
//       the PCA0CPLn register for the module.
//
//    When the PCA0L register increments and matches the PCA0CPLn register for
//    the selected module, the CEXn pin will be set high, except when the
//    ECOMn bit in PCA0CPMn is cleared to '0'.  By varying the value of the
//    PCA0CPHn register, the duty cycle of the waveform can also be varied.
//
//    When ECOMn = '1', the duty cycle of the PWM waveform is:
//
//       8-bit PWM Duty Cycle = (256 - PCA0CPLn) / 256
//
//    To set the duty cycle to 100%, a value of 0x00 should be loaded into the
//    PCA0CPHn register for the module being used (with ECOMn set to '1').
//    When the value of PCA0CPLn is equal to 0x00, the pin will never be
//    set low.
//
//    To set the duty cycle to 0%, the ECOMn bit in the PCA0CPMn register
//    should be cleared to 0.  This prevents the PCA0CPLn match from occuring,
//    which results in the pin never being set high.
//    -------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------
void PCA0_Init (void)
{
   char SFRPAGE_save = SFRPAGE;        // Save current SFR Page

   SFRPAGE = PCA0_PAGE;
   // configure PCA time base; overflow interrupt disabled
   PCA0CN = 0x00;                      // Stop counter; clear all flags
   PCA0MD = 0x08;                      // Use SYSCLK as time base

   PCA0CPM0 = 0x42;                    // Module 0 = 8-bit PWM mode

   // Configure initial PWM duty cycle = 50%
   PCA0CPH0 = 256 - (256 * 0.5);

   // Start PCA counter
   CR = 1;
   SFRPAGE = SFRPAGE_save;
}

//-----------------------------------------------------------------------------
// End Of File
//-----------------------------------------------------------------------------