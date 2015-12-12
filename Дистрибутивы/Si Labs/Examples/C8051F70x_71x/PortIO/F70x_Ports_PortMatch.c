//-----------------------------------------------------------------------------
// F700_Ports_PortMatch.c
//-----------------------------------------------------------------------------
// Copyright 2008 Silicon Laboratories, Inc.
// http://www.silabs.com
//
// Program Description:
//
// This program demonstrates how to use the Port Match feature.  The firmware
// enables the port match interrupt and puts the device in idle mode.  Pressing
// a switch wakes the device up from idle mode and turns on an LED in the Port
// Match ISR.  Releasing the switch turns off the LED and the re-enters idle
// mode.
//
//
// How To Test:
//
// 1) Download code to a 'F700 target board
// 2) Ensure that the Switch and LED pins on the J19 header are
//    properly shorted
// 3) Push and hold the switch (P1.1) and see that the LED is on
// 4) Release the switch and see that the LED turns off
//
//
// Target:         C8051F700
// Tool chain:     Keil C51 7.50 / Keil EVAL C51
// Command Line:   None
//
// Release 1.0 / 08 JUL 2008 (PD)
//    -Initial Revision

//-----------------------------------------------------------------------------
// Includes
//-----------------------------------------------------------------------------

#include <compiler_defs.h>
#include <C8051F700_defs.h>            // SFR declarations

//-----------------------------------------------------------------------------
// Pin Declarations
//-----------------------------------------------------------------------------

SBIT (LED, SFR_P1, 0);                 // LED == 1 means ON
SBIT (SW1, SFR_P1, 1);                 // SW1 == 0 means switch depressed

//-----------------------------------------------------------------------------
// Function Prototypes
//-----------------------------------------------------------------------------

void OSCILLATOR_Init (void);
void PORT_Init (void);
void PORT_Match_Init (void);

INTERRUPT_PROTO(PORT_Match_ISR, INTERRUPT_PORT_MATCH);

//-----------------------------------------------------------------------------
// main() Routine
//-----------------------------------------------------------------------------

void main (void)
{

   WDTCN = 0xDE;                       // Disable the watchdog timer
   WDTCN = 0xAD;

   PORT_Init ();                       // Initialize Port I/O
   OSCILLATOR_Init ();                 // Initialize Oscillator

   PORT_Match_Init ();                 // Enable the Port Match interrupt

   EA = 1;                             // Enable global interrupts

   while (1)
   {
	   // Put the device in idle mode
      LED = 0;                         // Turn off LED
      PCON |= 0x01;                    // Enter Idle mode
      PCON = PCON;                     // Safety dummy instruction

      // Pressing the switch pulls P1.1 low which triggers the Port Match
      // interrupt.  When the Port Match interrupt occurs, the device wakes up
      // from Idle mode and vectors to the Port_Match_ISR

      // Port Match is a level sensitive interrupt, so the firmware waits for
      // the switch to be released in the ISR before proceeding.

      // Once the ISR is finished, code execution resumes after the instruction
      // that put the device in idle mode
   }
}

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
// This function initializes the system clock to use the internal
// oscillator.
//
//-----------------------------------------------------------------------------
void OSCILLATOR_Init (void)
{
   // Save the current SFRPAGE
   U8 SFRPAGE_save = SFRPAGE;

   SFRPAGE = CONFIG_PAGE;

   OSCICN |= 0x80;                     // Enable the precision internal osc.

   RSTSRC = 0x06;                      // Enable missing clock detector and
                                       // leave VDD Monitor enabled.

   CLKSEL = 0x00;                      // Select precision internal osc.
                                       // divided by 1 as the system clock

   SFRPAGE = SFRPAGE_save;

}
//-----------------------------------------------------------------------------
// PORT_Init
//-----------------------------------------------------------------------------
//
// Return Value : None
// Parameters   : None
//
// This function configures the crossbar and ports pins.
//
// P1.0   digital   push-pull     LED
// P1.1   digital   open-drain    SW1
//-----------------------------------------------------------------------------

void PORT_Init (void)
{
   U8 SFRPAGE_save = SFRPAGE;
   SFRPAGE = CONFIG_PAGE;

   P1MDIN |= 0x01;                     // P1.0 is digital
   P1MDIN |= 0x02;                     // P1.0 is digital

   P1MDOUT |= 0x01;                    // P1.0 is push-pull
   P1MDOUT &= ~0x02;                   // P1.1 is open-drain

   P1     |= 0x02;                     // Set P1.1 latch to '1'

   XBR1    = 0x40;                     // Enable crossbar and enable
                                       // weak pull-ups

   SFRPAGE = SFRPAGE_save;
}

//-----------------------------------------------------------------------------
// PORT_Match_Init
//-----------------------------------------------------------------------------
//
// Return Value : None
// Parameters   : None
//
// Enable the Port Match interrupt for P1.1.  Whenever P1.1 is logic low, the
// Port Match interrupt is active.
//
//-----------------------------------------------------------------------------

void PORT_Match_Init (void)
{
   U8 SFRPAGE_save = SFRPAGE;
   SFRPAGE = LEGACY_PAGE;

   P1MASK = 0x02;                      // Set P1.1 as the pin to monitor
   P1MAT  = 0x02;                      // Interrupt when P1.1 is low

   EIE1 |= 0x02;                       // Enable Port Match Interrupts

   SFRPAGE = SFRPAGE_save;
}

//-----------------------------------------------------------------------------
// Interrupt Service Subroutines
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// PORT_Match_ISR
//-----------------------------------------------------------------------------
//
// Whenever the Port_Match ISR is active, turn on the LED.  Since the Port
// Match interrupt is level sensitive and not edge sensitive, the ISR
// waits for the switch to be released  and the interrupt is cleared before
// exiting Otherwise, the Port_Match ISR would constantly be triggered until
// the switch was released.
//
//-----------------------------------------------------------------------------

INTERRUPT(PORT_Match_ISR, INTERRUPT_PORT_MATCH)
{
   LED = 1;                            // Turn on LED

   while (SW1 == 0);                   // Wait for switch release
}

//-----------------------------------------------------------------------------
// End Of File
//-----------------------------------------------------------------------------