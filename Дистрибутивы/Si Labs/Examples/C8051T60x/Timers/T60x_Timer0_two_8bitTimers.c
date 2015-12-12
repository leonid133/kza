//-----------------------------------------------------------------------------
// T60x_Timer0_two_8bitTimers.c
//-----------------------------------------------------------------------------
// Copyright 2008 Silicon Laboratories, Inc.
// http://www.silabs.com
//
// Program Description:
//
// This program presents an example of use of the Timer0 of the C8051T60x's in
// two 8-bit counter/timer mode. 
//
// It sets the two timers to interrupt every millisecond.
//
// When the Timer0 low interrupts a counter (low_counter) increments and upon
// reaching the value defined by LED_TOGGLE_RATE toggles the LED.
// When the Timer0 high interrupts a counter (high_counter) increments and upon
// reaching the value defined by SIGNAL_TOGGLE_RATE toggles the SIGNAL pin.
//
// Pinout:
//
//    P0.2 -> LED (Toggled by TL0; T606)
//    P0.6 -> LED (Toggled by TL0; T600/1/2/3/4/5)
//
//    P0.7 -> SIGNAL (toggled by TH0)
//
//    all other port pins unused
//
// How To Test:
//
// 1) Define the chip number being used under CHIPNUM
// 2) If desired, change the number of interrupts (ms) to toggle an output:
//    LED_TOGGLE_RATE
//    SIGNAL_TOGGLE_RATE
// 3) Compile and download the code.
// 4) Verify the LED pins of J3 are populated on the 'T60x TB.
// 5) Run the code.
// 6) Verify that the LED is blinking and SIGNAL (P0.7) is toggling.
//
//
// Target:         C8051T60x
// Tool chain:     KEIL C51 8.0 / KEIL EVAL C51
// Command Line:   None
//
// Release 1.0 / 31 JUL 2008 (ADT)
//    - Initial Revision
//
//-----------------------------------------------------------------------------
// Includes
//-----------------------------------------------------------------------------
#include <compiler_defs.h>
#include <C8051T600_defs.h>            // SFR declarations

//-----------------------------------------------------------------------------
// Global Constants
//-----------------------------------------------------------------------------
#define CHIPNUM   600                  // Define the chip number being used
                                       // Options: 600, 601, 602, 603,
                                       //          604, 605, 606

#define SYSCLK             24500000/8  // SYSCLK in Hz (24.5 MHz internal
                                       // oscillator / 8)
                                       // the internal oscillator has a
                                       // tolerance of +/- 2%

#define TIMER_PRESCALER            48  // Based on Timer CKCON settings

// There are SYSCLK/TIMER_PRESCALER timer ticks per second, so
// SYSCLK/TIMER_PRESCALER/1000 timer ticks per millisecond.
#define TIMER_TICKS_PER_MS  SYSCLK/TIMER_PRESCALER/1000

// Note: TIMER_TICKS_PER_MS should not exceed 255 (0xFF) for the 8-bit timers

#define AUX1     -TIMER_TICKS_PER_MS

#define LED_TOGGLE_RATE           100  // LED toggle rate in milliseconds
                                       // if LED_TOGGLE_RATE = 1, the LED will
                                       // be on for 1 millisecond and off for
                                       // 1 millisecond

#define SIGNAL_TOGGLE_RATE         30  // SIGNAL pin toggle rate in
                                       // milliseconds
                                       // if SIGNAL_TOGGLE_RATE = 1, the
                                       // SIGNAL output will be on for 1
                                       // millisecond and off for 1
                                       // millisecond

#define TIMER0_RELOAD_HIGH       AUX1  // Reload value for Timer0 high byte
#define TIMER0_RELOAD_LOW        AUX1  // Reload value for Timer0 low byte

#if (CHIPNUM == 606)
SBIT (LED, SFR_P0, 2);                 // LED='1' means ON
#else
SBIT (LED, SFR_P0, 6);
#endif

SBIT (SIGNAL, SFR_P0, 7);              // SIGNAL to be used also by Timer0

//-----------------------------------------------------------------------------
// Function Prototypes
//-----------------------------------------------------------------------------

void Port_Init (void);                 // Port initialization routine
void Timer0_Init (void);               // Timer0 initialization routine

//-----------------------------------------------------------------------------
// main() Routine
//-----------------------------------------------------------------------------

void main (void)
{
   PCA0MD &= ~0x40;                    // Clear watchdog timer enable
   REG0CN |= 0x01;                     // OTP Power Controller Enable

   Timer0_Init ();                     // Initialize the Timer0
   Port_Init ();                       // Init Ports
   EA = 1;                             // Enable global interrupts

   while (1);                          // Loop forever
}

//-----------------------------------------------------------------------------
// Initialization Subroutines
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Port_Init
//-----------------------------------------------------------------------------
//
// Return Value : None
// Parameters   : None
//
// This function configures the crossbar and GPIO ports.
//
// P0.2   digital   push-pull     LED (toggled by TL0; T606)
// P0.6   digital   push-pull     LED (toggled by TL0; T600/1/2/3/4/5)
//
// P0.7   digital   push-pull     SIGNAL (toggled by TH0)
//
// all other port pins unused
//
//-----------------------------------------------------------------------------
void Port_Init (void)
{
   XBR2 = 0x40;                        // Enable crossbar
   P0MDOUT = 0x80;                     // SIGNAL is push-pull

#if (CHIPNUM == 606)
   P0MDOUT |= 0x04;
#else
   P0MDOUT |= 0x40;
#endif
}

//-----------------------------------------------------------------------------
// Timer0_Init
//-----------------------------------------------------------------------------
//
// Return Value : None
// Parameters   : None
//
// This function configures the Timer0 as two 8-bit timers, interrupt enabled.
// Using the internal osc. at 24.5MHz with a prescaler of 1:8 and reloading the
// TH0 register with TIMER0_RELOAD_HIGH and TL0 with TIMER0_RELOAD_HIGH.
//
// Note: The Timer0 uses a 1:48 prescaler.  If this setting changes, the
// TIMER_PRESCALER constant must also be changed.
//-----------------------------------------------------------------------------
void Timer0_Init(void)
{
   TH0 = TIMER0_RELOAD_HIGH;           // Init Timer0 High register
   TL0 = TIMER0_RELOAD_LOW;            // Init Timer0 Low register

   TMOD = 0x03;                        // Timer0 in two 8-bit mode
   CKCON = 0x02;                       // Timer0 uses a 1:48 prescaler
   ET0 = 1;                            // Timer0 interrupt enabled
   ET1 = 1;                            // Timer1 interrupt enabled
   TCON = 0x50;                        // Timer0 and timer1 ON
}


//-----------------------------------------------------------------------------
// Interrupt Service Routines
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Timer0_ISR
//-----------------------------------------------------------------------------
//
// Here we process the Timer0 interrupt and toggle the LED
//
//-----------------------------------------------------------------------------
INTERRUPT (Timer0_ISR, INTERRUPT_TIMER0)
{
   static U16 low_counter=0;  // Define counter variable

   TL0 = TIMER0_RELOAD_LOW;            // Reinit Timer0 Low register
   if((low_counter++) == LED_TOGGLE_RATE)
   {
      low_counter = 0;                 // Reset interrupt counter
      LED = ~LED;                      // Toggle the LED
   }
}

//-----------------------------------------------------------------------------
// Timer1_ISR
//-----------------------------------------------------------------------------
//
// Here we process the Timer1 interrupt and toggle the SIGNAL I/O
// Note: In this mode the Timer0 is effectively using the Timer1 interrupt
// vector. For more info check the Timers chapter of the datasheet.
//-----------------------------------------------------------------------------
INTERRUPT (Timer1_ISR, INTERRUPT_TIMER1)
{
   static U16 high_counter=0;

   TH0 = TIMER0_RELOAD_HIGH;           // Reinit Timer0 High register
   if((high_counter++) == SIGNAL_TOGGLE_RATE)
   {
      high_counter = 0;                // Reset interrupt counter
      SIGNAL = ~SIGNAL;                // Toggle Signal
   }
}

//-----------------------------------------------------------------------------
// End Of File
//-----------------------------------------------------------------------------