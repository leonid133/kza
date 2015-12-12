//-----------------------------------------------------------------------------
// F12x_UART1_Interrupt.c
//-----------------------------------------------------------------------------
// Copyright 2008 Silicon Laboratories, Inc.
// http://www.silabs.com
//
// Program Description:
//
// This program demonstrates how to configure the C8051F120 to write to and read
// from the UART interface. The program reads a word using the UART1 interrupts
// and outputs that word to the screen, with all characters in uppercase
//
// How To Test:
//
// 1) Download code to a 'F12x device that is connected to a UART transceiver
// 2) Verify jumpers J6 and J9 are populated on the 'F12x TB.
// 3) Connect serial cable from the transceiver to a PC
// 4) On the PC, open HyperTerminal (or any other terminal program) and connect
//    to the COM port at <BAUDRATE> and 8-N-1
// 5) Download and execute code on an 'F12x target board.
//
//
// Target:         C8051F12x
// Tool chain:     Keil C51 7.50 / Keil EVAL C51
// Command Line:   None
//
// Release 1.1
//    -Corrected "UART0" references to "UART1" in the UART1 ISR (BD)
//    -1 JUL 2008
//
// Release 1.0
//    -Initial Revision (SM)
//    -6 JUN 2007
//


//-----------------------------------------------------------------------------
// Includes
//-----------------------------------------------------------------------------

#include <c8051f120.h>                 // SFR declarations
#include <stdio.h>

//-----------------------------------------------------------------------------
// Global Constants
//-----------------------------------------------------------------------------

#define BAUDRATE            115200     // Baud rate of UART in bps
#define SYSTEMCLOCK       24500000

//-----------------------------------------------------------------------------
// Function Prototypes
//-----------------------------------------------------------------------------

void OSCILLATOR_Init (void);
void PORT_Init (void);
void UART1_Init (void);

//-----------------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------------

#define UART_BUFFERSIZE 64
unsigned char UART_Buffer[UART_BUFFERSIZE];
unsigned char UART_Buffer_Size = 0;
unsigned char UART_Input_First = 0;
unsigned char UART_Output_First = 0;
unsigned char TX_Ready =1;
static char Byte;

//-----------------------------------------------------------------------------
// main() Routine
//-----------------------------------------------------------------------------

void main (void)
{
   SFRPAGE = CONFIG_PAGE;

   WDTCN = 0xDE;                       // Disable watchdog timer
   WDTCN = 0xAD;

   OSCILLATOR_Init ();                 // Initialize oscillator
   PORT_Init ();                       // Initialize crossbar and GPIO

   UART1_Init ();                      // Initialize UART1

   EA = 1;

   SFRPAGE = UART1_PAGE;

   while (1)
   {  // If the complete word has been entered via the hyperterminal followed
      // by carriage return
      if((TX_Ready == 1) && (UART_Buffer_Size != 0) && (Byte == 13))
      {
         TX_Ready = 0;                 // Set the flag to zero

         TI1 = 1;                      // Set transmit flag to 1
      }
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
// This function initializes the system clock to 24.5 MHz
//
//-----------------------------------------------------------------------------
void OSCILLATOR_Init (void)
{
   char SFRPAGE_SAVE = SFRPAGE;        // Save Current SFR page

   SFRPAGE = CONFIG_PAGE;              // Set SFR page

   OSCICN = 0x83;                      // Set internal oscillator to run
                                       // at its slowest frequency

   SFRPAGE = SFRPAGE_SAVE;             // Restore SFRPAGE
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
// P0.0   digital   push-pull     UART TX
// P0.1   digital   open-drain    UART RX
//-----------------------------------------------------------------------------
void PORT_Init (void)
{
   char SFRPAGE_SAVE = SFRPAGE;        // Save Current SFR page

   SFRPAGE = CONFIG_PAGE;              // Set SFR page

   XBR0     = 0x00;
   XBR1     = 0x00;
   XBR2     = 0x44;                    // Enable crossbar and weak pull-up
                                       // Enable UART1

   P0MDOUT |= 0x01;                    // Set TX pin to push-pull
   P1MDOUT |= 0x40;                    // Set P1.6(LED) to push-pull

   SFRPAGE = SFRPAGE_SAVE;             // Restore SFR page
}


//-----------------------------------------------------------------------------
// UART1_Init
//-----------------------------------------------------------------------------
//
// Return Value : None
// Parameters   : None
//
// Configure the UART1 using Timer1, for <baudrate> and 8-N-1.
//
//-----------------------------------------------------------------------------
void UART1_Init (void)
{
   char SFRPAGE_SAVE = SFRPAGE;        // Save Current SFR page

   SFRPAGE = UART1_PAGE;
   SCON1   = 0x10;                     // SCON1: mode 0, 8-bit UART, enable RX

   SFRPAGE = TIMER01_PAGE;
   TMOD   &= ~0xF0;
   TMOD   |=  0x20;                    // TMOD: timer 1, mode 2, 8-bit reload


   if (SYSTEMCLOCK/BAUDRATE/2/256 < 1) {
      TH1 = -(SYSTEMCLOCK/BAUDRATE/2);
      CKCON |= 0x10;                   // T1M = 1; SCA1:0 = xx
   } else if (SYSTEMCLOCK/BAUDRATE/2/256 < 4) {
      TH1 = -(SYSTEMCLOCK/BAUDRATE/2/4);
      CKCON &= ~0x13;                  // Clear all T1 related bits
      CKCON |=  0x01;                  // T1M = 0; SCA1:0 = 01
   } else if (SYSTEMCLOCK/BAUDRATE/2/256 < 12) {
      TH1 = -(SYSTEMCLOCK/BAUDRATE/2/12);
      CKCON &= ~0x13;                  // T1M = 0; SCA1:0 = 00
   } else {
      TH1 = -(SYSTEMCLOCK/BAUDRATE/2/48);
      CKCON &= ~0x13;                  // Clear all T1 related bits
      CKCON |=  0x02;                  // T1M = 0; SCA1:0 = 10
   }

   TL1 = TH1;                          // init Timer1
   TR1 = 1;                            // START Timer1
   TX_Ready = 1;                       // Flag showing that UART can transmit
   EIE2      = 0x40;                   // Enable UART1 interrupts

   SFRPAGE = UART1_PAGE;
   EIP2    = 0x40;                     // Make UART high priority

   SFRPAGE = SFRPAGE_SAVE;             // Restore SFR page

}

//-----------------------------------------------------------------------------
// Interrupt Service Routines
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// UART1_Interrupt
//-----------------------------------------------------------------------------
//
// This routine is invoked whenever a character is entered or displayed on the
// Hyperterminal.
//
//-----------------------------------------------------------------------------

void UART1_Interrupt (void) interrupt 20
{
   if (RI1 == 1)
   {
      if( UART_Buffer_Size == 0)  {      // If new word is entered
         UART_Input_First = 0;    }

      RI1 = 0;                           // Clear interrupt flag

      Byte = SBUF1;                      // Read a character from UART

      if (UART_Buffer_Size < UART_BUFFERSIZE)
      {
         UART_Buffer[UART_Input_First] = Byte; // Store in array

         UART_Buffer_Size++;             // Update array's size

         UART_Input_First++;             // Update counter
      }
   }

   if (TI1 == 1)                         // Check if transmit flag is set
   {
      TI1 = 0;                           // Clear interrupt flag

      if (UART_Buffer_Size != 1)         // If buffer not empty
      {
         // If a new word is being output
         if ( UART_Buffer_Size == UART_Input_First ) {
              UART_Output_First = 0;  }

         // Store a character in the variable byte
         Byte = UART_Buffer[UART_Output_First];

         if ((Byte >= 0x61) && (Byte <= 0x7A)) { // If upper case letter
            Byte -= 32; }

         SBUF1 = Byte;                   // Transmit to Hyperterminal

         UART_Output_First++;            // Update counter

         UART_Buffer_Size--;             // Decrease array size

      }
      else
      {
         UART_Buffer_Size = 0;            // Set the array size to 0
         TX_Ready = 1;                    // Indicate transmission complete
      }
   }
}

//-----------------------------------------------------------------------------
// End Of File
//-----------------------------------------------------------------------------