//-----------------------------------------------------------------------------
// F500_FlashPrimitives.c
//-----------------------------------------------------------------------------
// Copyright 2008 Silicon Laboratories, Inc.
// http://www.silabs.com
//
// Program Description:
//
// This program contains several useful utilities for writing and updating
// FLASH memory.
//
// Target:         C8051F500/1/2/3/4/5/6/7/8/9/10/11
// Tool chain:     Keil C51 8.00
// Command Line:   None
//
//
// Release 1.0
//    -Initial Revision (GP)
//    -08 APR 2008
//

//-----------------------------------------------------------------------------
// Includes
//-----------------------------------------------------------------------------

#include <compiler_defs.h>
#include <c8051F500_defs.h>
#include "F500_FlashPrimitives.h"

//-----------------------------------------------------------------------------
// Function Prototypes
//-----------------------------------------------------------------------------

void  FLASH_ByteWrite (FLADDR addr, U8 byte);
U8    FLASH_ByteRead  (FLADDR addr);
void  FLASH_PageErase (FLADDR addr);

//-----------------------------------------------------------------------------
// FLASH_ByteWrite
//-----------------------------------------------------------------------------
//
// Return Value : None
// Parameters   :
//   1) FLADDR addr - address of the byte to write to
//                    valid range is 0x0000 to 0xFBFF for 64K Flash devices
//                    valid range is 0x0000 to 0x7FFF for 32K Flash devices
//   2) U8 byte - byte to write to Flash.
//
// This routine writes <byte> to the linear FLASH address <addr>.
//-----------------------------------------------------------------------------

void FLASH_ByteWrite (FLADDR addr, U8 byte)
{
   bit EA_SAVE = EA;                   // Preserve EA
   U8 xdata * data pwrite;             // FLASH write pointer

   U8 SFRPAGE_save = SFRPAGE;
   SFRPAGE = ACTIVE_PAGE;

   EA = 0;                             // Disable interrupts

   VDM0CN = 0xA0;                      // Enable VDD monitor and high threshold

   RSTSRC = 0x02;                      // Enable VDD monitor as a reset source

   pwrite = (U8 xdata *) addr;

   FLKEY  = 0xA5;                      // Key Sequence 1
   FLKEY  = 0xF1;                      // Key Sequence 2
   PSCTL |= 0x01;                      // PSWE = 1 which enables writes

   VDM0CN = 0xA0;                      // Enable VDD monitor and high threshold

   RSTSRC = 0x02;                      // Enable VDD monitor as a reset source

   *pwrite = byte;                     // Write the byte

   PSCTL &= ~0x01;                     // PSWE = 0 which disable writes

   EA = EA_SAVE;                       // Restore interrupts

   SFRPAGE = SFRPAGE_save;
}

//-----------------------------------------------------------------------------
// FLASH_ByteRead
//-----------------------------------------------------------------------------
//
// Return Value :
//      U8 - byte read from Flash
// Parameters   :
//   1) FLADDR addr - address of the byte to read to
//                    valid range is 0x0000 to 0xFBFF for 64K Flash devices
//                    valid range is 0x0000 to 0x7FFF for 32K Flash devices
//
// This routine reads a <byte> from the linear FLASH address <addr>.
//-----------------------------------------------------------------------------

U8 FLASH_ByteRead (FLADDR addr)
{
   bit EA_SAVE = EA;                   // Preserve EA
   U8 code * data pread;               // FLASH read pointer
   U8 byte;

   EA = 0;                             // Disable interrupts

   pread = (U8 code *) addr;

   byte = *pread;                      // Read the byte

   EA = EA_SAVE;                       // Restore interrupts

   return byte;
}

//-----------------------------------------------------------------------------
// FLASH_PageErase
//-----------------------------------------------------------------------------
//
// Return Value : None
// Parameters   :
//   1) FLADDR addr - address of any byte in the page to erase
//                    valid range is 0x0000 to 0xF9FF for 64K Flash devices
//                    valid range is 0x0000 to 0x7DFF for 32K Flash devices
//
// This routine erases the FLASH page containing the linear FLASH address
// <addr>.  Note that the page of Flash containing the Lock Byte cannot be
// erased from application code.
//-----------------------------------------------------------------------------

void FLASH_PageErase (FLADDR addr)
{
   bit EA_SAVE = EA;                   // Preserve EA
   U8 xdata * data pwrite;             // FLASH write pointer

   U8 SFRPAGE_save = SFRPAGE;
   SFRPAGE = ACTIVE_PAGE;

   EA = 0;                             // Disable interrupts

   VDM0CN = 0xA0;                      // Enable VDD monitor and high threshold

   RSTSRC = 0x02;                      // Enable VDD monitor as a reset source

   pwrite = (U8 xdata *) addr;

   FLKEY  = 0xA5;                      // Key Sequence 1
   FLKEY  = 0xF1;                      // Key Sequence 2
   PSCTL |= 0x03;                      // PSWE = 1; PSEE = 1

   VDM0CN = 0xA0;                      // Enable VDD monitor and high threshold

   RSTSRC = 0x02;                      // Enable VDD monitor as a reset source

   *pwrite = 0;                        // Initiate page erase

   PSCTL &= ~0x03;                     // PSWE = 0; PSEE = 0

   EA = EA_SAVE;                       // Restore interrupts

   SFRPAGE = SFRPAGE_save;
}

//-----------------------------------------------------------------------------
// End Of File
//-----------------------------------------------------------------------------