//-----------------------------------------------------------------------------
// C8051F930_lib.h
//-----------------------------------------------------------------------------
// Copyright (C) 2009 Silicon Laboratories, Inc.
// http://www.silabs.com
//
// Program Description:
//
// Driver for the 'F93x power management function.
//
// Target:         C8051F93x-C8051F90x
// Tool chain:     Generic
// Command Line:   None
//
// Release 1.0
//    -23 JUL 2009
//

//-----------------------------------------------------------------------------
// Includes
//-----------------------------------------------------------------------------

#include <smartclock.h>                // RTC Functionality
#include <power.h>                     // Power Management Functionality


//-----------------------------------------------------------------------------
// Global Configuration Options
//-----------------------------------------------------------------------------

#define SYSCLK  20000000

#define SMARTCLOCK_ENABLED   1