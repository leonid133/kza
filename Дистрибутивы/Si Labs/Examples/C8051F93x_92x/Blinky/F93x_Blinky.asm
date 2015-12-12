$NOMOD51
;-----------------------------------------------------------------------------
; F93x_Blinky.asm
;-----------------------------------------------------------------------------
; Copyright 2007 Silicon Laboratories, Inc.
; http://www.silabs.com
;
; Program Description:
;
; This program illustrates how to disable the watchdog timer,
; configure the Crossbar, configure a port and write to a port
; I/O pin.
;
;
; How To Test:
;
; 1) Download code to a 'F93x target board
; 2) Ensure that pins 2 and 3 are shorted together on the J8 header
; 3) Run the code and if the P1.6 LED blinks, the code works
;
;
; Target:         C8051F93x-C8051F92x
; Tool chain:     Keil C51 7.50 / Keil EVAL C51
; Command Line:   None
;
; Release 1.1 
;    - Compiled and tested for C8051F930-TB (JH)
;    - 06 JULY 2009
;
; Release 1.0
;    - Initial Revision (FB)
;    - 04 OCT 2007
;

$include (C8051F930.inc)               ; Include register definition file.

;------------------------------------------------------------------------------
; EQUATES
;------------------------------------------------------------------------------

LED_ON       equ     0                 ; LED states
LED_OFF      equ     1

YELLOW_LED   equ   P1.6                ; Yellow LED: '0' is ON

;------------------------------------------------------------------------------
; RESET and INTERRUPT VECTORS
;------------------------------------------------------------------------------

            ; Reset Vector
            cseg AT 0
            ljmp Main                  ; Locate a jump to the start of
                                       ; code at the reset vector.

;------------------------------------------------------------------------------
; CODE SEGMENT
;------------------------------------------------------------------------------


Blink       segment  CODE

            rseg     Blink             ; Switch to this code segment.
            using    0                 ; Specify register bank for the
                                       ; following program code.

Main:
            ; Disable the WDT.
            anl   PCA0MD,  #NOT(040h)   ; Clear Watchdog Enable bit

            ; Initialize the Port I/O Crossbar
            orl   P1SKIP,  #40h        ; Skip the LED pin from crossbar
            orl   P1MDOUT, #40h        ; Make LED pin output push-pull
            mov   XBR0,    #00h        ; No digital peripherals selected
            mov   XBR2,    #40h        ; Enable Crossbar

            ; Initialize LED to OFF
            clr   YELLOW_LED

            ; Simple delay loop.
Loop2:      mov   R7, #03h
Loop1:      mov   R6, #00h
Loop0:      mov   R5, #00h
            djnz  R5, $
            djnz  R6, Loop0
            djnz  R7, Loop1
            cpl   YELLOW_LED            ; Toggle LED.
            jmp   Loop2

;------------------------------------------------------------------------------
; End of file.

END

