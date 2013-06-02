/*******************************************************************************
*   $FILE:  main.c
*   Atmel Corporation:  http://www.atmel.com \n
*   Support email:  touch@atmel.com
******************************************************************************/

/*  License
*   Copyright (c) 2010, Atmel Corporation All rights reserved.
*
*   Redistribution and use in source and binary forms, with or without
*   modification, are permitted provided that the following conditions are met:
*
*   1. Redistributions of source code must retain the above copyright notice,
*   this list of conditions and the following disclaimer.
*
*   2. Redistributions in binary form must reproduce the above copyright notice,
*   this list of conditions and the following disclaimer in the documentation
*   and/or other materials provided with the distribution.
*
*   3. The name of ATMEL may not be used to endorse or promote products derived
*   from this software without specific prior written permission.
*
*   THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
*   WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
*   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY AND
*   SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT,
*   INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
*   (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
*   ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
*   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
*   THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/*----------------------------------------------------------------------------
                            compiler information
----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
                                include files
----------------------------------------------------------------------------*/
#include <avr/io.h>
#include <avr/interrupt.h>
#include "touch.h"
#include "touch_api.h"
/*----------------------------------------------------------------------------
                            manifest constants
----------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------
                            type definitions
----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
                                prototypes
----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
                            Structure Declarations
----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
                                    macros
----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
                                global variables
----------------------------------------------------------------------------*/
/* Timer period in msec. */
extern uint16_t qt_measurement_period_msec;

/*----------------------------------------------------------------------------
                                extern variables
----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
                                static variables
----------------------------------------------------------------------------*/

/* flag set by timer ISR when it's time to measure touch */
extern volatile uint8_t time_to_measure_touch;

/* current time, set by timer ISR */
extern volatile uint16_t current_time_ms_touch;

#if defined(__ATxmega32D4__)
/*============================================================================
Name    :   init_timer_isr
------------------------------------------------------------------------------
Purpose :   configure timer ISR to fire regularly
============================================================================*/

void init_timer_isr( void )
{
    /*  Set timer period    */
   TCC0.PER = TICKS_PER_MS * qt_measurement_period_msec;
   /*  select clock source */
   TCC0.CTRLA = (TOUCH_DATA_T)4;
   /*  Set Comparre A interrupt to low level   */
   TCC0.INTCTRLB = 1u;
   /*  enable low lever interrupts in power manager interrupt control  */
   PMIC.CTRL |= 1u;
}
/*============================================================================
Name    :   set_timer_period
------------------------------------------------------------------------------
Purpose :   changed the timer period runtime
Input   :   qt_measurement_period_msec
Output  :   n/a
Notes   :
============================================================================*/
void set_timer_period(uint16_t qt_measurement_period_msec)
{
	/*  set timer compare value (how often timer ISR will fire,set to 1 ms interrupt) */
  TCC0.PER = TICKS_PER_MS * qt_measurement_period_msec;
}

/*============================================================================
Name    :   CCP write helper function written in assembly.
------------------------------------------------------------------------------
Purpose :   This function is written in assembly because of the timecritial
operation of writing to the registers for xmega.
Input   :   address - A pointer to the address to write to.
value   - The value to put in to the register.
============================================================================*/
void CCPWrite( volatile uint8_t * address, uint8_t value )
{
   volatile uint8_t * tmpAddr = address;
#ifdef RAMPZ
   RAMPZ = 0;
#endif
   asm volatile(
                "movw r30,  %0"	"\n\t"
                "ldi  r16,  %2"	"\n\t"
                "out   %3, r16"	"\n\t"
                "st     Z,  %1"
                :
                : "r" (tmpAddr), "r" (value), "M" (CCP_IOREG_gc), "m" (CCP)
                : "r16", "r30", "r31"
               );
}

/*============================================================================
Name    :   init_system
------------------------------------------------------------------------------
Purpose :   initialise host app, pins, watchdog, etc
============================================================================*/
void init_system( void )
{
    uint8_t PSconfig;
   uint8_t clkCtrl;

   /*  Configure Oscillator and Clock source   */

   /*  Select Prescaler A divider as 4 and Prescaler B & C divider as (1,1) respectively.  */
   /*  Overall divide by 4 i.e. A*B*C  */
   PSconfig = (uint8_t) CLK_PSADIV_4_gc | CLK_PSBCDIV_1_1_gc;
   /*  Enable internal 32 MHz ring oscillator. */
   OSC.CTRL |= OSC_RC32MEN_bm;
   CCPWrite( &CLK.PSCTRL, PSconfig );
   /*  Wait until oscillator is ready. */
   while ( ( OSC.STATUS & OSC_RC32MRDY_bm ) == 0 );
   /*  Set the 32 MHz ring oscillator as the main clock source */
   clkCtrl = ( CLK.CTRL & ~CLK_SCLKSEL_gm ) | CLK_SCLKSEL_RC32M_gc;
   CCPWrite( &CLK.CTRL, clkCtrl );

   /*  Route clk signal to port pin    */
   /*  PORTCFG_CLKEVOUT = 0x03;    */
   /*  PORTE_DIRSET = 0x80;    */

}

ISR(TCC0_CCA_vect)
{
    /*  set flag: it's time to measure touch    */
    time_to_measure_touch = 1u;

    /*  update the current time  */
    current_time_ms_touch += qt_measurement_period_msec;
}
#endif





