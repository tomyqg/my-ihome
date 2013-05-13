
/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief Metering Algorithm implementation
 *
 *
 *
 * \par Application note:
 *      AVR1631: Energy Meter Reference Design with ATxmega32A4
 *
 * \par Documentation
 *      For comprehensive code documentation, supported compilers, compiler
 *      settings and supported devices see readme.html
 *
 * \author
 *      Atmel Corporation: http://www.atmel.com \n
 *      Support email: avr@atmel.com
 *
 * $Revision: 1.0     \n
 * $Date:   2012-07-01 10:10:10 +0530   \n 
 *
 * Copyright (c) 2008, Atmel Corporation All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. The name of ATMEL may not be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY AND
 * SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

#include "meter.h"

//! hold the status of mains power supply
uint8_t power_status_flag = 0;
//! lookup table for number of days in a month
const uint8_t days_lookup[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
//! structure for holding the time value
RTC_BCD_t rtcTime = {0,0,0,1,1,2012};


/*! brief This function initilize RTC to external clock and 1 sec interrupt.
*
*/
void rtc_init()
{
	CLK.RTCCTRL =  CLK_RTCSRC_TOSC_gc | CLK_RTCEN_bm;
	do {
	} while ( RTC_Busy() );
	RTC_Initialize( RTC_CYCLES_1S, 0, 0, RTC_PRESCALER_DIV1_gc );
        RTC.INTFLAGS = RTC_OVFIF_bm | RTC_COMPIF_bm;
	RTC_SetIntLevels( RTC_OVFINTLVL_HI_gc, RTC_COMPINTLVL_OFF_gc );
	PMIC.CTRL |= PMIC_HILVLEN_bm;        
}
	
/*! breif The RTC ISR occurs in every sec. The time and date is updated in this function and the rtc_flag is set
*  
*/
ISR(RTC_OVF_vect)
{
    SLEEP.CTRL &=  ~SLEEP_SEN_bm;
    uint8_t pin_read = 0;
    //! update the time and date
    update_time();
    //! check the status of mains power supply
    pin_read = PORTD.IN & PIN2_bm;
    //pin_read = PIN2_bm;
    if(pin_read == PIN2_bm)
    {
      //! if power is on, set the rtc_flag
      //TCC1.CTRLA = ( TCC1.CTRLA & ~TC1_CLKSEL_gm ) | TC_CLKSEL_OFF_gc; 
      TCC1.INTCTRLA = (TCC1.INTCTRLA & ~(TC1_OVFINTLVL_gm | TC1_ERRINTLVL_gm));
      rtc_flag = 1;
    }
    else
    {
      //! if power is off, clear the rtc_flag
      rtc_flag = 0;
      if(power_status_flag == POWERED_UP)
      power_status_flag = POWER_OFF_DETECTED;
    }
}


/*! breif This function updates the date and time
*
*/
void update_time()
{
        if ( ++rtcTime.sec > 59 )
	{
		rtcTime.sec = 0;
		rtcTime.min++;
	}
	if ( rtcTime.min > 59 ) 
	{
		rtcTime.min = 0;
		rtcTime.hr++;
	}
	if (rtcTime.hr > 23)
	{
		rtcTime.hr = 0;
		rtcTime.day++;
	}
	if (rtcTime.day > days_lookup[rtcTime.month-1] )
	{
		if (rtcTime.month == 2)
		{
			if (rtcTime.year % 4 == 0 && rtcTime.day == 29)
			{
				asm("nop");
			}
			else
			{
				rtcTime.day = 1;
				rtcTime.month++;
			}
		} 
		else
		{
		rtcTime.day = 1;
		rtcTime.month++;	
		}

	}
	if (rtcTime.month > 12)
	{
		rtcTime.year++;
		rtcTime.month = 1;
	}
}


