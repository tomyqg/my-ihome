/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief Meter Calibration routine
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

#ifndef METER_H
#define METER_H

#include <ioavr.h>
#include <math.h>
#include "clksys_driver.h"
#include "adc_driver.h"
#include "rtc_driver.h"
#include "meter_calculations.h"
#include "meter_calibration.h"
#include "meter_initialisation.h"
#include "meter_lcd_module.h"
#include "meter_rtc.h"
#include "avr_compiler.h"

#define RTC_CYCLES_1S     1024
#define	TAMPER_CURRENT_REVERSAL		PIN0_bm
#define TAMPER_EARTH_FAULT		PIN1_bm 
#define TAMPER_NEUTRAL_CUT		PIN3_bm
#define F_CPU                           32000000UL
#define F_TIMER                         500000UL            //F_TIMER = (F_CPU/64)
#define F_SAMPLING                      124                 //PER = F_TIMER/Sampling_freq - 1

#define POWERED_UP      	0 
#define POWER_OFF_DETECTED	1
#define POWER_OFF		2 
#define POWER_ON_DETECTED	2

extern uint8_t rtc_flag;
extern uint8_t cover_open_flag;


void tamper_check(void);


struct coefficient
{
  //! for Vrms scalling factor
  uint16_t volt_const;
  //! for Offset error correction in the phase line
  uint16_t shunt_offset[7];
  //! for Offset error in the watt calculation
  int32_t  watt_offset[7];
  //! energy
  float kwh;
  //! for Irms Scalling factor
  uint16_t shunt_const[7];
  //! for active power scaling factor
  float watt_const[7];
  //! for phase magnitude correction
  float A1[7];
  //! for phase angle correction
  float B1[7];
};
__eeprom extern struct coefficient eeprom_t;
extern struct coefficient meter_t;
#define eeprom  eeprom_t
#define meter  meter_t

////debug
int16_t adc_iadc_debug(void);
int16_t adc_vadc_debug(void);
extern int16_t debug_count, debug_count1;


#endif
