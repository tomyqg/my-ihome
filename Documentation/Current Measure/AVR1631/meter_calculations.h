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
#ifndef METER_CALCULATION_H
#define METER_CALCULATION_H

#include <math.h>

#define VOLT_GAIN		1
#define FREQUENCY_MAX           5000                        //100Hz, F_TIMER/100Hz 
#define FREQUENCY_MIN           25000                       //20Hz, F_TIMER/20Hz
#define LP_ORDER                6
#define ONE_PULSE_ENERGY_THRESHOLD (float)0.0003125
#define PULSE_ON_TIME    13

extern uint16_t Vrms[5], Irms[5],Nrms;
extern int16_t offset[7];
extern uint16_t adc_samples;
extern int32_t active_energy_signed,neutral_power;
extern uint16_t active_power[5],apparent_power;
extern int16_t power_factor;
extern int32_t volt_temp,ct_temp, shunt_temp;
extern uint8_t calibration_flag, calibration_count;
extern uint8_t gain_stage;
extern uint16_t frequency;
extern int64_t watts_sum, watts_sum_calib;
extern uint8_t freq_cnt,offset_cnt;
extern uint16_t max_demand;
extern float pulse_energy_2_5ms;

void calculate(void);
void calculate_freq(void);
void get_frequency(void);
int16_t lp_filter(int16_t *);
void meter_flush(void);


#endif

