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
#ifndef METER_CALIBRATION_H
#define METER_CALIBRATION_H


//! \brief Select the USART to be used
#define USART   	USARTC1

//! \brief Select which Port is used as USART.
#define USART_PORT      PORTC

//! \brief Configure USART Scale factor. Here,Selected as Zero
#define UART_BSCALE_VALUE  0          


/*!
 * \brief Configure for Baud Rate Value  
 *        E.g.:Target: Internal RC 32MHz (default) 
 *        For 38400bps, UBRR = 32MHz/(16*38400)-1 = 51.08
 */   
#define UART_BSEL_VALUE     51

//! \brief Define UART Character Size
#define UART_CHARACTER_SIZE     USART_CHSIZE_8BIT_gc 

//! \brief Define UART Parity Mode 
#define UART_PARITY_MODE_SIZE    USART_PMODE_DISABLED_gc

//! \brief Define OMEGA = (2 * pi * (Freq / Fsampling) )
#define OMEGA   (float)0.07853982                         

extern uint8_t rx_buffer[10];


void calibrate_current(void);
void calibrate_no_load(void);
void calibrate_phase(void);
void calibrate_time(void);
void calibrate_time_date(void);
void calibrate_date(void);
void calibrate_voltage(void);
void calibrate_watt(void);
void phase_calc(float, uint8_t);
void read_tx_array(uint8_t cnt);
void uart_getdata(uint8_t );
void uart_print(void);


#endif
