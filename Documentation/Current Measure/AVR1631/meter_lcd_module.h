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

#ifndef METER_LCD_MODULE_H
#define METER_LCD_MODULE_H

#define LCD				PORTE
#define LCD_READ		8		 //LCD_pin2	PE3
#define LCD_CS			4		 //LCD_pin1	PE2
#define LCD_WRITE		2		 //LCD_pin3	PE1
#define LCD_DATA		1		 //LCD_pin4	PE0

#define LCD_CS_SET		LCD.OUTSET|=LCD_CS;
#define LCD_CS_CLEAR	        LCD.OUTCLR=LCD_CS;

#define LCD_WRITE_SET		LCD.OUTSET|=LCD_WRITE;
#define LCD_WRITE_CLEAR		LCD.OUTCLR=LCD_WRITE;

#define LCD_READ_SET		LCD.OUTSET|=LCD_READ;
#define LCD_READ_CLEAR		LCD.OUTCLR=LCD_READ;

#define LCD_DATA_SET		LCD.OUTSET|=LCD_DATA;
#define LCD_DATA_OUT(a)		LCD.OUT = (LCD.OUT & ~LCD_DATA) | a;

#define LCD_COMMAND             0x04
#define LCD_DATA_WRITE          0x05
#define LCD_DATA_READ_WRITE     0x06

void lcd_disp_key(void);
void lcd_disp_date(void);
void lcd_disp_time(void);
void lcd_disp_active_power(void);
void lcd_disp_max_demand(void);
void lcd_disp_energy();
void lcd_disp_power_factor(int16_t);
void lcd_disp_frequency(uint16_t);
void lcd_disp_apparent_power();
void lcd_command(uint8_t, uint8_t);
void lcd_data(uint8_t, uint8_t, uint8_t);
void lcd_clear_all(void);
void lcd_show_all(void);
void lcd_disp_voltage(void);
void lcd_disp_current(void);
void lcd_read_write(uint8_t, uint8_t, uint8_t);

extern uint16_t number_arr[16];
extern uint8_t key_flag;

#endif