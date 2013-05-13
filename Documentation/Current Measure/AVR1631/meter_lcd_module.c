/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief Meter initialisation
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
__eeprom  RTC_BCD_t eepromTime @ 0x02;
//! lookup table displaying the numbers to the lcd module
        		  // 0	1  2  3   4  5  6  7   8   9   A   B  C   D  E  F
uint16_t number_arr[16] = {125,96,62,122,99,91,95,112,127,123,119,127,29,125,31,23};

/*! brief This function call the LCD display function based on the value in key_flag variable
*
*     key_flag            function
*         0               lcd_disp_time
*         1               lcd_disp_date
*         2               lcd_disp_voltage
*         3               lcd_disp_current
*         4               lcd_disp_active_power
*         5               lcd_disp_apparent_power
*         6               lcd_disp_power_factor
*         7               lcd_disp_energy
*         8               lcd_disp_max_demand
*         9               lcd_disp_frequency
*         10              lcd_show_all
*/
void lcd_disp_key(void)
{
  //! clear the lcd display
  lcd_clear_all();
  //! shows the LCD page number
  lcd_read_write(LCD_DATA_READ_WRITE,18,number_arr[key_flag]);
  //! call the appropriate function based on the input from key_flag
  switch(key_flag)
  {
      case 0x00:
              lcd_disp_time();
              break;
      case 0x01:
              lcd_disp_date();
              break;
      case 0x02:
              lcd_disp_voltage();
              break;
      case 0x03:
              lcd_disp_current();
              break;
      case 0x04:
              lcd_disp_active_power();
              break;
      case 0x05:
              lcd_disp_apparent_power();
              break;
      case 0x06:
              lcd_disp_power_factor(power_factor);
              break;
      case 0x07:
              lcd_disp_energy();
              break;
      case 0x08:
              lcd_disp_max_demand();
              break;
      case 0x09:
              lcd_disp_frequency(frequency);
              break;
      case 0x0A:
              lcd_show_all();
              break;	              
      default:
              lcd_disp_time();			
  }
  
}

/*! brief This function display the date in LCD
*
*/
void lcd_disp_date(void)
{
	uint32_t disp_time = 0;
	uint16_t disp_temp = 0;
	disp_time =  (uint32_t)rtcTime.day*10000;
	disp_time += (uint32_t)rtcTime.month*100;
	disp_time += (uint32_t)rtcTime.year%100;
	
	for (uint8_t k = 14;k > 3;k-=2)
	{
		disp_temp	=	disp_time % 10;
		disp_time	/=	10;
		lcd_data(LCD_DATA_WRITE,k,number_arr[disp_temp]);    // display the date
	}
	delay_us(1);
	lcd_read_write(LCD_DATA_READ_WRITE,6,128);                        // dispaly ':'
	delay_us(1);
	lcd_read_write(LCD_DATA_READ_WRITE,10,128);                       // display ':'
	delay_us(1);
	lcd_read_write(LCD_DATA_READ_WRITE,20,number_arr[0]);             // display 'D'
}

/*! brief This function display the time in LCD
*
*/
void lcd_disp_time(void)
{
	uint32_t disp_time = 0;
	uint8_t disp_temp = 0;
	disp_time = (int32_t)rtcTime.hr*10000;
	disp_time += (int32_t)rtcTime.min*100;
	disp_time += (int32_t)rtcTime.sec;
	
	for (uint8_t k = 14;k > 3;k-=2)
	{
		disp_temp	=   (uint8_t)(disp_time % 10);
		disp_time	=   (disp_time / 10);
		lcd_data(LCD_DATA_WRITE,k,number_arr[disp_temp]);
	}
	delay_us(1);
	lcd_read_write(LCD_DATA_READ_WRITE,6,128);                        // dispaly ':'
	delay_us(1);
	lcd_read_write(LCD_DATA_READ_WRITE,10,128);                       // dispaly ':'
	delay_us(1);
	lcd_read_write(LCD_DATA_READ_WRITE,22,8);                         // dispaly 'T'
}

/*! brief This function display the energy in LCD
*
*/
void lcd_disp_energy(void)
{
	uint16_t disp_temp = 0;
        uint32_t energy;
        energy = (uint32_t)(meter.kwh*100000);
	lcd_data(LCD_DATA_WRITE,26,3);		            //display kwh 
	for (uint8_t k =14;k>1;k-=2)
	{
		disp_temp = energy % 10;
		energy /= 10;
		lcd_data(LCD_DATA_WRITE,k,number_arr[disp_temp]);
	}
	lcd_read_write(LCD_DATA_READ_WRITE,4,128);                    //6,8,128 display '.'
}

/*! brief This function display the power factor in LCD
*
*/
void lcd_disp_power_factor(int16_t pf)
{
	uint16_t disp_temp = 0;
	lcd_read_write(LCD_DATA_READ_WRITE,24,64);		//disp pf
	for (uint8_t k =14;k>7;k-=2)
	{
		disp_temp = pf % 10;
		pf /= 10;
		lcd_data(LCD_DATA_WRITE,k,number_arr[disp_temp]);
	}
	lcd_read_write(LCD_DATA_READ_WRITE,8,128);
}

/*! brief This function display the line frequency in LCD
*
*/
void lcd_disp_frequency(uint16_t freq)
{
	uint16_t disp_temp = 0;
	for (uint8_t k =12;k>5;k-=2)
	{
		disp_temp = freq % 10;
		freq /= 10;
		lcd_data(LCD_DATA_WRITE,k,number_arr[disp_temp]);
	}
	lcd_read_write(LCD_DATA_READ_WRITE,8,128);
}


/*! brief This function display the active power in LCD
*
*/
void lcd_disp_active_power(void)
{
	uint8_t disp_temp;
        uint16_t act_power;
        
        act_power = (int32_t)active_power[0];
        lcd_data(LCD_DATA_WRITE,26,1);		
	for (uint8_t k =14;k>5;k-=2)
	{
		disp_temp = act_power % 10;
		act_power /= 10;
		lcd_data(LCD_DATA_WRITE,k,number_arr[disp_temp]);
	}
        lcd_read_write(LCD_DATA_READ_WRITE,8,128);
}

/*! brief This function display the max demand in LCD
*
*/
void lcd_disp_max_demand(void)
{
	uint8_t disp_temp;
        uint16_t act_power;
        
        act_power = (int32_t)max_demand;
        lcd_data(LCD_DATA_WRITE,26,1);
	for (uint8_t k =14;k>5;k-=2)
	{
		disp_temp = act_power % 10;
		act_power /= 10;
		lcd_data(LCD_DATA_WRITE,k,number_arr[disp_temp]);
	}
        lcd_read_write(LCD_DATA_READ_WRITE,8,128);
        lcd_read_write(LCD_DATA_READ_WRITE,2,128);
}



/*! brief This function display the apparent power in LCD
*
*/
void lcd_disp_apparent_power(void)
{
	uint8_t disp_temp = 0;
	uint16_t app_power;

        app_power = apparent_power;
        lcd_data(LCD_DATA_WRITE,26,48);
	for (uint8_t k =14;k>5;k-=2)
	{
          disp_temp = app_power % 10;
	  app_power = app_power / 10;
	  lcd_data(LCD_DATA_WRITE,k,number_arr[disp_temp]);
	}
        lcd_read_write(LCD_DATA_READ_WRITE,8,128);
}


/*! brief This function display the voltage in LCD
*
*/
void lcd_disp_voltage(void)
{
	uint8_t disp_temp = 0;
        uint16_t volt;
        
	volt = Vrms[0];
        lcd_data(LCD_DATA_WRITE,26,16);	
	for (uint8_t k =14;k>3;k-=2)
	{
          disp_temp = (uint8_t)(volt % 10);
	  volt = (volt / 10);
	  lcd_data(LCD_DATA_WRITE,k,number_arr[disp_temp]);
	}
	lcd_read_write(LCD_DATA_READ_WRITE,4,128);		//disp "."
}

/*! brief This function display the current in LCD
*
*/
void lcd_disp_current(void)
{
	uint8_t disp_temp = 0;
        uint16_t amp;
        amp = (uint16_t)Irms[0];
	lcd_data(LCD_DATA_WRITE,26,32);		//disp A
	for (uint8_t k =14;k>5;k-=2)
	{
          disp_temp = (uint8_t)(amp % 10);
	  amp = amp / 10;
	  lcd_data(LCD_DATA_WRITE,k,number_arr[disp_temp]);
	}
	lcd_read_write(LCD_DATA_READ_WRITE,8,128);		//disp "."
}


/*! brief This function writes the command 'cmd2' to the LCD 
* please refer the lcd datasheet for details
*/
void lcd_command(uint8_t cmd1, uint8_t cmd2)
{
	int cmd_count;
	cmd1 <<= 5;
	LCD_CS_CLEAR;
	asm("nop");
	delay_us(100);
	for (cmd_count = 0;cmd_count < 3;cmd_count++)
	{
		LCD_WRITE_CLEAR;
		delay_us(100);
		LCD_DATA_OUT((cmd1 & 0x80)>>7);
		cmd1	<<=	1;
		delay_us(100);
		LCD_WRITE_SET;
		delay_us(100);
	}
	
	for (cmd_count = 0;cmd_count < 9;cmd_count++)
	{
		LCD_WRITE_CLEAR;
		delay_us(100);
		LCD_DATA_OUT((cmd2 & 0x80)>>7);
		cmd2 <<= 1;
		delay_us(100);
		LCD_WRITE_SET;
		delay_us(100);
	}
	LCD_CS_SET;
	delay_us(100);
}

/*! brief This function write the data 'lr_data' to the address lr_addr
*  please refer the LCD datasheet for details
*/
void lcd_data(uint8_t cmd, uint8_t lr_addr, uint8_t lr_data)
{
	int cmd_count;
	cmd <<= 5;
	LCD_CS_CLEAR;
	asm("nop");
	asm("nop");
	for (cmd_count = 0;cmd_count < 3;cmd_count++)
	{
		LCD_WRITE_CLEAR;
		asm("nop");
		asm("nop");
		LCD_DATA_OUT((cmd & 0x80)>>7);
		cmd	<<=	1;
		asm("nop");
		asm("nop");
		LCD_WRITE_SET;
		asm("nop");
		asm("nop");
	}
	lr_addr <<=2;
	for (cmd_count = 0;cmd_count < 6 ;cmd_count++)
	{
		LCD_WRITE_CLEAR;
		asm("nop");
		asm("nop");
		LCD_DATA_OUT((lr_addr & 0x80)>>7);
		lr_addr <<= 1;
		asm("nop");
		asm("nop");
		LCD_WRITE_SET;
		asm("nop");
		asm("nop");
	}
	for (cmd_count = 0;cmd_count < 9 ;cmd_count++)
	{
		LCD_WRITE_CLEAR;
		asm("nop");
		asm("nop");
		LCD_DATA_OUT(lr_data & 0x01);
		lr_data >>= 1;
		asm("nop");
		asm("nop");
		LCD_WRITE_SET;
		asm("nop");
		asm("nop");
	}
	LCD_CS_SET;
	asm("nop");
	asm("nop");
	
}

/*! brief This function reads the address 'lr_addr' and add the 'lr_data' 
*   to the read data. The result is then written to the address 'lr_addr' 
*/
void lcd_read_write(uint8_t cmd, uint8_t lr_addr, uint8_t lr_data)
{
	int cmd_count = 0;
	uint16_t rd = 0;
	uint8_t lr_addr_b;
	cmd <<= 5;
	lr_addr_b = lr_addr;
	LCD_CS_CLEAR;
	delay_us(1);
	for (cmd_count = 0;cmd_count < 3;cmd_count++)
	{
		LCD_WRITE_CLEAR;
		delay_us(1);
		LCD_DATA_OUT((cmd & 0x80)>>7);
		cmd	<<=	1;
		delay_us(1);
		LCD_WRITE_SET;
		delay_us(1);
	}
	lr_addr <<=2;
	for (cmd_count = 0;cmd_count < 6 ;cmd_count++)
	{
		LCD_WRITE_CLEAR;
		delay_us(1);
		LCD_DATA_OUT((lr_addr & 0x80)>>7);
		lr_addr <<= 1;
		delay_us(1);
		LCD_WRITE_SET;
		delay_us(1);
	}
	LCD.DIRCLR = LCD_DATA;
	//LCD.PIN0CTRL = PORT_OPC_TOTEM_gc ;
	for (cmd_count = 0;cmd_count < 8 ;cmd_count++)
	{
		LCD_READ_CLEAR;
		delay_us(1);
		rd |= (LCD.IN & 0x01)<<cmd_count; 
		delay_us(1);
		LCD_READ_SET;
		delay_us(1);
	}
	LCD_CS_SET;
	LCD.DIRSET = LCD_DATA;
	delay_us(1);	
	rd += lr_data;
	lcd_data(LCD_DATA_WRITE,lr_addr_b,rd);
}


/*! brief This function clear the lcd
*
*/
void lcd_clear_all(void)
{
	for (int i = 0; i <= 0x1C; i+=2)
	{
		lcd_data(LCD_DATA_WRITE,i,0x00);
		asm("nop");
		asm("nop");
	}
}

/*! brief This function sets all the segments in the display
*
*/
void lcd_show_all(void)
{
	for (int i = 0; i <= 0x1C; i+=2)
	{
		lcd_data(LCD_DATA_WRITE,i,0xFF);
		asm("nop");
		asm("nop");
	}
}
