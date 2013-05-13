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

int main(void)
{
  //! set the power reduction register to minimize current consumption
  init_power_reduction();
  //! set CPU clock to 32MHz and enables the DFLL32MHz
  clock_init();           
  //! initialize the LCD module
  init_lcd();
  //! display all the segments in LCD
  lcd_show_all();        
  //! initialize the GPIO pins
  init_ioport();
  //! initializes the UART
  init_UART();
  //! load the calibration data from flash
  ADC_CalibrationValues_Load(&ADCA);
  //! get the ADC offset value
  get_offset();
  //! initialize the ADC
  init_ADC();
  //! load the calibration value from eeprom
  init_eeprom();
  //! initilize RTC to external clock and 1 sec interrupt
  rtc_init();
  //! initilize the timer
  init_timer();  

  while(1)
    {
      //! rtc_flag is set
      if (rtc_flag == 1)
      {
        //! perform the calculation of metering paramters
        calculate();
        //for debugging
        if(cover_open_flag ==1)
        PORTD.OUTTGL = PIN3_bm;
        //TCC1.CTRLA = ( TCC1.CTRLA & ~TC1_CLKSEL_gm ) | TC_CLKSEL_DIV64_gc;
        TCC1.INTCTRLA = (TCC1.INTCTRLA & ~(TC1_OVFINTLVL_gm | TC1_ERRINTLVL_gm))|TC1_OVFINTLVL0_bm;
        //! if power on detected
        if(power_status_flag == POWER_ON_DETECTED) 
        {
          //! change the clock frequency to 32MHz
          CLKSYS_Prescalers_Config( CLK_PSADIV_1_gc, CLK_PSBCDIV_1_1_gc );
          //! switch off the battery
          PORTD.PIN4CTRL	= PORT_OPC_WIREDANDPULL_gc;
          //PORTD.OUTSET	= PIN4_bm;
          //! initialize gpio pins
          init_ioport();
          //! change the sleep mode to ideal sleep
          SLEEP.CTRL = (SLEEP.CTRL & ~SLEEP_SMODE_gm)| SLEEP_SMODE_IDLE_gc;
          //! enable ADC & Timer
          ADC_Enable(&ADCA);
          TCC1.CTRLA = ( TCC1.CTRLA & ~TC1_CLKSEL_gm ) | TC_CLKSEL_DIV64_gc;
          //! initialize lcd module
          init_lcd();
          //! update power status flag
          power_status_flag = POWERED_UP;
        }
        //! update the LCD display
        lcd_disp_key();
        tamper_check();
        if( active_power[0] > max_demand )
        {
          max_demand = active_power[0];
        }
        //! check for calibration flag and proceed and call the appropriate calibration funtion
        if(calibration_flag != 0)
        {
          //! checking the calibration flag for calibrating the date & time
          if (calibration_flag == 1)
          { calibrate_time_date();  }
          //! checking the calibration flag for calibrating the voltage
          else if (calibration_flag == 2)
          { calibrate_voltage();    }
          //! checking the calibration flag for calculating the offset value
          else if (calibration_flag == 3)
          { calibrate_no_load();    }
          //! checking the calibration flag for calculating the phase angle variation
          else if (calibration_flag == 4)
          { calibrate_phase();	    }
          //! checking the calibration flag for calculating watt varition
          else if (calibration_flag == 5)
          { calibrate_watt();       }
          else if (calibration_flag == 6)
          //! checking the calibration flag for calibrating the current            
          { calibrate_current();    }
        }
        rtc_flag = 0;
        __watchdog_reset();
      }
      //! checking the power_status, if power off is detected
      else if(power_status_flag == POWER_OFF_DETECTED)
      {
        //! switch on the battery
        PORTD.PIN4CTRL	= PORT_OPC_WIREDAND_gc;
        //PORTD.OUTCLR  = PIN4_bm;
        //! change the sleep mode to power save mode
        SLEEP.CTRL = (SLEEP.CTRL & ~SLEEP_SMODE_gm)| SLEEP_SMODE_PSAVE_gc;  
        //! switch off the LCD
        lcd_command(LCD_COMMAND,0x02);
        //! change reset the gpio pin
        facilitatePowersaving();
        //! disable the ADC
        ADC_Disable(&ADCA);
        //! disable the timer
        TCC1.CTRLA = ( TCC1.CTRLA & ~TC1_CLKSEL_gm ) | TC_CLKSEL_OFF_gc;
        //! clear all the varilable used in for energy calculation
        meter_flush();
        //! reduce the cpu clock frequency to minimize power consumption
        CLKSYS_Prescalers_Config( CLK_PSADIV_64_gc, CLK_PSBCDIV_1_1_gc );
        //! update power status flag
        power_status_flag = POWER_OFF;
        __watchdog_reset();
      }
      //! goes to sleep
      SLEEP.CTRL |=  SLEEP_SEN_bm;
      asm("sleep");
    }
}



