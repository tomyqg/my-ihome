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


uint8_t rtc_flag = 0, key_flag=0;
uint8_t magnetic_tamper_flag = 0;
uint8_t cover_open_flag = 0;
/*! \brie This function checks for different types of tampering such as earth fault
*   current reversal, neutral cut, magnetic tampering
*/
void tamper_check(void)
{
  uint16_t Irms_tamper_low, Irms_tamper_high;
  static uint8_t rev_current = 0, earth_fault = 0;
  //! check for earth fault tamper (Load earthing)  \n
  //! compare the phase current is within 12% of the neutral current
  Irms_tamper_low = Irms[0] - ((Irms[0]*64)>>8);            
  Irms_tamper_high = Irms[0] + ((Irms[0]*64)>>8);
  if ((Nrms < Irms_tamper_low) | (Nrms > Irms_tamper_high))
  {
    earth_fault++;
  }
  else
  {
    earth_fault = 0;
  }
 //! if tampering persist for 15 secs update the led status
  if(earth_fault > 15)      //checking for 15 seconds
  {
    PORTB.OUTSET  = TAMPER_EARTH_FAULT;
    earth_fault   = 15;
  }
  else
  {
    PORTB.OUTCLR  = TAMPER_EARTH_FAULT;
  }
  
  //! check the status for Current Reversal tampering
  if (active_energy_signed  < -5 || neutral_power < -5)     //comparing the sign of neutral and pahse power
  {	rev_current = 1;	}
  else
  {	rev_current = 0;		}
  //! if tampering detected toggle the TAMPER_CURRENT_REVERSAL LED
  if (rev_current == 1)
  {	PORTB.OUTTGL	= TAMPER_CURRENT_REVERSAL;		}   // toggle the LED on current reversal detection
  else
  {	PORTB.OUTCLR	= TAMPER_CURRENT_REVERSAL;		}
  //! check the status for Neutral CUT tampering and configure the status led
  if ((Vrms[0] == 0) && (Irms[0] > 500))
  {	PORTB.OUTSET	= TAMPER_NEUTRAL_CUT;			}
  else
  {	PORTB.OUTCLR	= TAMPER_NEUTRAL_CUT;			}
  //! check the status of Magnetic tamper flag and update the LED
  if(magnetic_tamper_flag == 1)
  {
    lcd_read_write(LCD_DATA_READ_WRITE,24,1);
  }
}



/*! \brief This function is executed whenever the LCD key is pressed
*
*  \param key_flag is incremented on each press and is cleared on reaching
*   the top value
*/ 
ISR(PORTB_INT0_vect)
{
        SLEEP.CTRL &=  ~SLEEP_SEN_bm;
	PORTB.INT0MASK	= 0x00;
	key_flag++;
	if (key_flag > 10)
	{
		key_flag = 0;
	}
	delay_ms(50);           // to avoid switch bouncing
	PORTB.INTFLAGS = 0x00;
	PORTB.INT0MASK = PIN2_bm;
        init_lcd();	
}

/*! \brief This function is executed on key press
*
*  \param key_flag is incremented on each press and is cleared on reaching
*   the top value
*/
ISR(PORTC_INT0_vect)
{
  SLEEP.CTRL &=  ~SLEEP_SEN_bm;
  uint8_t pin_val = 0;
  PORTC.INT0MASK	= 0x00;
  delay_ms(20);
  pin_val = PORTC.IN;
  //! check for cover open pin status
  if((pin_val & PIN2_bm) == 0)      // cover open interrupt, currently used to store energy value to eeprom for debugging
   {
      cover_open_flag ^= 0x1;
   }
  //! check for magnetic tampering
  else if((pin_val & PIN4_bm) == 0x10)   
  {
    magnetic_tamper_flag = 1;
  }
  //! check if PIN5_bm is pressed
  else if((pin_val & PIN5_bm) == 0)        //currently used to change the display for testing
  {
    key_flag--;
    if (key_flag == 255)
    {
      key_flag = 10;
    }
  }  
  PORTC.INTFLAGS = 0x00;
  PORTC.INT0MASK = PIN2_bm |PIN4_bm |PIN5_bm;	
}


/*! \brief This function is executed whenever the power off is detected
*   It will switch on the battery and CPU goes to power save mode
*
*  \param key_flag is incremented on each press and is cleared on reaching
*   the top value
*/
ISR(PORTD_INT0_vect)
{
    SLEEP.CTRL    &=  ~SLEEP_SEN_bm;
    PORTD.PIN4CTRL	= PORT_OPC_WIREDAND_gc;
    //PORTD.OUTCLR  = PIN4_bm;
    __watchdog_reset();
}
