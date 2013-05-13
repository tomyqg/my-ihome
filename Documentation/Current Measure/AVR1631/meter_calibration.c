
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

#include "meter.h"

uint8_t rx_buffer[10] = {0},rx_complete_flag = 0,tx_count=0,rx_count = 0;
uint8_t tx_read[31], calibration_flag = 0, calibration_count;
__flash unsigned char tx_array[7][31] =	{
                                  {"                             \r\n"},
                                  {"\r\nEnter the Time            \r\n"},
                                  {"\r\nEnter the Date            \r\n"},
                                  {"\r\nEnter the voltage?        \r\n"},
                                  {"\r\nPut the meter in No Load? \r\n"},
                                  {"\r\nEnter the Watt value?     \r\n"},
                                  {"\r\nEnter the Current in Amps \r\n"}, };

/*! brief Tx interrupt vector call the function uart_print which sends the next data in the tx_read array 
*
*/

ISR(USARTC1_TXC_vect)
{
  SLEEP.CTRL &=  ~SLEEP_SEN_bm;  
  uart_print();
}

/*! brief  The UART RX ISR set the calibration_flag variable \n
*  press the switch and send the command through uart     \n
*  command t --> for calibrating date and time  \n
*  command v --> for calibrating voltage            \n
*  command n --> for offset calculation             \n
*  command i --> for current calibration            \n
*  command w --> for watt calibration               \n
*  command c --> for phase calibration              \n
*/
ISR(USARTC1_RXC_vect)
{
        SLEEP.CTRL &=  ~SLEEP_SEN_bm;  
	uint8_t uart_rx = 0;
	uart_rx = USART.DATA;
        //! //check whether the calibration switch is pressed during the command receive
        if ((PORTC.IN & PIN3_bm) == 0)    
        {
          rx_complete_flag = 0x00;
          if(uart_rx == 't') 
          {
            calibration_flag = 1;
          }
          else if (uart_rx == 'v')
          {
            calibration_flag = 2;
          }
          else if (uart_rx == 'n')
          {
            calibration_flag = 3;
          }
          else if (uart_rx == 'c')
          {
            calibration_flag = 4;
          }
          else if (uart_rx == 'w')
          {
            calibration_flag = 5;
          }
          else if (uart_rx == 'i')
          {
            calibration_flag = 6;
          }          
          else if(uart_rx == 13)          //checks whether the received character is carriage return
          {
            rx_complete_flag = 1;
          }
          else                          //store the recieve character in rx_buffer
          {
            rx_buffer[rx_count] = uart_rx-48;
            rx_count++;
          }
        // echo back the received signal
        while(USART.STATUS & USART_TXCIF_bm == 0 ){}
        USART.DATA = uart_rx;        
        }  
}

/*! brief This function send the character in tx_read array into uart buffer
* 
*/
void uart_print(void)
{
	USART.DATA = tx_read[tx_count];
        tx_count++;
        if(tx_count > 30)
        {
          //disable the tx interrupt  when all the data in the array is sent
          USART.CTRLA &= ~USART_TXCINTLVL_gm; 
          tx_count = 0;
        }
}

/*! brief This function collect the calibration data from the user through UART
*
*   \param rx_complete_flag is set on receiving complete data from the user
*/
void uart_getdata(uint8_t tx)
{
  //load the array to be printed in hyperterminal
  read_tx_array(tx);
  //clear the receive buffer
  rx_count = 0;
  for(uint8_t k = 0; k < 9; k++)
    rx_buffer[k] = 0;
  //enable transmit interrupt
  USART.CTRLA |= USART_TXCINTLVL0_bm;
  uart_print();
  while(rx_complete_flag == 0){__watchdog_reset();}
  rx_complete_flag = 0;
  while(USART.STATUS & USART_TXCIF_bm == 0 ){}
  USART.DATA = 10;
  while(USART.STATUS & USART_TXCIF_bm == 0 ){}
  USART.DATA = 13;
}

/*! brief This function load the array from flash
* 
*   \param tx_read[] holds the data to be transmitted through uart
*/
void read_tx_array(uint8_t cnt)
{
  for(uint8_t k = 0; k < 30; k++)
  {
    tx_read[k] = tx_array[cnt][k];
  }  
}

/*! brief This function updates the date and time received through UART,
*   RTC starts running with this new time
*/
void calibrate_time_date(void)
{
  uart_getdata(2);
  calibrate_date();  	
  uart_getdata(1);
  calibrate_time();
  calibration_flag = 0;
}

/*! brief This function updates the structure 'rtcTime' with the new time received through UART,
*   RTC starts running with this new time
*/
void calibrate_time(void)
{
	rtcTime.hr	= rx_buffer[0]*10+rx_buffer[1];
	rtcTime.min	= rx_buffer[3]*10+rx_buffer[4];
	rtcTime.sec	= rx_buffer[6]*10+rx_buffer[7];
        key_flag = 0;
}

/*! brief This function updates the structure 'rtcTime' with the new date received through UART,
*
*/
void calibrate_date(void)
{
	rtcTime.day		= rx_buffer[0]*10+rx_buffer[1];
	rtcTime.month	        = rx_buffer[3]*10+rx_buffer[4];
	rtcTime.year	        = rx_buffer[6]*1000+rx_buffer[7]*100+rx_buffer[8]*10+rx_buffer[9];
}


/*! brief This function take the average of current at noload for 30sec, caluculate
* the offset current value and stores the calibration value into eeprom
*
* /param calibration_temp accumulates the raw current value for calibration
* /param calibration_watt accumulates the raw active power value for calibration
*/
void calibrate_no_load(void)
{
	static float calibration_temp = 0;
        static int64_t calibration_watt = 0;
	if (calibration_count < 1)
	{
          uart_getdata(4);
          calibration_temp = 0;          
          if(rx_buffer[0] != 0x49)
          {
            calibration_flag = 0;
            calibration_count = 0xFF;
          }
          meter.shunt_offset[6] = 0;
          meter.watt_offset[6]  = 0;
	  gain_stage = 6;
          offset_cnt = 6;          
	}
        else
        {
          calibration_temp = calibration_temp + (float)shunt_temp;
          calibration_watt = calibration_watt + (int64_t)watts_sum_calib;
        }
        calibration_count++;			 
        while(USART.STATUS & USART_TXCIF_bm == 0 ){}
        USART.DATA = '.';
	if (calibration_count == 30)
	{
		calibration_temp /= calibration_count;
		meter.shunt_offset[6] = 	(uint16_t)calibration_temp;
        eeprom.shunt_offset[6] = meter.shunt_offset[6];
        calibration_watt  /= (calibration_count);
        meter.watt_offset[6]  = -1*(int32_t)calibration_watt;
        eeprom.watt_offset[6] = meter.watt_offset[6];
        calibration_watt = 0;                
        while(USART.STATUS & USART_TXCIF_bm == 0 ){}
        USART.DATA = 10;
        while(USART.STATUS & USART_TXCIF_bm == 0 ){}
        USART.DATA = 13;        
		calibration_temp = 0;
		calibration_flag = 0;
		calibration_count = 0;
	}
}

/*! brief This function take the average of voltage for 30sec, caluculate
* the volt constant and stores the calibration value into eeprom
*
* /param volt_input holds the input data from the user
* /param calibration_temp accumulates the raw voltage value for calibration
*/
void calibrate_voltage(void)
{
	static float calibration_temp = 0, volt_input = 0;
        
	if (calibration_count < 1)
	{
          //uart_getdata((unsigned char *)&tx_array[3][0]);
          uart_getdata(3);
          calibration_temp = 0;
    	  volt_input = (float) (rx_buffer[0]*100 + rx_buffer[1]*10 + rx_buffer[2]);
	  volt_input += (float) rx_buffer[4]/10 + (float)rx_buffer[5]/100;	 
	}
        calibration_count++;
        while(USART.STATUS & USART_TXCIF_bm == 0 ){}
        USART.DATA = '.';
	calibration_temp = calibration_temp + (float)volt_temp;
	if (calibration_count > 30)
	{
		calibration_temp /= calibration_count;	
		meter.volt_const = (uint16_t)(65536* (volt_input/sqrt(calibration_temp)));
        eeprom.volt_const = meter.volt_const;
		calibration_temp = 0;
		calibration_flag = 0;
		calibration_count = 0;
        while(USART.STATUS & USART_TXCIF_bm == 0 ){}
        USART.DATA = 10;
        while(USART.STATUS & USART_TXCIF_bm == 0 ){}
        USART.DATA = 13;        
	}	

}

/*! brief This function take the average of current for 30sec, caluculate
* the shunt constant and stores the calibration value into eeprom
*
* /param volt_input holds the input data from the user
* /param calibration_temp accumulates the raw voltage value for calibration
*/
void calibrate_current(void)
{
        float calib_temp;
        static float calibration_temp = 0, current_input = 0;
        static uint8_t cal_gain_stage = 0, cal_offset_cnt = 0;
	if (calibration_count < 1)
    {
        uart_getdata(6);
        calibration_temp = 0;            
        calib_temp  =  (float)(rx_buffer[0]*10 + rx_buffer[1]);
        calib_temp  += (float)rx_buffer[3]/10 + (float)rx_buffer[4]/100 + (float)rx_buffer[5]/1000;
        current_input = calib_temp;
        key_flag = 3;
        if(current_input < 0.500)
        {
          cal_gain_stage =6;
          cal_offset_cnt = 6;
        }
        else if(current_input > 0.500 && current_input < 1.500)
        {
          cal_gain_stage = 5;
          cal_offset_cnt = 6;
        }
        else if(current_input > 1.500 && current_input < 2.500)
        {
          cal_gain_stage = 4;
          cal_offset_cnt = 6;
        }
        else if(current_input > 2.500 && current_input < 4.000)
        {
          cal_gain_stage = 3;
          cal_offset_cnt = 6;
        }            
        else if(current_input > 4.000 && current_input < 7.000)
        {
          cal_gain_stage = 2;
          cal_offset_cnt = 6;
        }          
        else if(current_input > 7.000 && current_input < 14.500)
        {
          cal_gain_stage = 1;
          cal_offset_cnt = 6;
        }          
        else if(current_input > 14.500)
        {
          cal_gain_stage = 0;
          cal_offset_cnt = 5;
        }
        else
        {
          calibration_flag = 0;
          calibration_count = 0xFF; 
        }
    }
        else
        {
          calibration_temp = calibration_temp + (float)shunt_temp;
	}
        while(USART.STATUS & USART_TXCIF_bm == 0 ){}
        USART.DATA = '.';
        calibration_count++;
        gain_stage = cal_gain_stage;
        offset_cnt = cal_offset_cnt;
        if(cal_offset_cnt == 6 && offset_cnt!= 6)
        {
          ADC_Ch_InputMode_and_Gain_Config(&ADCA.CH0,
             ADC_CH_INPUTMODE_DIFFWGAIN_gc,
               ADC_CH_GAIN_64X_gc);
        }
        else if(cal_offset_cnt ==5 && offset_cnt!= 5)
        {
          ADC_Ch_InputMode_and_Gain_Config(&ADCA.CH0,
             ADC_CH_INPUTMODE_DIFFWGAIN_gc,
               ADC_CH_GAIN_32X_gc);          
        }
	if (calibration_count > 30)
	{
		calibration_temp /= (calibration_count-1);	
        meter.shunt_const[cal_gain_stage] = (uint16_t)(65536*(current_input/sqrt(calibration_temp)));
        eeprom.shunt_const[cal_gain_stage] = meter.shunt_const[cal_gain_stage];
        calibration_temp = 0;
		calibration_flag = 0;
		calibration_count = 0;
        while(USART.STATUS & USART_TXCIF_bm == 0 ){}
        USART.DATA = 10;
        while(USART.STATUS & USART_TXCIF_bm == 0 ){}
        USART.DATA = 13;        
	}
}

/*! brief This function take the average of active power for 30sec, caluculate
* the watt constant and stores the calibration value into eeprom
*
* /param watt_input holds the input data from the user
* /param calibration_watt accumulates the raw active power for calibration
*/
void calibrate_watt(void)
{
  	static int64_t calibration_watt = 0;
    static uint8_t cal_gain_stage = 0, cal_offset_cnt = 0;
	static uint16_t watt_input = 0;
    static float A1_temp, B1_temp;
    if (calibration_count < 1)
    {
        uart_getdata(5);
        calibration_watt = 0;
        watt_input  =  rx_buffer[0]*1000 + rx_buffer[1]*100 + rx_buffer[2]*10 + rx_buffer[3];
        key_flag = 4;
        if(Irms[0] < 500)
        {
          cal_gain_stage =6;
          cal_offset_cnt = 6;
        }
        else if(Irms[0] > 500 && Irms[0] < 1500)
        {
          cal_gain_stage = 5;
          cal_offset_cnt = 6;
        }
        else if(Irms[0] > 1500 && Irms[0] < 2500)
        {
          cal_gain_stage = 4;
          cal_offset_cnt = 6;
        }
        else if(Irms[0] > 2500 && Irms[0] < 4000)
        {
          cal_gain_stage = 3;
          cal_offset_cnt = 6;
        }            
        else if(Irms[0] > 4000 && Irms[0] < 7000)
        {
          cal_gain_stage = 2;
          cal_offset_cnt = 6;
        }          
        else if(Irms[0] > 7000 && Irms[0] < 14500)
        {
          cal_gain_stage = 1;
          cal_offset_cnt = 6;
        }          
        else if(Irms[0] > 14500)
        {
          cal_gain_stage = 0;
          cal_offset_cnt = 5;
        }
        else
        {
          calibration_flag = 0;
          calibration_count = 0xFF; 
        }
        A1_temp = meter.A1[cal_gain_stage];
        B1_temp = meter.B1[cal_gain_stage];
        meter.A1[cal_gain_stage] = 1;
        meter.B1[cal_gain_stage] = 0;
	}
    else
    {
      calibration_watt = calibration_watt + (int64_t)active_energy_signed;          
	}
    while(USART.STATUS & USART_TXCIF_bm == 0 ){}
    USART.DATA = '.';
    calibration_count++;
    if(cal_offset_cnt == 6  && offset_cnt!= 6)
    {
      ADC_Ch_InputMode_and_Gain_Config(&ADCA.CH0,
         ADC_CH_INPUTMODE_DIFFWGAIN_gc,
           ADC_CH_GAIN_64X_gc);
    }
    else if(cal_offset_cnt ==5  && offset_cnt!= 5)
    {
      ADC_Ch_InputMode_and_Gain_Config(&ADCA.CH0,
         ADC_CH_INPUTMODE_DIFFWGAIN_gc,
           ADC_CH_GAIN_32X_gc);          
    }
	if (calibration_count > 30)
	{
		calibration_watt /= (calibration_count-1);	
        meter.watt_const[cal_gain_stage] = (float)watt_input/abs(calibration_watt);
        eeprom.watt_const[cal_gain_stage] = meter.watt_const[cal_gain_stage];
        meter.A1[cal_gain_stage] = A1_temp;
        meter.B1[cal_gain_stage] = B1_temp;
        calibration_watt = 0;
		calibration_flag = 0;
		calibration_count = 0;
        while(USART.STATUS & USART_TXCIF_bm == 0 ){}
        USART.DATA = 10;
        while(USART.STATUS & USART_TXCIF_bm == 0 ){}
        USART.DATA = 13;        
	}
}

/*! brief This function take the average of power factor for 30sec, caluculate
* the phase constants and stores the calibration value into eeprom
*
* /param current_input holds the input data from the user
* /param calibration_temp accumulates the raw voltage value for calibration
* /param pf_temp calculates the actual power factor for phase calibration
*/
void calibrate_phase(void)
{
  	float calib_temp;
    static float calibration_temp = 0, current_input = 0;
    float pf_temp;
    static uint8_t cal_gain_stage = 0, cal_offset_cnt = 0;
	if (calibration_count < 1)
    {
        //uart_getdata((unsigned char *)&tx_array[6][0]);
        uart_getdata(6);
        calibration_temp = 0;
        calib_temp  =  (float)(rx_buffer[0]*10 + rx_buffer[1]);
        calib_temp  += (float)rx_buffer[3]/10 + (float)rx_buffer[4]/100 + (float)rx_buffer[5]/1000;
        current_input = calib_temp;
        key_flag = 6;
        if(current_input > 0.001 && current_input < 0.500)
        {
          cal_gain_stage =6;
          cal_offset_cnt = 6;
        }
        else if(current_input > 0.500 && current_input < 1.500)
        {
          cal_gain_stage = 5;
          cal_offset_cnt = 6;
        }
        else if(current_input > 1.500 && current_input < 2.500)
        {
          cal_gain_stage = 4;
          cal_offset_cnt = 6;
        }
        else if(current_input > 2.500 && current_input < 4.000)
        {
          cal_gain_stage = 3;
          cal_offset_cnt = 6;
        }            
        else if(current_input > 4.000 && current_input < 7.000)
        {
          cal_gain_stage = 2;
          cal_offset_cnt = 6;
        }          
        else if(current_input > 7.000 && current_input < 14.500)
        {
          cal_gain_stage = 1;
          cal_offset_cnt = 6;
        }          
        else if(current_input > 14.500)
        {
          cal_gain_stage = 0;
          cal_offset_cnt = 5;
        }
        else
        {
          calibration_flag = 0;
          calibration_count = 0xFF; 
        }
        meter.A1[cal_gain_stage] = 1;
        meter.B1[cal_gain_stage] = 0;
    }
    else
    {
        calibration_temp = calibration_temp + ((float)active_power[0]); 
	}
    while(USART.STATUS & USART_TXCIF_bm == 0 ){}
    USART.DATA = '.';
    calibration_count++;
    gain_stage = cal_gain_stage;
    offset_cnt = cal_offset_cnt;
    key_flag = 6;
	if (calibration_count > 60)
	{
		calibration_temp /= (calibration_count-1);
        pf_temp = calibration_temp / (float)(240 * current_input);
        phase_calc(pf_temp, cal_gain_stage);
        calibration_temp = 0;
		calibration_flag = 0;
		calibration_count = 0;
        while(USART.STATUS & USART_TXCIF_bm == 0 ){}
        USART.DATA = 10;
        while(USART.STATUS & USART_TXCIF_bm == 0 ){}
        USART.DATA = 13;        
	}  
  
}

/*! brief This function calculate the beta(B1) and A1 values
* 
*/
void phase_calc(float pf_avg, uint8_t g_stage)
{
  float phase_angle, g_delay, t1, t2, t3, beta, gain;
  phase_angle = acos(pf_avg) - acos(0.5);
  g_delay = phase_angle/OMEGA;
  
  t1 = (1-(2*g_delay)) *cos(OMEGA);
  t2= sqrt( pow((1-(2*g_delay)),2)  * (cos(OMEGA)*cos(OMEGA)) + 4*g_delay*(1-g_delay));
  t3 = 2*(1-g_delay);
  beta = -(t1-t2)/t3;
  eeprom.B1[g_stage] = beta;
  meter.B1[g_stage] = beta;
  gain = pow(pow( (cos(OMEGA) + beta),2) + pow(sin(OMEGA),2),-0.5);
  eeprom.A1[g_stage] = gain;
  meter.A1[g_stage] = gain;   
}


