
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

/*!
 * \brief variables used in metering calculations
 */
// STATIC VARIABLES DECLARATION
uint8_t gain_stage = 0;
uint32_t freq_sum = 0;
uint16_t frequency = 0, max_demand = 0;
uint16_t Vrms[5] = {0}, Irms[5] = {0}, Nrms = 0;
int16_t p_mean = 0, Vmean = 0, n_mean = 0;
int32_t phase_mean=0, neutral_mean=0, volt_mean=0;
uint64_t phase_sum = 0, volt_sum = 0, neutral_sum = 0;
int64_t watts_sum = 0, watts_sum_neutral = 0,watts_sum_calib = 0;
uint16_t adc_samples = 0;
int32_t active_energy_signed = 0, neutral_power = 0;
uint16_t active_power[5] = {0},apparent_power = 0;
int16_t power_factor, offset[7] = {0};
int32_t volt_temp,ct_temp, shunt_temp;
volatile int16_t adc_read_ch0, adc_read_ch1;
int16_t lp_phase[LP_ORDER] = {0}, lp_power[LP_ORDER] = {0}, lp_volt[LP_ORDER] = {0};
uint8_t freq_cnt,offset_cnt = 0;
const int16_t lp_coeff[LP_ORDER] ={5252,   5568,   5728,   5728,   5568,   5252};
float pulse_energy_2_5ms = 0;
uint8_t pulse_count = 0,On_Time_counter = 0;
float pulse_energy_acc = 0;

/*! \brief Timer/Counter 1 interrupt occur in every 250 uSec, (4KHz). 
*   The ISR routine enables ADC Conversion start on channel 0.
*   This function also perform the pulsing of the led for meter constant of 3200 impulse/kwh
*
*   \param pulse_energy_2_5ms hold the active energy for 2.5msec
*   \param pulse_energy_acc hold the accumulated energy on every 2.5msec
*   \param On_Time_counter hold the duration for which the pulse led to be ON
*/
ISR(TCC1_OVF_vect)
{
  SLEEP.CTRL &=  ~SLEEP_SEN_bm;
  //ADC_Ch_Conversion_Start(&ADCA.CH0);
  ADC_Conversions_Start(&ADCA,ADC_CH0START_bm | ADC_CH1START_bm);
  pulse_count++;
  if(pulse_count > 9)
  {
    pulse_count = 0;
    pulse_energy_acc = pulse_energy_acc + pulse_energy_2_5ms ;
    if(pulse_energy_acc >= ONE_PULSE_ENERGY_THRESHOLD)
    {
       pulse_energy_acc = pulse_energy_acc -  ONE_PULSE_ENERGY_THRESHOLD;
       // On the pulse LED
       PORTD.OUTSET = PIN0_bm;
       // On time counter 
       On_Time_counter = PULSE_ON_TIME;
    }
    
    if(On_Time_counter !=0)
    {
      On_Time_counter--;
      if(On_Time_counter == 0)
      {
        // Off the pulse LED
        PORTD.OUTCLR = PIN0_bm;
      }
    }
  }
}

/*! \brief ADC ISR channel 0 interrupt reads the phase value from shunt for Irms and energy calculation 
*
*   \param adc_read_ch0 hold the ADC channel result result register
*/	
ISR(ADCA_CH0_vect)			  //Phase - Shunt
{
  SLEEP.CTRL    &=  ~SLEEP_SEN_bm;
  adc_read_ch0  =  ADC_ResultCh_GetWord_Signed(&ADCA.CH0,offset[offset_cnt]);
}


/*! \brief ADC ISR channel 1 interrupt reads the voltage value for Vrms and energy calculation 
*  The ADC ISR also check for the sign change in voltage signal for frequency calucaulation.
*
*   \param adc_read_ch1 hold the ADC channel result register
*
*/
ISR(ADCA_CH1_vect)
{
  SLEEP.CTRL    &=  ~SLEEP_SEN_bm;
  adc_read_ch1  =   ADC_ResultCh_GetWord_Signed(&ADCA.CH1,offset[VOLT_GAIN]);
  get_frequency();
  ADC_Ch_Conversion_Start(&ADCA.CH2);
}

/*! \brief ADC ISR channel 2 interrupt reads the neutral value from CT and accumulate it for neutal current calculation 
*
*   \param p_mean subtract the DC offset value form the ADC_result register.
*   \param phase_sum contains the accumulated phase value
*   \param v_mean substract the DC offset value form the ADC_result register.
*   \param volt_sum contains the accumulated volt value
*   \param n_mean substract the DC offset value form the ADC_result register.
*   \param neutral_sum contains the accumulated neutral value
*   \param watts_sum contain the accumulated power value of active power calculation
*   \param lp_phase array contain the phase signal for low pass filtering
*   \param lp_volt array contain the volt signal for low pass filtering
*   \param lp_power array contain the instantaneous power signal for low pass filtering
*
*/
ISR(ADCA_CH2_vect)		  //CT
{
  SLEEP.CTRL &=  ~SLEEP_SEN_bm;
  int32_t i_temp, n_temp, v_temp;
  int16_t lp_powerout = 0;
  int16_t phase_read, neutral_read, volt_read, volt_read_filt1;
  static int16_t volt_read_filt2 = 0;
  neutral_read = ADC_ResultCh_GetWord_Signed(&ADCA.CH2, offset[1]);
  
  phase_mean += (int32_t)adc_read_ch0;
  lp_phase[0] = adc_read_ch0 - p_mean;
  // low pass filtering of the input signal
  phase_read =  lp_filter(lp_phase);
  
  //Removing DC offset from the ADC result.
  volt_mean += (int32_t)adc_read_ch1; 
  lp_volt[0] = adc_read_ch1 - Vmean;
  
  //Phase angle  correction  
  volt_read_filt1 = lp_filter(lp_volt);
  volt_read  = (int16_t)(meter.A1[gain_stage] * (volt_read_filt1 + meter.B1[gain_stage]*(float)volt_read_filt2));
  volt_read_filt2 = volt_read_filt1;
    
  //multiply and accumulating the phase value for energy calculation
  phase_sum += (uint64_t)((int32_t)phase_read * (int32_t)phase_read);        
    
  //multiply and accumulating the volt value for energy calculation
  volt_sum += (uint64_t)((int32_t)volt_read * (int32_t)volt_read); 


  //Removing DC offset from the ADC result.
  neutral_mean += (int32_t)neutral_read; 
  neutral_read -= n_mean;
  //multiply and accumulating the volt value for energy calculation
  neutral_sum += (uint64_t)((int32_t)(neutral_read) * (int32_t)(neutral_read));
  
  //active energy calculation
  i_temp = ((int32_t)meter.shunt_const[gain_stage] * (int32_t)phase_read);
  n_temp = (1583 * (int32_t)neutral_read)>>16;
  v_temp = ((int32_t)meter.volt_const * (int32_t)volt_read);
  lp_power[0] = ((int64_t)i_temp * (int64_t)v_temp)>>32;
  lp_powerout = lp_filter(lp_power);
  watts_sum  = watts_sum + lp_powerout;
  watts_sum_neutral += (n_temp * v_temp);
  adc_samples++;
}


/*! brief This function performs the low pass filtering of the input array
*
*   \param lp_coeff contains the coefficients for the low pass filtering
*   \param lp_output return the filtered value
*/
int16_t lp_filter(int16_t *inp_array)
{
  uint8_t i;
  int16_t lp_output = 0;
  //y(n)=h[0]*x[n]+..+h[N-1]x[n-(N-1)]
  for (i = 0; i< LP_ORDER; i++)
    lp_output += ((int32_t)lp_coeff[i] * (int32_t)*inp_array++)>>15;            
  //update sample data move "down"  
  for (i = LP_ORDER-1; i > 0; i--)                                              //from bottom of buffer
  {
    *inp_array--;
    *inp_array = *(inp_array-1);                                                
  }
  return(lp_output);
}

/*! brief This function is called on every 1sec and perform the calculation of metering paramters
*
*   \param Irms contains the root mean square value of current on phase line
*   \param Vrms contains the root mean square value of voltage
*   \param Nrms contains the root mean square value of current on neutral line
*   \param apparent_power holds the apparent energy (VA)
*   \param active_power holds the active energy (Watts)
*   \param power_factor hols the power factor(PF)
*/
void calculate(void)
{
  float kwh_temp=0;
  for(int8_t avg_count = 1; avg_count <4; avg_count++)
  {
    Irms[avg_count]          =  Irms[avg_count+1];
    Vrms[avg_count]          =  Vrms[avg_count+1];
    active_power[avg_count]  = active_power[avg_count+1];
  }
  //Current(rms) calculation on phase line
  shunt_temp =    (int32_t)((phase_sum/adc_samples)- meter.shunt_offset[offset_cnt]);
  if (shunt_temp < 5)
  { shunt_temp = 0;		}
  Irms[4]  = ((meter.shunt_const[gain_stage] * (int32_t)(sqrt(shunt_temp)*1000))>>16);
  //for smoothing the Irms
  Irms[0]  = ((int32_t)Irms[1] + (int32_t)Irms[2] + (int32_t)Irms[3] + (int32_t)Irms[4])>>2;
  Irms[2]  = Irms[0];
  
 //Voltage(rms) calculation  
  volt_temp =  (int32_t)(volt_sum/adc_samples);
  if(volt_temp < 35000)
  volt_temp = 0;
  Vrms[4]  = ((meter.volt_const * (int32_t)(sqrt(volt_temp)*100))>>16);
  //for smoothing the Vrms
  Vrms[0]  = ((int32_t)Vrms[1] + (int32_t)Vrms[2] + (int32_t)Vrms[3] + (int32_t)Vrms[4])>>2;
  Vrms[2]  = Vrms[0];

  //Current(rms) calculation on neutral line 
  ct_temp = (int32_t) (neutral_sum/adc_samples);
  if (ct_temp < 5)
  { ct_temp = 0;	}                
  Nrms = ( 1583 * (int64_t)((sqrt(ct_temp))*1000))>>16;

  //apparent energy calculation  
  apparent_power = ((uint64_t)Vrms[0] * (uint64_t)Irms[0])/100000;               //(Vrms/100) * (Irms/1000);
  //active energy calculation
  active_energy_signed =  ((watts_sum + meter.watt_offset[offset_cnt])  / adc_samples);
  watts_sum_calib = watts_sum;
  watts_sum = 0;
 
  //for smoothing the active power
  active_power[4] = (uint16_t)(meter.watt_const[gain_stage] *  abs(active_energy_signed));
  active_power[0] = ( (uint16_t)(active_power[1]) + (uint16_t)(active_power[2]) + (uint16_t)(active_power[3]) + (uint16_t)(active_power[4]) )>>2;   ////for smoothing the kW                
  active_power[2] = active_power[0];

  // neutral power calculation
  neutral_power = (watts_sum_neutral /adc_samples);                

  // changing the gain dynamically 
  if(Irms[4] < 500 && gain_stage != 6)
  {
    gain_stage =6;
    offset_cnt = 6;
    ADC_Ch_InputMode_and_Gain_Config(&ADCA.CH0,
                     ADC_CH_INPUTMODE_DIFFWGAIN_gc,
                     ADC_CH_GAIN_64X_gc);
  }
  else if(Irms[4] > 500 && Irms[4] < 1500 && gain_stage != 5)
  {
    gain_stage =5;
    offset_cnt = 6;
    ADC_Ch_InputMode_and_Gain_Config(&ADCA.CH0,
                   ADC_CH_INPUTMODE_DIFFWGAIN_gc,
                             ADC_CH_GAIN_64X_gc);
  }
  else if(Irms[4] > 1500 && Irms[4] < 2500 && gain_stage != 4)
  {
    gain_stage =4;
    offset_cnt = 6;
    ADC_Ch_InputMode_and_Gain_Config(&ADCA.CH0,
                     ADC_CH_INPUTMODE_DIFFWGAIN_gc,
                     ADC_CH_GAIN_64X_gc);
  }                
  else if (Irms[4] > 2500 && Irms[4] < 4000 && gain_stage != 3)
  {
    gain_stage =3;
    offset_cnt = 6;
    ADC_Ch_InputMode_and_Gain_Config(&ADCA.CH0,
                     ADC_CH_INPUTMODE_DIFFWGAIN_gc,
                     ADC_CH_GAIN_64X_gc);
  }
  else if (Irms[4] > 4000 && Irms[4] < 7000 && gain_stage != 2)
  {
    gain_stage = 2;
    offset_cnt = 6;
    ADC_Ch_InputMode_and_Gain_Config(&ADCA.CH0,
                     ADC_CH_INPUTMODE_DIFFWGAIN_gc,
                     ADC_CH_GAIN_64X_gc);
  }
  else if (Irms[4] > 7000 && Irms[4] < 14500 && gain_stage != 1)
  {
    gain_stage = 1;
    offset_cnt = 6;
    ADC_Ch_InputMode_and_Gain_Config(&ADCA.CH0,
                     ADC_CH_INPUTMODE_DIFFWGAIN_gc,
                     ADC_CH_GAIN_64X_gc);
  }
  else if (Irms[4] > 14500 && gain_stage !=0)
  {    
    gain_stage = 0;
    offset_cnt = 5;
    ADC_Ch_InputMode_and_Gain_Config(&ADCA.CH0,
                     ADC_CH_INPUTMODE_DIFFWGAIN_gc,
                     ADC_CH_GAIN_32X_gc);
  }
  
  
  if(active_energy_signed < 3 && active_energy_signed > -3)
  {
    active_power[0] = 0;
    Irms[0] = 0;
    apparent_power = 0;
  }
  if (Irms[0] < 10)
  {
    Irms[0] = 0;
    Nrms = 0;
    active_power[0] = 0;
    neutral_power=0;
    apparent_power = 0;
  }
  
  //power factor calculation
  power_factor = (int16_t)(1000 * (float)active_power[0]/(float)apparent_power);
  if(power_factor >1000)
  {
    apparent_power = active_power[0];
    power_factor = 1000;
  }
  else if(power_factor < 0)
  {
    power_factor = 0;
  }

  
  kwh_temp = ((float)(active_power[0])/3600000);
  meter.kwh += kwh_temp;  
  pulse_energy_2_5ms = kwh_temp / 400;
  calculate_freq();

  //Dc offset voltage calculation
  Vmean  = (int16_t)(volt_mean/adc_samples);
  p_mean = (int16_t)(phase_mean/adc_samples);
  n_mean = (int16_t)(neutral_mean/adc_samples);
  phase_mean = 0;
  volt_mean = 0;
  n_mean = 0;

  adc_samples = 0;
  volt_sum = 0;
  phase_sum = 0;
  neutral_sum = 0;
  watts_sum = 0;
  watts_sum_neutral = 0;
}



/*! brief This function calculate line frequency from the value
*
*  \param frequency hold the line frequency.
*  \param freq_avg hold the average timer value.
*/
void calculate_freq(void)
{
  
  uint16_t freq_avg;

  freq_avg = (uint16_t)(freq_sum/freq_cnt);
  frequency = (uint16_t)((F_TIMER *100) /freq_avg);

  if(frequency > 10100 || frequency < 2000)
    frequency = 0;
  freq_sum = 0;
  freq_cnt = 0;
}

/*! \This function check for the sign change in voltage signal for frequency calucaulation.
*
*   \param sign_flag updated when there is a sign change  \n
*          sign_flag = 0, during the negative half cycle  \n
*          sign_flag = 1, during the positive half cycle  \n
*   \param freq_val hold the timer value of one sine wave
*   \param freq_sum hold the accumulated timer counter for frequency calculation
*   \param freq_cnt hold the number of sine waves taken for frequency measument
*
*/
void get_frequency(void)
{
  static uint8_t sign_flag = 0;
  uint16_t freq_val;
  if(adc_read_ch1 < 0 && sign_flag != 0)
  {
    sign_flag = 0;
  }
  else if(adc_read_ch1 >= 0 && sign_flag != 1)
  {
    sign_flag = 1;
    freq_val = TCE0.CNT;
    //*! freq measurement limited is from 20 to 100hz //freq_val > 9090 && freq_val < 11111    
    if(freq_val > FREQUENCY_MAX && freq_val < FREQUENCY_MIN)        
    {
      freq_sum += freq_val;
      freq_cnt++;
    }
    TCE0.CNT =0;
  }
}


/*! brief This function clear all the varilable used in for energy calculation
*
*/

void meter_flush()
{
  phase_mean = 0;
  volt_mean = 0;
  n_mean = 0;
  for(int j=0; j < 5; j++)
  {
    Irms[j] = 0;
    Vrms[j] = 0;
    active_power[j] = 0;
  }
  phase_mean = 0;
  volt_mean = 0;
  n_mean = 0;
  freq_sum = 0;
  freq_cnt = 0;
  adc_samples = 0;
  volt_sum = 0;
  phase_sum = 0;
  neutral_sum = 0;
  watts_sum = 0;
  watts_sum_neutral = 0;
}

