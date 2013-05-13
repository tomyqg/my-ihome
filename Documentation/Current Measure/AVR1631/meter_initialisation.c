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

//! declaring the structure in EEPROM
__eeprom struct coefficient eeprom_t  @ 0x12;
struct coefficient meter_t;

/*! \brief Perform a delay of \c milliseconds.
 *
 *  \param ms contains the required delay in milli seconds
 */
void delay_ms(uint16_t ms)
{
  int x = 0;
  for (x=0;x<=ms;x++)
  {
          delay_us(1000);
  }
}

/* brief This function initialise the LCD Module 
*
*/
void init_lcd(void)
{
  LCD.DIRSET = LCD_CS | LCD_WRITE | LCD_DATA | LCD_READ;
  LCD.PIN3CTRL = PORT_OPC_PULLUP_gc;
  LCD.PIN2CTRL = PORT_OPC_PULLUP_gc;
  LCD.PIN1CTRL = PORT_OPC_PULLUP_gc;
  LCD.PIN0CTRL = PORT_OPC_PULLUP_gc;
  LCD_CS_SET;
  LCD_WRITE_SET;
  LCD_DATA_SET;
  LCD_READ_SET;
  lcd_command(LCD_COMMAND,0x01); //sys en
  delay_us(100);
  lcd_command(LCD_COMMAND,0x29); //bias 1/3, 10:4
  delay_us(100);
  lcd_command(LCD_COMMAND,0xE3);	//normal mode
  delay_us(100);
  lcd_command(LCD_COMMAND,0x03);	//LCD on
}



/*! \brief This function intialises the CPU clock to 32MHz and enable the DFLL for runtime
*    calibration of 32MHz internal Rc oscillator.
*
*   The 32.768KHz external crystal is used as reference clock for DFLL
*/
void clock_init(void)
{
  //! Enable the 32MHz internal RC oscillator
  CLKSYS_Enable( OSC_RC32MEN_bm );
  CLKSYS_Prescalers_Config( CLK_PSADIV_1_gc, CLK_PSBCDIV_1_1_gc );      
  do {} while ( CLKSYS_IsReady( OSC_RC32MRDY_bm ) == 0 );
  CLKSYS_Main_ClockSource_Select( CLK_SCLKSEL_RC32M_gc );
  CLKSYS_Disable( OSC_RC2MEN_bm | OSC_RC32KEN_bm | OSC_PLLEN_bm);
  delay_ms(1);
  
  //! Enable the DFLL for runtime calibration of 32MHz internal RC oscillator
  OSC.XOSCCTRL = OSC_XOSCSEL_32KHz_gc;     //0x02;
  CLKSYS_Enable(OSC_XOSCEN_bm);        
  do {} while ( CLKSYS_IsReady( OSC_XOSCRDY_bm ) == 0 );
  OSC.DFLLCTRL = ( OSC.DFLLCTRL & ~(OSC_RC32MCREF_gm | OSC_RC2MCREF_bm)) | OSC_RC32MCREF0_bm;
  DFLLRC32M.CTRL = DFLL_ENABLE_bm;
    delay_ms(1);
}

/*! \brief This function initializes the IOPORTS
*
*/
void init_ioport(void)
{
  
  /*! \brief PORTB GPIO pin confuguration \n
  *   PIN0_bm --> TAMPER_CURRENT_REVERSAL indication  \n
  *   PIN1_bm --> TAMPER_EARTH_FAULT indication       \n
  *   PIN2_bm --> LCD switch                          \n
  *   PIN3_bm --> TAMPER_NEUTRAL_CUT indication       \n
  *   
  */
  PORTB.DIRCLR	        = PIN2_bm;
  PORTB.DIRSET	        = TAMPER_CURRENT_REVERSAL | TAMPER_EARTH_FAULT | TAMPER_NEUTRAL_CUT;
  PORTB.PIN2CTRL        = PORT_OPC_PULLUP_gc;
  PORTB.PIN2CTRL	= PORT_ISC_FALLING_gc ;
  PORTB.OUTCLR	        = PIN0_bm | PIN1_bm | PIN2_bm | PIN3_bm;
  PORTB.INT0MASK	= PIN2_bm;
  PORTB.INTCTRL	        = PORT_INT0LVL_LO_gc;
  
  /*! \brief PORTC GPIO pin confuguration \n
  *   PIN0_bm --> TWI connector                      \n
  *   PIN1_bm --> TWI connector                      \n
  *   PIN2_bm --> Cover open switch                  \n
  *   PIN3_bm --> Calibration switch                 \n
  *   PIN4_bm --> Magnetic Tamper detect             \n
  *   PIN5_bm --> unused switch                      \n
  *   PIN6_bm --> UART Rx                            \n
  *   PIN7_bm --> UART Tx                            \n  
  */  
  PORTC.DIRCLR	        =  PIN0_bm | PIN1_bm |PIN2_bm |PIN3_bm |PIN4_bm | PIN5_bm;
  PORTC.PIN0CTRL	=  PORT_OPC_PULLUP_gc;
  PORTC.PIN1CTRL	=  PORT_OPC_PULLUP_gc;
  PORTC.PIN2CTRL	=  PORT_OPC_PULLUP_gc;
  PORTC.PIN2CTRL	|= PORT_ISC_FALLING_gc;
  PORTC.PIN3CTRL	=  PORT_OPC_PULLUP_gc;
  PORTC.PIN4CTRL	=  PORT_OPC_PULLUP_gc;
  PORTC.PIN4CTRL	|= PORT_ISC_RISING_gc;
  PORTC.PIN5CTRL	=  PORT_OPC_PULLUP_gc;       
  PORTC.PIN5CTRL	|= PORT_ISC_FALLING_gc;
  PORTC.OUTCLR	        =  PIN2_bm |PIN3_bm |PIN4_bm | PIN5_bm;
  PORTC.INT0MASK	=  PIN2_bm |PIN4_bm |PIN5_bm;
  PORTC.INTCTRL	        =  PORT_INT0LVL_LO_gc;
  
  /*! \brief PORTD GPIO pin confuguration \n
  *   PIN0_bm --> LED Pulse                          \n
  *   PIN1_bm --> LCD Backlight control              \n
  *   PIN2_bm --> sleep detect                       \n
  *   PIN3_bm --> LED debugging and status update    \n
  *   PIN4_bm --> Battery On/Off control             \n
  *   PIN5_bm --> USB D-                             \n
  *   PIN6_bm --> USB D+                             \n
  *   PIN7_bm --> LCD pin(optional)                  \n  
  */  
  PORTD.DIRSET	        = PIN0_bm | PIN1_bm | PIN3_bm;
  PORTD.DIRCLR          = PIN2_bm | PIN4_bm | PIN5_bm | PIN6_bm | PIN7_bm;
  PORTD.PIN0CTRL	= PORT_OPC_PULLUP_gc;
  PORTD.PIN1CTRL	= PORT_OPC_WIREDANDPULL_gc;
  PORTD.PIN2CTRL	= PORT_ISC_BOTHEDGES_gc;
  PORTD.PIN2CTRL	|= PORT_OPC_PULLDOWN_gc;
  PORTD.PIN2CTRL	|= PORT_SRLEN_bm;
  PORTD.PIN3CTRL	= PORT_OPC_PULLUP_gc;
  PORTD.PIN4CTRL	= PORT_OPC_WIREDANDPULL_gc;
  PORTD.PIN5CTRL	= PORT_OPC_PULLUP_gc;
  PORTD.PIN6CTRL	= PORT_OPC_PULLUP_gc;
  PORTD.PIN7CTRL	= PORT_OPC_PULLUP_gc;
  PORTD.INT0MASK        = PIN2_bm;
  PORTD.INTCTRL	        = PORT_INT0LVL_HI_gc;
  PORTD.OUTCLR	        = PIN0_bm | PIN1_bm | PIN3_bm | PIN4_bm;
}

/*! \brief This function initializes the UARTC1 to 8 Data bits, No Parity, 1 Stop bit, 38400bps
*
*/
void init_UART(void)
{
  //!  Set UART TXD Pin(Pin3) as Output and RXD(Pin 2) Pin as Input Port
  USART_PORT.DIRSET   = PIN7_bm;
  USART_PORT.DIRCLR   = PIN6_bm;

  /*!  Configure USART Control Register, Character Size,Parity,Stop Bit to 
       For example 8 Data bits, No Parity, 1 Stop bit*/
  USART.CTRLC = (uint8_t) UART_CHARACTER_SIZE | UART_PARITY_MODE_SIZE | false;

  //! Configure USART Baud Rate value, 38400bps
  USART.BAUDCTRLA = UART_BSEL_VALUE;

  //!  Enable Both USART Receiver and Transmitter
  USART.CTRLB |= USART_RXEN_bm;
  USART.CTRLB |= USART_TXEN_bm;
  
  USART.CTRLA = USART_RXCINTLVL0_bm;
}


/*! \brief This function loads the eeprom vaules during the startup
 *
 *    
 *  \param  refer the struct eeprom in meter.h
 */
void init_eeprom(void)
{
  for(uint8_t j = 0; j < 7; j++)
  {
    meter.watt_const[j]   = eeprom.watt_const[j];
    meter.shunt_const[j]  = eeprom.shunt_const[j];
    meter.shunt_offset[j] = eeprom.shunt_offset[j];
    meter.watt_offset[j]  = eeprom.watt_offset[j];
    meter.A1[j]           = eeprom.A1[j];
    meter.B1[j]           = eeprom.B1[j];
  }
  meter.volt_const     = eeprom.volt_const;              
  meter.kwh = eeprom.kwh;                           
}

/*! \brief This function disables the clock to unused peripherals and there by  
 *   reduces the current consumption
 *    
 */
void init_power_reduction(void)
{
    PR.PRGEN = PR_USB_bm | PR_AES_bm | PR_EBI_bm | PR_EVSYS_bm | PR_DMA_bm;
    PR.PRPA  = PR_DAC_bm | PR_AC_bm;
    PR.PRPB  = PR_DAC_bm | PR_AC_bm | PR_ADC_bm;
    PR.PRPC  = PR_TWI_bm | PR_USART0_bm                | PR_SPI_bm | PR_HIRES_bm | PR_TC0_bm;
    PR.PRPD  = PR_TWI_bm | PR_USART0_bm | PR_USART1_bm | PR_SPI_bm | PR_HIRES_bm | PR_TC1_bm;
    PR.PRPE  = PR_TWI_bm | PR_USART0_bm | PR_USART1_bm | PR_SPI_bm | PR_HIRES_bm | PR_TC1_bm;
    PR.PRPF  = PR_TWI_bm | PR_USART0_bm | PR_USART1_bm | PR_SPI_bm | PR_HIRES_bm | PR_TC1_bm;
}


/*! \brief This function disables the clock to unused peripherals and there by  
 *   reduces the current consumption
 *    
 */
void init_timer(void)
{
  TCC1.PER =  F_SAMPLING;
  TCC1.CTRLA = ( TCC1.CTRLA & ~TC1_CLKSEL_gm ) | TC_CLKSEL_DIV64_gc;
  TCC1.CTRLB = (TCC1.CTRLB & ~TC1_WGMODE_gm);
  TCC1.INTCTRLA = (TCC1.INTCTRLA & ~(TC1_OVFINTLVL_gm | TC1_ERRINTLVL_gm))|TC1_OVFINTLVL0_bm;
  SLEEP.CTRL = (SLEEP.CTRL & ~SLEEP_SMODE_gm)| SLEEP_SMODE_IDLE_gc;

  TCE0.PER =  0xFFFF;
  TCE0.CNT = 0;
  TCE0.CTRLA = ( TCE0.CTRLA & ~TC0_CLKSEL_gm ) | TC_CLKSEL_DIV64_gc;  
  TCE0.CTRLB = (TCE0.CTRLB & ~TC0_WGMODE_gm);
}

/*! brief This function initializes the ADC
*
* 
*/
void init_ADC()
{
  //! Set up ADC A to have signed conversion mode and 12 bit resolution. 
  ADC_ConvMode_and_Resolution_Config(&ADCA, ADC_ConvMode_Signed, ADC_RESOLUTION_12BIT_gc);
  
  //!  Set ADC clock frequency to FCPU/128
  ADC_Prescaler_Config(&ADCA, ADC_PRESCALER_DIV128_gc);
  
  //!  Set referance voltage on ADC A to be INT 1V.
  ADC_Reference_Config(&ADCA, ADC_REFSEL_INT1V_gc);

  ADC_Low_Impedance_Mode(&ADCA);

  //! *************** ADC Channel 0 ********************************
  //! \n connected to the shunt resistor for current measumemnt in Phase line  \n
 //! Setup channel 0 to have diff gain  input. 
  ADC_Ch_InputMode_and_Gain_Config(&ADCA.CH0,
                                   ADC_CH_INPUTMODE_DIFFWGAIN_gc,
                                   ADC_CH_GAIN_32X_gc);                                                         
  gain_stage = 0;
  offset_cnt = 5;

  //!  Set input to the channels in ADC A to be PIN 0 and PIN 4. 
  ADC_Ch_InputMux_Config(&ADCA.CH0, ADC_CH_MUXPOS_PIN0_gc, ADC_CH_MUXNEG_PIN4_gc);

  //! Enable medium level interrupts on ADCA channel 0, on conversion complete. */
  ADC_Ch_Interrupts_Config(&ADCA.CH0, ADC_CH_INTMODE_COMPLETE_gc, ADC_CH_INTLVL_MED_gc);
 //!**********************************************************************/
  
  
  
  //! *************** ADC Channel 1 ********************************
  //! \n connected to the voltage divider resistor for voltage measumemnt    \n
  //! Setup channel 1 to have diff gain  input. 
  ADC_Ch_InputMode_and_Gain_Config(&ADCA.CH1,
                                   ADC_CH_INPUTMODE_DIFFWGAIN_gc,
                                   ADC_CH_GAIN_2X_gc);
  //! Set input to the channels in ADC A to be PIN 0 and PIN 4.
  ADC_Ch_InputMux_Config(&ADCA.CH1, ADC_CH_MUXPOS_PIN1_gc, ADC_CH_MUXNEG_PIN5_gc);

  //! Enable medium level interrupts on ADCA channel 0, on conversion complete. 
  ADC_Ch_Interrupts_Config(&ADCA.CH1, ADC_CH_INTMODE_COMPLETE_gc, ADC_CH_INTLVL_MED_gc);

  
  //!  *********************************************************************/
  
  //! *************** ADC Channel 2 ********************************
  //! \n connected to the CT for current measumemnt in neutral line   \n
  //! Setup channel 1 to have diff gain  input.
  ADC_Ch_InputMode_and_Gain_Config(&ADCA.CH2,
                                   ADC_CH_INPUTMODE_DIFFWGAIN_gc,
                                   ADC_CH_GAIN_2X_gc);
                                                                   
  //! Set input to the channels in ADC A to be PIN 0 and PIN 4.
  ADC_Ch_InputMux_Config(&ADCA.CH2, ADC_CH_MUXPOS_PIN2_gc, ADC_CH_MUXNEG_PIN6_gc);

  //! Enable medium level interrupts on ADCA channel 0, on conversion complete.
  ADC_Ch_Interrupts_Config(&ADCA.CH2, ADC_CH_INTMODE_COMPLETE_gc, ADC_CH_INTLVL_MED_gc);
  
  
  //!  *******************************************************************/
  
  //*************** ADC Channel 3 *************************************/
  //! \n unused   \n
  // Setup channel 1 to have diff gain  input.
  ADC_Ch_InputMode_and_Gain_Config(&ADCA.CH3,
                                   ADC_CH_INPUTMODE_DIFFWGAIN_gc,
                                   ADC_CH_GAIN_2X_gc);
                                                                   
  // Set input to the channels in ADC A to be PIN 0 and PIN 4.
  ADC_Ch_InputMux_Config(&ADCA.CH3, ADC_CH_MUXPOS_PIN3_gc, ADC_CH_MUXNEG_PIN7_gc);

 // Enable low level interrupts on ADCA channel 0, on conversion complete.
  //ADC_Ch_Interrupts_Config(&ADCA.CH3, ADC_CH_INTMODE_COMPLETE_gc, ADC_CH_INTLVL_LO_gc);

//***********************************************************************/


  //! Enable PMIC interrupt level high.
  PMIC.CTRL |= (PMIC_LOLVLEX_bm | PMIC_MEDLVLEX_bm |PMIC_RREN_bm);

  //! Enable global interrupts.
  sei();
  
  ADC_Pipeline_Flush(&ADCA);
  
  //! Enable ADC A with free running mode, VCC reference and signed conversion.
  ADC_Enable(&ADCA);

  /*! Wait until common mode voltage is stable. Default clock is 32MHz and
   * therefore below the maximum frequency to use this function. */
  ADC_Wait_32MHz(&ADCA);
  //! clear all the varilable used in for energy calculation
  meter_flush();
  
  PORTA.DIRCLR    = PIN3_bm | PIN7_bm;
  PORTA.PIN3CTRL  =  PORT_OPC_PULLUP_gc;
  PORTA.PIN7CTRL  =  PORT_OPC_PULLUP_gc;
}



/*! brief This function calculate the ADC offset values for different gain stage.
*
*  \param offset[ ] This array hold the offset values at different gain stages.
*/
void get_offset()
{
	/* Set up ADC A to have signed conversion mode and 12 bit resolution. */
  	ADC_ConvMode_and_Resolution_Config(&ADCA, ADC_ConvMode_Signed, ADC_RESOLUTION_12BIT_gc);

	/* Set sample rate */
	ADC_Prescaler_Config(&ADCA, ADC_PRESCALER_DIV128_gc);

	/* Set referance voltage on ADC A to be VCC/1.6 V.*/
	ADC_Reference_Config(&ADCA, ADC_REFSEL_INT1V_gc);

   	/* Get offset value for ADC A. */
	ADC_Ch_InputMode_and_Gain_Config(&ADCA.CH0,
	                                 ADC_CH_INPUTMODE_DIFFWGAIN_gc,
	                                 ADC_CH_GAIN_1X_gc);

   	ADC_Ch_InputMux_Config(&ADCA.CH0, ADC_CH_MUXPOS_PIN4_gc, ADC_CH_MUXNEG_PIN4_gc);

	ADC_Enable(&ADCA);
	/* Wait until common mode voltage is stable. Default clk is 32MHz and
	 * therefore below the maximum frequency to use this function. */
	ADC_Wait_32MHz(&ADCA);
 	offset[0] = ADC_Offset_Get_Signed(&ADCA, &ADCA.CH0, true);

	delay_us(1000);
	ADC_Ch_InputMode_and_Gain_Config(&ADCA.CH0,
	                                 ADC_CH_INPUTMODE_DIFFWGAIN_gc,
	                                 ADC_CH_GAIN_2X_gc);
	delay_us(1000);
	offset[1] = ADC_Offset_Get_Signed(&ADCA, &ADCA.CH0, true);		


	delay_us(1000);
	ADC_Ch_InputMode_and_Gain_Config(&ADCA.CH0,
	                                 ADC_CH_INPUTMODE_DIFFWGAIN_gc,
	                                 ADC_CH_GAIN_4X_gc);
	delay_us(1000);
	offset[2] = ADC_Offset_Get_Signed(&ADCA, &ADCA.CH0, true);		


	delay_us(1000);
	ADC_Ch_InputMode_and_Gain_Config(&ADCA.CH0,
	                                 ADC_CH_INPUTMODE_DIFFWGAIN_gc,
	                                 ADC_CH_GAIN_8X_gc);
	delay_us(1000);
	offset[3] = ADC_Offset_Get_Signed(&ADCA, &ADCA.CH0, true);		


	delay_us(1000);
	ADC_Ch_InputMode_and_Gain_Config(&ADCA.CH0,
	                                 ADC_CH_INPUTMODE_DIFFWGAIN_gc,
	                                 ADC_CH_GAIN_16X_gc);
	delay_us(1000);
	offset[4] = ADC_Offset_Get_Signed(&ADCA, &ADCA.CH0, true);		


	delay_us(1000);
	ADC_Ch_InputMode_and_Gain_Config(&ADCA.CH0,
	                                 ADC_CH_INPUTMODE_DIFFWGAIN_gc,
	                                 ADC_CH_GAIN_32X_gc);
	delay_us(1000);
	offset[5] = ADC_Offset_Get_Signed(&ADCA, &ADCA.CH0, true);		

	delay_us(1000);
	ADC_Ch_InputMode_and_Gain_Config(&ADCA.CH0,
	                                 ADC_CH_INPUTMODE_DIFFWGAIN_gc,
	                                 ADC_CH_GAIN_64X_gc);
	delay_us(1000);
	offset[6] = ADC_Offset_Get_Signed(&ADCA, &ADCA.CH0, true);		
	 
    ADC_Disable(&ADCA);
  
}

/*! brief This function enable pullup on all port pins and set the pins to input mode
*
* 
*/
void facilitatePowersaving(void) 
{
  // Pullup on all ports, to ensure a defined state.
  PORTCFG.MPCMASK = 0xFF;
  PORTA.PIN0CTRL  = PORT_OPC_PULLUP_gc;
  PORTA.DIRCLR    = 0xFF;

  PORTCFG.MPCMASK = 0x0F;
  PORTB.PIN0CTRL  = PORT_OPC_PULLUP_gc;
  PORTB.DIRCLR    = 0x0F;
  
  PORTCFG.MPCMASK = ~PIN2_bm;
  PORTC.PIN0CTRL  = PORT_OPC_PULLUP_gc;
  PORTC.DIRCLR    = 0xFF;
  
  PORTCFG.MPCMASK = ~(PIN2_bm | PIN4_bm);
  PORTD.PIN0CTRL  = PORT_OPC_PULLUP_gc;
  PORTD.DIRCLR    = 0xFF;
  
  PORTCFG.MPCMASK = 0x0F;
  PORTE.PIN0CTRL  = PORT_OPC_PULLUP_gc;
  PORTE.DIRCLR    = 0x0F;
}

