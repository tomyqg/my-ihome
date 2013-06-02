/*******************************************************************************
*   $FILE:  main.c
*   Atmel Corporation:  http://www.atmel.com \n
*   Support email:  touch@atmel.com
******************************************************************************/

/*  License
*   Copyright (c) 2010, Atmel Corporation All rights reserved.
*
*   Redistribution and use in source and binary forms, with or without
*   modification, are permitted provided that the following conditions are met:
*
*   1. Redistributions of source code must retain the above copyright notice,
*   this list of conditions and the following disclaimer.
*
*   2. Redistributions in binary form must reproduce the above copyright notice,
*   this list of conditions and the following disclaimer in the documentation
*   and/or other materials provided with the distribution.
*
*   3. The name of ATMEL may not be used to endorse or promote products derived
*   from this software without specific prior written permission.
*
*   THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
*   WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
*   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY AND
*   SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT,
*   INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
*   (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
*   ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
*   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
*   THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/*----------------------------------------------------------------------------
                            compiler information
----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
                                include files
----------------------------------------------------------------------------*/
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <avr/sleep.h>

#define __delay_cycles(n)     __builtin_avr_delay_cycles(n)
#define __enable_interrupt()  sei()

#include "touch_api.h"
#include "touch.h"

#include "board_config.h"
#include "utils.h"
#include "usart_driver.h"
#include <avr/pgmspace.h>
#include "tc_driver.h"

/*----------------------------------------------------------------------------
                            manifest constants
----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
                                    macros
----------------------------------------------------------------------------*/

#define GET_SENSOR_STATE(SENSOR_NUMBER) qt_measure_data.qt_touch_status.sensor_states[(SENSOR_NUMBER/8)] & (1 << (SENSOR_NUMBER % 8))
//#define GET_ROTOR_SLIDER_POSITION(ROTOR_SLIDER_NUMBER) qt_measure_data.qt_touch_status.rotor_slider_values[ROTOR_SLIDER_NUMBER]

/*----------------------------------------------------------------------------
                            type definitions
----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
                                prototypes
----------------------------------------------------------------------------*/
extern void touch_measure();
extern void touch_init( void );
extern void init_system( void );
extern void init_timer_isr(void);
extern void set_timer_period(uint16_t);
/*----------------------------------------------------------------------------
                            Structure Declarations
----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
                                    macros
----------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------
                                global variables
----------------------------------------------------------------------------*/
/* Timer period in msec. */
uint16_t qt_measurement_period_msec = QT_MEASUREMENT_PERIOD_MS;
uint16_t time_ms_inc=0;
/*----------------------------------------------------------------------------
                                extern variables
----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
                                static variables
----------------------------------------------------------------------------*/

/* flag set by timer ISR when it's time to measure touch */
volatile uint8_t time_to_measure_touch = 0u;

/* current time, set by timer ISR */
volatile uint16_t current_time_ms_touch = 0u;

#ifdef MMSN_DEBUG

/* STDOUT */
static int uart_putchar(char c, FILE *stream);

static FILE mystdout = FDEV_SETUP_STREAM (uart_putchar, NULL, _FDEV_SETUP_WRITE);

static int uart_putchar(char a_inChar, FILE *stream)
{
	if (a_inChar == '\n')
	{
		uart_putchar('\r', stream);
	}
	
	// Wait for the transmit buffer to be empty
	// while ( !(USART_TERMINAL.STATUS & USART_DREIF_bm) );
	while ( !(USART_COMMUNICATION_BUS.STATUS & USART_DREIF_bm) );
	
	// Put our character into the transmit buffer
	// USART_TERMINAL.DATA = a_inChar;
	USART_COMMUNICATION_BUS.DATA = a_inChar;
	
	return 0;
};

/**
* \brief Configure and initialize communication terminal USART.
*
* This function performs all necessary USART configuration to enable receiving and transmission.
* During configuration global interrupts are disabled.
*
* \note Configuration is only valid for a chosen hardware USART, baud rate and peripheral clock.
*
* \param none.
*
* \retval none.
*/
static inline void usartCommTerminalInit(void)
{
	/* Set USART transmission 19200 baud rate @8MHz */
	/* BSCALE = -7		*/
	/* CLK2X = 0		*/
	/* BSEL = 3205		*/
	/* Error = 0,01%	*/
		
	/* USART initialization should use the following sequence:
		1. Set the TxD pin value high, and optionally set the XCK pin low.
		2. Set the TxD and optionally the XCK pin as output.
		3. Set the baud rate and frame format.
		4. Set the mode of operation (enables XCK pin output in synchronous mode).
		5. Enable the transmitter or the receiver, depending on the usage.
	For interrupt-driven USART operation, global interrupts should be disabled during the
	initialization. */	
	
	/* Disable global interrupts */
	cli();
		
	/* PC7 (TXD) as output - high */
	PORT_DIRSET(USART_TERMINAL_TX_IO);
	PORT_OUTSET(USART_TERMINAL_TX_IO);
	
	/* PC6 (RXD) as input */
	// USART_TERMINAL_PORT.DIRCLR = TPASTE3(PIN, 6, _bm);
	PORT_DIRCLR(USART_TERMINAL_RX_IO);
	
	/* Enable system clock to peripheral */
	// Should be enabled after restart - USARTC1
	PR.PRPD &= ~PR_USART0_bm;
	
	/* Set the baud rate: use BSCALE and BSEL */
	xmega_set_usart_baudrate(&USART_TERMINAL, 3205, -7);	// 19200bps
	
	/* Set frame format */
	xmega_set_usart_format(&USART_TERMINAL, USART_TERMINAL_CHAR_LENGTH, USART_TERMINAL_PARITY, USART_TERMINAL_STOP_BIT);

	/* Set mode */
	xmega_set_usart_mode(&USART_TERMINAL, USART_CMODE_ASYNCHRONOUS_gc);
	
	/* Set interrupts level */
	// xmega_set_usart_rx_interrupt_level(&USART_TERMINAL, USART_RXCINTLVL_LO_gc);
	// xmega_set_usart_tx_interrupt_level(&USART_TERMINAL, USART_TXCINTLVL_LO_gc);
	
	/* Enable transmitter and receiver */
	xmega_enable_usart_tx(&USART_TERMINAL);
	xmega_enable_usart_rx(&USART_TERMINAL);
	
}	// usartCommTerminalInit()
#endif

/********************************************//**
* Heartbeat timer configuration section
***********************************************/
static inline void xmega_timer_config(TC0_t *a_pTimer, TC_CLKSEL_t a_clockSelect, uint16_t a_timerPeriod )
{
	/*
	 * Enable the timer, and set it to count up.
	 * When it overflows, it triggers the message to the Host.
	 * 0.25Hz tick.
	 */
	a_pTimer->CTRLB = (a_pTimer->CTRLB & ~TC0_WGMODE_gm) | TC_WGMODE_NORMAL_gc;
	a_pTimer->CTRLFSET	= 0;																// set UP direction
	a_pTimer->PER		= a_timerPeriod;													// set defined period
	a_pTimer->INTCTRLA	= a_pTimer->INTCTRLA & ~TC0_OVFINTLVL_gm;							// set overflow interrupt
	a_pTimer->INTCTRLA	= a_pTimer->INTCTRLA | (TC_OVFINTLVL_LO_gc << TC0_OVFINTLVL_gp);	// set low-level overflow interrupt
	a_pTimer->CTRLA		= (a_pTimer->CTRLA & ~TC0_CLKSEL_gm) | a_clockSelect;	
};

uint16_t gCounter = 0;

ISR(TCC0_OVF_vect)
{
	/*  set flag: it's time to measure touch    */
	time_to_measure_touch = 1u;

	/*  update the current time  */
	current_time_ms_touch += qt_measurement_period_msec;
	
	// temp
	gCounter++;
}

/*============================================================================
Name    :   main
------------------------------------------------------------------------------
Purpose :   main code entry point
Input   :   n/a
Output  :   n/a
Notes   :
============================================================================*/

int main( void )
{

   /* initialize host app, pins, watchdog, etc */
    // init_system();

    /* configure timer ISR to fire regularly */
    // init_timer_isr();
	
	// Configure CPU and peripherals clock
	xmega_set_cpu_clock_to_32MHz();
	
	// Enable interrupts
	PMIC.CTRL = PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_HILVLEN_bm;
	
	// Power management - configure sleep mode to IDLE
	set_sleep_mode(SLEEP_SMODE_IDLE_gc);
	// Enable sleep mode
	sleep_enable();

#ifdef MMSN_DEBUG
	// Initialize serial communication terminal
	// usartCommTerminalInit();
	
	// Configure and initialize communication bus usart
	xmega_usart_configure();
		
	/* RS-485 PHYSICAL DEVICE CONFIGURATION */
	// Initialize GPIO related to RS-485 interface
	rs485_driver_gpio_initialize();
		
	// Enable driver to be able to send data
	rs485_driver_enable();

	// Redirect stream to standard output
	stdout = &mystdout;

	/* Print out welcome message */
	printf_P(PSTR("\nGeneric Board ver 1.00\n"));
#endif

	// Heartbeat timer - 8MHz prescales by 8 => 1MHz
	// Thus 1ms equals to 1000 ticks
	xmega_timer_config(&TIMER_HEARTBEAT, TC_CLKSEL_DIV8_gc, (TICKS_PER_MS * qt_measurement_period_msec));

	/* Initialize Touch sensors */
	touch_init();

	// Configure PIN6 as input
	PORT_DIRCLR(POWER_SUPPLY_MEASUREMENT_IO);
	
	// ADC conversion on pin 6
	uint16_t u16ConvResult = xmega_generate_adc_random_value(&ADCA, ADC_REFSEL_INT1V_gc | 0x02, ADC_CH_MUXPOS_PIN6_gc);
	printf("Power meter = %u\n", u16ConvResult);
	
	// Configure PIN5 as input
	PORT_DIRCLR(OPTO_IO);
	
	u16ConvResult = xmega_generate_adc_random_value(&ADCA, ADC_REFSEL_INT1V_gc | 0x02, ADC_CH_MUXPOS_PIN5_gc);
	printf("Opto = %u\n", u16ConvResult);

    /* loop forever */
    for(;;)
    {
        touch_measure();
		
		// Test every 2s
		if(gCounter >= 40)
		{
			printf("\n");
			printf("\nSensor[0]: %d - ", GET_SENSOR_STATE(0));
			printf("Sensor[1]: %d - ", GET_SENSOR_STATE(1));
			printf("Sensor[2]: %d - ", GET_SENSOR_STATE(2));
			printf("Sensor[3]: %d\n", GET_SENSOR_STATE(3));
			printf("Sensor[4]: %d - ", GET_SENSOR_STATE(4));
			printf("Sensor[5]: %d - ", GET_SENSOR_STATE(5));
			printf("Sensor[6]: %d - ", GET_SENSOR_STATE(6));
			printf("Sensor[7]: %d\n", GET_SENSOR_STATE(7));
			printf("Sensor[8]: %d - ", GET_SENSOR_STATE(8));
			printf("Sensor[9]: %d - ", GET_SENSOR_STATE(9));
			printf("Sensor[10]: %d - ", GET_SENSOR_STATE(10));
			printf("Sensor[11]: %d\n", GET_SENSOR_STATE(11));
			printf("Sensor[12]: %d - ", GET_SENSOR_STATE(12));
			printf("Sensor[13]: %d - ", GET_SENSOR_STATE(13));
			printf("Sensor[14]: %d - ", GET_SENSOR_STATE(14));
			printf("Sensor[15]: %d\n", GET_SENSOR_STATE(15));
			
			printf("Sensor[16]: %d - ", GET_SENSOR_STATE(16));
			printf("Sensor[17]: %d - ", GET_SENSOR_STATE(17));
			printf("Sensor[18]: %d - ", GET_SENSOR_STATE(18));
			printf("Sensor[19]: %d\n", GET_SENSOR_STATE(19));
			printf("Sensor[20]: %d - ", GET_SENSOR_STATE(20));
			printf("Sensor[21]: %d - ", GET_SENSOR_STATE(21));
			printf("Sensor[22]: %d - ", GET_SENSOR_STATE(22));
			printf("Sensor[23]: %d\n", GET_SENSOR_STATE(23));
			printf("Sensor[24]: %d - ", GET_SENSOR_STATE(24));
			printf("Sensor[25]: %d - ", GET_SENSOR_STATE(25));
			printf("Sensor[26]: %d - ", GET_SENSOR_STATE(26));
			printf("Sensor[27]: %d\n", GET_SENSOR_STATE(27));
			printf("Sensor[28]: %d - ", GET_SENSOR_STATE(28));
			printf("Sensor[29]: %d - ", GET_SENSOR_STATE(29));
			printf("Sensor[30]: %d - ", GET_SENSOR_STATE(30));
			printf("Sensor[31]: %d\n", GET_SENSOR_STATE(31));
			printf("\n");
			
			u16ConvResult = xmega_generate_adc_random_value(&ADCA, ADC_REFSEL_INT1V_gc, ADC_CH_MUXPOS_PIN5_gc);
			printf("Opto = %i\n", u16ConvResult);
			/*
			printf("\nCh_Sig[0]: %u	", qt_measure_data.channel_signals[0]);
			printf("Ch_Sig[1]: %u	", qt_measure_data.channel_signals[1]);
			printf("Ch_Sig[2]: %u	", qt_measure_data.channel_signals[2]);
			printf("Ch_Sig[3]: %u", qt_measure_data.channel_signals[3]);
			printf("\nCh_Sig[4]: %u	", qt_measure_data.channel_signals[4]);
			printf("Ch_Sig[5]: %u	", qt_measure_data.channel_signals[5]);
			printf("Ch_Sig[6]: %u	", qt_measure_data.channel_signals[6]);
			printf("Ch_Sig[7]: %u", qt_measure_data.channel_signals[7]);
			printf("\nCh_Sig[8]: %u	", qt_measure_data.channel_signals[8]);
			printf("Ch_Sig[9]: %u	", qt_measure_data.channel_signals[9]);
			
			printf("Ch_Sig[10]: %u	", qt_measure_data.channel_signals[10]);
			printf("Ch_Sig[11]: %u", qt_measure_data.channel_signals[11]);
			printf("\nCh_Sig[12]: %u	", qt_measure_data.channel_signals[12]);
			printf("Ch_Sig[13]: %u	", qt_measure_data.channel_signals[13]);
			printf("Ch_Sig[14]: %u	", qt_measure_data.channel_signals[14]);
			printf("Ch_Sig[15]: %u", qt_measure_data.channel_signals[15]);
			printf("\nCh_Sig[16]: %u	", qt_measure_data.channel_signals[16]);
			printf("Ch_Sig[17]: %u	", qt_measure_data.channel_signals[17]);
			printf("Ch_Sig[18]: %u	", qt_measure_data.channel_signals[18]);
			printf("Ch_Sig[19]: %u", qt_measure_data.channel_signals[19]);
			printf("\nCh_Sig[20]: %u	", qt_measure_data.channel_signals[20]);
			
			printf("Ch_Sig[21]: %u	", qt_measure_data.channel_signals[21]);
			printf("Ch_Sig[22]: %u	", qt_measure_data.channel_signals[22]);
			printf("Ch_Sig[23]: %u", qt_measure_data.channel_signals[23]);
			printf("\nCh_Sig[24]: %u	", qt_measure_data.channel_signals[24]);
			printf("Ch_Sig[25]: %u	", qt_measure_data.channel_signals[25]);
			printf("Ch_Sig[26]: %u	", qt_measure_data.channel_signals[26]);
			printf("Ch_Sig[27]: %u", qt_measure_data.channel_signals[27]);
			printf("\nCh_Sig[28]: %u	", qt_measure_data.channel_signals[28]);
			printf("Ch_Sig[29]: %u	", qt_measure_data.channel_signals[29]);
			printf("Ch_Sig[30]: %u	", qt_measure_data.channel_signals[30]);
			printf("Ch_Sig[31]: %u", qt_measure_data.channel_signals[31]); */
			
			printf("\ndelta[0]: %i	", qt_get_sensor_delta(0));
			printf("delta[1]: %i	", qt_get_sensor_delta(1));
			printf("delta[2]: %i	", qt_get_sensor_delta(2));
			printf("delta[3]: %i", qt_get_sensor_delta(3));
			printf("\ndelta[4]: %i	", qt_get_sensor_delta(4));
			printf("delta[5]: %i	", qt_get_sensor_delta(5));
			printf("delta[6]: %i	", qt_get_sensor_delta(6));
			printf("delta[7]: %i", qt_get_sensor_delta(7));
			printf("\ndelta[8]: %i	", qt_get_sensor_delta(8));
			printf("delta[9]: %i	", qt_get_sensor_delta(9));
			
			printf("delta[10]: %i	", qt_get_sensor_delta(10));
			printf("delta[11]: %i", qt_get_sensor_delta(11));
			printf("\ndelta[12]: %i	", qt_get_sensor_delta(12));
			printf("delta[13]: %i	", qt_get_sensor_delta(13));
			printf("delta[14]: %i	", qt_get_sensor_delta(14));
			printf("delta[15]: %i", qt_get_sensor_delta(15));
			printf("\ndelta[16]: %i	", qt_get_sensor_delta(16));
			printf("delta[17]: %i	", qt_get_sensor_delta(17));
			printf("delta[18]: %i	", qt_get_sensor_delta(18));
			printf("delta[19]: %i", qt_get_sensor_delta(19));
			printf("\ndelta[20]: %i	", qt_get_sensor_delta(20));
			
			printf("delta[21]: %i	", qt_get_sensor_delta(21));
			printf("delta[22]: %i	", qt_get_sensor_delta(22));
			printf("delta[23]: %i", qt_get_sensor_delta(23));
			printf("\ndelta[24]: %i	", qt_get_sensor_delta(24));
			printf("delta[25]: %i	", qt_get_sensor_delta(25));
			printf("delta[26]: %i	", qt_get_sensor_delta(26));
			printf("delta[27]: %i", qt_get_sensor_delta(27));
			printf("\ndelta[28]: %i	", qt_get_sensor_delta(28));
			printf("delta[29]: %i	", qt_get_sensor_delta(29));
			printf("delta[30]: %i	", qt_get_sensor_delta(30));
			printf("delta[31]: %i", qt_get_sensor_delta(31));
			
			gCounter = 0;
		}

    /*  Time Non-critical host application code goes here  */
    }
}
