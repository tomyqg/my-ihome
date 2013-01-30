/*
 * Multi_master_Network.c
 *
 * Created: 2013-01-23 18:08:26
 *  Author: fidectom
 */ 

#include <avr/io.h>
#include <stdio.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include "utils.h"
#include "board_config.h"
#include "usart_driver/usart_driver.h"
#include "nvm_driver/nvm_driver.h"
#include "tc_driver.h"
#include "smm_NetworkSM.h"
// #include "crc_driver/crc.h"
//#include <util/crc16.h>


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
	while ( !(USART_TERMINAL.STATUS & USART_DREIF_bm) );
	
	// Put our character into the transmit buffer
	USART_TERMINAL.DATA = a_inChar;
	
	return 0;
}

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
	// USART_TERMINAL_PORT.DIRSET = PIN7_bm;
	USART_TERMINAL_PORT.DIRSET = TPASTE3(PIN, 7, _bm);
	USART_TERMINAL_PORT.OUTSET = TPASTE3(PIN, 7, _bm);
	
	/* PC6 (RXD) as input */
	USART_TERMINAL_PORT.DIRCLR = TPASTE3(PIN, 6, _bm);
	
	/* Enable system clock to peripheral */
	// Should be enabled after restart - USARTC1
	PR.PRPD &= ~PR_USART1_bm;
	
	/* Set the baud rate: use BSCALE and BSEL */
	xmega_set_usart_baudrate(&USART_TERMINAL, 3205, -7);	// 19200bps
	
	/* Set frame format */
	xmega_set_usart_format(&USART_TERMINAL, USART_TERMINAL_CHAR_LENGTH, USART_TERMINAL_PARITY, USART_TERMINAL_STOP_BIT);

	/* Set mode */
	xmega_set_usart_mode(&USART_TERMINAL, USART_CMODE_ASYNCHRONOUS_gc);
	
	/* Set interrupts level */
	xmega_set_usart_rx_interrupt_level(&USART_TERMINAL, USART_RXCINTLVL_LO_gc);
	xmega_set_usart_tx_interrupt_level(&USART_TERMINAL, USART_TXCINTLVL_LO_gc);
	
	/* Enable transmitter and receiver */
	xmega_enable_usart_tx(&USART_TERMINAL);
	xmega_enable_usart_rx(&USART_TERMINAL);
	
	/* Enable global interrupts */
	sei();
	
}	// usartCommTerminalInit()

/**
* \brief Configure XMEGA oscillator and clock source.
*z 
* This function enables internal 32MHz oscillator and sets it as main cpu clock.
* Prescaler (A:B:C) are configured as (4:1:1) to provide 8MHz clock to peripherals.
*
* \param none.
*
* \retval none.
*/
void xmega_set_cpu_clock_to_32MHz(void)
{
	uint8_t u8PrescalerConfig;
	uint8_t u8ClockControl;
	
	/*  Enable internal 32MHz ring oscillator. */
	OSC.CTRL |= OSC_RC32MEN_bm;
	
	/*  Wait until oscillator is ready. */
	while ((OSC.STATUS & OSC_RC32MRDY_bm) == 0);

	/*  Select Prescaler A divider as 4 and Prescaler B & C divider as (1,1) respectively.  */
	/*  Overall divide by 4 i.e. A*B*C  */
	u8PrescalerConfig = (uint8_t)(CLK_PSADIV_4_gc | CLK_PSBCDIV_1_1_gc);
	
	/* Disable register security for clock update */
	CCP = CCP_IOREG_gc;
	CLK.PSCTRL = u8PrescalerConfig;
	
	/*  Set the 32 MHz ring oscillator as the main clock source */
	u8ClockControl = ( CLK.CTRL & ~CLK_SCLKSEL_gm ) | CLK_SCLKSEL_RC32M_gc;

	/* Disable register security for clock update */
	CCP = CCP_IOREG_gc;
	CLK.CTRL = u8ClockControl;
}

// CRC-16(CRC-CCITT) Polynom x^16 + x^12 + x^5 + 1 --> 0x1021
uint16_t calculate_crc16_checksum(uint8_t *a_pData, uint8_t count)
{
	uint8_t i = 0;
	uint16_t crc16_checksum = 0;
	
	// Die 4 Checksum Register auf 0x00 zurücksetzen
	CRC_CTRL |= CRC_RESET_RESET0_gc;
	
	// The CRC registers will be reset one peripheral clock cycle after the RESET[1] bit is set
	// asm("nop");
	_NOP();
	
	// Den CRC-Kern auf CRC16 konfigurieren
	CRC_CTRL &= ~CRC_CRC32_bm;
	
	// IO als Quelle für den CRC-Kern auswhählen
	CRC_CTRL = (CRC_CTRL & (~CRC_SOURCE_gm)) | CRC_SOURCE_IO_gc;
	
	for(i = 0; i < count; i++)
	{
		CRC_DATAIN = a_pData[i];		
	}

	// Signal CRC complete
	CRC_STATUS |= CRC_BUSY_bm;
	
	// Check if the CRC module is busy
	while((CRC_STATUS & CRC_BUSY_bm) == CRC_BUSY_bm);
	
	crc16_checksum = ((uint16_t)CRC_CHECKSUM0 & 0x00FF);
	crc16_checksum |= (((uint16_t)CRC_CHECKSUM1 << 8) & 0xFF00);
	
	// Den CRC-Kern anhalten, indem keine Quellec angegeben wird
	CRC_CTRL = (CRC_CTRL & (~CRC_SOURCE_gm)) | CRC_SOURCE_DISABLE_gc;
	
	return crc16_checksum;
}

/* *********************************************************************** */
/* ******************** INTERRUPT EVENT DEFINITIONS ********************** */
/* *********************************************************************** */
#define EVENT_BUSYLINE_TIMER_bm		(1 << 0)
#define EVENT_NORESPONSE_TIMER_bm	(1 << 1)

// Bitmasked flags that describe what interrupt has occurred
volatile uint16_t gInterruptEvents = 0;

/*! \brief Busy line timer overflow interrupt service routine.
 *
 *  Busy line timer overflow interrupt handler.
 *  Be as quick as possible and only set the flag of corresponding event.
 */
ISR(TCD0_OVF_vect)
{	
	// Signal busy line timer expiration
	gInterruptEvents |= EVENT_BUSYLINE_TIMER_bm;
}

/*! \brief No response timer overflow interrupt service routine.
 *
 *  No response timer overflow interrupt handler.
 *  Be as quick as possible and only set the flag of corresponding event.
 */
ISR(TCE0_OVF_vect)
{	
	// Signal no response timer expiration
	gInterruptEvents |= EVENT_NORESPONSE_TIMER_bm;
}

//! Storage for serial number
struct nvm_device_serial xmegaSerialNumber;

// CRC-16 checksum
uint16_t g_u16crc16_checksum;

int main(void)
{
	// Variable to read interrupt event flags
	uint16_t u16EventFlags;
 
    // Configure cpu and peripherals clock
	xmega_set_cpu_clock_to_32MHz();
	
	// Enable interrupts
	PMIC.CTRL = PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_HILVLEN_bm;
	
	// Power management - configure sleep mode to IDLE
	set_sleep_mode(SLEEP_SMODE_IDLE_gc);
	// Enable sleep
	sleep_enable();
	
	// Initialize serial communication terminal
	usartCommTerminalInit();
	
	// Redirect stream to standard output
	stdout = &mystdout;
	
	/* Print out welcome message */
	printf("Multi-master Network ver 1.0\n");

	// Initialize GPIO related to RS-485 driver
	rs485_driver_gpio_initialize();
	// Initially go LOW to enable receiver - start listening
	rs485_receiver_enable();

	/********************************************//**
	 * Busy Line timer configuration section
	 ***********************************************/
	
	/*
	 * Enable the timer, and set it to count up.
	 * When it overflows, it triggers the message to the Host.
	 * 0.25Hz tick.
	 */
	
	TIMER_BUSY_LINE.CTRLB		= (TIMER_BUSY_LINE.CTRLB & ~TC0_WGMODE_gm) | TC_WGMODE_NORMAL_gc;
	TIMER_BUSY_LINE.CTRLFSET	= 0;																	// set UP direction
	TIMER_BUSY_LINE.PER			= TIMER_BUSY_LINE_PERIOD;												// set defined period
	TIMER_BUSY_LINE.INTCTRLA	= TIMER_BUSY_LINE.INTCTRLA & ~TC0_OVFINTLVL_gm;							// set overflow interrupt
	TIMER_BUSY_LINE.INTCTRLA	= TIMER_BUSY_LINE.INTCTRLA | (TC_OVFINTLVL_LO_gc << TC0_OVFINTLVL_gp);	// set low-level overflow interrupt
	TIMER_BUSY_LINE.CTRLA		= (TIMER_BUSY_LINE.CTRLA & ~TC0_CLKSEL_gm) | TC_CLKSEL_DIV256_gc;
	
	/********************************************//**
	 * No response timer configuration section
	 ***********************************************/
	TIMER_NO_RESPONSE.CTRLB		= (TIMER_NO_RESPONSE.CTRLB & ~TC0_WGMODE_gm) | TC_WGMODE_NORMAL_gc;
	TIMER_NO_RESPONSE.CTRLFSET	= 0;																	// set UP direction
	TIMER_NO_RESPONSE.PER		= TIMER_NO_RESPONSE_PERIOD;												// set defined period
	TIMER_NO_RESPONSE.INTCTRLA	= TIMER_NO_RESPONSE.INTCTRLA & ~TC0_OVFINTLVL_gm;							// set overflow interrupt
	TIMER_NO_RESPONSE.INTCTRLA	= TIMER_NO_RESPONSE.INTCTRLA | (TC_OVFINTLVL_LO_gc << TC0_OVFINTLVL_gp);	// set low-level overflow interrupt
	TIMER_NO_RESPONSE.CTRLA		= (TIMER_NO_RESPONSE.CTRLA & ~TC0_CLKSEL_gm) | TC_CLKSEL_DIV1024_gc;
	
	// Read device serial number
	nvm_read_device_serial(&xmegaSerialNumber);

	// Print out device serial number elements
	/* printf("LOTNUM0: %02x\n", xmegaSerialNumber.lotnum0);
	printf("LOTNUM1: %02x\n", xmegaSerialNumber.lotnum1);
	printf("LOTNUM2: %02x\n", xmegaSerialNumber.lotnum2);
	printf("LOTNUM3: %02x\n", xmegaSerialNumber.lotnum3);
	printf("LOTNUM4: %02x\n", xmegaSerialNumber.lotnum4);
	printf("LOTNUM5: %02x\n", xmegaSerialNumber.lotnum5);
	printf("WAFNUM:  %02x\n", xmegaSerialNumber.wafnum);
	printf("COORDX0: %02x\n", xmegaSerialNumber.coordx0);
	printf("COORDX1: %02x\n", xmegaSerialNumber.coordx1);
	printf("COORDY0: %02x\n", xmegaSerialNumber.coordy0);
	printf("COORDY1: %02x\n", xmegaSerialNumber.coordy1); */

	// Calculate CRC-16 checksum for the data using ASF
	/* g_u32crc16_checksum = crc_io_checksum((void *)&xmegaSerialNumber.u8DataArray[0], 11, CRC_16BIT);
	printf("SNum crc-16: 0x%04x\n", g_u32crc16_checksum); */
	
	// Calculate CRC-16 (CRC-CCITT) using XMEGA hardware CRC peripheral
	g_u16crc16_checksum = calculate_crc16_checksum(&xmegaSerialNumber.u8DataArray[0], 11);
	printf("crc-16: 0x%04x\n", g_u16crc16_checksum);

	// CRC-16 calculation using AVR-GCC library. xmodem version */
	/* uint16_t crc = 0;
	for( uint8_t u8idx = 0; u8idx < 11; u8idx++)
		crc = _crc_xmodem_update(crc,xmegaSerialNumber.u8DataArray[u8idx]);

	printf("crc-16 xmodem: 0x%04x\n", crc); */

	// Start infinite main loop, go to sleep and wait for interruption
	for(;;)
    {
		// Atomic interrupt safe read of global variable storing event flags
        u16EventFlags = gInterruptEvents;
        
        while (u16EventFlags)
        {
	        // Note: Each handler will clear the relevant bit in global variable gInterruptEvents
	        
			if (u16EventFlags & EVENT_BUSYLINE_TIMER_bm)
	        {
		        // Busy line timer interrupt fired up
		        // HandleUSARTD0RXC();
				printf("BusyLine timer expired\n");
				
				// Clear corresponding event flag
				gInterruptEvents &= ~EVENT_BUSYLINE_TIMER_bm;
	        }
	        
	        if (u16EventFlags & EVENT_NORESPONSE_TIMER_bm)
	        {
		        // No response timer overflow interrupt fired up
		        // HandleHeartbeatTimer();
				printf("NoResponse timer expired\n");
				
				// Clear corresponding event flag
				gInterruptEvents &= ~EVENT_NORESPONSE_TIMER_bm;
	        }
	        
	        u16EventFlags = gInterruptEvents;
        }
        
        // Read the event register again without allowing any new interrupts
        // cpu_irq_disable();
		cli();
		
        if (0 == gInterruptEvents)
        {
	        // cpu_irq_enable();
			sei();
			
	        // Go to sleep, everything is handled by interrupts.
	        // An interrupt will cause a wake up and run the while loop
	        sleep_cpu();
        };
        
        sei();
    };
}