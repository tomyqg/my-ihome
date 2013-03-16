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
#include <avr/eeprom.h>
#include <util/atomic.h>
#include <string.h>
#include <stdlib.h>
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
	// USART_TERMINAL_PORT.DIRSET = TPASTE3(PIN, 7, _bm);
	// USART_TERMINAL_PORT.OUTSET = TPASTE3(PIN, 7, _bm);
	PORT_DIRSET(USART_TERMINAL_TX_IO);
	PORT_OUTSET(USART_TERMINAL_TX_IO);
	
	/* PC6 (RXD) as input */
	// USART_TERMINAL_PORT.DIRCLR = TPASTE3(PIN, 6, _bm);
	PORT_DIRCLR(USART_TERMINAL_RX_IO);
	
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
	// xmega_set_usart_rx_interrupt_level(&USART_TERMINAL, USART_RXCINTLVL_LO_gc);
	// xmega_set_usart_tx_interrupt_level(&USART_TERMINAL, USART_TXCINTLVL_LO_gc);
	
	/* Enable transmitter and receiver */
	xmega_enable_usart_tx(&USART_TERMINAL);
	xmega_enable_usart_rx(&USART_TERMINAL);
	
}	// usartCommTerminalInit()

/********************************************//**
* Busy Line timer configuration section
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

// Global bitmasked flags that describe what interrupt has occurred
volatile uint16_t gSystemEvents = 0;

// Variable to store current event flag
uint16_t u16EventFlags;

/* Flag to indicate that the line is busy */
extern volatile uint8_t gBusyLine;

// Structure for storing received frame
extern mmsn_comm_data_frame_t gCommDataFrameReceive; 
// Structure for storing frame to be transmitted
extern mmsn_comm_data_frame_t gCommDataFrameTransmit;

// Pointer to the transmitted frame
mmsn_comm_data_frame_t * pSendingFrame = &gCommDataFrameTransmit;

/* Current state of Network State Machine */
extern volatile eSM_StateType gNSM_CurrentState;

/* Previous state of Network State Machine */
extern volatile eSM_StateType gNSM_PreviousState;

// Lookup table containing a pointer to the FSM handlers
extern void (*SM_stateTable[])(void);

/*! \brief System heartbeat timer overflow interrupt service routine.
 *
 *  System heartbeat timer overflow interrupt handler.
 *  Be as quick as possible and only set the flag of corresponding event.
 */
ISR(TCC0_OVF_vect)
{	
	// Notify that no response timer expired
	gSystemEvents |= EVENT_IRQ_HEARTBEAT_TIMEOUT_bm;
}

/*! \brief Busy line timer overflow interrupt service routine.
 *
 *  Busy line timer overflow interrupt handler.
 *  Be as quick as possible and only set the flag of corresponding event.
 */
ISR(TCD0_OVF_vect)
{	
	// Signal busy line timer expiration
	gSystemEvents |= EVENT_IRQ_COLLISION_AVOIDANCE_TIMEOUT_bm;
}

/*! \brief No response timer overflow interrupt service routine.
 *
 *  No response timer overflow interrupt handler.
 *  Be as quick as possible and only set the flag of corresponding event.
 */
ISR(TCE0_OVF_vect)
{	
	// Notify that no response timer expired
	gSystemEvents |= EVENT_IRQ_WAIT_FOR_RESPONSE_TIMEOUT_bm;
}

/*! \brief Reception Complete Interrupt interrupt service routine.
 *
 *  USART data receive interrupt handler.
 *  Be as quick as possible and only set the flag of corresponding event.
 */
ISR(USARTD1_RXC_vect, ISR_BLOCK)
{
	// Notify that new data arrived on the bus
	gSystemEvents |= EVENT_IRQ_RECEIVE_COMPLETE_bm;
}

/* Data Register Empty Interrupt */
ISR(USARTD1_DRE_vect, ISR_BLOCK)
{
	// Set Data Register Empty event
	gSystemEvents |= EVENT_IRQ_DATA_REGISTER_EMPTY_bm;
	
	/* DREIF is cleared by writing DATA.
	   Disable DRE interrupt because DATA is read in corresponding FSM handler.
       Otherwise new interrupt will occur directly after the return from IRQ handler. */
	xmega_set_usart_dre_interrupt_level (&USART_COMMUNICATION_BUS, USART_DREINTLVL_OFF_gc);
}

/* Transmit Complete Interrupt */
ISR(USARTD1_TXC_vect, ISR_BLOCK)
{
	gSystemEvents |= EVENT_IRQ_TRANSMIT_COMPLETE_bm;
}

//! Storage for serial number
struct nvm_device_serial xmegaSerialNumber;

// Global variable for storing Collision Avoidance timer period value
uint16_t g_u16CollisionAvoidancePeriod;

// Global variable indicate device configuration status
eConfigStatus_t g_DeviceConfigStatus;

// Structure to store device configuration data
typedef struct
{
	uint8_t u8LogicalNetworkAddr;	// Logical address in the network
} ConfigData_t;

/* EEPROM variables section */

// Create variable in EEPROM without initial values
ConfigData_t EEMEM eeConfigurationData;

// Create variable in RAM 
ConfigData_t ConfigurationData;

inline void xmega_read_configuration_data(void)
{
	// Read a block of bytes from EEPROM
	eeprom_read_block(&ConfigurationData, &eeConfigurationData, sizeof(ConfigData_t));
}

inline void xmega_save_configuration_data(void)
{
	// Update a block of bytes to EEPROM address
	eeprom_update_block(&ConfigurationData, &eeConfigurationData, sizeof(ConfigData_t));
}

int main(void)
{
	
	// Configure CPU and peripherals clock
	xmega_set_cpu_clock_to_32MHz();
	
	// Enable interrupts
	PMIC.CTRL = PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_HILVLEN_bm;
	
	// Power management - configure sleep mode to IDLE
	set_sleep_mode(SLEEP_SMODE_IDLE_gc);
	// Enable sleep mode
	sleep_enable();
	
	// Initialize serial communication terminal
	usartCommTerminalInit();
	
	// Redirect stream to standard output
	stdout = &mystdout;
	
	/* Print out welcome message */
	printf("Multi-master Network ver 1.0\n");
	
	// Calculate CRC-16 value based on the random Internal SRAM memory content.
	// Move by 0x400 from the beginning to omit memory area related to global variables section.
	// Take 20 consecutive SRAM bytes for CRC-16 calculation.
	uint16_t u16RandomValue = xmega_calculate_checksum_crc16((uint8_t *)(INTERNAL_SRAM_START + 0x400), 20);
	// Pseudo-random number generator seeding with previously obtain value
	srand(u16RandomValue);
	
#ifdef MMSN_DEBUG
	printf("rnd = %d\n", u16RandomValue);
#endif	
	
	// Read XMEGA device serial number
	nvm_read_device_serial(&xmegaSerialNumber);

	// Read configuration data from EEPROM
	xmega_read_configuration_data();

	// Check if logical address was already assigned
	if (true == isLogicalNetworkAddrAssigned(&ConfigurationData.u8LogicalNetworkAddr))
	{
		// Logical Network Address was already assigned.
		g_DeviceConfigStatus = eLogicalAddrAssigned;
		
		// Calculate Collision Avoidance (back-off/busy line) timer period value.
		g_u16CollisionAvoidancePeriod = (ConfigurationData.u8LogicalNetworkAddr * TIMER_COLLISION_AVOIDANCE_520us_VALUE) +
									   TIMER_COLLISION_AVOIDANCE_32us_VALUE;
#ifdef MMSN_DEBUG
		printf("LNA_CA = %d\n", g_u16CollisionAvoidancePeriod);
#endif									   
	}
	else
	{
		// Logical Network Address was NOT assigned.
		g_DeviceConfigStatus = eLogicalAddrNotAssigned;
		
		// Calculate Collision Avoidance (back-off/busy line) timer period using random value.
		g_u16CollisionAvoidancePeriod = (xmega_generate_random_logical_network_address() * TIMER_COLLISION_AVOIDANCE_520us_VALUE) +
		TIMER_COLLISION_AVOIDANCE_32us_VALUE;

#ifdef MMSN_DEBUG
		printf("RND_CA = %d\n", g_u16CollisionAvoidancePeriod);
#endif		
	}
	
	/************************************************************************/
	/* TIMERS CONFIGURATION                                                 */
	/************************************************************************/
	
	// No response timer - configure but do not run
	xmega_timer_config(&TIMER_NO_RESPONSE, TC_CLKSEL_OFF_gc, TIMER_NO_RESPONSE_PERIOD);
	
	// Collision Avoidance timer - configure but do not run
	xmega_timer_config(&TIMER_COLLISION_AVOIDANCE, TC_CLKSEL_OFF_gc, g_u16CollisionAvoidancePeriod);
	
	// Heartbeat timer - configure but do not run
	xmega_timer_config(&TIMER_COLLISION_AVOIDANCE, TC_CLKSEL_OFF_gc, TIMER_HEARTBEAT_PERIOD);
	
	/************************************************************************/
	/* RS-485 PHYSICAL DEVICE CONFIGURATION						            */
	/************************************************************************/
	// Initialize GPIO related to RS-485 driver
	rs485_driver_gpio_initialize();
	// Initially go LOW to enable receiver - start listening
	rs485_receiver_enable();
		
	// Force the state of the SREG register on exit, disabling the Global Interrupt Status flag bit.
	/* ATOMIC_BLOCK(NONATOMIC_FORCEOFF)
	{
		// Clear busy line timeout event
		FLAG_CLEAR(gSystemEvents, EVENT_IRQ_COLLISION_AVOIDANCE_TIMEOUT_bm);
	}
	
	// Turn busy line timer off
	xmega_tc_select_clock_source(&TIMER_COLLISION_AVOIDANCE, TC_CLKSEL_OFF_gc); */
	
	/************************************************************************/
	/* Initialize Multi-Master Serial Network State Machine                 */
	/************************************************************************/
	SM_stateTable[eSM_Initialize]();
	
	// Turn on global interrupts
	sei();

	/************************************************************************/
	/* Start infinite main loop, go to sleep and wait for interruption      */
	/************************************************************************/
	for(;;)
    {
		// Force the state of the SREG register on exit, enabling the Global Interrupt Status flag bit.
        ATOMIC_BLOCK(ATOMIC_FORCEON)
		{
			// Atomic interrupt safe read of global variable storing event flags
			u16EventFlags = gSystemEvents;
		}
		
        while (u16EventFlags)
        {
	        // Note: Each handler will clear the relevant bit in global variable gSystemEvents
			// Corresponding event flag will be cleared in FSM handler
			
			// Call corresponding state machine handler
			SM_stateTable[gNSM_CurrentState]();
			
			// Read the flags again after handler return
			ATOMIC_BLOCK(ATOMIC_FORCEON)
			{
				// Atomic interrupt safe read of global variable storing event flags
				u16EventFlags = gSystemEvents;
			}
        }	// while(...)
        
        // Read the event register again without allowing any new interrupts
		cli();
		
        if (0 == gSystemEvents)
        {
			sei();
			
	        // Go to sleep, everything is handled by interrupts.
	        // An interrupt will cause a wake up and run the while loop
	        sleep_cpu();
        };
        
		// Set Global Interrupt Status flag
        sei();
    };
};

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
	
// CRC-16 calculation using AVR-GCC library. xmodem version */
	/* uint16_t crc = 0;
	for( uint8_t u8idx = 0; u8idx < 11; u8idx++)
		crc = _crc_xmodem_update(crc,xmegaSerialNumber.u8DataArray[u8idx]);

	printf("crc-16 xmodem: 0x%04x\n", crc); */
	
/************************************************************************/
/* RANDOM VALUE GENERATION                                              */
/************************************************************************/
// Get random value from ADC conversion on pin 5
/* uint16_t u16ConvResult = xmega_generate_adc_random_value(&ADCA, ADC_REFSEL_INT1V_gc, ADC_CH_MUXPOS_PIN5_gc);
printf("ADC conversion on PIN5 = %i\n", u16ConvResult); */ 	


// Clear data buffer for sending frame
// memset(pSendingFrame, 0x00, sizeof(mmsn_comm_data_frame_t));

// Compose data frame
//set_MMSN_DeviceType(MY_DEVICE_TYPE, pSendingFrame->u16Identifier);							// set Device Type
//set_MMSN_DeviceNumber(MMSN_DEFAULT_LOGICAL_NETWORK_ADDRESS, pSendingFrame->u16Identifier);	// set Device Number
//set_MMSN_RTR(eRTR_DataFrame, pSendingFrame->u16Identifier);									// set Remote Transmission Request
//set_MMSN_CTRLF(0, pSendingFrame->u16Identifier);											// set Control Field

// TODO: data buffer with Assign Network Address request

// Calculate CRC-16 (CRC-CCITT) using XMEGA hardware CRC peripheral
//uint16_t u16TempCRC;
//u16TempCRC = xmega_calculate_checksum_crc16(pSendingFrame->u8CommFrameArray, MMSN_FRAME_NOCRC_LENGTH);
//pSendingFrame->u16CRC16 = u16TempCRC;

// Indicate that frame is prepared for transmission
//gSystemEvents |= EVENT_SW_DATA_READY_TO_SEND_bm;

// Initialize GPIO related to RS-485 driver
//rs485_driver_gpio_initialize();
// Enable RS-485 driver for transmission
//rs485_driver_enable();