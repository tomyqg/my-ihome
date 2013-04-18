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
#include <avr/pgmspace.h>
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
#endif

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

// Multi-Master Serial Network FSM
extern MMSN_FSM_t mmsnFSM;

// MMSN event handlers table
extern mmsnFsmEventHandler mmsnFSMActionTable[][MMSN_MAX_EVENTS];

//! Pointer to Receiver FSM action function pointer array
// mmsnFsmEventHandler (* MMSNActionTablePtr)[MMSN_MAX_EVENTS];	

// Network Error descriptor
extern NetworkErrorDesc_t g_NetworkErrorDesc;

// Lookup table containing a pointer to the FSM handlers
extern void (*SM_stateTable[])(void);


/************************************************************************/
/* EVENT FIFO                                                           */
/************************************************************************/
/* Size of Network receiving FIFO buffer */
#define EVENT_FIFO_BUFFER_SIZE	(8)

/**
 * \brief Buffer to associate with event FIFO
 *
 * This buffer consists of \ref FIFO_RECEIVE_BUFFER_SIZE elements
 * capable of holding a byte
 */
uint8_t EventQueue[EVENT_FIFO_BUFFER_SIZE];

/**
 * \brief Event queue buffer descriptor.
 *
 * its size and where to read from or write to upon the next buffer pull or
 * push. This is required to access the FIFO buffer via the FIFO service.
 *
 * \pre The descriptor must be initialized with \ref fifo_init() before the FIFO
 * buffer can be used.
 *
 * \note The FIFO buffer should be used with only one of its supported data types
 * at a time, or the buffered values will be corrupted unless special conditions
 * are met.
 */
fifo_desc_t eventQueue_desc;

// Global variable to store arguments passed to event handler function
uint8_t g_u8GlobalArgument;

/************************************************************************/
/* SYSTEM EVENTS                                                        */
/************************************************************************/
//! System heartbeat timer timeout
#define SYSEV_HEARTBEAT_TIMEOUT_bm				(1 << 0)
//! Collision Avoidance timer timeout
#define SYSEV_COLLISION_AVOIDANCE_TIMEOUT_bm	(1 << 1)
//! No response timer timeout
#define SYSEV_NO_RESPONSE_TIMEOUT_bm			(1 << 2)
//! Received data from serial bus
#define SYSEV_RECEIVE_COMPLETE_bm				(1 << 3)
//! Data register empty
#define SYSEV_DATA_REGISTER_EMPTY_bm			(1 << 4)
//! Frame completely transmitted
#define SYSEV_TRANSMIT_COMPLETE_bm				(1 << 5)

/************************************************************************/
/* SYSTEM EVENT HANDLERS                                                */
/************************************************************************/
void sys_HeartbeatTimeout_Handler(void)
{
	// Clear corresponding event flag
	FLAG_CLEAR(gSystemEvents, SYSEV_HEARTBEAT_TIMEOUT_bm);
}

void sysev_CollisionAvoidanceTimeout_Handler(void)
{
	ADD_EVENT_TO_QUEUE(&eventQueue_desc, MMSN_COLLISION_AVOIDANCE_TIMEOUT);
	
	// Clear corresponding event flag
	FLAG_CLEAR(gSystemEvents, SYSEV_COLLISION_AVOIDANCE_TIMEOUT_bm);
}

void sysev_NoResponseTimeout_Handler(void)
{
	ADD_EVENT_TO_QUEUE(&eventQueue_desc, MMSN_NO_RESPONSE_TIMEOUT_EVENT);
	
	// Clear corresponding event flag
	FLAG_CLEAR(gSystemEvents, SYSEV_NO_RESPONSE_TIMEOUT_bm);
}

void sysev_ReceiveComplete_Handler(void)
{
	// Check for errors on a hardware level
	
	// Due to the buffering of the error flags, the status register must be read before the receive buffer
	// (DATA), since reading the DATA location changes the FIFO buffer.
	uint8_t u8UsartErrorFlags = 0;
	
	// Read USART receiver error flags
	u8UsartErrorFlags = USART_COMMUNICATION_BUS.STATUS & (USART_FERR_bm | USART_BUFOVF_bm | USART_PERR_bm);
	
	// Read the buffer. It will automatically clear RXCIF flag
	g_u8GlobalArgument = USART_COMMUNICATION_BUS.DATA;

	//printf("\nRXC=%d", g_u8GlobalArgument);

	// Decision making process based on receiver error flags
	if (0 == u8UsartErrorFlags)
	{
		// Received data is error free.
		ADD_EVENT_TO_QUEUE(&eventQueue_desc, MMSN_DATA_RECEIVED_EVENT);
	}
	else
	{
		// If data was erroneous at hardware level than discard the data.
		
		// Store network error type
		g_NetworkErrorDesc.currError = NE_USART_Receiver_Error;
	
		// Add current Network Error to the table
		add_commNetworkError(&g_NetworkErrorDesc);
		
		// Clear current network error in the descriptor
		g_NetworkErrorDesc.currError = NE_None;
		
		// Force transition to the ERROR state
		// mmsnFSM.CurrentState = eSM_Error;
	}
	
	// Clear corresponding system event flag
	FLAG_CLEAR(gSystemEvents, SYSEV_RECEIVE_COMPLETE_bm);
}

void sysev_DataRegisterEmpty_Handler(void)
{
	ADD_EVENT_TO_QUEUE(&eventQueue_desc, MMSN_DATA_REG_EMPTY_EVENT);
	
	// Clear corresponding event flag
	FLAG_CLEAR(gSystemEvents, SYSEV_DATA_REGISTER_EMPTY_bm);
}

void sysev_TransmitComplete_Handler(void)
{
	
	// Clear corresponding event flag
	FLAG_CLEAR(gSystemEvents, SYSEV_TRANSMIT_COMPLETE_bm);
}

/************************************************************************/
/* INTERRUPT HANDLERS                                                   */
/************************************************************************/

/*! \brief System heartbeat timer overflow interrupt service routine.
 *
 *  System heartbeat timer overflow interrupt handler.
 *  Be as quick as possible and only set the flag of corresponding event.
 */
ISR(TCC0_OVF_vect)
{	
	// Notify that system heartbeat timer expired
	FLAG_SET(gSystemEvents, SYSEV_HEARTBEAT_TIMEOUT_bm);
}

/*! \brief Busy line timer overflow interrupt service routine.
 *
 *  Busy line timer overflow interrupt handler.
 *  Be as quick as possible and only set the flag of corresponding event.
 */
ISR(TCD0_OVF_vect)
{	
	// Signal Collision Avoidance (busy line) timer expiration
	FLAG_SET(gSystemEvents, SYSEV_COLLISION_AVOIDANCE_TIMEOUT_bm);
}

/*! \brief No response timer overflow interrupt service routine.
 *
 *  No response timer overflow interrupt handler.
 *  Be as quick as possible and only set the flag of corresponding event.
 */
ISR(TCE0_OVF_vect)
{	
	// Notify that no response timer expired
	FLAG_SET(gSystemEvents, SYSEV_NO_RESPONSE_TIMEOUT_bm);
}

/*! \brief Reception Complete Interrupt interrupt service routine.
 *
 * \Note Hardware specific handler
 *
 *  USART data receive interrupt handler.
 *  Be as quick as possible and only set the flag of corresponding event.
 */
ISR(USARTD0_RXC_vect, ISR_BLOCK)
{
	// Notify that new data arrived on the serial bus
	FLAG_SET(gSystemEvents, SYSEV_RECEIVE_COMPLETE_bm);
}

/* Data Register Empty Interrupt   */
/* \Note Hardware specific handler */
ISR(USARTD0_DRE_vect, ISR_BLOCK)
{
	// Set Data Register Empty event
	FLAG_SET(gSystemEvents, SYSEV_DATA_REGISTER_EMPTY_bm);
	
	/* DREIF is cleared by writing DATA.
	 * Disable DRE interrupt because DATA is read in corresponding FSM handler.
     * Otherwise new interrupt will occur directly after the return from IRQ handler.
	 */
	xmega_set_usart_dre_interrupt_level (&USART_COMMUNICATION_BUS, USART_DREINTLVL_OFF_gc);
}

/* Transmit Complete Interrupt	   */
/* \Note Hardware specific handler */
ISR(USARTD0_TXC_vect, ISR_BLOCK)
{
	FLAG_SET(gSystemEvents, SYSEV_TRANSMIT_COMPLETE_bm);
}

/************************************************************************/
/* GLOBALS                                                              */
/************************************************************************/
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

/************************************************************************/
/* COMMUNICATION                                                        */
/************************************************************************/

void syscmd_Module_Serial_Number_Handler(void)
{
	/* XMEGA device short version of serial number
	 * is comprised of 7 lower bytes of full serial number.
	 */
	
	printf_P(PSTR("\nShortSN "));
	
	// Copy 7 lower bytes to TX frame buffer
	memcpy((mmsnFSM.ptrTXDataFrame->u8DataBuffer + 1), &xmegaSerialNumber.u8DataArray[4], 7);
	
	for (uint8_t u8idx = 0; u8idx < MMSN_DATA_LENGTH; u8idx++)
	{
		printf("%u:", mmsnFSM.ptrTXDataFrame->u8DataBuffer[u8idx]);
	}
}

CommandDescriptor_t CmdDescTable[COMMAND_COUNT] = 
{
	{ 0,								NULL },		// 0
	{ 0,								NULL },		// 1
	{ 0,								NULL },		// 2
	{ 0,								NULL },		// 3
	{ 0,								NULL },		// 4
	{ 0,								NULL },		// 5
	{ 0,								NULL },		// 6
	{ 0,								NULL },		// 7
	{ 0,								NULL },		// 8
	{ 0,								NULL },		// 9
	{ 0,								NULL },		// 10
	{ 0,								NULL },		// 11
	{ 0,								NULL },		// 12
	{ 0,								NULL },		// 13
	{ 0,								NULL },		// 14
	{ 0,								NULL },		// 15
	{ 0,								NULL },		// 16
	{ 0,								NULL },		// 17
	{ 0,								NULL },		// 18
	{ 0,								NULL },		// 19
	{ 0,								NULL },		// 20
	{ 0,								NULL },		// 21
	{ 0,								NULL },		// 22
	{ 0,								NULL },		// 23
	{ 0,								NULL },		// 24
	{ 0,								NULL },		// 25
	{ 0,								NULL },		// 26
	{ 0,								NULL },		// 27
	{ 0,								NULL },		// 28
	{ 0,								NULL },		// 29
	{ 0,								NULL },		// 30
	{ 0,								NULL },		// 31
	{ 0,								NULL },		// 32
	{ 0,								NULL },		// 33
	{ 0,								NULL },		// 34
	{ 0,								NULL },		// 35
	{ 0,								NULL },		// 36
	{ 0,								NULL },		// 37
	{ 0,								NULL },		// 38
	{ 0,								NULL },		// 39
	{ 0,								NULL },		// 40
	{ 0,								NULL },		// 41
	{ 0,								NULL },		// 42
	{ 0,								NULL },		// 43
	{ 0,								NULL },		// 44
	{ 0,								NULL },		// 45
	{ 0,								NULL },		// 46
	{ 0,								NULL },		// 47
	{ 0,								NULL },		// 48
	{ 0,								NULL },		// 49
	{ 0,								NULL },		// 50
	{ 0,								NULL },		// 51
	{ 0,								NULL },		// 52
	{ 0,								NULL },		// 53
	{ 0,								NULL },		// 54
	{ 0,								NULL },		// 55
	{ 0,								NULL },		// 56
	{ 0,								NULL },		// 57
	{ 0,								NULL },		// 58
	{ 0,								NULL },		// 59
	{ 0,								NULL },		// 60
	{ 0,								NULL },		// 61
	{ 0,								NULL },		// 62
	{ 0,								NULL },		// 63
	{ 0,								NULL },		// 64
	{ 0,								NULL },		// 65
	{ 0,								NULL },		// 66
	{ 0,								NULL },		// 67
	{ 0,								NULL },		// 68
	{ 0,								NULL },		// 69
	{ 0,								NULL },		// 70
	{ 0,								NULL },		// 71
	{ 0,								NULL },		// 72
	{ 0,								NULL },		// 73
	{ 0,								NULL },		// 74
	{ 0,								NULL },		// 75
	{ 0,								NULL },		// 76
	{ 0,								NULL },		// 77
	{ 0,								NULL },		// 78
	{ 0,								NULL },		// 79
	{ 0,								NULL },		// 80
	{ 0,								NULL },		// 81
	{ 0,								NULL },		// 82
	{ 0,								NULL },		// 83
	{ 0,								NULL },		// 84
	{ 0,								NULL },		// 85
	{ 0,								NULL },		// 86
	{ 0,								NULL },		// 87
	{ 0,								NULL },		// 88
	{ 0,								NULL },		// 89
	{ 0,								NULL },		// 90
	{ 0,								NULL },		// 91
	{ 0,								NULL },		// 92
	{ 0,								NULL },		// 93
	{ 0,								NULL },		// 94
	{ 0,								NULL },		// 95
	{ 0,								NULL },		// 96
	{ 0,								NULL },		// 97
	{ 0,								NULL },		// 98
	{ 0,								NULL },		// 99
	{ 0,								NULL },		// 100				
	
	{ SYSCMD_GROUP_RESTART_REQ,			NULL },		// 101
	{ SYSCMD_MODULE_RESTART_REQ,		NULL },		// 102
	{ SYSCMD_GROUP_SERIAL_NUMBER_REQ,	NULL },		// 103
	{ SYSCMD_MODULE_SERIAL_NUMBER_REQ,	syscmd_Module_Serial_Number_Handler },		// 104
	{ 0,								NULL },		// 105
	{ 0,								NULL },		// 106
	{ 0,								NULL },		// 107
	{ 0,								NULL },		// 108
	{ 0,								NULL },		// 109
	{ 0,								NULL },		// 110
	{ 0,								NULL },		// 111
	{ 0,								NULL },		// 112
	{ 0,								NULL },		// 113
	{ 0,								NULL },		// 114
	{ 0,								NULL },		// 115
	{ 0,								NULL },		// 116
	{ 0,								NULL },		// 117
	{ 0,								NULL },		// 118
	{ 0,								NULL },		// 119
	{ 0,								NULL },		// 120
	{ 0,								NULL },		// 121
	{ 0,								NULL },		// 122
	{ 0,								NULL },		// 123
	{ 0,								NULL },		// 124
	{ 0,								NULL },		// 125
	{ 0,								NULL },		// 126
	{ 0,								NULL }		// 127
};
// int arr_size = sizeof(CmdDescTable)/sizeof(CmdDescTable[0]); 

/**
 * \brief Obtain pointer to the Command function handler.
 */
funcCommandHandler get_CommandFunctionHandler(uint8_t a_u8CommandNumber)
{
	if (a_u8CommandNumber <= COMMAND_NUMBER_LAST)
	{
		return (CmdDescTable[a_u8CommandNumber].ptrCmdHandler);	
	}
	else
	{
		return NULL;
	}
};

/************************************************************************/
/* MAIN PROGRAM                                                         */
/************************************************************************/
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

#ifdef MMSN_DEBUG	
	// Initialize serial communication terminal
	usartCommTerminalInit();
	
	// Redirect stream to standard output
	stdout = &mystdout;
	
	/* Print out welcome message */
	//static const char WelcomeMessage[] PROGMEM = "Multi-Master Serial Network Manager ver 1.0";
	//printf("%S\n", WelcomeMessage);
	printf_P(PSTR("\nMulti-Master Serial Network Manager ver 1.0\n"));
	//printf("Multi-Master Serial Network Manager ver 1.0\n");
#endif
	
	// Calculate CRC-16 value based on the random Internal SRAM memory content.
	// Move by 0x400 from the beginning to omit memory area related to global variables section.
	// Take 20 consecutive SRAM bytes for CRC-16 calculation.
	uint16_t u16RandomValue = xmega_calculate_checksum_crc16((uint8_t *)(INTERNAL_SRAM_START + 1200), 20);
	// Pseudo-random number generator seeding with previously obtain value
	srand(u16RandomValue);
	
#ifdef MMSN_DEBUG
	printf("rnd = %u\n", u16RandomValue);
#endif	
	
	// Read XMEGA device serial number
	nvm_read_device_serial(&xmegaSerialNumber);

	// Read configuration data from EEPROM
	xmega_read_configuration_data();

	// Check if logical address was already assigned
	if (true == _isLogicalNetworkAddrAssigned(&ConfigurationData.u8LogicalNetworkAddr))
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
	xmega_timer_config(&TIMER_HEARTBEAT, TC_CLKSEL_OFF_gc, TIMER_HEARTBEAT_PERIOD);
	
	// Force the state of the SREG register on exit, disabling the Global Interrupt Status flag bit.
	/* ATOMIC_BLOCK(NONATOMIC_FORCEOFF)
	{
		// Clear busy line timeout event
		FLAG_CLEAR(gSystemEvents, EVENT_IRQ_COLLISION_AVOIDANCE_TIMEOUT_bm);
	}
	
	// Turn busy line timer off
	xmega_tc_select_clock_source(&TIMER_COLLISION_AVOIDANCE, TC_CLKSEL_OFF_gc); */
	
	/* Initialize event queue */
	fifo_init(&eventQueue_desc, &EventQueue[0], EVENT_FIFO_BUFFER_SIZE);
	
	/************************************************************************/
	/* Initialize Multi-Master Serial Network State Machine                 */
	/************************************************************************/
	// SM_stateTable[eSM_Initialize]();
	mmsn_InitializeStateMachine(&mmsnFSM);
	
	// Start heartbeat timer
	//xmega_tc_select_clock_source(&TIMER_HEARTBEAT, TC_CLKSEL_DIV64_gc);
	
	xmega_set_usart_rx_interrupt_level(&USART_COMMUNICATION_BUS, USART_RXCINTLVL_HI_gc);
	
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
		};
		
        while (u16EventFlags)
        {
	        // Note: Each handler will clear the relevant bit in global variable gSystemEvents
			// Corresponding event flag will be cleared in FSM handler		

			// Heartbeat timer interrupt fired up
			if (u16EventFlags & SYSEV_HEARTBEAT_TIMEOUT_bm)
			{
				sys_HeartbeatTimeout_Handler();
			};
			
			// Collision Avoidance timer interrupt fired up
			if (u16EventFlags & SYSEV_COLLISION_AVOIDANCE_TIMEOUT_bm)
			{
				sysev_CollisionAvoidanceTimeout_Handler();
			};
			
			// No response timer interrupt fired up
			if (u16EventFlags & SYSEV_NO_RESPONSE_TIMEOUT_bm)
			{
				sysev_NoResponseTimeout_Handler();
			};
			
			// Receive Complete (RXC) interrupt fired up
			if (u16EventFlags & SYSEV_RECEIVE_COMPLETE_bm)
			{
				sysev_ReceiveComplete_Handler();
			};
			
			// Data Register Empty (DRE) interrupt fired up
			if (u16EventFlags & SYSEV_DATA_REGISTER_EMPTY_bm)
			{
				sysev_DataRegisterEmpty_Handler();
			};
			
			// Transmit Complete (TXC) interrupt fired up
			if (u16EventFlags & SYSEV_TRANSMIT_COMPLETE_bm)
			{
				sysev_TransmitComplete_Handler();
			};
			
			// Dispatch events from the MMSN FSM queue
			while(!fifo_is_empty(&eventQueue_desc))
			{
				// Get event from the queue
				uint8_t currEvent = fifo_pull_uint8_nocheck(&eventQueue_desc);
				
				// Call corresponding FSM event handler
				if (mmsnFSMActionTable[mmsnFSM.CurrentState][currEvent])
				{
					mmsnFSMActionTable[mmsnFSM.CurrentState][currEvent](&mmsnFSM, currEvent, &g_u8GlobalArgument);
				}
			};
			
			// Read the system event flag again after handlers return
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

// TEST
	/* rs485_driver_enable();
	
	uint8_t u8tab[] = "\nHello_World!";
	uint8_t length = 13;
	
	for (uint8_t idx = 0; idx < length; idx++)
	{
		while( false == (USART_COMMUNICATION_BUS.STATUS & USART_DREIF_bm) );
		USART_COMMUNICATION_BUS.DATA = u8tab[idx];
	} */
// TEST