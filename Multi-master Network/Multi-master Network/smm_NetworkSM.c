/*
Serial Multi-Master Network State Machine
*/

#include "smm_NetworkSM.h"
#include "fsm_Receiver.h"

#include <avr/io.h>
#include <util/atomic.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "fifo/fifo.h"
#include "board_config.h"
#include "tc_driver.h"
#include "usart_driver/usart_driver.h"
#include "utils.h"

/**
 * \brief Buffer to associate with receiving FIFO buffer
 *
 * This buffer consists of \ref FIFO_RECEIVE_BUFFER_SIZE elements
 * capable of holding a byte
 */
uint8_t fifo_receive_buffer [FIFO_RECEIVE_BUFFER_SIZE];

/**
 * \brief Network receiving FIFO buffer descriptor.
 *
 * This descriptor contains information about the location of the FIFO buffer,
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
fifo_desc_t fifo_receive_buffer_desc;

/**
 * \brief Buffer to associate with sending FIFO buffer
 *
 * This buffer consists of \ref FIFO_SEND_BUFFER_SIZE elements
 * capable of holding a byte
 */
uint8_t fifo_send_buffer [FIFO_SEND_BUFFER_SIZE];

/**
 * \brief Network sending FIFO buffer descriptor.
 *
 * This descriptor contains information about the location of the FIFO buffer,
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
fifo_desc_t fifo_send_buffer_desc;

/* Bitmasked flags that describe what event has occurred */
extern volatile uint16_t gSystemEvents;

// Variable to store current event flag
extern uint16_t u16EventFlags;

/* Flag to indicate that the line is busy */
volatile uint8_t gBusyLine;

/* Current state of Network State Machine */
volatile eSM_StateType gNSM_CurrentState;

/* Previous state of Network State Machine */
volatile eSM_StateType gNSM_PreviousState;

// Structure for storing received frame
mmsn_comm_data_frame_t gCommDataFrameReceive;
// Structure for storing frame to be transmitted
mmsn_comm_data_frame_t gCommDataFrameTransmit;

// Variable to store transmitted data counter
volatile uint8_t g_u8DataTransmitCounter;

// Variable to store retries count
uint8_t g_u8RetriesCount;

// Transmitted message type
eTransmitMessageType_t gTxMsgType;

/************************************************************************/
/* FSM RECEIVER                                                         */
/************************************************************************/
FSMReceiver_t ReceiverFSM;

/************************************************************************/
/* NETWORK ERROR HANDLING                                               */
/************************************************************************/
CommNetworkErrorDesc_t CommNetworkErrorTable[NETWORK_ERROR_TABLE_SIZE];

// Network Error descriptor
NetworkErrorDesc_t g_NetworkErrorDesc;

void init_commNetworkError(NetworkErrorDesc_t * a_pNetworkErrorDesc, CommNetworkErrorDesc_t * a_pCommNetErrTbl, uint8_t a_u8TblSize)
{
	// Initialize error descriptor
	a_pNetworkErrorDesc->u8Iterator		 = 0;
	a_pNetworkErrorDesc->u16ErrorCounter = 0;
	a_pNetworkErrorDesc->currError		 = eNE_None;
	
	// Clear communication network error table
	memset(a_pCommNetErrTbl, 0x00, a_u8TblSize);
};

void add_commNetworkError(NetworkErrorDesc_t * a_pNetworkErrorDesc)
{
	// Increase buffer iterator and wrap if necessary
	a_pNetworkErrorDesc->u8Iterator = (a_pNetworkErrorDesc->u8Iterator + 1) & NETWORK_ERROR_TABLE_SIZE_MASK;
	
	// Increase consecutive error counter. Will wrap if max value reached.
	a_pNetworkErrorDesc->u16ErrorCounter++;
	
	// Add another error to the table
	CommNetworkErrorTable[a_pNetworkErrorDesc->u8Iterator].eErrorReported = a_pNetworkErrorDesc->currError;
	CommNetworkErrorTable[a_pNetworkErrorDesc->u8Iterator].u16ErrorNumber = a_pNetworkErrorDesc->u16ErrorCounter;
};

eNetworkError_t get_commNetworkError(uint16_t a_u16ErrorNumber)
{
	return (CommNetworkErrorTable[a_u16ErrorNumber].eErrorReported);
};

/************************************************************************/
/* COMMUNICATION                                                        */
/************************************************************************/

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
	{ SYSCMD_MODULE_SERIAL_NUMBER_REQ,	NULL },		// 104
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

/* Lookup table containing a pointer to the function to call in each state */
void (*SM_stateTable[])(void) =
{
	fsm_Initialize,
	fsm_Idle,
	fsm_Receive,
	fsm_ProcessData,
	fsm_ExecuteCommand,
	fsm_Send,
	fsm_WaitForResend,
	fsm_Retransmission,
	fsm_WaitForResponse,
	fsm_Error
};

/**
 *  \brief Sets busy line flag.
 *
 *  \param none.
 */
static inline void fsm_SetBusyLine(void)
{
	gBusyLine = BUSY;
};

/**
 *  \brief Clears busy line flag.
 *
 *  \param none.
 */
static inline void fsm_ClearBusyLine(void)
{
	gBusyLine = FREE;
};

/**
 *  \brief Copy communication data frame from FIFO to destination
 *
 *  \param none.
 */

void mmsn_Copy_Comm_Frame(fifo_desc_t *a_pFifoDesc, mmsn_comm_data_frame_t *a_pDstDataFrame)
{
	uint8_t __attribute__((unused)) u8FifoStatus;
	
	// Pull out first two bytes with packed Address and Control Field
	u8FifoStatus = fifo_pull_uint16(a_pFifoDesc, &(a_pDstDataFrame->u16Identifier));

	// Pull out all data bytes
	for (uint8_t u8Idx = 0; u8Idx < MMSN_DATA_LENGTH; u8Idx++)
	{
		u8FifoStatus = fifo_pull_uint8(a_pFifoDesc, &(a_pDstDataFrame->u8DataTable[u8Idx]));
	}
	
	// Pull out CRC-16 16bit value
	u8FifoStatus = fifo_pull_uint16(a_pFifoDesc, &(a_pDstDataFrame->u16CRC16));
	
} // mmsn_Copy_Comm_Frame()

/**
 *  \brief Common helper function to handle Collision Detection timeout.
 *
 *  \param none.
 */
static void fsm_CollisionAvoidanceTimeoutHandler(void)
{
	// Stop collision avoidance timer by setting clock source to OFF state
	xmega_tc_select_clock_source(&TIMER_COLLISION_AVOIDANCE, TC_CLKSEL_OFF_gc);

	// Clear busy line flag.
	fsm_ClearBusyLine();
	
	// Clear corresponding flag
	ATOMIC_BLOCK(ATOMIC_FORCEON)
	{
		// Atomic interrupt safe set of global variable storing event flags
		FLAG_CLEAR(gSystemEvents, EVENT_IRQ_COLLISION_AVOIDANCE_TIMEOUT_bm);
	}
}

/************************************************************************/
/* FINITE STATE MACHINE HANDLERS                                        */
/************************************************************************/

/* INITIALIZE */
void fsm_Initialize(void)
{
	/* Set current state to \ref eSM_Initialize */
	gNSM_CurrentState = eSM_Initialize;
	/* Set previous state to current state */
	gNSM_PreviousState = gNSM_CurrentState;
	
	/* Initialize sending and receiving FIFOs */
	fifo_init(&fifo_receive_buffer_desc, &fifo_receive_buffer[0], FIFO_RECEIVE_BUFFER_SIZE);
	fifo_init(&fifo_send_buffer_desc,	 &fifo_send_buffer[0],	  FIFO_SEND_BUFFER_SIZE);
	
	// Initialize communication data frame receiving buffer with zeros
	memset(&(gCommDataFrameReceive.u8CommFrameTable[0]), 0x00, MMSN_COMM_FRAME_SIZE);
	// Initialize communication data frame transmission buffer with zeros
	memset(&(gCommDataFrameTransmit.u8CommFrameTable[0]), 0x00, MMSN_COMM_FRAME_SIZE);
		
	g_u8DataTransmitCounter = 0;
	
	// Initialize Receiver FSM
	fsmReceiverInitialize(&ReceiverFSM);
	
	// Initialize Network Error Descriptor
	init_commNetworkError(&g_NetworkErrorDesc, &CommNetworkErrorTable[0], sizeof(CommNetworkErrorTable));
	
	/************************************************************************/
	/* HARDWARE SETUP														*/
	/************************************************************************/
	
	/* RS-485 PHYSICAL DEVICE CONFIGURATION */
	// Initialize GPIO related to RS-485 interface
	rs485_driver_gpio_initialize();
	// Initially go LOW to enable receiver and start listening
	rs485_receiver_enable();
	
	/* USART INTERRUPTS CONFIGURATION - RECEIVING */
	// Turn on USART RXC interrupt
	xmega_set_usart_rx_interrupt_level(&USART_COMMUNICATION_BUS, USART_RXCINTLVL_MED_gc);
	
	// Turn off TXC and DRE interrupts
	xmega_set_usart_tx_interrupt_level(&USART_COMMUNICATION_BUS, USART_TXCINTLVL_OFF_gc);
	xmega_set_usart_dre_interrupt_level(&USART_COMMUNICATION_BUS, USART_DREINTLVL_OFF_gc);
	
	// Go to IDLE state
	gNSM_CurrentState = eSM_Idle;
};

/* IDLE state handler */
void fsm_Idle(void)
{
	// Check for receiving and sending data events. Receiving has higher priority than sending.
	
	// Check for data received event
	if (u16EventFlags & EVENT_IRQ_RECEIVE_COMPLETE_bm)
	{
		// USART data was received. Change current state to \ref fsm_Receive.
		// Don't clear \ref EVENT_IRQ_RECEIVE_COMPLETE_bm event.
		// This will cause that processing will be handled in the next state.
		gNSM_CurrentState = eSM_Receive;
		
		// Return immediately here. Sending will be handled after data processing cycle.
		return;
	};
	
	// Now check if frame is ready to be sent
	if (u16EventFlags & EVENT_SW_DATA_READY_TO_SEND_bm)
	{
		// Complete frame is ready to be sent. Change state to \ref eSM_Send.
		// Don't clear \ref EVENT_SW_DATA_READY_TO_SEND_bm event.
		// This will cause that processing will be handled in the next state.
		gNSM_CurrentState = eSM_Send;
	};
	
	// Check if busy line timer timed out
	if (u16EventFlags & EVENT_IRQ_COLLISION_AVOIDANCE_TIMEOUT_bm)
	{
		fsm_CollisionAvoidanceTimeoutHandler();
	};

};	// fsm_Idle()

/* RECEIVE state handler */
void fsm_Receive(void)
{
	if (u16EventFlags & EVENT_IRQ_RECEIVE_COMPLETE_bm)
	{
		/* The data was received. It means that the bus currently is or was in use.
		 * To avoid collision (CA) start or restart busy line timer if already running.
		 */
	
		// Turn collision avoidance timer on
		xmega_tc_select_clock_source(&TIMER_COLLISION_AVOIDANCE, TC_CLKSEL_DIV64_gc);
		
		// Force Restart of busy line timer
		xmega_tc_restart(&TIMER_COLLISION_AVOIDANCE);

		// Due to the buffering of the error flags, the status register must be read before the receive buffer
		// (DATA), since reading the DATA location changes the FIFO buffer.
		uint8_t u8UsartErrorFlags = 0;
		// Read USART receiver error flags
		u8UsartErrorFlags = USART_COMMUNICATION_BUS.STATUS & (USART_FERR_bm | USART_BUFOVF_bm | USART_PERR_bm);
		// Read the buffer. It will automatically clear RXCIF flag
		uint8_t u8Data = USART_COMMUNICATION_BUS.DATA;

		// Decision making process based on receiver error flags
		if (0 == u8UsartErrorFlags)
		{
			// Received data is error free.
		
			// Push new element into receiving FIFO. No check, the buffer should have enough capacity to hold it.
			fifo_push_uint8_nocheck(&fifo_receive_buffer_desc, u8Data);
		
			// Move to processing the data state
			gNSM_CurrentState = eSM_ProcessData;
			
			// Set data processing software event
			ATOMIC_BLOCK(ATOMIC_FORCEON)
			{
				FLAG_SET(gSystemEvents, EVENT_SW_RECEIVE_DATA_NO_ERROR_bm);
			}
		} 
		else
		{
			// If data was erroneous at hardware level than discard the data.
		
			// Report network error
			g_NetworkErrorDesc.currError = eNE_USART_Receiver_Error;
		
			// Make transition to the ERROR state
			gNSM_CurrentState = eSM_Error;
			
			// Set error in data processing software event
			ATOMIC_BLOCK(ATOMIC_FORCEON)
			{
				FLAG_SET(gSystemEvents, EVENT_SW_RECEIVE_DATA_ERROR_bm);
			}
		}
		
		// Always clear receive complete event flag
		ATOMIC_BLOCK(ATOMIC_FORCEON)
		{
			FLAG_CLEAR(gSystemEvents, EVENT_IRQ_RECEIVE_COMPLETE_bm);
		}
	};
	
	// Check if busy line timer timed out
	if (u16EventFlags & EVENT_IRQ_COLLISION_AVOIDANCE_TIMEOUT_bm)
	{
		fsm_CollisionAvoidanceTimeoutHandler();
	};
};	// fsm_Receive()

/* PROCESS_DATA state handler */
void fsm_ProcessData(void)
{
	// Check if complete frame was received.
	uint16_t g_u16crc16_checksum;
	
	if (u16EventFlags & EVENT_SW_RECEIVE_DATA_NO_ERROR_bm)
	{
		// Check if complete data frame was already received
		uint8_t u8FifoSize = fifo_get_used_size(&fifo_receive_buffer_desc);
		
		// Set corresponding state and event
		if (MMSN_COMM_FRAME_SIZE == u8FifoSize)
		{
			/* Complete frame was received.
			 */
			
			// Make a working copy of received data frame
			mmsn_Copy_Comm_Frame(&fifo_receive_buffer_desc, &gCommDataFrameReceive);
			
			// Clear receiving FIFO
			fifo_flush(&fifo_receive_buffer_desc);
			
			/* Calculate CRC-16 (CRC-CCITT) using XMEGA hardware CRC peripheral
			 * excluding last 2 bytes with CRC-16.
			 */
			g_u16crc16_checksum = xmega_calculate_checksum_crc16(&gCommDataFrameReceive.u8CommFrameTable[0], MMSN_FRAME_NOCRC_LENGTH);
			
			// Check data integrity
			if (g_u16crc16_checksum == gCommDataFrameReceive.u16CRC16)
			{
				// Calculated and received CRC-16 value matched. Go to command execution state.
				
				//Go to \ref eSM_ExecuteCommand state.
				gNSM_CurrentState = eSM_ExecuteCommand;
				
				// Set communication frame OK software event
				ATOMIC_BLOCK(ATOMIC_FORCEON)
				{
					FLAG_SET(gSystemEvents, EVENT_SW_COMM_FRAME_COMPLETE_bm);
				};
			}
			else
			{
				/* Received and calculated CRC-16 value does not match.
				 * Report CRC integrity error.
				 */
				g_NetworkErrorDesc.currError = eNE_Frame_CRC;
				
				// Go to \ref eSM_Error state
				gNSM_CurrentState = eSM_Error;
				
				// Set error in data integrity software event
				ATOMIC_BLOCK(ATOMIC_FORCEON)
				{
					FLAG_SET(gSystemEvents, EVENT_SW_COMM_FRAME_CRC_ERROR_bm);
				};
			}
		}
		else
		{
			// Still incomplete frame. Go to \ref eSM_Idle state.
			gNSM_CurrentState = eSM_Idle;
			
			// Set communication frame incomplete software event
			ATOMIC_BLOCK(ATOMIC_FORCEON)
			{
				FLAG_SET(gSystemEvents, EVENT_SW_COMM_FRAME_INCOMPLETE_bm);
			};
		}
		
		// Always clear data error free flag
		ATOMIC_BLOCK(ATOMIC_FORCEON)
		{
			// Atomic interrupt safe set of global variable storing event flags
			FLAG_CLEAR(gSystemEvents, EVENT_SW_RECEIVE_DATA_NO_ERROR_bm);
		};
	};
	
	// Check if collision avoidance timer timed out
	if (u16EventFlags & EVENT_IRQ_COLLISION_AVOIDANCE_TIMEOUT_bm)
	{
		fsm_CollisionAvoidanceTimeoutHandler();
	};

};	// fsm_ProcessData()

void fsm_ExecuteCommand(void)
{
	/* Complete frame was received and is ready to be processed.
	 * Check if this message is handled by this device.
	 */
	
	funcCommandHandler ptrCommandHandler = NULL;
	
	// Check for frame complete event
	if (u16EventFlags & EVENT_SW_COMM_FRAME_COMPLETE_bm)
	{
		uint8_t u8DeviceType;
		uint8_t u8DeviceNumber;
		
		// Retrieve device type from the message
		get_MMSN_DeviceType(gCommDataFrameReceive.u16Identifier, u8DeviceType);
		
		// Retrieve device number/command from the message
		get_MMSN_DeviceNumber(gCommDataFrameReceive.u16Identifier, u8DeviceNumber);
		
#ifdef MMSN_DEBUG
		printf("Exec:dev_typ = %d\n", u8DeviceType);
		printf("Exec:dev_num = %d\n", u8DeviceNumber);
#endif		
		// Obtain appropriate function handler
		ptrCommandHandler = get_CommandFunctionHandler(u8DeviceNumber);
		
		// Execute if needed
		if (NULL == ptrCommandHandler)
		{
			/* Command is NOT handled by this device
			*/
		} 
		else
		{
			/* Command is handled by this device.
			*/
		}
		
		// Clear receiving data buffer
		memset(&(gCommDataFrameReceive.u8CommFrameTable[0]), 0x00, MMSN_COMM_FRAME_SIZE);
		
		/* All clean-up done.
		 * Go to \ref eSM_Idle state.
		 */
		gNSM_CurrentState = eSM_Idle;
		
		// Clear corresponding flag
		ATOMIC_BLOCK(ATOMIC_FORCEON)
		{
			// Atomic interrupt safe set of global variable storing event flags
			FLAG_CLEAR(gSystemEvents, EVENT_SW_COMM_FRAME_COMPLETE_bm);
		};
	}
	
	/* Check if collision avoidance timer timed out
	 */
	if (u16EventFlags & EVENT_IRQ_COLLISION_AVOIDANCE_TIMEOUT_bm)
	{
		// Handle event and clear the flag
		fsm_CollisionAvoidanceTimeoutHandler();
	};
};

/* SEND state handler
 * The complete frame was already prepared and loaded into transmit buffer.
 */
void fsm_Send(void)
{
	/* Initiate data sending event */
	if (u16EventFlags & EVENT_SW_DATA_READY_TO_SEND_bm)
	{
		/* At first check is the line is free (Collision Avoidance).
		 * If the line is still busy than back-off and wait until timer times out.
		 * The back-off timer value is determined by Logical Address in Network.
		 * Go waiting for free bus.
		 */
		if (BUSY == gBusyLine)
		{
			/* Do not clear the Data Ready To Send event
			 * Transition to Wait For Resending state and wait for free bus.
			 */
			gNSM_CurrentState = eSM_WaitForResend;
			
			// return immediately with \ref EVENT_SW_DATA_READY_TO_SEND_bm event set
			return;
		}
		else
		{
			/* The line is free:
			 * 1. Turn off receiving (automatically if RS-485 driver and receiver pins are tight together).
			 * 2. Transmit entire frame by putting first byte and than using DRE interrupt.
			 * 3. Regular and ACKnowledge messages have the same length (for simplicity reasons).
			 * 4. Complete frame with calculated CRC-16 value is already stored in \ref gCommDataFrameTransmit buffer.
			*/
			
			// Set RS-485 transceiver to transmit mode
			rs485_driver_enable();
			
			// Nevertheless turn off USART RXC interrupt during sending the data
			xmega_set_usart_rx_interrupt_level(&USART_COMMUNICATION_BUS, USART_RXCINTLVL_OFF_gc);
			
			// Turn on TXC and DRE interrupts
			xmega_set_usart_tx_interrupt_level(&USART_COMMUNICATION_BUS, USART_TXCINTLVL_MED_gc);
			xmega_set_usart_dre_interrupt_level(&USART_COMMUNICATION_BUS, USART_DREINTLVL_MED_gc);
			
			// Reset data transmit counter
			g_u8DataTransmitCounter = 0;
			
			// Send first data byte. The remaining bytes will be sent in DRE event handler directly.
			USART_COMMUNICATION_BUS.DATA = gCommDataFrameTransmit.u8CommFrameTable[0];			
			
			ATOMIC_BLOCK(ATOMIC_FORCEON)
			{
				// Mark that the first byte was sent
				g_u8DataTransmitCounter++;
			};
		}
		
		// Clear data ready to be sent event flag
		ATOMIC_BLOCK(ATOMIC_FORCEON)
		{
			FLAG_CLEAR(gSystemEvents, EVENT_SW_DATA_READY_TO_SEND_bm);
		}
	};
	
	// Data Register Empty (DRE) IRQ handling
	if (u16EventFlags & EVENT_IRQ_DATA_REGISTER_EMPTY_bm)
	{
		if (g_u8DataTransmitCounter < MMSN_COMM_FRAME_SIZE)
		{
			// Data still waiting to be sent
			
			/* Get next data byte and put it to USART register for sending.
			 */
			USART_COMMUNICATION_BUS.DATA = gCommDataFrameTransmit.u8CommFrameTable[g_u8DataTransmitCounter];
			
			// Increase transmission counter
			ATOMIC_BLOCK(ATOMIC_FORCEON)
			{
				// Another byte was sent
				g_u8DataTransmitCounter++;
			};
			
			// Turn on DRE interrupt
			xmega_set_usart_dre_interrupt_level (&USART_COMMUNICATION_BUS, USART_DREINTLVL_MED_gc);
			
			// Do not change the state
		}
		else
		{
			/* Complete frame was sent out */
			
			// Set RS-485 transceiver for receiving
			rs485_receiver_enable();
			
			// Turn on USART RXC interrupt
			xmega_set_usart_rx_interrupt_level(&USART_COMMUNICATION_BUS, USART_RXCINTLVL_MED_gc);
			
			// Turn off TXC and DRE interrupts
			xmega_set_usart_tx_interrupt_level(&USART_COMMUNICATION_BUS, USART_TXCINTLVL_OFF_gc);
			xmega_set_usart_dre_interrupt_level(&USART_COMMUNICATION_BUS, USART_DREINTLVL_OFF_gc);
			
			// Start waiting for response timer
			xmega_tc_select_clock_source(&TIMER_NO_RESPONSE, TC_CLKSEL_DIV64_gc);
			
			/* Make transition to waiting for a response. State machine will be woken up after USART RXC IRQ.
			 */
			 gNSM_CurrentState = eSM_WaitForResponse;
		}
		
		// Always clear data register empty event flag
		ATOMIC_BLOCK(ATOMIC_FORCEON)
		{
			FLAG_CLEAR(gSystemEvents, EVENT_IRQ_DATA_REGISTER_EMPTY_bm);
		};		
	}
};

/* Waiting for resending state handler
 */
void fsm_WaitForResend(void)
{
	/* In this state 2 event are possible.
	 * First is related to collision avoidance timer timeout.
	 * The latter is USART data receiving.
	 */
	
	// Check for USART data received event
	if (u16EventFlags & EVENT_IRQ_RECEIVE_COMPLETE_bm)
	{
		/* USART data was received. Change current state to \ref fsm_Receive.
	     * Don't clear \ref EVENT_IRQ_RECEIVE_COMPLETE_bm event.
		 * This will cause that processing will be handled there.
		 */
		gNSM_CurrentState = eSM_Receive;
		
		// Return immediately here. Sending will be handled after data processing cycle.
		return;
	};
	
	/* Check if collision avoidance timer timed out
	 */
	if (u16EventFlags & EVENT_IRQ_COLLISION_AVOIDANCE_TIMEOUT_bm)
	{
		// Handle event and clear the flag.
		// The bus is free now.
		fsm_CollisionAvoidanceTimeoutHandler();
		
		// Go back to Send state
		gNSM_CurrentState = eSM_Send;
	};
};

/* Waiting for a response state handler
 */
void fsm_WaitForResponse(void)
{
	/* If data was received in this state than collect the data.
	 * Could be acknowledge frame if requested [RTR=1].
	 */ 
	
	
	
	
	
	// EVENT_IRQ_RECEIVE_COMPLETE_bm

	
	// Restart waiting for response timer
	
	// Each time restart busy line timer
	
	// If busy line timer times out than set line to free - ?? in IRQ handler??
	
};

/* This state is to decide whether message is to be sent again
 * or maximum retries count was reached and message will be dropped.
 * It means that send out message was not received or processed properly.
 */
void fsm_Retransmission(void)
{
	/* Wait for response time out or 
	 * Response data frame was not valid
	 */
	
	if (u16EventFlags & EVENT_SW_COMM_FRAME_RETRANSMIT_bm)
	{
		// Check if maximum retries count was reached
		if (MMSN_MAX_RETRIES <= g_u8RetriesCount)
		{
			/* Maximum retries count reached.
			 * Report an error.
			 */
			g_NetworkErrorDesc.currError = eNE_MaximumRetries;
				
			// Go to \ref eSM_Error state
			gNSM_CurrentState = eSM_Error;
			
			// Clear retries counter
			g_u8RetriesCount = 0;
				
			// Set error max retries reached software event
			ATOMIC_BLOCK(ATOMIC_FORCEON)
			{
				FLAG_CLEAR(gSystemEvents, EVENT_SW_COMM_FRAME_RETRANSMIT_bm);
				FLAG_SET(gSystemEvents, EVENT_SW_MAX_RETRIES_COUNT_REACHED_bm);
			};									
		} 
		else
		{
			// We can still give it a try
		
			// Increase retransmissions counter
			g_u8RetriesCount++;
			
			// Reset data transmission counter
			g_u8DataTransmitCounter = 0;
			
			// Go to \ref eSM_Send state to send complete message again
			gNSM_CurrentState = eSM_Send;
			
			// Force send message again
			ATOMIC_BLOCK(ATOMIC_FORCEON)
			{
				FLAG_CLEAR(gSystemEvents, EVENT_SW_COMM_FRAME_RETRANSMIT_bm);
				FLAG_SET(gSystemEvents, EVENT_SW_DATA_READY_TO_SEND_bm);
			};
		}		
	}
	
	/* Check if collision avoidance timer timed out
	 */
	if (u16EventFlags & EVENT_IRQ_COLLISION_AVOIDANCE_TIMEOUT_bm)
	{
		/* Handle event and clear the flag.
		 * The bus is free now.
		 */
		fsm_CollisionAvoidanceTimeoutHandler();
	}
};

// Error state handler
void fsm_Error(void)
{
	/* Data error at USART hardware level.
	 */
	if (u16EventFlags & EVENT_SW_RECEIVE_DATA_ERROR_bm)
	{
		// Add current Network Error to the table
		add_commNetworkError(&g_NetworkErrorDesc);
		
		// Clear network error descriptor
		g_NetworkErrorDesc.currError = eNE_None;
		
		// Flush receiving FIFO
		fifo_flush(&fifo_receive_buffer_desc);
		
		// Clear corresponding event
		ATOMIC_BLOCK(ATOMIC_FORCEON)
		{
			// Atomic interrupt safe set of global variable storing event flags
			FLAG_CLEAR(gSystemEvents, EVENT_SW_RECEIVE_DATA_ERROR_bm);
		};
		
		// Go to the \ref eSM_Idle state
		gNSM_CurrentState = eSM_Idle;
	}
	
	/* Data frame integrity error (CRC-16 does not match).
	 */
	if (u16EventFlags & EVENT_SW_COMM_FRAME_CRC_ERROR_bm)
	{
		// Add current Network Error to the table
		add_commNetworkError(&g_NetworkErrorDesc);
		
		// Clear network error descriptor
		g_NetworkErrorDesc.currError = eNE_None;
		
		// Clear receiving buffer
		memset(&(gCommDataFrameReceive.u8CommFrameTable[0]), 0x00, MMSN_COMM_FRAME_SIZE);
		
		// Flush receiving FIFO
		fifo_flush(&fifo_receive_buffer_desc);
		
		// Clear corresponding event
		ATOMIC_BLOCK(ATOMIC_FORCEON)
		{
			// Atomic interrupt safe set of global variable storing event flags
			FLAG_CLEAR(gSystemEvents, EVENT_SW_COMM_FRAME_CRC_ERROR_bm);
		};
		
		// Go to the \ref eSM_Idle state
		gNSM_CurrentState = eSM_Idle;
	}
	
	/* Maximum retries count reached. Message to be sent is lost.
	 */
	if (u16EventFlags & EVENT_SW_MAX_RETRIES_COUNT_REACHED_bm)
	{
		// Add current Network Error to the table
		add_commNetworkError(&g_NetworkErrorDesc);
		
		// Clear network error descriptor
		g_NetworkErrorDesc.currError = eNE_None;
		
		// Clear transmission buffer
		memset(&gCommDataFrameTransmit.u8CommFrameTable[0], 0x00, MMSN_COMM_FRAME_SIZE);
		
		// Clear transmission FIFO
		fifo_flush(&fifo_send_buffer_desc);
		
		// Reset retries count
		g_u8RetriesCount = 0;
		
		// Clear transmitted bytes counter
		g_u8DataTransmitCounter = 0;
		
		// Clear corresponding event
		ATOMIC_BLOCK(ATOMIC_FORCEON)
		{
			// Atomic interrupt safe set of global variable storing event flags
			FLAG_CLEAR(gSystemEvents, EVENT_SW_MAX_RETRIES_COUNT_REACHED_bm);
		};
		
		// Go to the \ref eSM_Idle state
		gNSM_CurrentState = eSM_Idle;
	}
	
	/* Check if collision avoidance timer timed out
	 */
	if (u16EventFlags & EVENT_IRQ_COLLISION_AVOIDANCE_TIMEOUT_bm)
	{
		// Handle event and clear the flag
		fsm_CollisionAvoidanceTimeoutHandler();
	};
};

// Determine if Logical Network Address was assigned to the device
bool isLogicalNetworkAddrAssigned(uint8_t *a_pu8LogicalNetworkAddr)
{
	return (!(MMSN_DEFAULT_LOGICAL_NETWORK_ADDRESS == (*a_pu8LogicalNetworkAddr)));
};

/**
 * \brief Get the shortened XMEGA device serial number.
 */
void xmega_get_shortened_serial_num(struct nvm_device_serial *a_pInCompleteSerialNum, xmega_shortened_serial_number_t *a_pOutShortenedSerialNum)
{
	// !Note that functions arguments must be properly provided
	
	// Magic numbers left on purpose
	memcpy(&(a_pOutShortenedSerialNum->u8DataArray[0]), &(a_pInCompleteSerialNum->u8DataArray[4]), 7);
};

// Function generates random logical network address.
uint8_t xmega_generate_random_logical_network_address(void)
{
	return ((rand() % 127) + 1);
};