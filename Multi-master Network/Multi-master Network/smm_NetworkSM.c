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

// Structure for storing received frame
mmsn_comm_data_frame_t g_RxCommFrameBuffer;
// Structure for storing frame to be transmitted
mmsn_comm_data_frame_t g_TxCommFrameBuffer;

// Event queue descriptor
extern fifo_desc_t eventQueue_desc;

/************************************************************************/
/* FSM RECEIVER                                                         */
/************************************************************************/
extern fsmReceiverActionHandler FSMReceiverActionHandlerTable[][FSMR_MAX_EVENTS];

// Multi-Master Serial Network FSM
MMSN_FSM_t mmsnFSM;

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
	a_pNetworkErrorDesc->currError		 = NE_None;
	
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



/* Lookup table containing a pointer to the function to call in each state */
/* void (*SM_stateTable[])(void) =
{
	fsm_Idle,
	fsm_Receive,
	fsm_ProcessData,
	fsm_ExecuteCommand,
	fsm_Send,
	fsm_WaitForResend,
	fsm_Retransmission,
	fsm_WaitForResponse,
	fsm_Error
}; */

/* Function event handlers pointer table */
mmsnFsmEventHandler mmsnFSMActionTable[MMSN_MAX_STATES][MMSN_MAX_EVENTS] =
{
	/* MMSN_IDLE_STATE - idle */
	/* MMSN_DATA_RECEIVED_EVENT, MMSN_COLLISION_AVOIDANCE_TIMEOUT, MMSN_ERROR_EVENT, MMSN_FRAME_PROCESS_EVENT */
	/* MMSN_EXECUTE_COMMAND_EVENT, MMSN_SEND_DATA_EVENT */
	{ mmsn_Idle_DataReceived_Handler, mmsn_Generic_CollisionAvoidanceTimeout_Handler, NULL, NULL,
	  NULL, NULL },
	
	/* MMSN_RECEIVE_STATE */
	/* MMSN_DATA_RECEIVED_EVENT, MMSN_COLLISION_AVOIDANCE_TIMEOUT, MMSN_ERROR_EVENT, MMSN_FRAME_PROCESS_EVENT */
	/* MMSN_EXECUTE_COMMAND_EVENT, MMSN_SEND_DATA_EVENT */
	{ mmsn_Receive_DataReceived_Handler, mmsn_Generic_CollisionAvoidanceTimeout_Handler, NULL, NULL,
	  NULL, NULL },

	/* MMSN_PROCESS_DATA_STATE */
	/* MMSN_DATA_RECEIVED_EVENT, MMSN_COLLISION_AVOIDANCE_TIMEOUT, MMSN_ERROR_EVENT, MMSN_FRAME_PROCESS_EVENT */
	/* MMSN_EXECUTE_COMMAND_EVENT, MMSN_SEND_DATA_EVENT */
	{ NULL, mmsn_Generic_CollisionAvoidanceTimeout_Handler, NULL, mmsn_ProcessData_FrameProcess_Handler,
	  NULL, NULL },

	/* MMSN_EXECUTE_COMMAND_STATE */
	/* MMSN_DATA_RECEIVED_EVENT, MMSN_COLLISION_AVOIDANCE_TIMEOUT, MMSN_ERROR_EVENT, MMSN_FRAME_PROCESS_EVENT */
	/* MMSN_EXECUTE_COMMAND_EVENT, MMSN_SEND_DATA_EVENT */
	{ NULL, mmsn_Generic_CollisionAvoidanceTimeout_Handler, NULL, NULL,
	  mmsn_ExecuteCommand_ExecuteCommandEvent_Handler, NULL },

	/* MMSN_SEND_STATE */
	
	{ NULL },
		
	/* MMSN_WAIT_FOR_RESEND_STATE */
	//{ NULL },
		
	/* MMSN_WAIT_FOR_RESPOND_STATE */
	{ NULL },
		
	/* MMSN_RECEIVE_RESPONSE_STATE */
	{ NULL },
		
	/* MMSN_PROCESS_RESPONSE_STATE */
	{ NULL },
		
	/* MMSN_RETRANSMIT_STATE */
	{ NULL },
		
	/* MMSN_ERROR_STATE */
	/* MMSN_DATA_RECEIVED_EVENT, MMSN_COLLISION_AVOIDANCE_TIMEOUT, MMSN_ERROR_EVENT, MMSN_FRAME_PROCESS_EVENT */
	/* MMSN_EXECUTE_COMMAND_EVENT, MMSN_SEND_DATA_EVENT */
	{ NULL, mmsn_Generic_CollisionAvoidanceTimeout_Handler, mmsn_Error_ErrorEvent_Handler, NULL,
	  NULL, NULL }
};

/* Common functions */
static inline void _restartCollisionAvoidanceTimer(void)
{
	// Turn collision avoidance timer on
	xmega_tc_select_clock_source(&TIMER_COLLISION_AVOIDANCE, TC_CLKSEL_DIV64_gc);
	
	// Force Restart of Collision Avoidance timer
	xmega_tc_restart(&TIMER_COLLISION_AVOIDANCE);
};

/**
 * \brief Make a working copy of received data frame.
 *
 *  \param none.
 */
void _copyDataFrame(fifo_desc_t * a_pFifoDesc, mmsn_comm_data_frame_t * a_pDstDataFrame)
{
	uint8_t u8Tmp1, u8Tmp2;
	
	// Pull out first two bytes with packed Address and Control Field
	u8Tmp1 = fifo_pull_uint8_nocheck(a_pFifoDesc);
	u8Tmp2 = fifo_pull_uint8_nocheck(a_pFifoDesc);
	MMSN_BYTES_2_WORD(u8Tmp1, u8Tmp2, a_pDstDataFrame->u16Identifier);
	
	// Pull out all data bytes
	for (u8Tmp1 = 0; u8Tmp1 < MMSN_DATA_LENGTH; u8Tmp1++)
	{
		a_pDstDataFrame->u8DataBuffer[u8Tmp1] = fifo_pull_uint8_nocheck(a_pFifoDesc);
	};
	
	// Pull out 2 bytes with CRC-16 16bit value
	u8Tmp1 = fifo_pull_uint8_nocheck(a_pFifoDesc);
	u8Tmp2 = fifo_pull_uint8_nocheck(a_pFifoDesc);
	MMSN_BYTES_2_WORD(u8Tmp1, u8Tmp2, a_pDstDataFrame->u16CRC16);
} // _copyDataFrame()

/************************************************************************/
/* EVENT HANDLERS                                                       */
/************************************************************************/
uint8_t mmsn_Idle_DataReceived_Handler(MMSN_FSM_t * a_pFSM, uint8_t a_u8Event, void * a_pEventArg)
{
	/* The error free data was received. It means that the bus currently is or was in use.
	* To avoid collision (CA) start or restart busy line timer if already running.
	*/
	_restartCollisionAvoidanceTimer();

	// Set bus to busy state
	a_pFSM->u8LineState = MMSN_BUSY_BUS;

	// In this state no other than DLE data is expected and ignore return from Receiver FSM.
	
	// Obtain pointer to ReceiverFSM action handler
	if( GET_EV_HDL_P(a_pFSM->pFSMRActionTable, a_pFSM->ReceiverFSM, FSMR_DATA_RECEIVED))
	{
		uint8_t u8RetCode;
	
		// Call action handler	
		u8RetCode = CALL_EV_HDL(a_pFSM->pFSMRActionTable, a_pFSM->ReceiverFSM, FSMR_DATA_RECEIVED, (*(uint8_t *)a_pEventArg));
		
		/* Receiver FSM will always return with IGNORE until |DLE|STX| sequence is received.
		 * |DLE|STX| data pair is a frame begin marker. When received change the state to RECEIVE.
		 * Otherwise ignore the data and do not change IDLE state.
		 */
		if (FSMR_FRAME_BEGIN == u8RetCode)
		{
			// Change state to Receive
			a_pFSM->CurrentState = MMSN_RECEIVE_STATE;
			
			// Set frame status to begin
			a_pFSM->FrameStatus = MMSN_FrameBegin;
			
			// Wait for another received data system event
		}
	}
	else
	{
		// Undefined function pointer to handle this event.
		g_NetworkErrorDesc.currError = NE_ReceiverFSM_UndefinedFuncPtr;
		
		// Change state to Error
		a_pFSM->CurrentState = MMSN_ERROR_STATE;
		
		// Add software event to queue. This will trigger immediate FSM run to handle error state.
		ADD_EVENT_TO_QUEUE(a_pFSM->ptrEventQueueDesc, MMSN_ERROR_EVENT);
	}	

	return MMSN_OK;
};

uint8_t mmsn_Generic_CollisionAvoidanceTimeout_Handler(MMSN_FSM_t * a_pFSM, uint8_t a_u8Event, void * a_pEventArg)
{
	// Stop collision avoidance timer by setting clock source to OFF state
	xmega_tc_select_clock_source(&TIMER_COLLISION_AVOIDANCE, TC_CLKSEL_OFF_gc);

	// Clear busy line flag.
	a_pFSM->u8LineState = MMSN_FREE_BUS;
	
	return MMSN_OK;
}

uint8_t mmsn_Receive_DataReceived_Handler(MMSN_FSM_t * a_pFSM, uint8_t a_u8Event, void * a_pEventArg)
{
	// Restart collision avoidance timer
	_restartCollisionAvoidanceTimer();
	
	// Set bus to busy state
	a_pFSM->u8LineState = MMSN_BUSY_BUS;
	
	// Obtain received data
	uint8_t u8Data = (*(uint8_t *)a_pEventArg);

	// Obtain pointer to ReceiverFSM action handler
	if( GET_EV_HDL_P(a_pFSM->pFSMRActionTable, a_pFSM->ReceiverFSM, FSMR_DATA_RECEIVED))
	{
		uint8_t u8RetCode;
		
		u8RetCode = CALL_EV_HDL(a_pFSM->pFSMRActionTable, a_pFSM->ReceiverFSM, FSMR_DATA_RECEIVED, u8Data);
		
		// Check if data should be stored in RX fifo buffer
		switch (u8RetCode)
		{
			case FSMR_FRAME_BEGIN:
				/* Check if frame begin was not already received by probing internal frame status.
				 * If so then this frame is malfunctioned - |DLE|STX| ... |DLE|STX|
				 */ 
				if( MMSN_FrameBegin == a_pFSM->FrameStatus)
				{
					// Report begin of frame error. All cleanup will be done in Error state.
					g_NetworkErrorDesc.currError = NE_Frame_Malfunction_STX;
					
					// Change state to Error
					a_pFSM->CurrentState = MMSN_ERROR_STATE;
					
					// Add software event to queue. This will trigger immediate FSM run to handle error state.
					ADD_EVENT_TO_QUEUE(a_pFSM->ptrEventQueueDesc, MMSN_ERROR_EVENT);
				}
				else
				{
					// Set frame status to begin
					a_pFSM->FrameStatus = MMSN_FrameBegin;
					
					// Do not change the state
				}
				break;
				
			case FSMR_FRAME_END:
				/* Check if frame end was already received.
				 * If true then it means frame malfunction - |DLE|ETX| ... |DLE|ETX|
				 */
				if( MMSN_FrameEnd == a_pFSM->FrameStatus)
				{
					// Report end of frame error. All cleanup will be done in Error state.
					g_NetworkErrorDesc.currError = NE_Frame_Malfunction_ETX;
					
					// Change state to Error
					a_pFSM->CurrentState = MMSN_ERROR_STATE;
					
					// Add software event to queue. This will trigger immediate FSM run to handle error state.
					ADD_EVENT_TO_QUEUE(a_pFSM->ptrEventQueueDesc, MMSN_ERROR_EVENT);
				}
				else
				{
					// Set frame status to end
					a_pFSM->FrameStatus = MMSN_FrameEnd;
					
					// We are ready to process complete data frame
					a_pFSM->CurrentState = MMSN_PROCESS_DATA_STATE;
					
					// Add software event to queue. This will trigger immediate FSM run to handle frame processing.
					ADD_EVENT_TO_QUEUE(a_pFSM->ptrEventQueueDesc, MMSN_FRAME_PROCESS_EVENT);
				}
				break;
				
			case FSMR_IGNORE_BYTE:
				// Do nothing and wait for another data
				break;
				
			case FSMR_COLLECT_BYTE:
				/* Try to add another element to the buffer
				 * and check for buffer overflow.
				 */
				if (FIFO_ERROR_OVERFLOW == fifo_push_uint8(&fifo_receive_buffer_desc, u8Data))
				{
					/* Not enough space in the buffer to hold additional data.
					 * It means that frame is broken. Current data will be lost.
					 */
					g_NetworkErrorDesc.currError = NE_RX_Buffer_Overflow;
					
					// Change state to Error
					a_pFSM->CurrentState = MMSN_ERROR_STATE;
					
					// Add software event to queue. This will trigger immediate FSM run to handle error state.
					ADD_EVENT_TO_QUEUE(a_pFSM->ptrEventQueueDesc, MMSN_ERROR_EVENT);
				}
				else
				{
					// Change frame status to collect
					a_pFSM->FrameStatus = MMSN_FrameCollect;
				}
				break;
				
			default:
				// Unhanded return code. Report an error.
				g_NetworkErrorDesc.currError = NE_ReceiverFSM_UnknownState;
				
				// Change state to Error
				a_pFSM->CurrentState = MMSN_ERROR_STATE;
				
				// Add software event to queue. This will trigger immediate FSM run to handle error state.
				ADD_EVENT_TO_QUEUE(a_pFSM->ptrEventQueueDesc, MMSN_ERROR_EVENT);
				 
				break;
		}	// end of switch
	}
	else
	{
		// Undefined function pointer to handle this event.
		g_NetworkErrorDesc.currError = NE_ReceiverFSM_UndefinedFuncPtr;
		
		// Change state to Error
		a_pFSM->CurrentState = MMSN_ERROR_STATE;
		
		// Add software event to queue. This will trigger immediate FSM run to handle error state.
		ADD_EVENT_TO_QUEUE(a_pFSM->ptrEventQueueDesc, MMSN_ERROR_EVENT);
	}
	
	return MMSN_OK;
};

uint8_t	mmsn_ExecuteCommand_ExecuteCommandEvent_Handler(MMSN_FSM_t * a_pFSM, uint8_t a_u8Event, void * a_pEventArg)
{
	/* Complete frame was received and is ready to be processed.
	 * Check if this message is handled by this device.
	 */
	
	funcCommandHandler	commandHandlerPtr = NULL;
	uint8_t				u8DeviceType;
	uint8_t				u8DeviceNumber = 0;
		
	// Retrieve device type from the message
	get_MMSN_DeviceType(a_pFSM->ptrRXDataFrame->u16Identifier, u8DeviceType);
		
	// Retrieve device number/command from the message
	get_MMSN_DeviceNumber(a_pFSM->ptrRXDataFrame->u16Identifier, u8DeviceNumber);
		
#ifdef MMSN_DEBUG
	printf("\nExec:dev_typ = %d", u8DeviceType);
	printf("\nExec:dev_num = %d", u8DeviceNumber);
#endif
	
	// Obtain appropriate function handler
	commandHandlerPtr = get_CommandFunctionHandler(u8DeviceNumber);
		
	// Execute if needed
	if (NULL == commandHandlerPtr)
	{
		/* Command is NOT handled by this device
		*/
		printf("\nExecute: NO");
	} 
	else
	{
		printf("\nExecute: YES");
		
		/* Command is handled by this device. */
		commandHandlerPtr();
	}

	// Clear internal receiving data buffer
	// memset(a_pFSM->ptrRXDataFrame->u8CommFrameTable, 0, MMSN_COMM_FRAME_SIZE);
	memset(a_pFSM->ptrRXDataFrame, 0, MMSN_COMM_FRAME_SIZE);
	
	/* All clean-up done. Go to \ref eSM_Idle state. */
	a_pFSM->CurrentState = eSM_Idle;

	return MMSN_OK;
}

uint8_t mmsn_Error_ErrorEvent_Handler(MMSN_FSM_t * a_pFSM, uint8_t a_u8Event, void * a_pEventArg)
{
	printf("\nError: %u", g_NetworkErrorDesc.currError);
	
	// Add current Network Error to the table
	add_commNetworkError(&g_NetworkErrorDesc);
	
	// Perform suitable action
	switch (g_NetworkErrorDesc.currError)
	{
		case NE_USART_Receiver_Error:
		case NE_Frame_CRC:
		case NE_Frame_Malfunction_STX:
		case NE_Frame_Malfunction_ETX:
		case NE_ReceiverFSM_UnknownState:
		case NE_ReceiverFSM_UndefinedFuncPtr:
		case NE_ReceiverFSM_Malfunction:
		case NE_RX_Buffer_Overflow:
		case NE_RX_Buffer_Underflow:
			/* All errors are related to data receiving */
			
			// Clear working RX buffer
			memset(a_pFSM->ptrRXDataFrame->u8FrameBuffer, 0, MMSN_COMM_FRAME_SIZE);
			
			// Flush receiving FIFO
			fifo_flush(&fifo_receive_buffer_desc);
			
			// Reset Receiver FSM
			fsmReceiverInitialize(&a_pFSM->ReceiverFSM);
		break;
		
		default:
		/* Your code here */
		break;
	} 

	// Clear network error descriptor
	g_NetworkErrorDesc.currError = NE_None;

	// Go to the \ref eSM_Idle state
	a_pFSM->CurrentState = eSM_Idle;
	
	return MMSN_OK;
}

uint8_t mmsn_ProcessData_FrameProcess_Handler(MMSN_FSM_t * a_pFSM, uint8_t a_u8Event, void * a_pEventArg)
{
	uint16_t u16CRC16;
	uint8_t  u8FrameSize;
	
	// Get RX buffer size
	u8FrameSize = fifo_get_used_size(&fifo_receive_buffer_desc);
	
	printf("\nProcessData: %d", u8FrameSize);
	
	// Check if expected data size was collected
	if (MMSN_COMM_FRAME_SIZE == u8FrameSize)
	{
		/* Complete frame was received. */
		
		// Make a working copy of received data frame
		_copyDataFrame(&fifo_receive_buffer_desc, a_pFSM->ptrRXDataFrame);
	
		/* printf("\nRX data = ");
		for (uint8_t u8idx = 0; u8idx < MMSN_COMM_FRAME_SIZE; u8idx++)
		{
			printf("%u, ", fifo_receive_buffer[u8idx]);
		}
		
		printf("\nCopied data = ");
		for (uint8_t u8idx = 0; u8idx < MMSN_COMM_FRAME_SIZE; u8idx++)
		{
			printf("%u, ", a_pFSM->ptrRXDataFrame->u8FrameBuffer[u8idx]);
		} */
		
		// Clear receiving FIFO
		fifo_flush(&fifo_receive_buffer_desc);
			
		/* Calculate CRC-16 (CRC-CCITT) using XMEGA hardware CRC peripheral
		 * excluding last 2 bytes with CRC-16.
		 */
		u16CRC16 = xmega_calculate_checksum_crc16(a_pFSM->ptrRXDataFrame->u8FrameBuffer, MMSN_FRAME_NOCRC_LENGTH);

		/* printf("\nmsg.CRC-16=%u", a_pFSM->ptrRXDataFrame->u16CRC16);
		printf("\nCRC-16=%u", u16CRC16); */
		
		// Check received data integrity
		
		if (u16CRC16 == a_pFSM->ptrRXDataFrame->u16CRC16)
		{
			// Calculated and received CRC-16 value matched. Go to command execution state.

			//Go to \ref eSM_ExecuteCommand state
			a_pFSM->CurrentState = MMSN_EXECUTE_COMMAND_STATE;
				
			// Add software event to queue. This will trigger immediate FSM run to handle next event.
			ADD_EVENT_TO_QUEUE(a_pFSM->ptrEventQueueDesc, MMSN_EXECUTE_COMMAND_EVENT);
		}
		else
		{
#ifdef MMSN_DEBUG
			// printf("\nFrame CRC error = %u", u16CRC16);
#endif			
			/* Received and calculated CRC-16 value does not match.
			 * Report CRC integrity error.
			 */
			g_NetworkErrorDesc.currError = NE_Frame_CRC;
				
			// Go to \ref eSM_Error state
			a_pFSM->CurrentState = MMSN_ERROR_STATE;
			
			// Add software event to queue. This will trigger immediate FSM run to handle error state.
			ADD_EVENT_TO_QUEUE(a_pFSM->ptrEventQueueDesc, MMSN_ERROR_EVENT);
		}
	}
	else
	{
#ifdef MMSN_DEBUG
		// printf("\nFrame size error = %d", u8FrameSize);
#endif
		/* Frame size does not match with expected value. Only buffer underflow should be possible.
		 * Buffer overflow condition is checked in Receive state.
		 */
		g_NetworkErrorDesc.currError = NE_RX_Buffer_Underflow;
		
		// Go to \ref eSM_Error state
		a_pFSM->CurrentState = MMSN_ERROR_STATE;
		
		// Add software event to queue. This will trigger immediate FSM run to handle error state.
		ADD_EVENT_TO_QUEUE(a_pFSM->ptrEventQueueDesc, MMSN_ERROR_EVENT);
	}

	return MMSN_OK;
}

uint8_t mmsn_Idle_SendData_Handler(MMSN_FSM_t * a_pFSM, uint8_t a_u8Event, void * a_pEventArg)
{
	/* At first check is the line is free (Collision Avoidance).
	 * If the line is still busy than back-off and wait until timer times out.
	 * The back-off timer value is determined by Logical Address in Network.
	 * Go waiting for free bus.
	 */
	if (MMSN_BUSY_BUS == a_pFSM->u8LineState)
	{
		/* Do not clear the Data Ready To Send event
		 * Transition to Wait For Resending state and wait for free bus.
		 */
		
		
		// Add software event to queue. This will trigger immediate FSM run.
		ADD_EVENT_TO_QUEUE(a_pFSM->ptrEventQueueDesc, );
		
		
		
		mmsnFSM.CurrentState = eSM_WaitForResend;
			
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
			mmsnFSM.u8TXDataCounter = 0;
			
			// Send first data byte. The remaining bytes will be sent in DRE event handler directly.
			USART_COMMUNICATION_BUS.DATA = mmsnFSM.ptrTXDataFrame->u8FrameBuffer[0];
			
			ATOMIC_BLOCK(ATOMIC_FORCEON)
			{
				// Mark that the first byte was sent
				mmsnFSM.u8TXDataCounter++;
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
		if (mmsnFSM.u8TXDataCounter < MMSN_COMM_FRAME_SIZE)
		{
			// Data still waiting to be sent
			
			/* Get next data byte and put it to USART register for sending.
			 */
			USART_COMMUNICATION_BUS.DATA = mmsnFSM.ptrTXDataFrame->u8FrameBuffer[mmsnFSM.u8TXDataCounter];
			
			// Increase transmission counter
			ATOMIC_BLOCK(ATOMIC_FORCEON)
			{
				// Another byte was sent
				mmsnFSM.u8TXDataCounter++;
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
			 mmsnFSM.CurrentState = eSM_WaitForResponse;
		}
		
		// Always clear data register empty event flag
		ATOMIC_BLOCK(ATOMIC_FORCEON)
		{
			FLAG_CLEAR(gSystemEvents, EVENT_IRQ_DATA_REGISTER_EMPTY_bm);
		};		
	}
	
	return MMSN_OK;
}







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
	//fsm_ClearBusyLine();
	
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

/* Initialize Multi-Master Serial Network FSM */
void mmsn_InitializeStateMachine(MMSN_FSM_t * a_pFSM)
{
	// Initialize network manager FSM
	a_pFSM->u8LineState			= MMSN_BUSY_BUS;		//! Set bus state to busy
	a_pFSM->CurrentState		= eSM_Idle;				//! Set current state to \ref eSM_Initialize
	a_pFSM->PreviousState		= eSM_Idle;				//! Set previous state to \ref eSM_Initialize
	a_pFSM->ptrRXDataFrame		= &g_RxCommFrameBuffer;	//! Pointer to internal copy of received data frame
	a_pFSM->ptrTXDataFrame		= &g_TxCommFrameBuffer;	//! Pointer to internal copy of transmitted data frame
	a_pFSM->u8TXDataCounter		= 0;					//! Transmitted data counter
	a_pFSM->u8RetriesCount		= 0;					//! Retries counter
	a_pFSM->FrameStatus			= MMSN_FrameUnknown;	//! Start with an unknown frame state
	a_pFSM->ptrEventQueueDesc	= &eventQueue_desc;		//! Store pointer to event queue
	
	// Assign pointer to receiver FSM action handler table
	a_pFSM->pFSMRActionTable = FSMReceiverActionHandlerTable;
	
	// Initialize receiver FSM
	fsmReceiverInitialize(&a_pFSM->ReceiverFSM);
	
	/* Initialize sending and receiving FIFOs */
	fifo_init(&fifo_receive_buffer_desc, &fifo_receive_buffer[0], FIFO_RECEIVE_BUFFER_SIZE);
	fifo_init(&fifo_send_buffer_desc,	 &fifo_send_buffer[0],	  FIFO_SEND_BUFFER_SIZE);
	
	// Initialize Network Error Descriptor
	init_commNetworkError(&g_NetworkErrorDesc, &CommNetworkErrorTable[0], sizeof(CommNetworkErrorTable));
	
	/************************************************************************/
	/* HARDWARE SETUP														*/
	/************************************************************************/
	
	// Configure and initialize communication bus usart
	xmega_usart_configure();
	
	/* RS-485 PHYSICAL DEVICE CONFIGURATION */
	// Initialize GPIO related to RS-485 interface
	rs485_driver_gpio_initialize();
	// Initially go LOW to enable receiver and start listening
	rs485_receiver_enable();
	
	/* USART INTERRUPTS CONFIGURATION - RECEIVING */
	// Turn on USART RXC interrupt
	xmega_set_usart_rx_interrupt_level(&USART_COMMUNICATION_BUS, USART_RXCINTLVL_HI_gc);
	
	// Turn off TXC and DRE interrupts
	xmega_set_usart_tx_interrupt_level(&USART_COMMUNICATION_BUS, USART_TXCINTLVL_OFF_gc);
	xmega_set_usart_dre_interrupt_level(&USART_COMMUNICATION_BUS, USART_DREINTLVL_OFF_gc);
};

/* IDLE state handler */
void fsm_Idle(void)
{
};	// fsm_Idle()

/* RECEIVE state handler */
void fsm_Receive(void)
{

};	// fsm_Receive()

/* PROCESS_DATA state handler */
void fsm_ProcessData(void)
{
};	// fsm_ProcessData()

void fsm_ExecuteCommand(void)
{
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
		if (MMSN_BUSY_BUS == mmsnFSM.u8LineState)
		{
			/* Do not clear the Data Ready To Send event
			 * Transition to Wait For Resending state and wait for free bus.
			 */
			mmsnFSM.CurrentState = eSM_WaitForResend;
			
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
			mmsnFSM.u8TXDataCounter = 0;
			
			// Send first data byte. The remaining bytes will be sent in DRE event handler directly.
			USART_COMMUNICATION_BUS.DATA = mmsnFSM.ptrTXDataFrame->u8FrameBuffer[0];
			
			ATOMIC_BLOCK(ATOMIC_FORCEON)
			{
				// Mark that the first byte was sent
				mmsnFSM.u8TXDataCounter++;
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
		if (mmsnFSM.u8TXDataCounter < MMSN_COMM_FRAME_SIZE)
		{
			// Data still waiting to be sent
			
			/* Get next data byte and put it to USART register for sending.
			 */
			USART_COMMUNICATION_BUS.DATA = mmsnFSM.ptrTXDataFrame->u8FrameBuffer[mmsnFSM.u8TXDataCounter];
			
			// Increase transmission counter
			ATOMIC_BLOCK(ATOMIC_FORCEON)
			{
				// Another byte was sent
				mmsnFSM.u8TXDataCounter++;
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
			 mmsnFSM.CurrentState = eSM_WaitForResponse;
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
		mmsnFSM.CurrentState = eSM_Receive;
		
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
		mmsnFSM.CurrentState = eSM_Send;
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
		if (MMSN_MAX_RETRIES <= mmsnFSM.u8RetriesCount)
		{
			/* Maximum retries count reached.
			 * Report an error.
			 */
			g_NetworkErrorDesc.currError = NE_MaximumRetries;
				
			// Go to \ref eSM_Error state
			mmsnFSM.CurrentState = eSM_Error;
			
			// Clear retries counter
			mmsnFSM.u8RetriesCount = 0;
				
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
			mmsnFSM.u8RetriesCount++;
			
			// Reset data transmission counter
			mmsnFSM.u8TXDataCounter = 0;
			
			// Go to \ref eSM_Send state to send complete message again
			mmsnFSM.CurrentState = eSM_Send;
			
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
		g_NetworkErrorDesc.currError = NE_None;
		
		// Flush receiving FIFO
		fifo_flush(&fifo_receive_buffer_desc);
		
		// Clear corresponding event
		ATOMIC_BLOCK(ATOMIC_FORCEON)
		{
			// Atomic interrupt safe set of global variable storing event flags
			FLAG_CLEAR(gSystemEvents, EVENT_SW_RECEIVE_DATA_ERROR_bm);
		};
		
		// Go to the \ref eSM_Idle state
		mmsnFSM.CurrentState = eSM_Idle;
	}
	
	/* Data frame integrity error (CRC-16 does not match).
	 */
	if (u16EventFlags & EVENT_SW_COMM_FRAME_CRC_ERROR_bm)
	{
		// Add current Network Error to the table
		add_commNetworkError(&g_NetworkErrorDesc);
		
		// Clear network error descriptor
		g_NetworkErrorDesc.currError = NE_None;
		
		// Clear receiving buffer
		// memset(mmsnFSM.ptrRXDataFrame->u8CommFrameTable, 0, MMSN_COMM_FRAME_SIZE);
		
		// Flush receiving FIFO
		fifo_flush(&fifo_receive_buffer_desc);
		
		// Clear corresponding event
		ATOMIC_BLOCK(ATOMIC_FORCEON)
		{
			// Atomic interrupt safe set of global variable storing event flags
			FLAG_CLEAR(gSystemEvents, EVENT_SW_COMM_FRAME_CRC_ERROR_bm);
		};
		
		// Go to the \ref eSM_Idle state
		mmsnFSM.CurrentState = eSM_Idle;
	}
	
	/* Maximum retries count reached. Message to be sent is lost.
	 */
	if (u16EventFlags & EVENT_SW_MAX_RETRIES_COUNT_REACHED_bm)
	{
		// Add current Network Error to the table
		add_commNetworkError(&g_NetworkErrorDesc);
		
		// Clear network error descriptor
		g_NetworkErrorDesc.currError = NE_None;
		
		// Clear transmission buffer
		memset(mmsnFSM.ptrTXDataFrame->u8FrameBuffer , 0, MMSN_COMM_FRAME_SIZE);
		
		// Clear transmission FIFO
		fifo_flush(&fifo_send_buffer_desc);
		
		// Reset retries count
		mmsnFSM.u8RetriesCount = 0;
		
		// Clear transmitted bytes counter
		mmsnFSM.u8TXDataCounter = 0;
		
		// Clear corresponding event
		ATOMIC_BLOCK(ATOMIC_FORCEON)
		{
			// Atomic interrupt safe set of global variable storing event flags
			FLAG_CLEAR(gSystemEvents, EVENT_SW_MAX_RETRIES_COUNT_REACHED_bm);
		};
		
		// Go to the \ref eSM_Idle state
		mmsnFSM.CurrentState = eSM_Idle;
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
bool _isLogicalNetworkAddrAssigned(uint8_t *a_pu8LogicalNetworkAddr)
{
	return (!(MMSN_DEFAULT_LOGICAL_NETWORK_ADDRESS == (*a_pu8LogicalNetworkAddr)));
};

// Function generates random logical network address.
uint8_t xmega_generate_random_logical_network_address(void)
{
	return ((rand() % 127) + 1);
};
