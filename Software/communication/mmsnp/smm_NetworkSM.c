/*
Serial Multi-Master Network State Machine
*/

#include "smm_NetworkSM.h"
#include "fsm_Receiver.h"

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/atomic.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <board_config.h>
#include <tc_driver.h>
#include <usart_driver.h>
#include <utils.h>

#ifndef __HAS_XMEGA_HARDWARE_CRC
	#include <util/crc16.h>
#endif

/**
 * \brief Buffer to associate with receiving FIFO buffer
 *
 * This buffer consists of \ref FIFO_RECEIVE_BUFFER_SIZE elements
 * capable of holding a byte
 */
uint8_t fifo_receive_buffer [MMSNP_FIFO_RECEIVE_BUFFER_SIZE];

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
// uint8_t fifo_send_buffer [FIFO_SEND_BUFFER_SIZE];

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
// fifo_desc_t fifo_send_buffer_desc;

/* Bitmasked flags that describe what event has occurred */
extern volatile uint16_t gSystemEvents;

// Variable to store current event flag
extern uint16_t u16EventFlags;

// Structure for storing received frame
mmsn_receive_data_frame_t g_RxCommFrameBuffer;
// Structure for storing frame to be transmitted
// mmsn_comm_data_frame_t g_TxCommFrameBuffer;

// Event queue descriptor
extern fifo_desc_t eventQueue_desc;

// Global storage for sending data frame buffer
mmsnp_send_data_frame_t g_SendDataFrame;

/************************************************************************/
/* FSM RECEIVER                                                         */
/************************************************************************/
extern fsmReceiverActionHandler FSMReceiverActionHandlerTable[][FSMR_MAX_EVENTS];

// Multi-Master Serial Network FSM
MMSN_FSM_t g_oMMSNP;

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

/************************************************************************/
/* Interface functions: Global storage for sending data                 */
/************************************************************************/

void _sendData_FrameBuffer_Init(mmsnp_send_data_frame_t *a_pFrameBuffer)
{
	// Clear all data within frame buffer
	memset(a_pFrameBuffer, 0, sizeof(mmsnp_send_data_frame_t));
};

// Note that no check against source data size is made
void _sendData_FrameBuffer_Write(mmsnp_send_data_frame_t *a_pDst, uint8_t a_u8DataSize, bool a_Response, uint8_t * a_pSrcData, uint8_t a_u8SrcDataSize)
{
	a_pDst->u8DataSize			= a_u8DataSize;
	a_pDst->u8IsResponseNeeded	= a_Response;
	memcpy(a_pDst->u8SendDataBuffer, a_pSrcData, a_u8SrcDataSize);
};

void _sendData_FrameBuffer_Copy(const mmsnp_send_data_frame_t *a_pSrc, mmsnp_send_data_frame_t *a_pDst)
{
	memcpy(a_pDst, a_pSrc, sizeof(mmsnp_send_data_frame_t));
};

/* Function event handlers pointer table */
mmsnFsmEventHandler mmsnFSMActionTable[MMSN_MAX_STATES][MMSNP_MAX_EVENTS] =
{
	/* MMSN_IDLE_STATE */
	/* MMSN_DATA_RECEIVED_EVENT, MMSN_COLLISION_AVOIDANCE_TIMEOUT, MMSN_ERROR_EVENT, MMSN_FRAME_PROCESS_EVENT	   */
	/* MMSN_EXECUTE_COMMAND_EVENT, MMSN_SEND_DATA_EVENT, MMSN_DATA_REG_EMPTY_EVENT, MMSN_NO_RESPONSE_TIMEOUT_EVENT */
	/* MMSN_RETRANSMISSION_EVENT */
	{ mmsn_Idle_DataReceived_Handler, mmsn_Idle_CollisionAvoidanceTimeoutEvent_Handler, NULL, NULL,
	  NULL, mmsn_Idle_SendDataEvent_Handler, NULL, NULL,
	  NULL },
	
	/* MMSN_RECEIVE_STATE */
	/* MMSN_DATA_RECEIVED_EVENT, MMSN_COLLISION_AVOIDANCE_TIMEOUT, MMSN_ERROR_EVENT, MMSN_FRAME_PROCESS_EVENT	   */
	/* MMSN_EXECUTE_COMMAND_EVENT, MMSN_SEND_DATA_EVENT, MMSN_DATA_REG_EMPTY_EVENT, MMSN_NO_RESPONSE_TIMEOUT_EVENT */
	/* MMSN_RETRANSMISSION_EVENT */
	{ mmsn_Receive_DataReceived_Handler, mmsn_xxx_CollisionAvoidanceTimeoutEvent_Handler, NULL, NULL,
	  NULL, NULL, NULL, NULL,
	  NULL },

	/* MMSN_PROCESS_DATA_STATE */
	/* MMSN_DATA_RECEIVED_EVENT, MMSN_COLLISION_AVOIDANCE_TIMEOUT, MMSN_ERROR_EVENT, MMSN_FRAME_PROCESS_EVENT	   */
	/* MMSN_EXECUTE_COMMAND_EVENT, MMSN_SEND_DATA_EVENT, MMSN_DATA_REG_EMPTY_EVENT, MMSN_NO_RESPONSE_TIMEOUT_EVENT */
	/* MMSN_RETRANSMISSION_EVENT */
	{ NULL, mmsn_xxx_CollisionAvoidanceTimeoutEvent_Handler, NULL, mmsn_ProcessData_FrameProcess_Handler,
	  NULL, NULL, NULL, NULL,
	  NULL },

	/* MMSN_EXECUTE_COMMAND_STATE */
	/* MMSN_DATA_RECEIVED_EVENT, MMSN_COLLISION_AVOIDANCE_TIMEOUT, MMSN_ERROR_EVENT, MMSN_FRAME_PROCESS_EVENT	   */
	/* MMSN_EXECUTE_COMMAND_EVENT, MMSN_SEND_DATA_EVENT, MMSN_DATA_REG_EMPTY_EVENT, MMSN_NO_RESPONSE_TIMEOUT_EVENT */
	/* MMSN_RETRANSMISSION_EVENT */
	{ NULL, mmsn_xxx_CollisionAvoidanceTimeoutEvent_Handler, NULL, NULL,
	  mmsn_ExecuteCommand_ExecuteCommandEvent_Handler, NULL, NULL, NULL,
	  NULL },

	/*** MMSN_SEND_STATE ***/
	/* MMSN_DATA_RECEIVED_EVENT, MMSN_COLLISION_AVOIDANCE_TIMEOUT, MMSN_ERROR_EVENT, MMSN_FRAME_PROCESS_EVENT	   */
	/* MMSN_EXECUTE_COMMAND_EVENT, MMSN_SEND_DATA_EVENT, MMSN_DATA_REG_EMPTY_EVENT, MMSN_NO_RESPONSE_TIMEOUT_EVENT */
	/* MMSN_RETRANSMISSION_EVENT */
	{ NULL, NULL, NULL, NULL,
	  NULL, NULL, mmsn_Send_DataRegEmptyEvent_Handler, NULL,
	  NULL },
				
	/* MMSN_WAIT_FOR_RESPONSE_STATE */
	/* MMSN_DATA_RECEIVED_EVENT, MMSN_COLLISION_AVOIDANCE_TIMEOUT, MMSN_ERROR_EVENT, MMSN_FRAME_PROCESS_EVENT	   */
	/* MMSN_EXECUTE_COMMAND_EVENT, MMSN_SEND_DATA_EVENT, MMSN_DATA_REG_EMPTY_EVENT, MMSN_NO_RESPONSE_TIMEOUT_EVENT */
	/* MMSN_RETRANSMISSION_EVENT */
	{ mmsn_WaitForResponse_DataReceivedEvent_Handler, mmsn_xxx_CollisionAvoidanceTimeoutEvent_Handler, NULL, NULL,
	  NULL, NULL, NULL, mmsn_xxx_NoResponseTimeoutEvent_Handler,
	  NULL },

	/* MMSN_RECEIVE_RESPONSE_STATE */
	/* MMSN_DATA_RECEIVED_EVENT, MMSN_COLLISION_AVOIDANCE_TIMEOUT, MMSN_ERROR_EVENT, MMSN_FRAME_PROCESS_EVENT	   */
	/* MMSN_EXECUTE_COMMAND_EVENT, MMSN_SEND_DATA_EVENT, MMSN_DATA_REG_EMPTY_EVENT, MMSN_NO_RESPONSE_TIMEOUT_EVENT */
	/* MMSN_RETRANSMISSION_EVENT */
	{ mmsn_ReceiveResponse_DataReceivedEvent_Handler, mmsn_xxx_CollisionAvoidanceTimeoutEvent_Handler, NULL, NULL,
	  NULL, NULL, NULL, mmsn_xxx_NoResponseTimeoutEvent_Handler,
	  NULL },

	/* MMSN_PROCESS_RESPONSE_STATE */
	/* MMSN_DATA_RECEIVED_EVENT, MMSN_COLLISION_AVOIDANCE_TIMEOUT, MMSN_ERROR_EVENT, MMSN_FRAME_PROCESS_EVENT	   */
	/* MMSN_EXECUTE_COMMAND_EVENT, MMSN_SEND_DATA_EVENT, MMSN_DATA_REG_EMPTY_EVENT, MMSN_NO_RESPONSE_TIMEOUT_EVENT */
	/* MMSN_RETRANSMISSION_EVENT */
	{ NULL, mmsn_xxx_CollisionAvoidanceTimeoutEvent_Handler, NULL, mmsn_ProcessResponse_FrameProcessEvent_Handler,
	  NULL, NULL, NULL, NULL,
	  NULL },

	/* MMSN_RETRANSMIT_STATE */
	/* MMSN_DATA_RECEIVED_EVENT, MMSN_COLLISION_AVOIDANCE_TIMEOUT, MMSN_ERROR_EVENT, MMSN_FRAME_PROCESS_EVENT	   */
	/* MMSN_EXECUTE_COMMAND_EVENT, MMSN_SEND_DATA_EVENT, MMSN_DATA_REG_EMPTY_EVENT, MMSN_NO_RESPONSE_TIMEOUT_EVENT */
	/* MMSN_RETRANSMISSION_EVENT */
	{ NULL, mmsn_Retransmit_CollisionAvoidanceTimeoutEvent_Handler, NULL, NULL,
	  NULL, NULL, NULL, NULL,
	  mmsn_Retransmit_RetransmissionEvent_Handler },

	/* MMSN_ERROR_STATE */
	/* MMSN_DATA_RECEIVED_EVENT, MMSN_COLLISION_AVOIDANCE_TIMEOUT, MMSN_ERROR_EVENT, MMSN_FRAME_PROCESS_EVENT	   */
	/* MMSN_EXECUTE_COMMAND_EVENT, MMSN_SEND_DATA_EVENT, MMSN_DATA_REG_EMPTY_EVENT, MMSN_NO_RESPONSE_TIMEOUT_EVENT */
	/* MMSN_RETRANSMISSION_EVENT */
	{ NULL, mmsn_xxx_CollisionAvoidanceTimeoutEvent_Handler, mmsn_Error_ErrorEvent_Handler, NULL,
	  NULL, NULL, NULL, NULL,
	  NULL }
};

/* Common functions */
static inline void _restartCollisionAvoidanceTimer(void)
{
	ATOMIC_BLOCK(ATOMIC_FORCEON)
	{
		/* Turn collision avoidance timer on */
		xmega_tc_select_clock_source(&TIMER_COLLISION_AVOIDANCE, TC_CLKSEL_DIV64_gc);
	
		/* Force Restart of Collision Avoidance timer */
		xmega_tc_restart(&TIMER_COLLISION_AVOIDANCE);
	};
};

/**
 * \brief Make a working copy of received data frame.
 *
 *  \param none.
 */
void _copyDataFrame(fifo_desc_t * a_pFifoDesc, mmsn_receive_data_frame_t * a_pDstDataFrame)
{
	/* Pull out first two bytes with packed Header.
	 * Dev_Type:4 | LAddr:7 | RTR:1 | Control_Field:4 |
	 */
	a_pDstDataFrame->u8HeaderHiByte = fifo_pull_uint8_nocheck(a_pFifoDesc);
	a_pDstDataFrame->u8HeaderLoByte = fifo_pull_uint8_nocheck(a_pFifoDesc);
	
	/* Pull out all data bytes */
	for (uint8_t u8Idx = 0; u8Idx < MMSN_DATA_LENGTH; u8Idx++)
	{
		a_pDstDataFrame->u8DataBuffer[u8Idx] = fifo_pull_uint8_nocheck(a_pFifoDesc);
	};
	
	/* Pull out 2 bytes with CRC-16 16bit value.
	 * This value is send over the Network using big endian order CRC16 = (Hi_byte|Lo_byte)
	 */
	a_pDstDataFrame->u8CRC16HiByte = fifo_pull_uint8_nocheck(a_pFifoDesc);
	a_pDstDataFrame->u8CRC16LoByte = fifo_pull_uint8_nocheck(a_pFifoDesc);
};	// _copyDataFrame()

/**
 * \brief Clear all resources utilized during receiving process.
 *
 *  \param a_pFSM Pointer to Multi-master FSM
 */
void _ClearRxResources(MMSN_FSM_t * a_pFSM)
{
	// Clear working RX buffer
	memset(a_pFSM->ptrRxDataFrame->u8FrameBuffer, 0, MMSNP_COMM_FRAME_SIZE);
	
	// Flush receiving FIFO
	fifo_flush(&fifo_receive_buffer_desc);
	
	// Reset Receiver FSM
	fsmReceiverInitialize(&a_pFSM->ReceiverFSM);
	
	// Reset bus state
	a_pFSM->u8LineState = MMSNP_FREE_BUS;
	
	// Reset frame status
	a_pFSM->FrameStatus = MMSN_FrameUnknown;
}

/**
 * \brief Clear all resources utilized during transmission process.
 *
 *  \param a_pFSM Pointer to Multi-Master Serial Network Protocol FSM Manager
 */
void _resetSendDataAttributes(MMSN_FSM_t * a_pFSM)
{
	/* Clear sending data attributes */
	a_pFSM->SendDataAttr.u8DataSize			= 0;
	a_pFSM->SendDataAttr.u8IsResponseNeeded = false;
	
	/* Reset transmission counter */
	a_pFSM->SendDataAttr.u8DataCounter = 0;
};

/************************************************************************/
/* EVENT HANDLERS                                                       */
/************************************************************************/
uint8_t mmsn_Idle_DataReceived_Handler(MMSN_FSM_t * a_pFSM, uint8_t a_u8Event, void * a_pEventArg)
{
	ATOMIC_BLOCK(ATOMIC_FORCEON)
	{
		/* The error free data was received. It means that the bus currently is or was in use.
		* To avoid collision (CA) start busy line timer or restart if already running.
		*/
		_restartCollisionAvoidanceTimer();

		// Set bus to busy state
		a_pFSM->u8LineState = MMSNP_BUSY_BUS;

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
				/* Change next state to Receive */
				a_pFSM->CurrentState = MMSN_RECEIVE_STATE;
				/* Update previous state */
				a_pFSM->PreviousState = MMSN_IDLE_STATE;
			
				/* Set frame status to begin */
				a_pFSM->FrameStatus = MMSN_FrameBegin;
			
				// Wait for another received data system event
			}
		}
		else
		{
			// Undefined function pointer to handle this event.
			g_NetworkErrorDesc.currError = NE_ReceiverFSM_UndefinedFuncPtr;
		
			/* Change state to Error */
			a_pFSM->CurrentState = MMSN_ERROR_STATE;
			/* Update previous state */
			a_pFSM->PreviousState = MMSN_IDLE_STATE;
		
			// Add software event to queue. This will trigger immediate FSM run to handle error state.
			ADD_EVENT_TO_QUEUE(a_pFSM->ptrEventQueueDesc, MMSN_ERROR_EVENT);
		}
	};

	return MMSNP_OK;
};

uint8_t mmsn_xxx_CollisionAvoidanceTimeoutEvent_Handler(MMSN_FSM_t * a_pFSM, uint8_t a_u8Event, void * a_pEventArg)
{
	ATOMIC_BLOCK(ATOMIC_FORCEON)
	{
		/* Stop collision avoidance timer by setting clock source to OFF state */
		xmega_tc_select_clock_source(&TIMER_COLLISION_AVOIDANCE, TC_CLKSEL_OFF_gc);

		/* Clear busy line flag */
		a_pFSM->u8LineState = MMSNP_FREE_BUS;
	};
	
	return MMSNP_OK;
}

uint8_t mmsn_Idle_CollisionAvoidanceTimeoutEvent_Handler(MMSN_FSM_t * a_pFSM, uint8_t a_u8Event, void * a_pEventArg)
{
	ATOMIC_BLOCK(ATOMIC_FORCEON)
	{
		/* Perform common activities related to Collision Avoidance timer timeout.
		 * 1. Stop the timer.
		 * 2. Indicate that the bus is free.
		 */
	
		/* Stop collision avoidance timer by setting clock source to OFF state */
		xmega_tc_select_clock_source(&TIMER_COLLISION_AVOIDANCE, TC_CLKSEL_OFF_gc);
		
		/* Clear busy line flag. */
		a_pFSM->u8LineState = MMSNP_FREE_BUS;
	
		// Check if there is data pending to be sent
		if (true == a_pFSM->u8IsDataToSend)
		{
			/* Data is pending. Send first byte and go to sending state.
			 * Remaining data will be handled in \ref MMSN_DATA_REG_EMPTY_EVENT event handler.
			 */
		
			/* Turn off receiving (automatically if RS-485 driver and receiver pins are tight together)
			 * and set RS-485 transceiver to transmit mode
			 */
			RS485_DRIVER_ENABLE();
		
			/* Nevertheless turn off USART RXC interrupt during sending the data */
			xmega_set_usart_rx_interrupt_level(&USART_COMMUNICATION_BUS, USART_RXCINTLVL_OFF_gc);
		
			/* Turn on TXC and DRE interrupts */
			// xmega_set_usart_tx_interrupt_level(&USART_COMMUNICATION_BUS, USART_TXCINTLVL_HI_gc);
			xmega_set_usart_dre_interrupt_level(&USART_COMMUNICATION_BUS, USART_DREINTLVL_HI_gc);
		
			/* Reset data transmit counter */
			a_pFSM->SendDataAttr.u8DataCounter = 0;
		
			/* Send first data byte. The remaining bytes will be sent in DRE event handler directly. */
			USART_COMMUNICATION_BUS.DATA = a_pFSM->SendDataAttr.pu8DataBuffer[0];
		
			/* Change state to \ref MMSN_SEND_STATE */
			a_pFSM->CurrentState = MMSN_SEND_STATE;
			/* Set previous state */
			a_pFSM->PreviousState = MMSN_IDLE_STATE;
		
			/* Clear flag indicating data waiting to be sent */
			a_pFSM->u8IsDataToSend = false;
		}
		else
		{
			// Do nothing here
		}
	};
	
	return MMSNP_OK;
};

uint8_t mmsn_Receive_DataReceived_Handler(MMSN_FSM_t * a_pFSM, uint8_t a_u8Event, void * a_pEventArg)
{
	ATOMIC_BLOCK(ATOMIC_FORCEON)
	{
		// Restart collision avoidance timer
		_restartCollisionAvoidanceTimer();
	
		// Set bus to busy state
		a_pFSM->u8LineState = MMSNP_BUSY_BUS;
	
		// Obtain received data
		uint8_t u8Data = (*(uint8_t *)a_pEventArg);

		// Obtain pointer to ReceiverFSM action handler
		if( GET_EV_HDL_P(a_pFSM->pFSMRActionTable, a_pFSM->ReceiverFSM, FSMR_DATA_RECEIVED))
		{
			uint8_t u8RetCode;
		
			u8RetCode = CALL_EV_HDL(a_pFSM->pFSMRActionTable, a_pFSM->ReceiverFSM, FSMR_DATA_RECEIVED, u8Data);
		
			/* Check if data should be stored in RX fifo buffer */
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
					
						/* Change state to Error */
						a_pFSM->CurrentState = MMSN_ERROR_STATE;
						/* Update previous state */
						a_pFSM->PreviousState = MMSN_RECEIVE_STATE;
					
						// Add software event to queue. This will trigger immediate FSM run to handle error state.
						ADD_EVENT_TO_QUEUE(a_pFSM->ptrEventQueueDesc, MMSN_ERROR_EVENT);
					}
					else
					{
						/* Set frame status to begin */
						a_pFSM->FrameStatus = MMSN_FrameBegin;
					
						// Do not change the state.
						/* Update previous state */
						a_pFSM->PreviousState = MMSN_RECEIVE_STATE;
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
						/* Update previous state */
						a_pFSM->PreviousState = MMSN_RECEIVE_STATE;
					
						// Add software event to queue. This will trigger immediate FSM run to handle error state.
						ADD_EVENT_TO_QUEUE(a_pFSM->ptrEventQueueDesc, MMSN_ERROR_EVENT);
					}
					else
					{
						/* Set frame status to end */
						a_pFSM->FrameStatus = MMSN_FrameEnd;
					
						/* We are ready to process complete data frame */
						a_pFSM->CurrentState = MMSN_PROCESS_DATA_STATE;
						/* Update previous state */
						a_pFSM->PreviousState = MMSN_RECEIVE_STATE;
					
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
					
						/* Change state to Error */
						a_pFSM->CurrentState = MMSN_ERROR_STATE;
						/* Update previous state */
						a_pFSM->PreviousState = MMSN_RECEIVE_STATE;
					
						// Add software event to queue. This will trigger immediate FSM run to handle error state.
						ADD_EVENT_TO_QUEUE(a_pFSM->ptrEventQueueDesc, MMSN_ERROR_EVENT);
					}
					else
					{
						/* Change frame status to collect */
						a_pFSM->FrameStatus = MMSN_FrameCollect;
					}
					break;
				
				default:
					// Unhanded return code. Report an error.
					g_NetworkErrorDesc.currError = NE_ReceiverFSM_UnknownState;
				
					// Change state to Error
					a_pFSM->CurrentState = MMSN_ERROR_STATE;
					/* Update previous state */
					a_pFSM->PreviousState = MMSN_RECEIVE_STATE;
				
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
	};
	
	return MMSNP_OK;
};

uint8_t	mmsn_ExecuteCommand_ExecuteCommandEvent_Handler(MMSN_FSM_t * a_pFSM, uint8_t a_u8Event, void * a_pEventArg)
{
	// ATOMIC_BLOCK(ATOMIC_FORCEON)
	{
		/* Complete frame was received and is ready to be processed.
		 * Check if this message is handled by this device.
		 */
		funcCommandHandler	commandHandlerPtr = NULL;
		// uint8_t				u8DeviceType;
		uint8_t				u8DeviceNumber = 0;
		uint16_t			u16Header;	
	
		// Convert two bytes in big endian order to 16bit variable
		MMSN_BYTES_2_WORD(a_pFSM->ptrRxDataFrame->u8HeaderHiByte, a_pFSM->ptrRxDataFrame->u8HeaderLoByte, u16Header);
	
		// Retrieve device type from the message
		// get_MMSN_DeviceType(u16Header, u8DeviceType);
		
		// Retrieve device number or system command from the message
		get_MMSN_DeviceNumber(u16Header, u8DeviceNumber);
		
#ifdef MMSNP_DEBUG
		// printf("\nExec:dev_typ = %d", u8DeviceType);
		//printf("\nExec:dev_num = %d", u8DeviceNumber);
#endif
		/* Obtain appropriate function handler */
		commandHandlerPtr = get_CommandFunctionHandler(u8DeviceNumber);
		
		// Execute if needed
		if (NULL == commandHandlerPtr)
		{
			/* Command is NOT handled by this device
			*/
			// printf("\nExecute: NO");
		} 
		else
		{
			// printf("\nExecute: YES");
		
			/* Command is handled by this device. */
			commandHandlerPtr(a_pFSM->ptrRxDataFrame);
		};

		/* Clear internal receiving data buffer */
		memset(a_pFSM->ptrRxDataFrame, 0, MMSNP_COMM_FRAME_SIZE);
	
		/* All clean-up done. Go to \ref MMSN_IDLE_STATE state */
		a_pFSM->CurrentState = MMSN_IDLE_STATE;
		/* Update previous state */
		a_pFSM->PreviousState = MMSN_EXECUTE_COMMAND_STATE;
	};

	return MMSNP_OK;
};

uint8_t mmsn_Send_DataRegEmptyEvent_Handler(MMSN_FSM_t *a_pFSM, uint8_t a_u8Event, void *a_pEventArg)
{
	/* Check if any data is left by comparing actual counter value with data size */
	if( a_pFSM->SendDataAttr.u8DataCounter < a_pFSM->SendDataAttr.u8DataSize)
	{
		/* Data still waiting to be sent */
		
		/* Increase transmission counter */
		a_pFSM->SendDataAttr.u8DataCounter++;
		
		/* Get next data byte and put it to USART register for sending */
		USART_COMMUNICATION_BUS.DATA = a_pFSM->SendDataAttr.pu8DataBuffer[a_pFSM->SendDataAttr.u8DataCounter];	
			
		/* Do not change the state until any data left in transmission buffer */
			
		/* Update previous state */
		a_pFSM->PreviousState = MMSN_SEND_STATE;
		
		/* Turn on DRE interrupt */
		xmega_set_usart_dre_interrupt_level(&USART_COMMUNICATION_BUS, USART_DREINTLVL_HI_gc);
	}
	else
	{
		/* Complete frame was sent out */
		
		/* Wait until all data is shifted out.
			* 1. Wait for TX complete.
			* 2. Clear TX interrupt flag
			*/
		while(!(USART_COMMUNICATION_BUS.STATUS & USART_TXCIF_bm));
		USART_COMMUNICATION_BUS.STATUS |= USART_TXCIF_bm;
			
		/* Turn off TXC and DRE interrupts */
		// xmega_set_usart_tx_interrupt_level(&USART_COMMUNICATION_BUS, USART_TXCINTLVL_OFF_gc);
		xmega_set_usart_dre_interrupt_level(&USART_COMMUNICATION_BUS, USART_DREINTLVL_OFF_gc);
			
		/* Set RS-485 transceiver for receiving */
		RS485_RECEIVER_ENABLE();
			
		/* Turn on USART RXC interrupt */
		xmega_set_usart_rx_interrupt_level(&USART_COMMUNICATION_BUS, USART_RXCINTLVL_HI_gc);

		/* If response is needed then
			* - start timer waiting for a response
			* - initialize receiver FSM
			* - go to the \ref MMSN_WAIT_FOR_RESPONSE_STATE state
			*
			* Otherwise
			* - clear things up
			* - go to the \ref MMSN_IDLE_STATE state
			*/
		if (true == a_pFSM->SendDataAttr.u8IsResponseNeeded)
		{
			/* Start waiting for response timer */
			xmega_tc_select_clock_source(&TIMER_NO_RESPONSE, TC_CLKSEL_DIV64_gc);
			
			/* Make transition to waiting for a response.
				* State machine will be woken up after USART RXC IRQ.
				*/
			a_pFSM->CurrentState = MMSN_WAIT_FOR_RESPONSE_STATE;
			/* Update previous state */
			a_pFSM->PreviousState = MMSN_SEND_STATE;
		}
		else
		{
			/* Reset transmission related resources */
			_resetSendDataAttributes(a_pFSM);
			
			/* Make transition to IDLE state. */
			a_pFSM->CurrentState = MMSN_IDLE_STATE;
			/* Update previous state */
			a_pFSM->PreviousState = MMSN_SEND_STATE;
		}

		/* Always clear receiver related resources */
		_ClearRxResources(a_pFSM);
	}
	
	return MMSNP_OK;
};

uint8_t mmsn_WaitForResponse_DataReceivedEvent_Handler(MMSN_FSM_t *a_pFSM, uint8_t a_u8Event, void *a_pEventArg)
{
	ATOMIC_BLOCK(ATOMIC_FORCEON)
	{
		/* The error free data was received. It means that the bus currently is or was in use.
		* To avoid collision (CA) start busy line timer or restart if already running.
		*/
		_restartCollisionAvoidanceTimer();

		// Set bus to busy state
		a_pFSM->u8LineState = MMSNP_BUSY_BUS;

		// In this state no other than DLE data is expected and ignore return value from Receiver FSM.
	
		// Obtain pointer to ReceiverFSM action handler
		if( GET_EV_HDL_P(a_pFSM->pFSMRActionTable, a_pFSM->ReceiverFSM, FSMR_DATA_RECEIVED))
		{
			uint8_t u8RetCode;
	
			/* Call action handler */
			u8RetCode = CALL_EV_HDL(a_pFSM->pFSMRActionTable, a_pFSM->ReceiverFSM, FSMR_DATA_RECEIVED, (*(uint8_t *)a_pEventArg));
		
			/* Receiver FSM will always return with IGNORE until |DLE|STX| sequence is received.
			 * |DLE|STX| data pair is a frame begin marker. When received change the state to \ref MMSN_RECEIVE_RESPONSE_STATE.
			 * Otherwise ignore the data and do not change current state.
			 */
			if (FSMR_FRAME_BEGIN == u8RetCode)
			{
				/* Change state to ReceiveResponse and wait for another received data system event */
				a_pFSM->CurrentState = MMSN_RECEIVE_RESPONSE_STATE;
				/* Update previous state */
				a_pFSM->PreviousState = MMSN_WAIT_FOR_RESPONSE_STATE;
			
				/* Set frame status to begin */
				a_pFSM->FrameStatus = MMSN_FrameBegin;
			}
			else
			{
				// No action needed. Simply ignore.
				
				/* Update previous state */
				a_pFSM->PreviousState = MMSN_WAIT_FOR_RESPONSE_STATE;
			}
		}
		else
		{
			// Undefined function pointer to handle this event.
			g_NetworkErrorDesc.currError = NE_ReceiverFSM_UndefinedFuncPtr;
		
			// Change state to Error
			a_pFSM->CurrentState = MMSN_ERROR_STATE;
			/* Update previous state */
			a_pFSM->PreviousState = MMSN_WAIT_FOR_RESPONSE_STATE;
		
			// Add software event to queue. This will trigger immediate FSM run to handle error state.
			ADD_EVENT_TO_QUEUE(a_pFSM->ptrEventQueueDesc, MMSN_ERROR_EVENT);
		}
	};
	
	return MMSNP_OK;
};

/**
 * \brief Generic handler for No Response Timeout event
 *
 * \param a_pFSM Pointer to Network FSM.
 * \param a_u8Event Event.
 * \param a_pEventArg Pointer to event argument.
 */
uint8_t mmsn_xxx_NoResponseTimeoutEvent_Handler(MMSN_FSM_t * a_pFSM, uint8_t a_u8Event, void * a_pEventArg)
{
	ATOMIC_BLOCK(ATOMIC_FORCEON)
	{
		/* Stop no response timer by setting clock source to OFF state */
		xmega_tc_select_clock_source(&TIMER_NO_RESPONSE, TC_CLKSEL_OFF_gc);
	
		/* Report no response timeout error */
		g_NetworkErrorDesc.currError = NE_NoResponseTimeout;
	
		/* Change state to Retransmit */
		a_pFSM->CurrentState = MMSN_RETRANSMIT_STATE;
	
		// Add software event to queue. This will trigger immediate FSM run to handle event in the next state.
		ADD_EVENT_TO_QUEUE(a_pFSM->ptrEventQueueDesc, MMSN_RETRANSMISSION_EVENT);
	};
	
	return MMSNP_OK;
};

uint8_t mmsn_ReceiveResponse_DataReceivedEvent_Handler(MMSN_FSM_t * a_pFSM, uint8_t a_u8Event, void * a_pEventArg)
{
	ATOMIC_BLOCK(ATOMIC_FORCEON)
	{
		// Restart collision avoidance timer
		_restartCollisionAvoidanceTimer();
	
		// Set bus to busy state
		a_pFSM->u8LineState = MMSNP_BUSY_BUS;
	
		// Obtain received data byte from event argument
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
						// Report begin of frame error. All cleanup will be done in Retransmit state.
						g_NetworkErrorDesc.currError = NE_Frame_Malfunction_STX;
					
						/* Change state to Retransmit */
						a_pFSM->CurrentState = MMSN_RETRANSMIT_STATE;
						/* Update previous state */
						a_pFSM->PreviousState = MMSN_RECEIVE_RESPONSE_STATE;
					
						/* Add software event to queue. This will trigger immediate FSM run to handle error state. */
						ADD_EVENT_TO_QUEUE(a_pFSM->ptrEventQueueDesc, MMSN_RETRANSMISSION_EVENT);
					}
					else
					{
						// Set frame status to begin
						a_pFSM->FrameStatus = MMSN_FrameBegin;
					
						// Do not change the state and wait for incoming data
						/* Update previous state */
						a_pFSM->PreviousState = MMSN_RECEIVE_RESPONSE_STATE;
					}
					break;
				
				case FSMR_FRAME_END:
					/* Check if frame end was already received.
					 * If true then it means frame malfunction - |DLE|ETX| ... |DLE|ETX|
					 */
					if( MMSN_FrameEnd == a_pFSM->FrameStatus)
					{
						// Report end of frame error. All cleanup will be done in Retransmit state.
						g_NetworkErrorDesc.currError = NE_Frame_Malfunction_ETX;
					
						// Change state to Retransmission
						a_pFSM->CurrentState = MMSN_RETRANSMIT_STATE;
						/* Update previous state */
						a_pFSM->PreviousState = MMSN_RECEIVE_RESPONSE_STATE;
					
						// Add software event to queue. This will trigger immediate FSM run to handle retransmission.
						ADD_EVENT_TO_QUEUE(a_pFSM->ptrEventQueueDesc, MMSN_RETRANSMISSION_EVENT);
					}
					else
					{
						// Set frame status to end
						a_pFSM->FrameStatus = MMSN_FrameEnd;
					
						// We are ready to process complete response data frame
						a_pFSM->CurrentState = MMSN_PROCESS_RESPONSE_STATE;
						/* Update previous state */
						a_pFSM->PreviousState = MMSN_RECEIVE_RESPONSE_STATE;
					
						// Add software event to queue. This will trigger immediate FSM run to handle response frame processing.
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
					
						// Change state to Retransmission
						a_pFSM->CurrentState = MMSN_RETRANSMIT_STATE;
						/* Update previous state */
						a_pFSM->PreviousState = MMSN_RECEIVE_RESPONSE_STATE;
					
						// Add software event to queue. This will trigger immediate FSM run to handle retransmission.
						ADD_EVENT_TO_QUEUE(a_pFSM->ptrEventQueueDesc, MMSN_RETRANSMISSION_EVENT);
					}
					else
					{
						// Change frame status to collect
						a_pFSM->FrameStatus = MMSN_FrameCollect;
					}
					break;
				
				default:
					// Unhanded return code. Report an fatal error.
					g_NetworkErrorDesc.currError = NE_ReceiverFSM_UnknownState;
				
					// Change state to Error
					a_pFSM->CurrentState = MMSN_ERROR_STATE;
					/* Update previous state */
					a_pFSM->PreviousState = MMSN_RECEIVE_RESPONSE_STATE;
				
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
			/* Update previous state */
			a_pFSM->PreviousState = MMSN_RECEIVE_RESPONSE_STATE;
		
			// Add software event to queue. This will trigger immediate FSM run to handle error state.
			ADD_EVENT_TO_QUEUE(a_pFSM->ptrEventQueueDesc, MMSN_ERROR_EVENT);
		}
	};
	
	return MMSNP_OK;
};

uint8_t mmsn_Retransmit_RetransmissionEvent_Handler(MMSN_FSM_t * a_pFSM, uint8_t a_u8Event, void * a_pEventArg)
{
	ATOMIC_BLOCK(ATOMIC_FORCEON)
	{
		/* Log current Network Error which causes retransmission event */
		add_commNetworkError(&g_NetworkErrorDesc);
	
		// Check if retransmission procedure should be applied
		if (a_pFSM->u8RetriesCount >= MMSNP_MAX_RETRIES)
		{
			/* Maximum retries number reached */
			g_NetworkErrorDesc.currError = NE_MaximumRetries;
		
			/* Change state to Error */
			a_pFSM->CurrentState = MMSN_ERROR_STATE;
			/* Update previous state */
			a_pFSM->PreviousState = MMSN_RETRANSMIT_STATE;
		
			/* Add software event to queue. This will trigger immediate FSM run to handle error state. */
			ADD_EVENT_TO_QUEUE(a_pFSM->ptrEventQueueDesc, MMSN_ERROR_EVENT);
		} 
		else
		{
			// Try again to send data frame
		
			// Reset all data on receiver side
			_ClearRxResources(a_pFSM);
		
			// Reset all data on transmission side
			_resetSendDataAttributes(a_pFSM);
	
			// Clear network error descriptor
			g_NetworkErrorDesc.currError = NE_None;
		
			/* Another try to send the data frame */
			a_pFSM->u8RetriesCount++;
		
			// Obtain data size
			a_pFSM->SendDataAttr.u8DataSize = _sendData_FrameBuffer_Read_DataSize(&g_SendDataFrame);
		
			/* Obtain information if response is needed */
			a_pFSM->SendDataAttr.u8IsResponseNeeded = _sendData_FrameBuffer_Read_ResponseNeed(&g_SendDataFrame);

			/* Check if the line is free (Collision Avoidance).
			 * If the line is busy then wait for a free bus
			 * - stay in the same state
			 *
			 * If the line is free:
			 * - send the first byte
			 * - go to SEND state, the rest will be handled by DRE interrupt
			 */
			if (MMSNP_BUSY_BUS == a_pFSM->u8LineState)
			{
				// Indicate that there is data waiting to be sent.
				a_pFSM->u8IsDataToSend = true;
		
				/* Exit and wait for Collision Avoidance timer timeout.
				 * The rest will be handled there.
				 */
			}
			else
			{
				/* Data is pending. Send first byte and go to sending state.
				 * Remaining data will be handled in \ref MMSN_DATA_REG_EMPTY_EVENT event handler.
				 */
		
				/* Turn off receiving (automatically if RS-485 driver and receiver pins are tight together)
				 * and set RS-485 transceiver to transmit mode
				 */
				RS485_DRIVER_ENABLE();
		
				// Nevertheless turn off USART RXC interrupt during sending the data
				xmega_set_usart_rx_interrupt_level(&USART_COMMUNICATION_BUS, USART_RXCINTLVL_OFF_gc);
		
				// Turn on TXC and DRE interrupts
				// xmega_set_usart_tx_interrupt_level(&USART_COMMUNICATION_BUS, USART_TXCINTLVL_HI_gc);
				xmega_set_usart_dre_interrupt_level(&USART_COMMUNICATION_BUS, USART_DREINTLVL_HI_gc);
		
				// Reset data transmit counter
				a_pFSM->SendDataAttr.u8DataCounter = 0;
		
				// Send first data byte. The remaining bytes will be sent in DRE event handler directly.
				USART_COMMUNICATION_BUS.DATA = a_pFSM->SendDataAttr.pu8DataBuffer[a_pFSM->SendDataAttr.u8DataCounter];
			
				// Change state to \ref MMSN_SEND_STATE
				a_pFSM->CurrentState = MMSN_SEND_STATE;
				/* Update previous state */
				a_pFSM->PreviousState = MMSN_RETRANSMIT_STATE;
		
				// Clear flag indicating data waiting to be sent
				a_pFSM->u8IsDataToSend = false;
			};
		}
	};
	
	return MMSNP_OK;
}

uint8_t mmsn_Retransmit_CollisionAvoidanceTimeoutEvent_Handler(MMSN_FSM_t * a_pFSM, uint8_t a_u8Event, void * a_pEventArg)
{
	/* Event handler has the same functionality as IDLE state CollisionAvoidance timeout event handler */
	return mmsn_Idle_CollisionAvoidanceTimeoutEvent_Handler(a_pFSM, a_u8Event, a_pEventArg);
};

uint8_t mmsn_ProcessResponse_FrameProcessEvent_Handler(MMSN_FSM_t * a_pFSM, uint8_t a_u8Event, void * a_pEventArg)
{
	uint16_t u16CalculatedCRC = 0x00;	//!< \brief Initial seed value for CRC-16 computation
	uint16_t u16ResponseCRC;
	uint8_t  u8ResponseSize;
	
	ATOMIC_BLOCK(ATOMIC_FORCEON)
	{
		/* Get size of received response */
		u8ResponseSize = fifo_get_used_size(&fifo_receive_buffer_desc);
	
		/* Check if expected data size was collected */
		if (MMSNP_COMM_FRAME_SIZE == u8ResponseSize)
		{
			/* Complete response data frame was received */
		
			/* Make a working copy of received data frame */
			_copyDataFrame(&fifo_receive_buffer_desc, a_pFSM->ptrRxDataFrame);
	
			/* Clear receiving FIFO */
			fifo_flush(&fifo_receive_buffer_desc);
			
			/* Calculate CRC-16 (CRC-CCITT) using XMEGA hardware CRC peripheral
			 * excluding last 2 bytes with CRC-16.
			 * Not valid for A/B revision! Use AVR LibC function.
			 */
#ifdef __HAS_XMEGA_HARDWARE_CRC
			u16CalculatedCRC = xmega_calculate_checksum_crc16(a_pFSM->ptrRxDataFrame->u8FrameBuffer, MMSN_FRAME_NOCRC_LENGTH);
#else
			for( uint8_t u8idx = 0; u8idx < MMSN_FRAME_NOCRC_LENGTH; u8idx++)
			{
				u16CalculatedCRC = _crc_xmodem_update(u16CalculatedCRC, a_pFSM->ptrRxDataFrame->u8FrameBuffer[u8idx]);
			};
#endif
			/* Retrieve CRC-16 from the response message.
			 * Note that received CRC16 value is transmitted in big endian order.
			 * AVR architecture is little endian, so received bytes should be changed.
			 */
			MMSN_BYTES_2_WORD(a_pFSM->ptrRxDataFrame->u8CRC16LoByte, a_pFSM->ptrRxDataFrame->u8CRC16HiByte, u16ResponseCRC);

			/* Check received data integrity */
			if (u16CalculatedCRC == u16ResponseCRC)
			{
				/* Calculated and received CRC-16 value matched. */

				/* When Received data frame is an acknowledge type response then
				 * received data must be exact match with the data being transmitted.
				 * It is enough to check received CRC-16 value with previously 
				 * transmitted data frame CRC-16 value.
				 *
				 * If values does not match then retransmission should be requested.
				 * Otherwise communication block is finished.
				 */
				
				
				/* Go to \ref MMSN_IDLE_STATE state */
				a_pFSM->CurrentState = MMSN_IDLE_STATE;
				/* Update previous state */
				a_pFSM->PreviousState = MMSN_PROCESS_RESPONSE_STATE;
				
				/* Clear Rx resources */
				_ClearRxResources(a_pFSM);
				
				/* Clear sending data attributes */
				_resetSendDataAttributes(a_pFSM);
				
				/* Reset sending parameters */
				a_pFSM->u8RetriesCount	= 0;
				a_pFSM->u8IsDataToSend  = false;
			}
			else
			{
				/* Received and calculated CRC-16 value does not match.
				 * Report CRC integrity error.
				 */
				g_NetworkErrorDesc.currError = NE_Frame_CRC;
				
				/* Go to \ref MMSN_RETRANSMIT_STATE state */
				a_pFSM->CurrentState = MMSN_RETRANSMIT_STATE;
				/* Update previous state */
				a_pFSM->PreviousState = MMSN_PROCESS_RESPONSE_STATE;
			
				/* Add software event to queue. This will trigger immediate FSM run to handle error state. */
				ADD_EVENT_TO_QUEUE(a_pFSM->ptrEventQueueDesc, MMSN_RETRANSMISSION_EVENT);
			}
		}
		else
		{
			/* Frame size does not match with expected value. Only buffer underflow should be possible.
			 * Buffer overflow condition is checked in Receive state.
			 */
			g_NetworkErrorDesc.currError = NE_RX_Buffer_Underflow;
		
			/* Go to \ref MMSN_RETRANSMIT_STATE state */
			a_pFSM->CurrentState = MMSN_RETRANSMIT_STATE;
			/* Update previous state */
			a_pFSM->PreviousState = MMSN_PROCESS_RESPONSE_STATE;
		
			/* Add software event to queue. This will trigger immediate FSM run to handle error state. */
			ADD_EVENT_TO_QUEUE(a_pFSM->ptrEventQueueDesc, MMSN_RETRANSMISSION_EVENT);
		}
	};
	
	return MMSNP_OK;
}

uint8_t mmsn_Error_ErrorEvent_Handler(MMSN_FSM_t * a_pFSM, uint8_t a_u8Event, void * a_pEventArg)
{
	/* Add current Network Error to the table */
	add_commNetworkError(&g_NetworkErrorDesc);
	
#ifdef MMSNP_DEBUG
	dprintf(NEWLINESTR "Err: %u", g_NetworkErrorDesc.currError);
#endif
	
	/* Perform suitable action */
	switch (g_NetworkErrorDesc.currError)
	{
		case NE_USART_Receiver_Error:
		case NE_Frame_CRC:
		case NE_Frame_Malfunction_STX:
		case NE_Frame_Malfunction_ETX:
		case NE_ReceiverFSM_Malfunction:
		case NE_ReceiverFSM_UnknownState:
		case NE_ReceiverFSM_UndefinedFuncPtr:		
		case NE_RX_Buffer_Overflow:
		case NE_RX_Buffer_Underflow:
			/* All errors are related to data receiving */
			_ClearRxResources(a_pFSM);
		break;
		
		case NE_MaximumRetries:
			// Clear Rx resources
			_ClearRxResources(a_pFSM);
			
			// Clear sending data attributes
			_resetSendDataAttributes(a_pFSM);
			
			// Reset sending parameters of MMSN
			a_pFSM->u8RetriesCount	= 0;
			a_pFSM->u8IsDataToSend  = false;
		
		default:
		/* Your code here */
		break;
	} 

	/* Clear network error descriptor */
	g_NetworkErrorDesc.currError = NE_None;

	/* Set next state to \ref MMSN_IDLE_STATE state */
	a_pFSM->CurrentState = MMSN_IDLE_STATE;
		
	/* Set previous state to \ref MMSN_IDLE_STATE state */
	a_pFSM->PreviousState = MMSN_ERROR_STATE;
		
	/* RS-485 physical device configuration
		* Initialize GPIO related to RS-485 interface.
		*/
	RS485_DRIVER_GPIO_INITIALIZE();
		
	/* Enable RS-485 receiver */
	RS485_RECEIVER_ENABLE();
	
	return MMSNP_OK;
};

uint8_t mmsn_ProcessData_FrameProcess_Handler(MMSN_FSM_t * a_pFSM, uint8_t a_u8Event, void * a_pEventArg)
{
	uint16_t u16CalculatedCRC = 0x00;	// Initial seed value for CRC-16 calculation
	uint16_t u16ReceivedCRC;
	uint8_t  u8FrameSize;
	
	ATOMIC_BLOCK(ATOMIC_FORCEON)
	{
		/* Get RX buffer size */
		u8FrameSize = fifo_get_used_size(&fifo_receive_buffer_desc);
		
		/* Check if expected data size was collected */
		if (MMSNP_COMM_FRAME_SIZE == u8FrameSize)
		{
			/* Complete frame was received. */
		
			/* Make a working copy of received data frame */
			_copyDataFrame(&fifo_receive_buffer_desc, a_pFSM->ptrRxDataFrame);
		
			// Clear receiving FIFO
			fifo_flush(&fifo_receive_buffer_desc);
			
			/* Calculate CRC-16 (CRC-CCITT) using XMEGA hardware CRC peripheral
			 * excluding last 2 bytes with CRC-16.
			 * Not valid for A/B revision! Use AVR LibC function.
			 */
#ifdef __HAS_XMEGA_HARDWARE_CRC			
			u16CalculatedCRC = xmega_calculate_checksum_crc16(a_pFSM->ptrRxDataFrame->u8FrameBuffer, MMSN_FRAME_NOCRC_LENGTH);
#else
			for( uint8_t u8idx = 0; u8idx < MMSN_FRAME_NOCRC_LENGTH; u8idx++)
			{
				u16CalculatedCRC = _crc_xmodem_update(u16CalculatedCRC, a_pFSM->ptrRxDataFrame->u8FrameBuffer[u8idx]);
			};
#endif
			/* Retrieve CRC-16 from the response message. */
			MMSN_BYTES_2_WORD(a_pFSM->ptrRxDataFrame->u8CRC16HiByte, a_pFSM->ptrRxDataFrame->u8CRC16LoByte, u16ReceivedCRC);
			
			// Check received data integrity
			if (u16CalculatedCRC == u16ReceivedCRC)
			{
				/* printf("\ncrc ok\n"); */
				/* Calculated and received CRC-16 value matched. Go to command execution state. */

				/* Go to \ref eSM_ExecuteCommand state */
				a_pFSM->CurrentState = MMSN_EXECUTE_COMMAND_STATE;
				/* Update previous state */
				a_pFSM->PreviousState = MMSN_PROCESS_DATA_STATE;
				
				/* Add software event to queue. This will trigger immediate FSM run to handle next event. */
				ADD_EVENT_TO_QUEUE(a_pFSM->ptrEventQueueDesc, MMSN_EXECUTE_COMMAND_EVENT);
			}
			else
			{
				/* printf("\ncrc nok\n");
				printf("\ncrc rx: %u-%u %u\n", a_pFSM->ptrRxDataFrame->u8CRC16LoByte, a_pFSM->ptrRxDataFrame->u8CRC16HiByte, u16ReceivedCRC);
				printf("\ncrc calc: %u\n", u16CalculatedCRC); */
				/* Received and calculated CRC-16 value does not match.
				 * Report CRC integrity error.
				 */
				g_NetworkErrorDesc.currError = NE_Frame_CRC;
				
				/* Go to \ref eSM_Error state */
				a_pFSM->CurrentState = MMSN_ERROR_STATE;
				/* Update previous state */
				a_pFSM->PreviousState = MMSN_PROCESS_DATA_STATE;
			
				/* Add software event to queue. This will trigger immediate FSM run to handle error state. */
				ADD_EVENT_TO_QUEUE(a_pFSM->ptrEventQueueDesc, MMSN_ERROR_EVENT);
			}
		}
		else
		{
			/* Frame size does not match with expected value. Only buffer underflow should be possible.
			 * Buffer overflow condition is checked in Receive state.
			 */
			g_NetworkErrorDesc.currError = NE_RX_Buffer_Underflow;
		
			/* Go to \ref eSM_Error state */
			a_pFSM->CurrentState = MMSN_ERROR_STATE;
			/* Update previous state */
			a_pFSM->PreviousState = MMSN_PROCESS_DATA_STATE;
		
			/* Add software event to queue. This will trigger immediate FSM run to handle error state. */
			ADD_EVENT_TO_QUEUE(a_pFSM->ptrEventQueueDesc, MMSN_ERROR_EVENT);
		};
	};

	return MMSNP_OK;
};

/* FSM was triggered by \ref MMSN_SEND_DATA_EVENT event.
 * It means that data frame is ready to be sent.
 */
uint8_t mmsn_Idle_SendDataEvent_Handler(MMSN_FSM_t *a_pFSM, uint8_t a_u8Event, void *a_pEventArg)
{
	// uint8_t u8EventArg;

	// Retrieve event arguments. Send data frame attributes in this case.
	// u8EventArg = (*(uint8_t *)a_pEventArg);

	/************************************************************************/
	/* Get attributes of the data to be sent: size and response necessity   */
	/************************************************************************/
	/* This is the first place where send request event is consumed         */
	/************************************************************************/
	
	ATOMIC_BLOCK(ATOMIC_FORCEON)
	{
		// Obtain data size in bytes
		// get_MMSN_SEND_DATA_SIZE(u8EventArg, a_pFSM->SendDataAttr.u8DataSize);
		a_pFSM->SendDataAttr.u8DataSize = _sendData_FrameBuffer_Read_DataSize(&g_SendDataFrame);
	
		// Obtain response needed indication
		// get_MMSN_SEND_RESPONSE_NEEDED(u8EventArg, a_pFSM->SendDataAttr.u8IsResponseNeeded);
		a_pFSM->SendDataAttr.u8IsResponseNeeded = _sendData_FrameBuffer_Read_ResponseNeed(&g_SendDataFrame);

		/* Valid data is already in a global sending storage. Just send it out. */
	
		/* Check if the line is free (Collision Avoidance mechanism).
		 * If the line is busy then wait for free bus
		 * - stay in the same state
		 *
		 * If the line is free:
		 * - send the first byte
		 * - go to sending state
		 */
		if (MMSNP_BUSY_BUS == a_pFSM->u8LineState)
		{
			// Indicate that there is data waiting to be sent.
			a_pFSM->u8IsDataToSend = true;
		
			/* Exit and wait for Collision Avoidance timer timeout.
			 * The rest will be handled there.
			 */
		}
		else
		{
			/* Data is pending. Send first byte and go to sending state.
			 * Remaining data will be handled in \ref MMSN_DATA_REG_EMPTY_EVENT event handler.
			 */
			
			/* Turn off receiving (automatically if RS-485 driver and receiver pins are tight together)
			 * and set RS-485 transceiver to transmit mode
			 */
			RS485_DRIVER_ENABLE();
		
			/* Nevertheless turn off USART RXC interrupt during sending the data */
			xmega_set_usart_rx_interrupt_level(&USART_COMMUNICATION_BUS, USART_RXCINTLVL_OFF_gc);
		
			/* Turn on TXC and DRE interrupts */
			// xmega_set_usart_tx_interrupt_level(&USART_COMMUNICATION_BUS, USART_TXCINTLVL_HI_gc);
			xmega_set_usart_dre_interrupt_level(&USART_COMMUNICATION_BUS, USART_DREINTLVL_HI_gc);
		
			// Reset data transmit counter
			a_pFSM->SendDataAttr.u8DataCounter = 0;
		
			// printf("\nsend_idle: %u", a_pFSM->SendDataAttr.pu8SendDataBuffer[15]);
		
			// Send first data byte. The remaining bytes will be sent in DRE event handler routine.
			USART_COMMUNICATION_BUS.DATA = a_pFSM->SendDataAttr.pu8DataBuffer[0];
				
			/* Change state to \ref MMSN_SEND_STATE */
			a_pFSM->CurrentState = MMSN_SEND_STATE;
			/* Update previous state */
			a_pFSM->PreviousState = MMSN_IDLE_STATE;
		
			// Clear flag indicating data waiting to be sent
			a_pFSM->u8IsDataToSend = false;
		}
	};
	
	return MMSNP_OK;
}

/************************************************************************/
/* FINITE STATE MACHINE HANDLERS                                        */
/************************************************************************/

/* Initialize Multi-Master Serial Network FSM */
void mmsn_Initialize(MMSN_FSM_t * a_pFSM)
{
	/* Initialize network manager FSM */
	a_pFSM->u8LineState			= MMSNP_FREE_BUS;		//! Set communication bus state to free
	a_pFSM->CurrentState		= MMSN_IDLE_STATE;		//! Set current state to \ref MMSN_IDLE_STATE
	a_pFSM->PreviousState		= MMSN_IDLE_STATE;		//! Set previous state to \ref MMSN_IDLE_STATE
	a_pFSM->ptrRxDataFrame		= &g_RxCommFrameBuffer;	//! Pointer to internal received data frame storage
	a_pFSM->u8RetriesCount		= 0;					//! Retries counter
	a_pFSM->FrameStatus			= MMSN_FrameUnknown;	//! Start with an unknown frame state
	a_pFSM->ptrEventQueueDesc	= &eventQueue_desc;		//! Store pointer to event queue
	a_pFSM->u8IsDataToSend      = false;
	
	/* Sending data attributes */
	a_pFSM->SendDataAttr.u8DataSize			= 0;								//! Transmitted data size
	a_pFSM->SendDataAttr.u8IsResponseNeeded	= false;							//! Indication that response message is needed
	a_pFSM->SendDataAttr.pu8DataBuffer		= g_SendDataFrame.u8SendDataBuffer;	//! Global storage for sending data
	a_pFSM->SendDataAttr.u8DataCounter		= 0;								//! Transmitted data counter
	
	/* Assign pointer to receiver FSM action handler table */
	a_pFSM->pFSMRActionTable = FSMReceiverActionHandlerTable;
	
	/* Initialize receiver FSM */
	fsmReceiverInitialize(&a_pFSM->ReceiverFSM);
	
	/* Initialize sending and receiving FIFOs */
	fifo_init(&fifo_receive_buffer_desc, &fifo_receive_buffer[0], MMSNP_FIFO_RECEIVE_BUFFER_SIZE);
	// fifo_init(&fifo_send_buffer_desc,	 &fifo_send_buffer[0],	  FIFO_SEND_BUFFER_SIZE);
	
	/* Initialize Network Error Descriptor object */
	// TODO: initialize table or read from EEPROM?
	init_commNetworkError(&g_NetworkErrorDesc, CommNetworkErrorTable, sizeof(CommNetworkErrorTable));
	
	/************************************************************************/
	/* HARDWARE SETUP														*/
	/************************************************************************/
	
	// Configure and initialize communication bus usart
	// xmega_usart_configure();
	
	/* RS-485 PHYSICAL DEVICE CONFIGURATION */
	// Initialize GPIO related to RS-485 interface
	// rs485_driver_gpio_initialize();
	// Initially go LOW to enable receiver and start listening
	// rs485_receiver_enable();
	
	/* USART INTERRUPTS CONFIGURATION - RECEIVING */
	// Turn on USART RXC interrupt
	// xmega_set_usart_rx_interrupt_level(&USART_COMMUNICATION_BUS, USART_RXCINTLVL_HI_gc);
	
	// Turn off TXC and DRE interrupts
	// xmega_set_usart_tx_interrupt_level(&USART_COMMUNICATION_BUS, USART_TXCINTLVL_OFF_gc);
	// xmega_set_usart_dre_interrupt_level(&USART_COMMUNICATION_BUS, USART_DREINTLVL_OFF_gc);
};

/**
 *  \brief Function to determine if Logical Network Address was assigned to the device.
 */
bool _isLogicalNetworkAddrAssigned(uint8_t *a_pu8LogicalNetworkAddr)
{
	return (!(MMSNP_DEFAULT_LOGICAL_NETWORK_ADDRESS == (*a_pu8LogicalNetworkAddr)));
};

/**
 *  \brief Function to generate random logical network address.
 */
uint8_t xmega_generate_random_logical_network_address(void)
{
	uint8_t u8RndValue;
	
	u8RndValue = xmega_generate_adc_random_value(&ADCA, ADC_REFSEL_INT1V_gc, RANDOM_ADC_PIN);
	
	return ((u8RndValue % MMSNP_MAX_DEVICES_COUNT) + 1);
};

/**
 *  \brief KISS function for byte stuffing.
 */
uint8_t doByteStuffing(uint8_t *a_pDstBuf, uint8_t a_DstBufLen, const uint8_t *a_pSrcBuf, uint8_t a_SrcBufLen)
{
	const uint8_t *endOfSrcBufPtr = a_pSrcBuf + a_SrcBufLen;	//! Set pointer to end of source buffer
	uint8_t *endOfDstBufPtr		  = a_pDstBuf + a_DstBufLen;	//! Set pointer to end of destination buffer
	uint8_t *writingPtr			  = a_pDstBuf;					//! Set writing pointer to the beginning of destination buffer
	uint8_t u8SrcByte;											//! Source data byte
	
	// Check if there is anything to do
	if (a_SrcBufLen == 0)
	{
		return 0;
	};
	
	// Walk through all bytes from source buffer
	for(;;)
	{
		// Check if destination buffer still has enough space to store two data bytes (with stuffing)
		if ((writingPtr + 1) > endOfDstBufPtr)
		{
			// Finish stuffing here
			break;
		};
		
		// Get another data byte
		u8SrcByte = *a_pSrcBuf++;
		
		// Check another byte for DLE value
		if (FSMR_DLE_BYTE_VALUE == u8SrcByte)
		{
			// DLE data byte found in source buffer
			// Do the stuff
			*writingPtr++ = FSMR_DLE_BYTE_VALUE;
			*writingPtr++ = FSMR_DLE_BYTE_VALUE;
			
			// Check for completeness
			if (a_pSrcBuf >= endOfSrcBufPtr)
			{
				break;
			}
		}
		else
		{
			// Copy source data
			*writingPtr++ = u8SrcByte;
			
			// Check for completeness
			if (a_pSrcBuf >= endOfSrcBufPtr)
			{
				break;
			}
		}
	}; // for(;;)
	
	/* Return length of the buffer */
	return (writingPtr - a_pDstBuf);
};

/**
 *  \brief KISS function to compose complete sending data frame.
 */
uint8_t _composeSendDataFrame(const mmsn_receive_data_frame_t *a_pSrcBuf, mmsnp_send_data_frame_t *a_pDstBuf, bool a_IsResponseNeeded)
{
	uint8_t u8MessageSize = 0;
	
	// Clear destination buffer
	memset(a_pDstBuf, 0, sizeof(mmsnp_send_data_frame_t));
	
	// Add Start of Transmission special characters: (DLE|STX)
	a_pDstBuf->u8SendDataBuffer[0] = FSMR_DLE_BYTE_VALUE;
	a_pDstBuf->u8SendDataBuffer[1] = FSMR_STX_BYTE_VALUE;
	
	// Do byte stuffing on source data starting after two leading special characters
	u8MessageSize = doByteStuffing((a_pDstBuf->u8SendDataBuffer + 2), MMSN_COMM_SEND_DATA_SIZE, a_pSrcBuf->u8FrameBuffer, MMSNP_COMM_FRAME_SIZE);
	
	// Increase sending message size after byte stuffing by two. Due to leading characters.
	u8MessageSize += 2;
	
	// Add End of Transmission special characters: (DLE|ETX)
	a_pDstBuf->u8SendDataBuffer[u8MessageSize++] = FSMR_DLE_BYTE_VALUE;
	a_pDstBuf->u8SendDataBuffer[u8MessageSize] = FSMR_ETX_BYTE_VALUE;
	
	// Increase sending message size by one to get correct length value and not array index
	u8MessageSize++;
	
	// Add data size and response needed indication
	a_pDstBuf->u8DataSize		  = u8MessageSize;
	a_pDstBuf->u8IsResponseNeeded = a_IsResponseNeeded;
	
	return u8MessageSize;
};