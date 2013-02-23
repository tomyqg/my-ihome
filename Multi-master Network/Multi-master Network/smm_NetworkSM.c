/*
Serial Multi-Master Network State Machine
*/

#include <avr/io.h>
#include <util/atomic.h>
#include <string.h>
#include <stdbool.h>
#include "fifo/fifo.h"
#include "smm_NetworkSM.h"
#include "board_config.h"
#include "tc_driver.h"
#include "usart_driver/usart_driver.h"

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

uint8_t gNetworkError;

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

// Own network address
extern uint16_t u16OwnNetworkAddress;

// Structure for storing received frame
mmsn_comm_data_frame_t gCommDataFrameReceive;
// Structure for storing frame to be transmitted
mmsn_comm_data_frame_t gCommDataFrameTransmit;

// Variable to store transmitted data counter
volatile uint8_t gDataTransmitCounter;

// Transmitted message type
eTransmitMessageType_t gTxMsgType;

/* Lookup table containing a pointer to the function to call in each state */
void (*SM_stateTable[])(void) =
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
		u8FifoStatus = fifo_pull_uint8(a_pFifoDesc, &(a_pDstDataFrame->u8DataArray[u8Idx]));
	}
	
	// Pull out CRC-16 16bit value
	u8FifoStatus = fifo_pull_uint16(a_pFifoDesc, &(a_pDstDataFrame->u16CRC16));
	
} // mmsn_Copy_Comm_Frame()

extern uint16_t xmega_calculate_checksum_crc16(uint8_t *a_pData, uint8_t count);

bool mmsn_IsCommandSupported(uint8_t a_u8Command)
{
	return true;
};


/************************************************************************/
/* FINITE STATE MACHINE HANDLERS                                        */
/************************************************************************/

/* INITIALIZE */
void fsm_InitializeStateMachine(void)
{
	/* Set current state to \ref eSM_Initialize */
	gNSM_CurrentState = eSM_Initialize;
	/* Set previous state to current state */
	gNSM_PreviousState = gNSM_CurrentState;
	
	/* Initialize sending and receiving FIFOs */
	fifo_init(&fifo_receive_buffer_desc, &fifo_receive_buffer[0], FIFO_RECEIVE_BUFFER_SIZE);
	fifo_init(&fifo_send_buffer_desc,	 &fifo_send_buffer[0],	  FIFO_SEND_BUFFER_SIZE);
	
	// Set own network address to default value
	u16OwnNetworkAddress = DEFAULT_NETWORK_ADDRESS;
	// Initialize communication data frame with zeros
	memset(&gCommDataFrameReceive.u8CommFrameArray[0], 0x00, MMSN_COMM_FRAME_SIZE);
	
	gDataTransmitCounter = 0;
	
	/* TODO: reset hardware  - hardware cleanup */
	
	// Go to IDLE state
	gNSM_CurrentState = eSM_Idle;
}

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
	if (u16EventFlags & EVENT_IRQ_BUSY_LINE_TIMEOUT_bm)
	{
		// Stop busy line timer by setting clock source to OFF state
		xmega_tc_select_clock_source(&TIMER_BUSY_LINE, TC_CLKSEL_OFF_gc);

		// Clear busy line flag. The bus is free for transmitting the data.
		fsm_ClearBusyLine();
		
		// Stay in the same state.
		
		// Clear corresponding flag 
		ATOMIC_BLOCK(ATOMIC_FORCEON)
		{
			// Atomic interrupt safe set of global variable storing event flags
			gSystemEvents &= ~EVENT_IRQ_BUSY_LINE_TIMEOUT_bm;
		}
	};

};	// fsm_Idle()

/* RECEIVE state handler */
void fsm_Receive(void)
{
	if (u16EventFlags & EVENT_IRQ_RECEIVE_COMPLETE_bm)
	{
		// The data was received. It means that the bus currently is or was in use.
		// To avoid collision (CA) start or restart busy line timer if already running.
	
		// Force Restart of busy line timer
		xmega_tc_restart(&TIMER_BUSY_LINE);

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
				gSystemEvents |= EVENT_SW_RECEIVE_DATA_NO_ERROR_bm;
			}
		} 
		else
		{
			// If data was erroneous at hardware level than discard the data.
		
			// Save error
			gNetworkError = eNE_USART_Receiver_Error;
		
			// Make transition to the ERROR state
			gNSM_CurrentState = eSM_Error;
			
			// Set error in data processing software event
			ATOMIC_BLOCK(ATOMIC_FORCEON)
			{
				gSystemEvents |= EVENT_SW_RECEIVE_DATA_ERROR_bm;
			}
		}
		
		// Always clear receive complete event flag
		ATOMIC_BLOCK(ATOMIC_FORCEON)
		{
			gSystemEvents &= ~EVENT_IRQ_RECEIVE_COMPLETE_bm;
		}
	};
	
	// Check if busy line timer timed out
	if (u16EventFlags & EVENT_IRQ_BUSY_LINE_TIMEOUT_bm)
	{
		// Stop busy line timer by setting clock source to OFF state
		xmega_tc_select_clock_source(&TIMER_BUSY_LINE, TC_CLKSEL_OFF_gc);

		// Clear busy line flag. The bus is free for transmitting the data.
		fsm_ClearBusyLine();
		
		// Stay in the same state.
		
		// Clear corresponding flag
		ATOMIC_BLOCK(ATOMIC_FORCEON)
		{
			// Atomic interrupt safe set of global variable storing event flags
			gSystemEvents &= ~EVENT_IRQ_BUSY_LINE_TIMEOUT_bm;
		}
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
			// Complete frame was received.
			
			// Make a working copy of received data frame
			mmsn_Copy_Comm_Frame(&fifo_receive_buffer_desc, &gCommDataFrameReceive);
			
			// Calculate CRC-16 (CRC-CCITT) using XMEGA hardware CRC peripheral
			// Get all data bytes without last 2 CRC bytes
			g_u16crc16_checksum = xmega_calculate_checksum_crc16(&gCommDataFrameReceive.u8CommFrameArray[0], MMSN_FRAME_NOCRC_LENGTH);
			
			// Check data integrity
			if (g_u16crc16_checksum == gCommDataFrameReceive.u16CRC16)
			{
				// CRC-16 OK continue processing
				
				// Check if command should be handled by this device
				//if(true == mmsn_IsCommandSupported((uint8_t)gCommDataFrameReceive.nDeviceNumber_SystemCommand))
				if(0)
				{
					//Go to \ref eSM_ExecuteCommand state.
					gNSM_CurrentState = eSM_ExecuteCommand;
					
					// Set communication frame OK software event
					ATOMIC_BLOCK(ATOMIC_FORCEON)
					{
						gSystemEvents |= EVENT_SW_COMM_FRAME_COMPLETE_bm;
					}
				}
				else
				{
					// Frame not for this network element. Go to \ref eSM_Idle state.
					gNSM_CurrentState = eSM_Idle;
					// Set no processing communication frame software event
					ATOMIC_BLOCK(ATOMIC_FORCEON)
					{
						gSystemEvents |= EVENT_SW_COMM_FRAME_NO_PROCESSING_bm;
					};
				}
			}
			else
			{
				// Save CRC integrity error.
				gNetworkError = eNE_Frame_CRC;
				
				// Go to \ref eSM_Error state
				gNSM_CurrentState = eSM_Error;
				
				// Set error in data integrity software event
				ATOMIC_BLOCK(ATOMIC_FORCEON)
				{
					gSystemEvents |= EVENT_SW_COMM_FRAME_CRC_ERROR_bm;
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
				gSystemEvents |= EVENT_SW_COMM_FRAME_INCOMPLETE_bm;
			};
		}
		
		// Always clear data error free flag
		ATOMIC_BLOCK(ATOMIC_FORCEON)
		{
			// Atomic interrupt safe set of global variable storing event flags
			gSystemEvents &= ~EVENT_SW_RECEIVE_DATA_NO_ERROR_bm;
		}
	};
	
	// Check if busy line timer timed out
	if (u16EventFlags & EVENT_IRQ_BUSY_LINE_TIMEOUT_bm)
	{
		// Stop busy line timer by setting clock source to OFF state
		xmega_tc_select_clock_source(&TIMER_BUSY_LINE, TC_CLKSEL_OFF_gc);

		// Clear busy line flag. The bus is free for transmitting the data.
		fsm_ClearBusyLine();
		
		// Stay in the same state.
		
		// Clear corresponding flag
		ATOMIC_BLOCK(ATOMIC_FORCEON)
		{
			// Atomic interrupt safe set of global variable storing event flags
			gSystemEvents &= ~EVENT_IRQ_BUSY_LINE_TIMEOUT_bm;
		}
	};

};	// fsm_ProcessData()

void fsm_ExecuteCommand(void)
{
	// Complete frame was received and should be processed by this device.
	
	// Check for frame complete event
	if (u16EventFlags & EVENT_SW_COMM_FRAME_COMPLETE_bm)
	{
		// Callback appropriate function
		
		// Check return value and mark errors if any
		
		// Make cleanup??
		
		// Return to IDLE state
		gNSM_CurrentState = eSM_Idle;
		
		// Clear corresponding flag
		ATOMIC_BLOCK(ATOMIC_FORCEON)
		{
			// Atomic interrupt safe set of global variable storing event flags
			gSystemEvents &= ~EVENT_SW_COMM_FRAME_COMPLETE_bm;
		}
	}
	else
	{
		// We have received unexpected event
		
		// Save unexpected event error
		gNetworkError = eNE_Unexpected_Event;
		
		// Go to \ref eSM_Error state
		gNSM_CurrentState = eSM_Error;
		
		// Set unexpected event error software event
		ATOMIC_BLOCK(ATOMIC_FORCEON)
		{
			gSystemEvents |= EVENT_SW_UNEXPECTED_EVENT_RECEIVED_bm;
		};
	}	
};

/* SEND state handler */
// The complete frame is formatted and put into transmit buffer.
void fsm_Send(void)
{
	if (u16EventFlags & EVENT_SW_DATA_READY_TO_SEND_bm)
	{
		// At first check is the line is busy (Collision Avoidance).
		// If the line is still busy than back-off and wait until timer times out. Go waiting for free line state.
		if (BUSY == gBusyLine)
		{
			gNSM_CurrentState = eSM_WaitForResend;
			// TODO:
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
			
			// Reset transmit counter
			gDataTransmitCounter = 0;
			
			// Send first data byte. The remaining bytes will be sent in DRE event handler.
			USART_COMMUNICATION_BUS.DATA = gCommDataFrameTransmit.u8CommFrameArray[0];			
			
			ATOMIC_BLOCK(ATOMIC_FORCEON)
			{
				// Mark that the first byte was sent
				gDataTransmitCounter++;
			};
		}
		
		// Clear data ready to be sent event flag
		ATOMIC_BLOCK(ATOMIC_FORCEON)
		{
			gSystemEvents &= ~EVENT_SW_DATA_READY_TO_SEND_bm;
		}
	};
	
	// Data Register Empty (DRE) IRQ handling
	if (u16EventFlags & EVENT_IRQ_DATA_REGISTER_EMPTY_bm)
	{
		if (gDataTransmitCounter < MMSN_COMM_FRAME_SIZE)
		{
			// Data still waiting to be sent
			
			// Get next data byte. The remaining bytes will be sent in DRE event handler.
			USART_COMMUNICATION_BUS.DATA = gCommDataFrameTransmit.u8CommFrameArray[gDataTransmitCounter];
			
			ATOMIC_BLOCK(ATOMIC_FORCEON)
			{
				// Another byte was sent
				gDataTransmitCounter++;
			};
			
			// Turn on DRE interrupt
			xmega_set_usart_dre_interrupt_level (&USART_COMMUNICATION_BUS, USART_DREINTLVL_MED_gc);
		}
		else
		{
			// Complete frame was sent out. Wait for response?
			
			
		}
		
		
		
		
		
		
		// Make transition to waiting for a response
		gNSM_CurrentState = eSM_WaitForResponse;
		
		// Clear data register empty event flag
		ATOMIC_BLOCK(ATOMIC_FORCEON)
		{
			gSystemEvents &= ~EVENT_IRQ_DATA_REGISTER_EMPTY_bm;
		}
	}
};

void fsm_WaitForResend(void)
{
	// TODO:
};

void fsm_Retransmission(void)
{
	// Increase retransmissions counter
	
	// If maximum retransmission value not reached than send complete frame again
	// by going to send state
	gNSM_CurrentState = eSM_Send;
	
	// Otherwise go to Error state and report error
	gNetworkError = eNE_MaximumRetries;
	gNSM_CurrentState = eSM_Error;
};

void fsm_WaitForResponse(void)
{
	// Variable to read event flags
	uint16_t u16EventFlags;
	
	// Atomic interrupt safe read of global variable storing event flags
	u16EventFlags = gSystemEvents;
	
	// If data was received in this state than collect the data. Should be acknowledge frame.
	if (u16EventFlags & EVENT_SW_DATA_READY_TO_SEND_bm)
	{
		// We have new data
	}
	
	// Waiting for response timer timed out. Go to retransmission state.
	if (u16EventFlags & EVENT_IRQ_WAIT_FOR_RESPONSE_TIMEOUT_bm)
	{
		gNSM_CurrentState = eSM_Retransmission;
		
		return;
	}
	
	// Restart waiting for response timer
	
	// Each time restart busy line timer
	
	// If busy line timer times out than set line to free - ?? in IRQ handler??
	
};

void fsm_Error(void)
{
	// Add current Network error to log
	
	// Clear error
	gNetworkError = eNE_None;
	
	// Make cleanup
	
	
	
	
	// Flush receiving FIFO
	fifo_flush(&fifo_receive_buffer_desc);
	
	// Flush transmission FIFO
	fifo_flush(&fifo_send_buffer_desc);
	
	// Go to the IDLE state
	gNSM_CurrentState = eSM_Idle;
};