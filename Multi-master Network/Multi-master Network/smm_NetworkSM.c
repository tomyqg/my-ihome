/*
Serial Multi-Master Network State Machine
*/

#include <avr/io.h>
#include "fifo/fifo.h"
#include "smm_NetworkSM.h"
#include "board_config.h"
#include "tc_driver.h"

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

/* Current state of StateMachine */
volatile uint8_t u8sm_CurrentState;
/* Previous state of StateMachine */
volatile uint8_t u8sm_PreviousState;

uint8_t gNetworkError;

/* Bitmasked flags that describe what event has occurred */
extern volatile uint16_t gSystemEvents;

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

void fsm_InitializeStateMachine(void)
{
	/* Set current state to \ref eSM_Initialize */
	u8sm_CurrentState = eSM_Initialize;
	/* Set previous state to current state */
	u8sm_PreviousState = u8sm_CurrentState;
	
	/* Initialize sending and receiving FIFOs */
	fifo_init(&fifo_receive_buffer_desc, &fifo_receive_buffer[0], FIFO_RECEIVE_BUFFER_SIZE);
	fifo_init(&fifo_send_buffer_desc,	 &fifo_send_buffer[0],	  FIFO_SEND_BUFFER_SIZE);
	
	/* TODO: reset hardware  - hardware cleanup */
	
	
}

/* IDLE state handler */
void fsm_Idle(void)
{
	// Variable to read event flags
	uint16_t u16EventFlags;
	
	// Atomic interrupt safe read of global variable storing event flags
	u16EventFlags = gSystemEvents;
	
	// Check for receiving and sending data events. Receiving has higher priority than sending.
	// Go for receiving check first.
	if (u16EventFlags & EVENT_RECEIVED_DATA_bm)
	{
		// USART data was received and put into the queue. Change current state to ...
		// Processing will be handled in the next state.
		gNSMCurrentState = eSM_Receive;
	};
	
	// Now check sending
	if (u16EventFlags & EVENT_DATA_READY_TO_SEND_bm)
	{
		// Complete frame is ready to be sent
		// Change state to Send
		gNSMCurrentState = eSM_Send;
	};
};

void fsm_Receive(void)
{
	// We have received the data. It means that the line currently is or was in use.
	// To avoid collision start or restart busy line timer if already running.
	
	// Mark the bus line as busy
	gBusyLine = BUSY;
	
	// Force Restart of busy line timer
	xmega_tc_restart(&TIMER_BUSY_LINE);

	// If received data was error free than move to processing the data
	gNSMCurrentState = eSM_ProcessData;
	
	// If data was erroneous than discard the data, make cleanup and go to IDLE state
	gNSMCurrentState = eSM_Idle;
};

void fsm_ProcessData(void)
{
	// Check if complete frame was received.
	
	// If complete than check if frame processing is needed (is it for me?)
	
		// If so, go to command executing state
		gNSMCurrentState = eSM_ExecuteCommand;
		// Otherwise make cleanup and go back to IDLE state
		// TODO: clanup()
		gNSMCurrentState = eSM_Idle;
	
	// If frame is still incomplete than go to IDLE state
	gNSMCurrentState = eSM_Idle;
};

void fsm_ExecuteCommand(void)
{
	// Complete frame was received and should be processed by this device.
	// Callback appropriate function
	
	// Check return value and mark errors if any
	
	// Make cleanup
	
	// Return to IDLE state
	gNSMCurrentState = eSM_Idle;
};

void fsm_Send(void)
{
	// At first check is the line is busy (Collision Avoidance).
	// If the line is still busy than back-off and wait until timer times out. Go waiting for free line state.
	if (BUSY == gBusyLine)
	{
		gNSMCurrentState = eSM_WaitForResend;
		
		return;
	}
	
	/* If the line is free:
	 * 1. Turn off receiving (automatically if RS-485 driver and receiver pins are tight together)
	 * 2. Transmit entire frame by putting first byte and than using DRE interrupt
	 * 2a. Request for ACKnowledge message
	 */
	
	// Make transition to waiting for a response
	gNSMCurrentState = eSM_WaitForResponse;
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
	gNSMCurrentState = eSM_Send;
	
	// Otherwise go to Error state and report error
	gNetworkError = eNE_MaximumRetries;
	gNSMCurrentState = eSM_Error;
};

void fsm_WaitForResponse(void)
{
	// Variable to read event flags
	uint16_t u16EventFlags;
	
	// Atomic interrupt safe read of global variable storing event flags
	u16EventFlags = gSystemEvents;
	
	// If data was received in this state than collect the data. Should be acknowledge frame.
	if (u16EventFlags & EVENT_DATA_READY_TO_SEND_bm)
	{
		// We have new data
	}
	
	// Waiting for response timer timed out. Go to retransmission state.
	if (u16EventFlags & EVENT_WAIT_FOR_RESPONSE_TIMEOUT_bm)
	{
		gNSMCurrentState = eSM_Retransmission;
		
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
	
	// Go to Idle state
	gNSMCurrentState = eSM_Idle;
};