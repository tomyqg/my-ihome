/*
 * fsm_Receiver.c
 *
 * Created: 2013-03-24 20:39:01
 *  Author: Tomasz Fidecki
 */ 

#include "fsm_Receiver.h"

/* Function event handlers pointer table */
fsmReceiverActionHandler FSMReceiverActionHandlerTable[][FSMR_MAX_EVENTS] =
{
	/* FSMR_WAIT_FOR_DLE - waiting for DLE character */
	{ fsmr_WaitForDLE_DataReceived_Handler },
	
	/* FSMR_WAIT_FOR_STX - waiting for STX or ETX flag meaning start of the frame */
	{ fsmr_WaitForSTX_DataReceived_Handler },

	/* FSMR_PROCESS_DATA - processing incoming data within the frame */
	{ fsmr_ProcessData_DataReceived_Handler },
	
	/* FSMR_PROCESS_DLE - processing Control Escape data byte within the frame */
	{ fsmr_ProcessDLE_DataReceived_Handler }
};

/* Function initializes Receiver FSM */
void fsmReceiverInitialize(FSMReceiver_t *a_pFSM)
{
	// Set default state which waits for STX/ETX flag byte
	a_pFSM->u8State = FSMR_WAIT_FOR_DLE;
};

/* Function handles \ref FSMR_DATA_RECEIVED event in \ref FSMR_WAIT_FOR_DLE state */
uint8_t fsmr_WaitForDLE_DataReceived_Handler(FSMReceiver_t * a_pFSM, uint8_t a_u8Event, void * a_pEventArg)
{
	// Ignore any data until DLE is received
	
	if (FSMR_DLE_BYTE_VALUE == (*(uint8_t *)a_pEventArg))
	{
		// Change state to one waiting for STX byte to recognize start of frame
		a_pFSM->u8State = FSMR_WAIT_FOR_STX;
	}
	
	// Always return with ignore action
	return FSMR_IGNORE_BYTE;
};

/* Function handles \ref FSMR_DATA_RECEIVED event in \ref FSMR_WAIT_FOR_STX state */
uint8_t fsmr_WaitForSTX_DataReceived_Handler(FSMReceiver_t * a_pFSM, uint8_t a_u8Event, void * a_pEventArg)
{
	if (FSMR_STX_BYTE_VALUE == (*(uint8_t *)a_pEventArg))
	{
		// STX data received (start of frame)
		
		// Go to data processing state
		a_pFSM->u8State = FSMR_PROCESS_DATA;
		
		// Indicate start of frame
		return FSMR_FRAME_BEGIN;
	}
	else
	{
		// Change the state to waiting for another DLE if received data is not STX
		// Go to data processing state
		a_pFSM->u8State = FSMR_WAIT_FOR_DLE;
		
		// Indicate that this byte should be ignored
		return FSMR_IGNORE_BYTE;
	}
};

/* Function handles \ref FSMR_DATA_RECEIVED event in \ref FSMR_PROCESS_DATA state */
uint8_t fsmr_ProcessData_DataReceived_Handler(FSMReceiver_t * a_pFSM, uint8_t a_u8Event, void * a_pEventArg)
{
	if (FSMR_DLE_BYTE_VALUE == (*(uint8_t *)a_pEventArg))
	{
		// DLE byte indicating that following data is escaped
		
		// Go to \ref FSMR_PROCESS_DLE state
		a_pFSM->u8State = FSMR_PROCESS_DLE;
		
		// Ignore this data
		return FSMR_IGNORE_BYTE;
	}
	else
	{
		// Any other data byte should be collected
		return FSMR_COLLECT_BYTE;
	}
};

/**
 * \brief Function handles \ref FSMR_DATA_RECEIVED event at \ref FSMR_PROCESS_DLE state.
 *
 * This function is invoked right after DLE byte was received in input stream.
 * Every received data should be collected unless it is ETX.
 * //Event argument is XOR-ed with 0x20 to get the original value - un-stuff process.
 *
 * //Note that event argument value is modified after function return.
 *
 * \param a_pFSM		pointer to the structure storing Receiver FSM.
 * \param a_u8Event		event value.
 * \param a_pEventArg	pointer to the event argument.
 *
 * \retval FSMR_IGNORE_BYTE if received data is ETX.
 * \retval FSMR_COLLECT_BYTE otherwise.
 */
uint8_t fsmr_ProcessDLE_DataReceived_Handler(FSMReceiver_t * a_pFSM, uint8_t a_u8Event, void * a_pEventArg)
{
	if (FSMR_ETX_BYTE_VALUE == (*(uint8_t *)a_pEventArg))
	{
		// The data frame is completed. Go to waiting for DLE|STX pair state
		a_pFSM->u8State = FSMR_WAIT_FOR_DLE;
		
		// Return with ETX indication
		return FSMR_FRAME_END;
	}
	else
	{
		// Go back to \ref FSMR_PROCESS_DATA state
		a_pFSM->u8State = FSMR_PROCESS_DATA;
	
		// XOR incoming byte to obtain correct value
		// (*(uint8_t *)a_pEventArg) ^= FSMR_RFC1662_XOR_VALUE;
	
		// Any data varying from ETX should be collected
		return FSMR_COLLECT_BYTE;
	}
};

/* 
fsmRun(FSM* thiz)
{
	int event;
	while( event = getEvent() )
	{
		void* eventArg = getEventArg();
		if(fsmActionTable[thiz->state][event])
		fsmActionTable[thiz->state][event]( thiz, event, eventArg );
	}
}
*/