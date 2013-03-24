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
	/* FSMR_WAIT_FOR_FLAG - waiting for STX or ETX flag meaning start of the frame */
	{ fsmr_WaitForFlag_EventHandler },

	/* FSMR_DATA_IN - processing incoming data within the frame */
	{ fsmr_DataIn_EventHandler },
	
	/* FSMR_DLE_IN - processing ESC data byte within the frame */
	{ fsmr_DLEIn_EventHandler }
};

/* Function initializes Receiver FSM */
void fsmReceiverInitialize(FSMReceiver_t *a_pFSM)
{
	// Set default state which waits for STX/ETX flag byte
	a_pFSM->u8State = FSMR_WAIT_FOR_FLAG;
};

/* Function handles \ref DATA_IN event at \ref FSMR_WAIT_FOR_FLAG state */
uint8_t fsmr_WaitForFlag_EventHandler(FSMReceiver_t * a_pFSM, uint8_t a_u8Event, void * a_pEventArg)
{
	if (FSMR_STX_ETX_BYTE_VALUE == (*(uint8_t *)a_pEventArg))
	{
		// STX or ETX data byte spotted
		
		// Go to DATA_IN state
		a_pFSM->u8State = FSMR_DATA_IN;
		
		// Ignore this byte
		return FSMR_IGNORE_BYTE;
	}
	else
	{
		// Do not change the state and ignore any other byte
		return FSMR_IGNORE_BYTE;
	}
};

/* Function handles \ref DATA_IN event at \ref FSMR_DATA_IN state */
uint8_t fsmr_DataIn_EventHandler(FSMReceiver_t * a_pFSM, uint8_t a_u8Event, void * a_pEventArg)
{
	if (FSMR_STX_ETX_BYTE_VALUE == (*(uint8_t *)a_pEventArg))
	{
		// ETX byte indicating end of frame
		
		// Go to \ref FSMR_WAIT_FOR_FLAG state
		a_pFSM->u8State = FSMR_WAIT_FOR_FLAG;
		
		// Indicate ETX byte
		return FSMR_ETX_BYTE;
	}
	else if (FSMR_DLE_BYTE_VALUE == (*(uint8_t *)a_pEventArg))
	{
		// DLE byte - escape data byte
		
		// Go to \ref FSMR_DLE_IN state
		a_pFSM->u8State = FSMR_DLE_IN;
		
		// Ignore byte
		return FSMR_IGNORE_BYTE;
	}
	else
	{
		// Any other data byte should be collected
		return FSMR_COLLECT_BYTE;
	}
};

/**
 * \brief Function handles \ref DATA_IN event at \ref FSMR_DLE_IN state.
 *
 * This function is invoked right after DLE byte was received in input stream.
 * The FSM always transits to \ref FSMR_DATA_IN state.
 * Event argument is XOR-ed with 0x20 to get the original value - un-stuff process.
 *
 * Note that event argument value is modified after function return.
 *
 * \param a_pFSM		pointer to the structure storing Receiver FSM.
 * \param a_u8Event		event value.
 * \param a_pEventArg	pointer to the event argument.
 *
 * \retval FSMR_COLLECT_BYTE.
 */
uint8_t fsmr_DLEIn_EventHandler(FSMReceiver_t * a_pFSM, uint8_t a_u8Event, void * a_pEventArg)
{
	// Go to \ref FSMR_DATA_IN state
	a_pFSM->u8State = FSMR_DATA_IN;
	
	// XOR incoming byte to obtain correct value
	(*(uint8_t *)a_pEventArg) ^= FSMR_RFC1662_XOR_VALUE;
	
	// Any data byte followed by DLE should be collected
	return FSMR_COLLECT_BYTE;
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