/*
 * fsm_Receiver.h
 *
 * Created: 2013-03-24 20:38:32
 *  Author: Tomasz Fidecki
 */ 

#ifndef FSM_RECEIVER_H_
#define FSM_RECEIVER_H_

#include <stdint.h>

/* Data bytes magic values */
#define FSMR_STX_BYTE_VALUE		0x7E
#define FSMR_ETX_BYTE_VALUE		0x7F
#define FSMR_DLE_BYTE_VALUE		0x7D
#define FSMR_RFC1662_XOR_VALUE	0x20

/* FSM states */
enum eFSMReceiverState
{
	FSMR_WAIT_FOR_DLE = 0,	// Waiting for Control Escape character (DLE)
	FSMR_WAIT_FOR_STX,		// Waiting for STX (start) or ETX (end) flag
	FSMR_PROCESS_DATA,		// Collecting data
	FSMR_PROCESS_DLE		// Data Link Escape
};

/* FSM Events */
enum eFSMReceiverEvent
{
	FSMR_DATA_RECEIVED = 0,
	FSMR_MAX_EVENTS	
};

/* FSM return codes */
enum eFSMReceiverReturnCode
{
	FSMR_FRAME_BEGIN	= 0,	//! Start of frame byte detected
	FSMR_FRAME_END		= 1,	//! End of frame byte detected
	FSMR_DLE_BYTE		= 2,	//! Data Link Escape byte detected
	FSMR_COLLECT_BYTE	= 3,	//! Collect Byte
	FSMR_IGNORE_BYTE	= 4,	//! Ignore byte
	
	FSMR_ERROR			= 0xFF	//! Error during processing
	
} eFSMReceiverReturnCode_t;

/* Structure holding Receiver FSM */
typedef struct FSMReceiver
{
	uint8_t	u8State;
	
} FSMReceiver_t;

/**
 * \brief Function to initialize Receiver FSM.
 *
 * This function initializes FSM structure with startup values.
 *
 * \param a_pFSM pointer to the structure storing Receiver FSM.
 *
 * \retval none.
 */
void fsmReceiverInitialize(FSMReceiver_t *a_pFSM);

/**
 * \brief Definition of Receiver FSM action handler function.
 *
 * This function runs all necessary actions related with corresponding FSM state.
 *
 * \param a_pFSM		pointer to the structure storing Receiver FSM.
 * \param a_u8Event		event value.
 * \param a_pEventArg	pointer to the event argument.
 *
 * \retval returned value encoded with eFSMReceiverReturnCode enumeration.
 * \retval FSMR_ERROR in case of processing error.
 */
typedef uint8_t (* fsmReceiverActionHandler) (FSMReceiver_t * a_pFSM, uint8_t a_u8Event, void * a_pEventArg);

/* Event function handler prototypes */
uint8_t fsmr_WaitForDLE_DataReceived_Handler (FSMReceiver_t * a_pFSM, uint8_t a_u8Event, void * a_pEventArg);
uint8_t fsmr_WaitForSTX_DataReceived_Handler (FSMReceiver_t * a_pFSM, uint8_t a_u8Event, void * a_pEventArg);
uint8_t fsmr_ProcessData_DataReceived_Handler(FSMReceiver_t * a_pFSM, uint8_t a_u8Event, void * a_pEventArg);
uint8_t fsmr_ProcessDLE_DataReceived_Handler (FSMReceiver_t * a_pFSM, uint8_t a_u8Event, void * a_pEventArg);

#endif /* FSM_RECEIVER_H_ */