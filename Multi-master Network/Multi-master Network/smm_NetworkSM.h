/*
Serial Multi-Master Network State Machine
*/

#ifndef SMM_NETWORKSM_H_
#define SMM_NETWORKSM_H_

/** EVENT definitions */

//! Received data from RS-485 bus
#define EVENT_RECEIVED_DATA_bm				(1 << 0)
//! Data is ready to be sent
#define EVENT_DATA_READY_TO_SEND_bm			(1 << 1)
//! Busy line timer time out
#define EVENT_BUSY_LINE_TIMEOUT_bm			(1 << 2)
//! Data register empty??
#define EVENT_DATA_REG_EMPTY_bm				(1 << 3)
//! Waiting for response timer time out
#define EVENT_WAIT_FOR_RESPONSE_TIMEOUT_bm	(1 << 4)

/* Error types */
typedef enum eNetworkError
{
	eNE_None = 0,
	eNE_MaximumRetries,
	
	eNE_MAX
} eNetworkError_Type;

/* Line free/busy indicators */
enum eBusyLine {FREE = 0, BUSY = 1};

/* Flag to indicate that the line is busy */
volatile uint8_t gBusyLine;

/* Size of Network receiving FIFO buffer */
#define FIFO_RECEIVE_BUFFER_SIZE	(12)

/* Size of Network receiving FIFO buffer */
#define FIFO_SEND_BUFFER_SIZE		(12)


typedef enum eSM_State
{
	eSM_Initialize = 0,
	eSM_Idle,
	eSM_Receive,
	eSM_ProcessData,
	eSM_ExecuteCommand,
	eSM_Send,
	eSM_WaitForResend,
	eSM_Retransmission,
	eSM_WaitForResponse,
	eSM_Error
} eSM_StateType;

/* Current state of Network State Machine */
eSM_StateType gNSMCurrentState;

/* Function prototypes to handle individual state */
void fsm_Idle(void);
void fsm_Receive(void);
void fsm_ProcessData(void);
void fsm_ExecuteCommand(void);
void fsm_Send(void);
void fsm_WaitForResend(void);
void fsm_Retransmission(void);
void fsm_WaitForResponse(void);
void fsm_Error(void);

#endif /* SMM_NETWORKSM_H_ */