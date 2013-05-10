/*
Serial Multi-Master Network State Machine
*/

#ifndef SMM_NETWORKSM_H_
#define SMM_NETWORKSM_H_

#include <stdbool.h>
#include <stdlib.h>
#include "nvm_driver/nvm_driver.h"
#include "fifo/fifo.h"
#include "fsm_Receiver.h"

/** EVENT definitions */

// Add event to the queue
#define ADD_EVENT_TO_QUEUE(p_eventQDesc, _event)	\
do {	\
		ATOMIC_BLOCK(ATOMIC_FORCEON)	\
		{	\
			fifo_push_uint8_nocheck(p_eventQDesc, _event);	\
		};	\
} while(0)


//! Interrupt type events
//! Received data from serial bus
#define EVENT_IRQ_RECEIVE_COMPLETE_bm				(1 << 0)
//! Data register empty
#define EVENT_IRQ_DATA_REGISTER_EMPTY_bm			(1 << 1)
//! Frame completely transmitted
#define EVENT_IRQ_TRANSMIT_COMPLETE_bm				(1 << 2)
//! Data is ready to be sent
#define EVENT_SW_DATA_READY_TO_SEND_bm				(1 << 3)
//! Busy line timer time out
#define EVENT_IRQ_COLLISION_AVOIDANCE_TIMEOUT_bm	(1 << 4)
//! Waiting for response timer time out
#define EVENT_IRQ_WAIT_FOR_RESPONSE_TIMEOUT_bm		(1 << 5)
//! System heartbeat timer time out
#define EVENT_IRQ_HEARTBEAT_TIMEOUT_bm				(1 << 6)

//! Software events
//! Received data is error free
#define EVENT_SW_RECEIVE_DATA_NO_ERROR_bm			(1 << 7)
//! Received data is erroneous
#define EVENT_SW_RECEIVE_DATA_ERROR_bm				(1 << 8)
//! Communication frame data integrity error
#define EVENT_SW_COMM_FRAME_CRC_ERROR_bm			(1 << 9)
//! Communication frame complete
#define EVENT_SW_COMM_FRAME_COMPLETE_bm				(1 << 10)
//! Communication frame incomplete
#define EVENT_SW_COMM_FRAME_INCOMPLETE_bm			(1 << 11)
//! Communication frame was executed
#define EVENT_SW_COMM_FRAME_RETRANSMIT_bm			(1 << 12)
//! Communication frame was executed
#define EVENT_SW_MAX_RETRIES_COUNT_REACHED_bm		(1 << 13)
//! Communication frame without processing
#define EVENT_SW_UNEXPECTED_EVENT_RECEIVED_bm		(1 << 14)

//! Maximum retries count
#define MMSN_MAX_RETRIES	(5)

/* Line free/busy indicators */
enum eBusyLine
{
	MMSN_FREE_BUS = 0,
	MMSN_BUSY_BUS = 1
};

/* Device configuration status */
typedef enum eConfigStatus
{
	eLogicalAddrNotAssigned = 0,
	eLogicalAddrAssigned = 1
} eConfigStatus_t;

/* Size of Network receiving FIFO buffer */
#define FIFO_RECEIVE_BUFFER_SIZE	(16)

/* Size of Network receiving FIFO buffer */
#define FIFO_SEND_BUFFER_SIZE		(16)

/* Communication staff temporarily here */

// DATA FRAME
// [12b]	[4b]		   [8B]  [2B]
// [Address][Control Field][DATA][CRC-16]
// [Address] = [12bits] = [DeviceType, 4bits][DeviceNumber/SystemCommand, 7bits][RemoteTransmissionRequest (RTR), 1bit]
#define MMSN_COMM_FRAME_SIZE	(12)

/** Default device logical address in the network (DeviceNumber, 7bit value).
 *  Every device has default address after startup.
 */
#define MMSN_DEFAULT_LOGICAL_NETWORK_ADDRESS	(0xFF)

// Multi-Master Serial Network Destination Address offset
//#define MMSN_DST_ADDRESS_OFFSET	(0)
// Multi-Master Serial Network Source Address offset
//#define MMSN_SRC_ADDRESS_OFFSET	(2)
// Multi-Master Serial Network data start offset
//#define MMSN_DATA_OFFSET		(4)
// Multi-Master Serial Network CRC-16 value offset
//#define MMSN_CRC16_OFFSET		(12)

// Multi-Master Serial Network data length
#define MMSN_DATA_LENGTH		(8)
// CRC-16 data length
#define MMSN_CRC_LENGTH			(2)
// Multi-Master Serial Network frame without CRC-16 value length
#define MMSN_FRAME_NOCRC_LENGTH	(MMSN_COMM_FRAME_SIZE - MMSN_CRC_LENGTH)

/**
 * \brief Structure containing the Multi-Master Serial Network Communication Data Frame
 *
 * This structure can be used to store the communication frame.
 */
struct mmsn_comm_data_frame {
	union {
		struct {
			uint16_t u16Identifier;
			uint8_t	 u8DataBuffer[MMSN_DATA_LENGTH];
			uint16_t u16CRC16;
		};
		uint8_t u8FrameBuffer[MMSN_COMM_FRAME_SIZE];
	};
};

typedef struct mmsn_comm_data_frame mmsn_comm_data_frame_t;

/* All the magic need for frame processing macros */
#define MMSN_ADDRESS_bm	0xFFF0	/* Multi-Master Serial Network Address bit mask */
#define MMSN_ADDRESS_bp	4		/* Multi-Master Serial Network Address bit position */
#define MMSN_DEVTYPE_bm	0xF000	/* Multi-Master Serial Network Device Type bit mask */
#define MMSN_DEVTYPE_bp	12		/* Multi-Master Serial Network Device Type bit position */
#define MMSN_DEVNUM_bm  0x0FE0	/* Multi-Master Serial Network Device Number bit mask */
#define MMSN_DEVNUM_bp  5		/* Multi-Master Serial Network Device Number bit position */
#define MMSN_RTR_bm		0x0010	/* Multi-Master Serial Network Remote Transmission Request bit mask */
#define MMSN_RTR_bp		4		/* Multi-Master Serial Network Remote Transmission Request bit position */
#define MMSN_CTRLF_bm	0x000F	/* Multi-Master Serial Network Control Field bit mask */
#define MMSN_CTRLF_bp	0		/* Multi-Master Serial Network Control Field bit position */

/* Macros to compose and decode frame */
#define get_MMSN_Address(_u16Identifier, _u16Address)	\
	_u16Address = ((_u16Identifier & MMSN_ADDRESS_bm) >> MMSN_ADDRESS_bp)

#define set_MMSN_Address(_u16Address, _u16Identifier)	\
	_u16Identifier = (_u16Identifier & (~MMSN_ADDRESS_bm)) | (_u16Address << MMSN_ADDRESS_bp)

#define get_MMSN_DeviceType(_u16Identifier, _u8DeviceType)	\
	_u8DeviceType = ((_u16Identifier & MMSN_DEVTYPE_bm) >> MMSN_DEVTYPE_bp)
	
#define set_MMSN_DeviceType(_u8DeviceType, _u16Identifier)	\
	_u16Identifier = (_u16Identifier & (~MMSN_DEVTYPE_bm)) | (_u8DeviceType << MMSN_DEVTYPE_bp)
	
#define get_MMSN_DeviceNumber(_u16Identifier, _u8DeviceNum)	\
	_u8DeviceNum = ((_u16Identifier & MMSN_DEVNUM_bm) >> MMSN_DEVNUM_bp)

#define set_MMSN_DeviceNumber(_u8DeviceNum, _u16Identifier)	\
	_u16Identifier = (_u16Identifier & (~MMSN_DEVNUM_bm)) | ((_u8DeviceNum & 0x7F) << MMSN_DEVNUM_bp)
	
#define get_MMSN_RTR(_u16Identifier, _u8RTR)	\
	_u8RTR = ((_u16Identifier & MMSN_RTR_bm) >> MMSN_RTR_bp)

#define set_MMSN_RTR(_u8RTR, _u16Identifier)	\
	_u16Identifier = (_u16Identifier & (~MMSN_RTR_bm)) | (_u8RTR << MMSN_RTR_bp)

#define get_MMSN_CTRLF(_u16Identifier, _u8CtrlF)	\
	_u8CtrlF = ((_u16Identifier & MMSN_CTRLF_bm) >> MMSN_CTRLF_bp)

#define set_MMSN_CTRLF(_u8CtrlF, _u16Identifier)	\
	_u16Identifier = (_u16Identifier & (~MMSN_CTRLF_bm)) | (_u8CtrlF << MMSN_CTRLF_bp)

#define MMSN_BYTES_2_WORD(_InByte1, _InByte2, _OutWord)	\
do {	\
	_OutWord = (((_InByte1 << 8) & 0xFF00) | (_InByte2));	\
} while (0);

// Global argument macros

// Get data size waiting to be sent. Encoded with 7 lower bits.
#define get_MMSN_SEND_DATA_SIZE(_InByte, _OutByte)	\
do {	\
	_OutByte = (_InByte & 0x7F);	\
} while (0);

// Set data size waiting to be sent. Encoded with 7 lower bits.
#define set_MMSN_SEND_DATA_SIZE(_InByte, _OutByte)	\
do {	\
	_OutByte = ((_OutByte & (~0x7F)) | (_InByte & 0x7F));	\
} while (0);

// Get data size waiting to be sent. Encoded with 7 lower bits.
#define get_MMSN_SEND_RESPONSE_NEEDED(_InByte, _OutByte)	\
do {	\
	_OutByte = ((_InByte & 0x80) >> 7);	\
} while (0);

// Set data size waiting to be sent. Encoded with 7 lower bits.
#define set_MMSN_SEND_RESPONSE_NEEDED(_InByte, _OutByte)	\
do {	\
	_OutByte = ((_OutByte & (~0x80)) | ((_InByte & 0x80) << 7));	\
} while (0);

// Device Types
#define	MMSN_ConfigurationUnit	(0x00)
#define	MMSN_SupervisorUnit		(0x01)
#define	MMSN_ButtonUnit			(0x02)
#define	MMSN_RelayUnit			(0x03)
#define	MMSN_InfraRedUnit		(0x04)
#define	MMSN_WiFiUnit			(0x05)
#define	MMSN_TemperatureUnit	(0x06)
#define	MMSN_DimmerUnit			(0x07)
#define	MMSN_BlindDriverUnit	(0x08)
#define	MMSN_InputUnit			(0x09)
#define	MMSN_InterfaceUnit		(0x0A)
#define	MMSN_Reserved_1			(0x0B)
#define MMSN_Reserved_2			(0x0C)
#define	MMSN_Reserved_3			(0x0D)
#define	MMSN_Reserved_4			(0x0E)
#define	MMSN_Reserved_5			(0x0F)
#define	MMSN_Reserved_6			(0x10)

enum eRemoteTransmissionRequest
{
	eRTR_DataFrame = 0,
	eRTR_RemoteFrame = 1
};

// Device types definitions of Multi-Master Serial Network
/*
enum eMMSN_DeviceType
{
	eMMSN_ConfigurationUnit = 0x00,
	eMMSN_SupervisorUnit	= 0x01,
	eMMSN_ButtonUnit		= 0x02,
	eMMSN_RelayUnit			= 0x03,
	eMMSN_InfraRedUnit		= 0x04,
	eMMSN_WiFiUnit			= 0x05,
	eMMSN_TemperatureUnit	= 0x06,
	eMMSN_DimmerUnit		= 0x07,
	eMMSN_BlindDriverUnit	= 0x08,
	eMMSN_InputUnit			= 0x09,
	eMMSN_InterfaceUnit		= 0x0A,
	eMMSN_Reserved_1		= 0x0B,
	eMMSN_Reserved_2		= 0x0C,
	eMMSN_Reserved_3		= 0x0D,
	eMMSN_Reserved_4		= 0x0E,
	eMMSN_Reserved_5		= 0x0F,
	eMMSN_Reserved_6		= 0x10,
}; */

// Processing routines prototypes
void processCommand_Status(void);

// Establish table of pointers to processing functions
// void (* processingFunctions[])(void) = { processCommand_Status };

typedef enum eMMSN_FrameStatus
{
	MMSN_FrameUnknown = 0,
	MMSN_FrameBegin,
	MMSN_FrameEnd,
	MMSN_FrameCollect
	
} eMMSN_FrameStatus_t;

typedef enum eSM_State
{
	eSM_Idle = 0,
	eSM_Receive,
	eSM_ProcessData,
	eSM_ExecuteCommand,
	eSM_Send,
	eSM_WaitForResend,
	eSM_Retransmission,
	eSM_WaitForResponse,
	eSM_Error
} eSM_StateType;

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


/* Multi-Master Serial Network FSM states */
enum eMMSN_FSMState
{
	MMSN_IDLE_STATE = 0,					// 0
	MMSN_RECEIVE_STATE,						// 1
	MMSN_PROCESS_DATA_STATE,				// 2
	MMSN_EXECUTE_COMMAND_STATE,				// 3
	MMSN_SEND_STATE,						// 4
	MMSN_WAIT_FOR_RESPONSE_STATE,			// 5
	MMSN_RECEIVE_RESPONSE_STATE,			// 6
	MMSN_PROCESS_RESPONSE_STATE,			// 7
	MMSN_RETRANSMIT_STATE,
	MMSN_ERROR_STATE,
	MMSN_MAX_STATES
};

/* FSM Events */
enum eMMSN_FSMEvent
{
	MMSN_DATA_RECEIVED_EVENT = 0,				// 0
	MMSN_COLLISION_AVOIDANCE_TIMEOUT,			// 1
	MMSN_ERROR_EVENT,							// 2
	MMSN_FRAME_PROCESS_EVENT,					// 3
	MMSN_EXECUTE_COMMAND_EVENT,					// 4
	MMSN_SEND_DATA_EVENT,						// 5
	MMSN_DATA_REG_EMPTY_EVENT,					// 6
	MMSN_NO_RESPONSE_TIMEOUT_EVENT,				// 7
	MMSN_RETRANSMISSION_EVENT,					// 8
	
	MMSN_MAX_EVENTS
};

/* MMSN FSM return codes */
enum eMMSN_FSMReturnCode
{
	MMSN_OK = 0,
	MMSN_ERROR = 0xFF	//! Error during processing
	
} eMMSN_FSMReturnCode_t;

typedef struct MMSN_SEND_ATTRIBUTES 
{
	uint8_t u8DataSize;
	bool	u8IsResponseNeeded;
} mmsn_send_attributes_t;

typedef struct MMSN_FSM
{
	uint8_t						u8LineState;		//! Flag to indicate that the line is busy
	eSM_StateType				CurrentState;
	eSM_StateType				PreviousState;
	mmsn_comm_data_frame_t	    *ptrRxDataFrame;	//! Pointer to structure holding frame being received
	mmsn_comm_data_frame_t		*ptrTxDataFrame;	//! Pointer to structure holding frame to be transmitted
	uint8_t						u8TxDataCounter;	//! Transmitted data counter
	uint8_t						u8RetriesCount;		//! Retransmission counter
	eMMSN_FrameStatus_t			FrameStatus;		//! Data Frame status
	uint8_t						u8IsDataToSend;		//! Indicate that data is waiting to be sent
	mmsn_send_attributes_t		SendDataAttr;		//! Attributes of the data to be sent
	fifo_desc_t					*ptrEventQueueDesc;	//! Pointer to communication event queue
	
	FSMReceiver_t				ReceiverFSM;		//! Receiver FSM
	fsmReceiverActionHandler	(* pFSMRActionTable)[FSMR_MAX_EVENTS];	//! Pointer to Receiver FSM action function pointer array
	
} MMSN_FSM_t;

// Helper macro to obtain pointer to action handler function
#define GET_EV_HDL_P(fsmActionTablePtr, fsm, event) \
(* (fsmActionTablePtr + fsm.u8State))[event]

// Helper macro to call action handler function
#define CALL_EV_HDL(fsmActionTablePtr, fsm, event, arg)	\
(* (fsmActionTablePtr + fsm.u8State))[event](&fsm, event, &arg)


void mmsn_InitializeStateMachine(MMSN_FSM_t * a_pFSM);

/**
 * \brief Definition of Multi-Master Serial Network FSM event handler function.
 *
 * This function handles all necessary events related with corresponding FSM state.
 *
 * \param a_pFSM		pointer to the structure holding MMSN FSM.
 * \param a_u8Event		event value.
 * \param a_pEventArg	pointer to the event argument.
 *
 * \retval returned value encoded with eMMSN_FSMReturnCode enumeration.
 * \retval MMSN_ERROR in case of processing error.
 */
typedef uint8_t (* mmsnFsmEventHandler) (MMSN_FSM_t * a_pFSM, uint8_t a_u8Event, void * a_pEventArg);

uint8_t mmsn_Idle_DataReceived_Handler(MMSN_FSM_t * a_pFSM, uint8_t a_u8Event, void * a_pEventArg);
uint8_t mmsn_Idle_CollisionAvoidanceTimeout_Handler(MMSN_FSM_t * a_pFSM, uint8_t a_u8Event, void * a_pEventArg);
uint8_t mmsn_xxx_CollisionAvoidanceTimeout_Handler(MMSN_FSM_t * a_pFSM, uint8_t a_u8Event, void * a_pEventArg);
uint8_t mmsn_Receive_DataReceived_Handler(MMSN_FSM_t * a_pFSM, uint8_t a_u8Event, void * a_pEventArg);
uint8_t mmsn_ProcessData_FrameProcess_Handler(MMSN_FSM_t * a_pFSM, uint8_t a_u8Event, void * a_pEventArg);
uint8_t	mmsn_ExecuteCommand_ExecuteCommandEvent_Handler(MMSN_FSM_t * a_pFSM, uint8_t a_u8Event, void * a_pEventArg);
uint8_t mmsn_Idle_SendDataEvent_Handler(MMSN_FSM_t * a_pFSM, uint8_t a_u8Event, void * a_pEventArg);
uint8_t mmsn_Send_DataRegEmptyEvent_Handler(MMSN_FSM_t * a_pFSM, uint8_t a_u8Event, void * a_pEventArg);
uint8_t mmsn_WaitForResponse_DataReceivedEvent_Handler(MMSN_FSM_t * a_pFSM, uint8_t a_u8Event, void * a_pEventArg);
uint8_t mmsn_xxx_NoResponseTimeoutEvent_Handler(MMSN_FSM_t * a_pFSM, uint8_t a_u8Event, void * a_pEventArg);
uint8_t mmsn_ReceiveResponse_DataReceivedEvent_Handler(MMSN_FSM_t * a_pFSM, uint8_t a_u8Event, void * a_pEventArg);
uint8_t mmsn_Retransmit_RetransmissionEvent_Handler(MMSN_FSM_t * a_pFSM, uint8_t a_u8Event, void * a_pEventArg);

uint8_t mmsn_Error_ErrorEvent_Handler(MMSN_FSM_t * a_pFSM, uint8_t a_u8Event, void * a_pEventArg);

/************************************************************************/
/* COMMUNICATION                                                        */
/************************************************************************/
// 0x00 (0) First Command Number
#define COMMAND_NUMBER_FIRST	(0)
// 0X7F (127) Last Command Number
#define COMMAND_NUMBER_LAST		(0x7F)
// Commands count
#define COMMAND_COUNT			(0X80)

// 0x65 (101) First System Command Number
#define SYSTEM_COMMAND_NUMBER_FIRST 0x65
// 0x7F (127) Last System Command Number
#define SYSTEM_COMMAND_NUMBER_LAST 0x7F
// (127 - 101) = 26
#define SYSTEM_COMMAND_COUNT (26)

#define SYSCMD_GROUP_RESTART_REQ		(0x65)
#define SYSCMD_MODULE_RESTART_REQ		(0x66)
#define SYSCMD_GROUP_SERIAL_NUMBER_REQ	(0x67)
#define SYSCMD_MODULE_SERIAL_NUMBER_REQ	(0x68)

// Command function handler
typedef void (* funcCommandHandler)(void);

typedef struct CommandDescriptor
{
	uint8_t			   u8SysCmdNumber;	// System Command Number
	funcCommandHandler ptrCmdHandler;	// Pointer to function handler
} CommandDescriptor_t;


/**
 * \brief Obtain pointer to the Command function handler.
 * 
 * This function gets the pointer to the command processing function.
 *
 * \param a_u8CommandNumber Command Number.
 *
 * \retval Pointer to the function handler.
 * \retval NULL if command number does not match.
 */
funcCommandHandler get_CommandFunctionHandler(uint8_t a_u8CommandNumber);

/************************************************************************/
/* NETWORK ERROR HANDLING                                               */
/************************************************************************/

// Size of network error storage table. Power of 2 for easier size management.
#define NETWORK_ERROR_TABLE_SIZE 64
// Create a mask to speed up the table management (index swapping).
// mask = (2 * size) - 1 => mask = 63 (0x3F)
#define NETWORK_ERROR_TABLE_SIZE_MASK 0x3F

/* Network Error types */
typedef enum eNetworkError
{
	NE_None = 0,						// 0
	NE_MaximumRetries,					// 1
	NE_USART_Receiver_Error,			// 2
	NE_Frame_CRC,						// 3
	NE_Frame_Malfunction_STX,			// 4
	NE_Frame_Malfunction_ETX,			// 5
	NE_ReceiverFSM_Malfunction,			// 6
	NE_ReceiverFSM_UnknownState,		// 7
	NE_ReceiverFSM_UndefinedFuncPtr,	// 8
	NE_RX_Buffer_Overflow,				// 9
	NE_RX_Buffer_Underflow,				// 10
	NE_Unexpected_Event,				// 11
	NE_NoResponseTimeout,				// 12
	
	eNE_MAX
} eNetworkError_t;

typedef struct CommNetworkErrorDesc
{
	uint16_t		u16ErrorNumber;
	eNetworkError_t	eErrorReported;
	
} CommNetworkErrorDesc_t;

typedef struct NetworkErrorDesc
{
	uint8_t			u8Iterator;			// Buffer iterator
	uint16_t		u16ErrorCounter;	// Consecutive error number
	eNetworkError_t	currError;			// Currently reported error
	
} NetworkErrorDesc_t;

void			init_commNetworkError(NetworkErrorDesc_t * a_pNetworkErrorDesc, CommNetworkErrorDesc_t * a_pCommNetErrTbl, uint8_t a_u8TblSize);
void			add_commNetworkError(NetworkErrorDesc_t * a_pNetworkErrorDesc);
eNetworkError_t get_commNetworkError(uint16_t a_u16ErrorNumber);

/************************************************************************/
/* Helper functions                                                     */
/************************************************************************/

/**
 * \brief Determine if Logical Network Address was assigned to the device.
 *
 * This function checks if logical network address was already assigned and stored
 * in the EEPROM. 
 *
 * \param a_pu8LogicalNetworkAddr pointer to the variable holding logical address.
 *
 * \retval true if logical address was already assigned.
 * \retval false if logical address was not assigned.
 */
bool _isLogicalNetworkAddrAssigned(uint8_t *a_pu8LogicalNetworkAddr);

/**
 *  \brief Function generates random logical network address.
 *		   Function utilizes rand() function to compute a pseudo-random integer
 *		   in the range of 1 to 127.
 *
 *  \param none.
 *
 *  \return unsigned 8bit random value within range <1..127>
 */
uint8_t xmega_generate_random_logical_network_address(void);

#endif /* SMM_NETWORKSM_H_ */