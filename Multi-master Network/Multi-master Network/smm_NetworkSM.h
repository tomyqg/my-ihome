/*
Serial Multi-Master Network State Machine
*/

#ifndef SMM_NETWORKSM_H_
#define SMM_NETWORKSM_H_

/** EVENT definitions */

//! Interrupt type events
//! Received data from serial bus
#define EVENT_IRQ_RECEIVE_COMPLETE_bm			(1 << 0)
//! Data register empty
#define EVENT_IRQ_DATA_REGISTER_EMPTY_bm		(1 << 1)
//! Frame completely transmitted
#define EVENT_IRQ_TRANSMIT_COMPLETE_bm			(1 << 2)
//! Data is ready to be sent
#define EVENT_SW_DATA_READY_TO_SEND_bm			(1 << 3)
//! Busy line timer time out
#define EVENT_IRQ_BUSY_LINE_TIMEOUT_bm			(1 << 4)
//! Waiting for response timer time out
#define EVENT_IRQ_WAIT_FOR_RESPONSE_TIMEOUT_bm	(1 << 5)

//! Software events
//! Received data is error free
#define EVENT_SW_RECEIVE_DATA_NO_ERROR_bm		(1 << 6)
//! Received data is erroneous
#define EVENT_SW_RECEIVE_DATA_ERROR_bm			(1 << 7)
//! Communication frame data integrity error
#define EVENT_SW_COMM_FRAME_CRC_ERROR_bm		(1 << 8)
//! Communication frame complete
#define EVENT_SW_COMM_FRAME_COMPLETE_bm			(1 << 9)
//! Communication frame incomplete
#define EVENT_SW_COMM_FRAME_INCOMPLETE_bm		(1 << 10)
//! Communication frame without processing
#define EVENT_SW_COMM_FRAME_NO_PROCESSING_bm	(1 << 11)
//! Communication frame without processing
#define EVENT_SW_UNEXPECTED_EVENT_RECEIVED_bm	(1 << 12)


/* Error types */
typedef enum eNetworkError
{
	eNE_None = 0,
	eNE_MaximumRetries,
	eNE_USART_Receiver_Error,
	eNE_Frame_CRC,
	eNE_Unexpected_Event,
	
	eNE_MAX
} eNetworkError_Type;

/* Line free/busy indicators */
enum eBusyLine {FREE = 0, BUSY = 1};

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

// Default network address identifier. Every device has default address after startup.
#define DEFAULT_NETWORK_ADDRESS	(0xFFFF)

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
			uint16_t nDeviceType				 : 4;
			uint16_t nDeviceNumber_SystemCommand : 7;
			uint16_t nRTR						 : 1;
			uint16_t nControlField				 : 4;
			uint8_t	 u8DataArray[8];
			uint16_t u16CRC16;
		};
		uint8_t u8CommFrameArray[MMSN_COMM_FRAME_SIZE];
	};
};

typedef struct mmsn_comm_data_frame mmsn_comm_data_frame_t;

typedef enum eTransmitMessageType
{
	NORMAL = 0,
	ACK,
	NACK
} eTransmitMessageType_t;

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