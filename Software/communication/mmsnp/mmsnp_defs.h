/*
 * mmsnp_defs.h
 *
 * Created: 2013-08-05 20:54:32
 * Author: Tomasz Fidecki
 * Email: t.fidecki@gmail.com
 */

#ifndef MMSNP_DEFS_H_
#define MMSNP_DEFS_H_

/* Actor with 8 Relay Channels */
#define  A8CH_DATA_FRAME_SIZE (8)

/* Device types */
#define A8CH_DEVICE_TYPE (0x00)

/* Maximum relay number */
#define A8CH_MAX_RELAY_INDEX (7)

#define A8CH_SET_RELAY_INDEX_DATA_BYTE	(0)
#define A8CH_SET_RELAY_STATE_DATA_BYTE	(1)
#define A8CH_SET_DELAY_TIMER_DATA_BYTE	(2)
#define A8CH_SET_UPTIME1_DATA_BYTE		(3)
#define A8CH_SET_UPTIME2_DATA_BYTE		(4)
#define A8CH_SET_UPTIME3_DATA_BYTE		(5)

/************************************************************************/
/* Touch Panel Device                                                   */
/************************************************************************/
/* Data order */
#define MMSNP_TP_BUTTON_SENSOR_STATE_INDEX_DATA_BYTE	(0x00)
#define MMSNP_TP_SCENE_SENSOR_STATE_INDEX_DATA_BYTE		(0x01)
#define MMSNP_TP_TEMPERATURE_INDEX_DATA_BYTE			(0x02)
#define MMSNP_TP_ROLLER_BLINDER_STATE_INDEX_DATA_BYTE	(0x03)
#define MMSNP_TP_RECUPERATOR_STATE_INDEX_DATA_BYTE		(0x04)

/* Bit positions */
#define MMSNP_TP_BUTTON_SENSOR_STATE_MAX_bp	(0x05)
#define MMSNP_TP_SCENE_SENSOR_STATE_MAX_bp	(0x02)

typedef enum MMSNP_A8CH_RelayStates
{
	MMSNP_A8CH_RELAY_OFF = 0,
	MMSNP_A8CH_RELAY_ON  = 1
	
} MMSNP_A8CH_RelayStates_t;

typedef enum MMSNP_TOUCH_PANEL_SENSOR_STATE
{
	MMSNP_TP_SENSOR_OFF = 0,
	MMSNP_TP_SENSOR_ON  = 1
} MMSNP_TOUCH_PANEL_SENSOR_STATE_t;


/* Association */

typedef enum MMSNP_EVENT_TYPE
{
	MMSNP_ET_UNASSIGNED = 0,
	MMSNP_ET_BUTTON_STATE,
	MMSNP_ET_SCENE_STATE,
	MMSNP_ET_SYSTEM_TEMPERATURE,
	MMSNP_ET_USER_TEMPERATURE,
	MMSNP_ET_RECUPERATOR_STATE,
	MMSNP_ET_ROLLER_BLINDER_STATE,
	
	MMSNP_ET_MAX
} MMSNP_EVENT_TYPE_t;

typedef enum MMSNP_ACTION_TYPE
{
	MMSNP_AT_NO_ACTION = 0,
	MMSNP_AT_RELAY_ON,
	MMSNP_AT_RELAY_OFF,
	MMSNP_AT_RELAY_ON_WITH_DELAY,
	MMSNP_AT_RELAY_OFF_WITH_DELAY,
	
	MMSNP_AT_MAX
} MMSNP_ACTION_TYPE_t;

typedef struct MMSNP_EVENT_SOURCE
{
	uint8_t u8DeviceType;
	uint8_t u8LogicalAddress;
	
} MMSNP_EVENT_SOURCE_t;

typedef uint_fast32_t MMSNP_DELAY_TIME_t;

typedef struct MMSNP_CHANNEL_DESCRIPTOR
{
	MMSNP_EVENT_TYPE_t	 event;
	MMSNP_ACTION_TYPE_t	 action;
	MMSNP_DELAY_TIME_t	 delay;
	MMSNP_EVENT_SOURCE_t source;

} MMSNP_CHANNEL_DESCRIPTOR_t;

#endif /* MMSNP_DEFS_H_ */