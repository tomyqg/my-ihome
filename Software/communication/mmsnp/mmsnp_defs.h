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

#endif /* MMSNP_DEFS_H_ */