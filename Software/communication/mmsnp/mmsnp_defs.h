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

typedef enum MMSNP_A8CH_RelayStates
{
	MYHOME_A8CH_RELAY_OFF = 0,
	MYHOME_A8CH_RELAY_ON  = 1 
	
} MMSNP_A8CH_RelayStates_t;

#endif /* MMSNP_DEFS_H_ */