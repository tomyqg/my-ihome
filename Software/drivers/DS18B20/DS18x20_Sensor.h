/*
 * DS18x20_Sensor.h
 *
 * Created: 2013-06-23 19:56:26
 *  Author: teef_000
 */ 


#ifndef DS18X20_SENSOR_H_
#define DS18X20_SENSOR_H_

#include "OneWireMaster.h"

#define DS18B20_OK			(0)
#define DS18B20_CRC_ERROR	(1)
#define DS18B20_ERROR		(2)

#define FAMILY_CODE_DS18S20	(0x10)
#define FAMILY_CODE_DS18B20	(0x28)
#define FAMILY_CODE_DS1822	(0x22)

// ROM Commands
#define SEARCH_ROM_18B20	(0xF0)		/*!< Search ROM process to identify ROM of all slaves									*/
#define READ_ROM_18B20		(0x33)		/*!< Read the slave's ROM code - one slave on the bus only								*/
#define	MATCH_ROM_18B20		(0x55)		/*!< Address a specific slave device on a bus											*/
#define	SKIP_ROM_18B20		(0xCC)		/*!< Address all slaves on the bus simultaneously w/o sending any ROM code				*/
#define	ALARM_SEARCH_18B20	(0xEC)		/*!< Identical to Search ROM command but only slaves with a set alarm flag will respond */

// Function Command Set
#define CONVERT_T_18B20			(0x44)	/*!< Initiates temperature conversion													*/
#define READ_SCRATCHPAD_18B20	(0xBE)	/*!< Reads the entire scratchpad including the CRC byte									*/
#define WRITE_SCRATCHPAD_18B20	(0x4E)	/*!< Writes data into scratchpad bytes 2, 3 and 4 (Th, Tl and configuration registers)	*/
#define COPY_SCRATCHPAD_18B20	(0x48)	/*!< Copies Th, Tl and configuration register data from the scratchpad to EEPROM		*/
#define RECALL_EE_18B20			(0xB8)	/*!< Recalls Th, Tl and configuration register data from EEPROM to the scratchpad		*/
#define READ_POWER_SUPPLY_18B20	(0xB4)	/*!< Signals DS18B20 power supply mode to the master									*/

// DS18B20 thermometer resolution configuration
#define RESOLUTION_BYTEPOS_DS18B20	(4)					/*!< Resolution byte position in configuration register		*/
#define RESOLUTION_BITMASK_DS18B20	((1<<5) | (1<<6))	/*!< Resolution configuration register bitmask				*/
#define RESOLUTION_9BITS_DS18B20	(0x00)				/*!< Max conversion time 93.75ms (tconv/8)					*/
#define RESOLUTION_10BITS_DS18B20	(1 << 5)			/*!< Max conversion time 187.5ms (tconv/4)					*/
#define RESOLUTION_11BITS_DS18B20	(1 << 6)			/*!< Max conversion time 375ms (tconv/2)					*/
#define RESOLUTION_12BITS_DS18B20	((1<<5) | (1<<6))	/*!< Max conversion time 750ms (tconv). Power-up default	*/

// Undefined bits definitions for resolution type
#define UNDEFINED_9BITS_RES_18B20	((1<<0) | (1<<1) | (1<<2))
#define UNDEFINED_10BITS_RES_18B20	((1<<0) | (1<<1))
#define UNDEFINED_11BITS_RES_18B20	((1<<0))

#define SCRATCHPAD_SIZE_18B20	(9)
#define ROM_CODE_SIZE_18B20		(8)

// DS18B20 memory map - scratchpad
typedef uint8_t DS18B20_Scratchpad_t[SCRATCHPAD_SIZE_18B20];

// DS18B20 unique 64-bit lasered ROM code
typedef	uint8_t DS18B20_ROMCode[ROM_CODE_SIZE_18B20];


typedef struct DS18B20_Manager
{
	DS18B20_Scratchpad_t scratchpad;
	DS18B20_ROMCode ROMCode;
	OneWireBusMaster_t *oneWireBusMasterPtr;
	
} DS18B20_Manager_t;

uint8_t DS18B20_Manager_ctor(DS18B20_Manager_t *a_pMe, OneWireBusMaster_t *a_pOneWire, uint8_t a_ConversionRes);
uint8_t DS18B20_Manager_startConvT(DS18B20_Manager_t *a_pMe);
uint8_t DS18B20_Manager_readScratchpad(DS18B20_Manager_t *a_pMe);
int16_t DS18B20_Manager_convertRawToCelsius(DS18B20_Manager_t *a_pMe);
uint8_t DS18B20_Manager_writeScratchpad(DS18B20_Manager_t *a_pMe, uint8_t a_Th, uint8_t a_Tl, uint8_t a_ConfReg);

#endif /* DS18X20_SENSOR_H_ */