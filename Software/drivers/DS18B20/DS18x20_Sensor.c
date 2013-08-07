/*
 * DS18x20_Sensor.c
 *
 * Created: 2013-06-23 21:16:45
 *  Author: Tomasz Fidecki (t.fidecki@gmail.com)
 */ 

#include "DS18x20_Sensor.h"
#include <string.h>
#include <stdbool.h>


uint8_t DS18B20_Manager_ctor(DS18B20_Manager_t *a_pMe, OneWireBusMaster_t *a_pOneWire, uint8_t a_ConversionRes)
{
	OneWireBusStatus_t oneWireBusStatus;
	uint8_t u8Idx;
	uint8_t retVal = DS18B20_OK;
	
	// Set OneWireBusMaster
	a_pMe->oneWireBusMasterPtr = a_pOneWire;
	
	// Clear the buffer for ROM code and Scratchpad memory
	memset(&(a_pMe->ROMCode), 0, ROM_CODE_SIZE_18B20);
	memset(&(a_pMe->scratchpad), 0, SCRATCHPAD_SIZE_18B20);
	
	/* Perform identification procedure for one slave only
	 * Transaction needed:
	 * 1) Initialization
	 * 2) ROM command
	 * 3) Function command
	 */
	
	// Initialize OneWire bus which ends with reset pulse functionality
	oneWireBusStatus = OneWireMaster_initialize(a_pMe->oneWireBusMasterPtr);
	
	if (OneWireBus_DEVICE_PRESENT == oneWireBusStatus)
	{
		// Device is present. Issue Read ROM command
		OneWireBusMaster_writeByte(a_pOneWire, READ_ROM_18B20);
		
		// Read the ROM code: Family code (1B) | Serial Number (6B) | CRC Byte (1B)
		for (u8Idx = 0; u8Idx < ROM_CODE_SIZE_18B20; u8Idx++)
		{
			a_pMe->ROMCode[u8Idx] = OneWireBusMaster_readByte(a_pMe->oneWireBusMasterPtr);
		}

		// Check CRC8
		if (OneWireBus_CRC8_ERROR == OneWireBusMaster_checkROMcrc(a_pMe->oneWireBusMasterPtr, a_pMe->ROMCode))
		{
			retVal = DS18B20_CRC_ERROR;
		} 
		else
		{
			// Nothing to do OK returned value already set
		}
	}
	else
	{
		// No device present or short circuit
		retVal = DS18B20_ERROR;
	}
	
	return retVal;
}

uint8_t DS18B20_Manager_startConvT(DS18B20_Manager_t *a_pMe)
{
	uint8_t retVal = DS18B20_OK;
	OneWireBusStatus_t oneWireBusStatus;
	
	oneWireBusStatus = OneWireMaster_reset(a_pMe->oneWireBusMasterPtr);
	
	if (OneWireBus_DEVICE_PRESENT == oneWireBusStatus)
	{
		// Device is present. Issue Skip ROM command to address all devices.
		OneWireBusMaster_writeByte(a_pMe->oneWireBusMasterPtr, SKIP_ROM_18B20);
		
		// Device is present. Issue Read ROM command
		OneWireBusMaster_writeByte(a_pMe->oneWireBusMasterPtr, CONVERT_T_18B20);
	}
	else
	{
		// No device present or short circuit
		retVal = DS18B20_ERROR;
	}
	
	return retVal;
}

uint8_t DS18B20_Manager_readScratchpad(DS18B20_Manager_t *a_pMe)
{
	uint8_t retVal = DS18B20_OK;
	OneWireBusStatus_t oneWireBusStatus;
	uint8_t u8crc8 = 0;	// CRC8 initial value
	uint8_t u8Idx;
	
	// Send reset pulse to sensor
	oneWireBusStatus = OneWireMaster_reset(a_pMe->oneWireBusMasterPtr);
	
	if (OneWireBus_DEVICE_PRESENT == oneWireBusStatus)
	{
		
		// Device is present. Issue Skip ROM command to address all devices.
		OneWireBusMaster_writeByte(a_pMe->oneWireBusMasterPtr, SKIP_ROM_18B20);
		
		// Issue read scratchpad function command
		OneWireBusMaster_writeByte(a_pMe->oneWireBusMasterPtr, READ_SCRATCHPAD_18B20);
		
		// Read scratchpad: 9 bytes 
		for (u8Idx = 0; u8Idx < SCRATCHPAD_SIZE_18B20; u8Idx++)
		{
			a_pMe->scratchpad[u8Idx] = OneWireBusMaster_readByte(a_pMe->oneWireBusMasterPtr);
		}

		// Calculate CRC8 for received scratchpad data
		for (u8Idx = 0; u8Idx < (SCRATCHPAD_SIZE_18B20 - 1); u8Idx++)
		{
			u8crc8 = OneWire_crc8(a_pMe->scratchpad[u8Idx], u8crc8);
		}
		
		// Check data integrity
		if (u8crc8 != a_pMe->scratchpad[SCRATCHPAD_SIZE_18B20-1])
		{
			retVal = DS18B20_CRC_ERROR;
		}
		else
		{
			// Nothing to do
		}
	}
	else
	{
		// No device present or short circuit
		retVal = DS18B20_ERROR;
	}
	
	return retVal;
}

int16_t DS18B20_Manager_convertRawToCelsius(DS18B20_Manager_t *a_pMe)
{
	uint16_t u16RawTemp;
	uint8_t u8Resolution;
	bool	bNegative = false;
	uint8_t u8IntPart;
	uint8_t u8FractPart = 0;
	int16_t i16RetVal;
	
	/* Get raw temperature data from the scratchpad memory
	 * Byte[0]: temperature LSB
	 * Byte[1]: temperature MSB
	 */
	u16RawTemp = (a_pMe->scratchpad[1] << 8) | a_pMe->scratchpad[0];
	
	// Get family code from the ROM Code and clear undefined bits if needed
	if (FAMILY_CODE_DS18S20 != a_pMe->ROMCode[0])
	{
		// Get actual resolution from scratchpad
		u8Resolution = (a_pMe->scratchpad[RESOLUTION_BYTEPOS_DS18B20] & RESOLUTION_BITMASK_DS18B20);
	
		// Clear undefined bits when resolution is not 12-bit	
		if (RESOLUTION_9BITS_DS18B20 == u8Resolution)
		{
			// 9-bit resolution, bits 2, 1, 0 are undefined
			u16RawTemp &= ~(UNDEFINED_9BITS_RES_18B20);
		} 
		else if (RESOLUTION_10BITS_DS18B20 == u8Resolution)
		{
			// 10-bit resolution, bits 1, 0 are undefined
			u16RawTemp &= ~(UNDEFINED_10BITS_RES_18B20);
		}
		else if (RESOLUTION_11BITS_DS18B20 == u8Resolution)
		{
			// 11-bit resolution, bit 0 is undefined
			u16RawTemp &= ~(UNDEFINED_11BITS_RES_18B20);
		}
		else
		{
			// 12-bit resolution all bits are relevant
		}
	}		
	
	// Check raw temperature for negative sign
	if (u16RawTemp & 0x8000)
	{
		bNegative = true;
		u16RawTemp ^= 0xFFFF;	// two's complement
		u16RawTemp += 1;
	}
	
	if (FAMILY_CODE_DS18S20 == a_pMe->ROMCode[0])
	{
		// Discard sign and divide by 0.5 to get correct value
		// Use shift right bitwise operation for performance reasons
		u8IntPart = (uint8_t)((u16RawTemp & 0xFF) >> 1);
		
		// Round up if 0.5C bit is present
		if (u16RawTemp & 0x0001)
		{
			u8IntPart++;
		}
	}
	else
	{
		// Make conversion to the following format [+/- xxx.xx]
		u8IntPart = (uint8_t)((u16RawTemp >> 4) & 0x7F);			// integer part
		u8FractPart = (uint8_t)(((u16RawTemp & 0x0F) * 625) / 100);	// fractional part
	}
	
	// Make final computations
	i16RetVal = u8IntPart;
		
	// Rounding
	if (u8FractPart > 50)
	{
		i16RetVal++;
	}
	
	// Add sign
	if (bNegative)
	{
		i16RetVal = -i16RetVal;
	}
	
	return i16RetVal;
}

uint8_t DS18B20_Manager_writeScratchpad(DS18B20_Manager_t *a_pMe, uint8_t a_Th, uint8_t a_Tl, uint8_t a_ConfReg)
{
	uint8_t retVal = DS18B20_OK;
	OneWireBusStatus_t oneWireBusStatus;
	
	oneWireBusStatus = OneWireMaster_reset(a_pMe->oneWireBusMasterPtr);
	
	if (OneWireBus_DEVICE_PRESENT == oneWireBusStatus)
	{
		// Device is present. Issue Skip ROM command to address all devices.
		OneWireBusMaster_writeByte(a_pMe->oneWireBusMasterPtr, SKIP_ROM_18B20);
		
		// Device is present. Issue Write Scratchpad function command
		OneWireBusMaster_writeByte(a_pMe->oneWireBusMasterPtr, WRITE_SCRATCHPAD_18B20);
		
		// Transmit 3 bytes: Th, Tl and Configuration Register
		OneWireBusMaster_writeByte(a_pMe->oneWireBusMasterPtr, a_Th);
		OneWireBusMaster_writeByte(a_pMe->oneWireBusMasterPtr, a_Tl);
		OneWireBusMaster_writeByte(a_pMe->oneWireBusMasterPtr, a_ConfReg);
	}
	else
	{
		// No device present or short circuit
		retVal = DS18B20_ERROR;
	}
	
	return retVal;
}