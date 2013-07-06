/*
 * OneWireMaster.c
 *
 * Created: 2013-06-22 15:13:30
 *  Author: Tomasz Fidecki (t.fidecki@gmail.com)
 */ 

#include "OneWireMaster.h"
#include <util/delay.h>
#include <util/atomic.h>

void OneWireMaster_ctor(OneWireBusMaster_t *a_pMe, PORT_t *a_pPort, uint8_t a_u8Pin)
{
	a_pMe->portPtr = a_pPort;
	a_pMe->u8Pin   = a_u8Pin;		
}

OneWireBusStatus_t OneWireMaster_initialize(OneWireBusMaster_t *a_pMe)
{
	// Drive output high
	a_pMe->portPtr->DIRSET = a_pMe->u8Pin;
	a_pMe->portPtr->OUTSET = a_pMe->u8Pin;
	
	// Invoke reset function to detect device presence
	return OneWireMaster_reset(a_pMe);
}

OneWireBusStatus_t OneWireMaster_reset(OneWireBusMaster_t *a_pMe)
{
	OneWireBusStatus_t busState;
	
	// Drive output LOW of port dedicated to 1-Wire bus transmission
	// Master Reset Pulse
	a_pMe->portPtr->DIRSET = a_pMe->u8Pin;
	a_pMe->portPtr->OUTCLR = a_pMe->u8Pin;
	
	// Make it atomic and wait to detect device presence
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		// Wait 480us
		_delay_us(480);
		
		// Toggle pin direction and set to input
		// and let it float due to pull-up resistor
		a_pMe->portPtr->DIRTGL = a_pMe->u8Pin;
		
		// Wait 80us
		_delay_us(80);
		
		// Read pin value - 1-Wire device presence pulse
		busState = (a_pMe->portPtr->IN & a_pMe->u8Pin);
		if (busState == 0)
		{
			busState = OneWireBus_DEVICE_PRESENT;
		}
		else
		{
			busState = OneWireBus_NO_DEVICES;
		}
	}
	
	// After up to 240us in total device should release the bus
	// which will be pulled-up to high by pull-up resistor
	_delay_us(420);
	
	// Read again pin value to detect shot circuit
	if ((a_pMe->portPtr->IN & a_pMe->u8Pin) == 0)
	{
		busState = OneWireBus_SHORT_CIRCUIT;
	}
	else
	{
		// Do nothing here, status already set
	}

	return busState;
}

inline void OneWireBusMaster_writeBit(OneWireBusMaster_t *a_pMe, uint8_t a_bit)
{
	if (a_bit)
	{
		cli();
		// Pull the bus low
		a_pMe->portPtr->DIRSET = a_pMe->u8Pin;
		a_pMe->portPtr->OUTCLR = a_pMe->u8Pin;
		_delay_us(6);
		
		// Release the bus, the bus will be pulled high by pull-up resistor
		a_pMe->portPtr->DIRTGL = a_pMe->u8Pin;
		sei();
		
		_delay_us(64);
	}
	else
	{
		cli();
		// Pull the bus low
		a_pMe->portPtr->DIRSET = a_pMe->u8Pin;
		a_pMe->portPtr->OUTCLR = a_pMe->u8Pin;
		_delay_us(60);
		
		// Release the bus, the bus will be pulled high by pull-up resistor
		a_pMe->portPtr->DIRTGL = a_pMe->u8Pin;
		sei();
		
		_delay_us(10);
	}
}

inline uint8_t OneWireBusMaster_readBit(OneWireBusMaster_t *a_pMe)
{
	uint8_t readBit;
	
	cli();
	
	// Pull the bus low
	a_pMe->portPtr->DIRSET = a_pMe->u8Pin;
	a_pMe->portPtr->OUTCLR = a_pMe->u8Pin;
	_delay_us(6);
	
	// Release the bus, the bus will be pulled high by pull-up resistor
	a_pMe->portPtr->DIRTGL = a_pMe->u8Pin;
	_delay_us(9);
	
	// Read the bit on the bus
	readBit = a_pMe->portPtr->IN & a_pMe->u8Pin;
	sei();
	
	_delay_us(55);
	
	return readBit;
}

uint8_t OneWireBusMaster_readByte(OneWireBusMaster_t *a_pMe)
{
	uint8_t bits = 8;
	uint8_t readByte = 0;
	
	while (bits--)
	{
		readByte >>= 1;
		if (OneWireBusMaster_readBit(a_pMe))
		{
			readByte |= 0x80;
		}
	}
	
	return readByte;
}

void OneWireBusMaster_writeByte(OneWireBusMaster_t *a_pMe, uint8_t a_byte)
{
	uint8_t bits = 8;
	
	while (bits--)
	{
		OneWireBusMaster_writeBit(a_pMe, a_byte & 0x01);
		a_byte >>= 1;
	}
}

uint8_t OneWire_crc8(uint8_t a_u8InData, uint8_t a_u8Seed)
{
	uint8_t bits = 8;
	uint8_t u8Value;

	do
	{
		u8Value = ((a_u8Seed ^ a_u8InData) & 0x01);
		if (u8Value == 0)
		{
			a_u8Seed >>= 1;
		} 
		else
		{
			a_u8Seed ^= 0x18;	// CRC8 polynomial 0x18 = X^8 + X^5 + X^4 + X^0
			a_u8Seed >>= 1;
			a_u8Seed |= 0x80;
		}
		
		a_u8InData >>= 1;
		bits--;
	} while (bits);
	
	return a_u8Seed;
}

uint8_t OneWireBusMaster_checkROMcrc(OneWireBusMaster_t *a_pMe, uint8_t *a_pROMcode)
{
	uint8_t u8Idx;
	uint8_t u8crc8 = 0;	// CRC8 initial value

	// Iterate through first 7 bytes of 64-bit ROM code
	for (u8Idx = 0; u8Idx < 7; u8Idx++)
	{
		u8crc8 = OneWire_crc8(*a_pROMcode, u8crc8);
		a_pROMcode++;
	}

	// Check calculated ROM code CRC against received one (last byte)
	if (u8crc8 == *a_pROMcode)
	{
		return OneWireBus_CRC8_OK;
	}
	else
	{
		return OneWireBus_CRC8_ERROR;
	}
}