/*
 * OneWireMaster.h
 *
 * Created: 2013-06-22 14:53:20
 *  Author: teef_000
 */ 


#ifndef ONEWIREMASTER_H_
#define ONEWIREMASTER_H_

#include <avr/io.h>

typedef enum OneWireBusStatus
{
	OneWireBus_NO_DEVICES	  = 0,
	OneWireBus_DEVICE_PRESENT = 1,
	OneWireBus_SHORT_CIRCUIT  = 2,
	OneWireBus_CRC8_OK		  = 3,
	OneWireBus_CRC8_ERROR	  = 4
	
} OneWireBusStatus_t;

typedef struct OneWireBusMaster
{
	volatile PORT_t *portPtr;
	uint8_t			 u8Pin;
} OneWireBusMaster_t;

void OneWireMaster_ctor(OneWireBusMaster_t *a_pMe, PORT_t *a_pPort, uint8_t a_u8Pin);
OneWireBusStatus_t OneWireMaster_initialize(OneWireBusMaster_t *a_pMe);
OneWireBusStatus_t OneWireMaster_reset(OneWireBusMaster_t *a_pMe);
void OneWireBusMaster_writeBit(OneWireBusMaster_t *a_pMe, uint8_t a_bit);
uint8_t OneWireBusMaster_readBit(OneWireBusMaster_t *a_pMe);
uint8_t OneWireBusMaster_readByte(OneWireBusMaster_t *a_pMe);
void OneWireBusMaster_writeByte(OneWireBusMaster_t *a_pMe, uint8_t a_byte);

uint8_t OneWire_crc8(uint8_t a_u8InData, uint8_t a_u8Seed);
uint8_t OneWireBusMaster_checkROMcrc(OneWireBusMaster_t *a_pMe, uint8_t *a_pROMcode);

#endif /* ONEWIREMASTER_H_ */