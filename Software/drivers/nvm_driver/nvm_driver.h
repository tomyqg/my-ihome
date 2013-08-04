/*
 * nvm_driver.h
 * Driver implementation for XMEGA Non Volatile Memory Controller (NVM).
 *
 * Created: 2013-01-25 09:16:20
 *  Author: fidectom
 */ 

#ifndef NVM_DRIVER_H_
#define NVM_DRIVER_H_

#include <stdint.h>

/**
 * \brief Structure containing the device serial
 *
 * This structure can be used to store the serial number of a device.
 */
struct nvm_device_serial {
	union {
		struct {
			uint8_t lotnum0;
			uint8_t lotnum1;
			uint8_t lotnum2;
			uint8_t lotnum3;
			uint8_t lotnum4;
			uint8_t lotnum5;
			uint8_t wafnum;
			uint8_t coordx0;
			uint8_t coordx1;
			uint8_t coordy0;
			uint8_t coordy1;
		};
		uint8_t u8DataArray[11];
	};
};

/**
 * \brief Read one byte from the production signature row.
 */
uint8_t nvm_production_signature_row_byte(uint8_t a_u8AddressOffset);

/**
 * \brief Read the XMEGA device serial
 */
void nvm_read_device_serial(struct nvm_device_serial *a_pStorage);

#endif /* NVM_DRIVER_H_ */