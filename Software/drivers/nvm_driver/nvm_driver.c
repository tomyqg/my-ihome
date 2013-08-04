/*
 * nvm_driver.c
 * Driver implementation for XMEGA Non Volatile Memory Controller (NVM).
 *
 * Created: 2013-01-25 09:19:02
 *  Author: fidectom
 */ 

#include <avr/io.h>
#include <avr/pgmspace.h> 
#include <stddef.h>
#include "nvm_driver.h"

/**
 * \brief Read one byte from the production signature row.
 * 
 * This function reads one byte from the production signature row of the device
 * at the given address.
 *
 * \note This function is modifying the NVM.CMD register.
 *       If the application are using program space access in interrupts
 *       (__flash pointers in IAR EW or pgm_read_byte in GCC) interrupts
 *       needs to be disabled when running EEPROM access functions. If not
 *       the program space reads will be corrupted.
 *
 * \param a_u8AddressOffset Byte offset into the signature row.
 *
 * \retval unsigned 8 bit value with the read out production signature
 */
uint8_t nvm_production_signature_row_byte(uint8_t a_u8AddressOffset)
{
	uint8_t u8Result;

	/* Load the NVM Command register to read the calibration row. */
	NVM_CMD = NVM_CMD_READ_CALIB_ROW_gc;
	u8Result = pgm_read_byte(a_u8AddressOffset);

	/* Clean up NVM Command register. */
	NVM_CMD = NVM_CMD_NO_OPERATION_gc;

	return( u8Result );
}

/**
 * \brief Read the device serial
 *
 * This function returns the device serial stored in the device.
 *
 * \note This function is modifying the NVM.CMD register.
 *       If the application are using program space access in interrupts
 *       (__flash pointers in IAR EW or pgm_read_byte in GCC) interrupts
 *       needs to be disabled when running EEPROM access functions. If not
 *       the program space reads will be corrupted.
 *
 * \retval a_pStorage Pointer to the structure where to store the device serial
 */
void nvm_read_device_serial(struct nvm_device_serial *a_pStorage)
{
	a_pStorage->lotnum0 = nvm_production_signature_row_byte(
		offsetof(NVM_PROD_SIGNATURES_t, LOTNUM0));
			
	a_pStorage->lotnum1 = nvm_production_signature_row_byte(
		offsetof(NVM_PROD_SIGNATURES_t, LOTNUM1));
	
	a_pStorage->lotnum2 = nvm_production_signature_row_byte(
		offsetof(NVM_PROD_SIGNATURES_t, LOTNUM2));
					
	a_pStorage->lotnum3 = nvm_production_signature_row_byte(
		offsetof(NVM_PROD_SIGNATURES_t, LOTNUM3));
	
	a_pStorage->lotnum4 = nvm_production_signature_row_byte(
		offsetof(NVM_PROD_SIGNATURES_t, LOTNUM4));
	
	a_pStorage->lotnum5 = nvm_production_signature_row_byte(
		offsetof(NVM_PROD_SIGNATURES_t, LOTNUM5));

	a_pStorage->wafnum  = nvm_production_signature_row_byte(
		offsetof(NVM_PROD_SIGNATURES_t, WAFNUM));

	a_pStorage->coordx0 = nvm_production_signature_row_byte(
		offsetof(NVM_PROD_SIGNATURES_t, COORDX0));
	
	a_pStorage->coordx1 = nvm_production_signature_row_byte(
		offsetof(NVM_PROD_SIGNATURES_t, COORDX1));
	
	a_pStorage->coordy0 = nvm_production_signature_row_byte(
		offsetof(NVM_PROD_SIGNATURES_t, COORDY0));
	
	a_pStorage->coordy1 = nvm_production_signature_row_byte(
		offsetof(NVM_PROD_SIGNATURES_t, COORDY1));
}
