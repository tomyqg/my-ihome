/*
 * utils.h
 *
 * Created: 2013-01-30 17:56:03
 *  Author: fidectom
 */ 

#ifndef UTILS_H_
#define UTILS_H_

#include <avr/io.h>

/**
   \def _NOP

   Execute a <i>no operation</i> (NOP) CPU instruction.  This
   should not be used to implement delays, better use the functions
   from <util/delay_basic.h> or <util/delay.h> for this.  For
   debugging purposes, a NOP can be useful to have an instruction that
   is guaranteed to be not optimized away by the compiler, so it can
   always become a breakpoint in the debugger.
*/
#define _NOP() __asm__ __volatile__("nop":::"memory") 

/**
   \def _MemoryBarrier

   Implement a read/write <i>memory barrier</i>.  A memory
   barrier instructs the compiler to not cache any memory data in
   registers beyond the barrier.  This can sometimes be more effective
   than blocking certain optimizations by declaring some object with a
   \c volatile qualifier.

   See \ref optim_code_reorder for things to be taken into account
   with respect to compiler optimizations.
*/
#define _MemoryBarrier() __asm__ __volatile__("":::"memory")

/************************************************************************/
/* Clock Management                                                     */
/************************************************************************/

/**
* \brief Configure XMEGA oscillator and clock source.
*
* This function enables internal 32MHz oscillator and sets it as main cpu clock.
* Prescaler (A:B:C) are configured as (4:1:1) to provide 8MHz clock to peripherals.
*
* \param none.
*
* \retval none.
*/
void xmega_set_cpu_clock_to_32MHz(void);

/**
 *  \brief Function calculates CRC-16 value using XMEGA CRC peripheral.
 *         Calculation is performed on the data and data count provided as parameters.
 *
 *	\note CRC-16(CRC-CCITT) Polynomial x^16 + x^12 + x^5 + 1 --> 0x1021
 *
 *  \param a_pData		Pointer to the data.
 *  \param a_u8Count	Count of elements used for calculation.
 *
 *  \return CRC-16 value.
 */
uint16_t xmega_calculate_checksum_crc16(uint8_t *a_pData, uint8_t a_u8Count);

/**
 *  \brief Function generates random value based on the analog-to-digital conversion.
 *         ADC works in the single-ended input mode. Conversion is performed on the chosen pin.
 *		   Conversion result is a 12-bit right adjusted value.
 *
 *	\note Clock prescaler is fixed at 64. Thus results in 125kHz clock for ADC.
 *
 *  \param a_pADC				Pointer to the Analog-to-Digital Converter structure.
 *  \param a_VoltageReference	Voltage reference selection enum.
 *	\param a_InputMux			Positive input multiplexer selection enum.
 *
 *  \return Conversion result.
 */
uint16_t xmega_generate_adc_random_value(ADC_t * a_pADC, ADC_REFSEL_t a_VoltageReference, ADC_CH_MUXPOS_t a_InputMux);

/**
 *  \brief Function generates random value based on the Internal SRAM content.
 *		   Function takes count of elements and offset as arguments. Reads the SRAM content
 *		   and calculates CRC-16 value based on provided data.
 *
 *	\note Providing parameters make sure that its within range <INTERNAL_SRAM_START .. INTERNAL_SRAM_END>.
 *
 *  \param a_u16Offset		Offset from the beginning from Internal SRAM start. Useful to jump over .data section.
 *  \param a_u16ElemCount	Count of elements used for CRC-16 calculation.
 *
 *  \return unsigned 16bit random value.
 */
uint16_t xmega_generate_sram_random_value(uint16_t a_u16Offset, uint16_t a_u16ElemCount);

/**
* \brief Configure and initialize xmega USART.
*
* This function performs all necessary USART configuration to enable receiving and transmission.
* During configuration global interrupts are disabled.
*
* \note Configuration is only valid for a chosen USART, baud rate and peripheral clock.
*
* \param none.
*
* \retval none.
*/
void xmega_usart_configure(void);

/************************************************************************/
/* Bit manipulation                                                     */
/************************************************************************/

// Set a bit
#define FLAG_SET(arg1, arg2)	arg1 |= arg2;

// Clear a bit 
#define	FLAG_CLEAR(arg1, arg2)	arg1 &= ~(arg2)

// Port Direction Set
#define PORT_DIRSET(...)	port_dirset_(__VA_ARGS__)
#define port_dirset_(arg1, arg2)	arg1.DIRSET = arg2

// Port Direction Clear
#define PORT_DIRCLR(...)	port_dirclr_(__VA_ARGS__)
#define port_dirclr_(arg1, arg2)	arg1.DIRCLR = arg2

// Port Output Clear
#define PORT_OUTCLR(...)	port_outclr_(__VA_ARGS__)
#define port_outclr_(arg1, arg2)	arg1.OUTCLR = arg2

// Port Output Set
#define PORT_OUTSET(...)	port_outset_(__VA_ARGS__)
#define port_outset_(arg1, arg2)	arg1.OUTSET = arg2


// Length of Xmega short serial number in bytes
#define XMEGA_SHORT_SERIAL_NUM_LEN	  (7)
// Offset of original data array where Xmega short serial starts
#define XMEGA_SHORT_SERIAL_NUM_OFFSET (4)

/**
 * \brief Structure containing the xmega shortened serial number.
 *
 * This structure is used to store shortened (7 bytes) device serial number.
 * This would be needed when supervisor is requesting uC serial number encoded on 7 bytes.
 * Shortened serial number is comprised of lotnum4, lotnum5, ..., and coordy1 bytes.
 * These are 7 lowest bytes which are more unique for a device.
 */
typedef struct xmega_shortened_serial_number
{
	union {
		struct {
			uint8_t lotnum4;
			uint8_t lotnum5;
			uint8_t wafnum;
			uint8_t coordx0;
			uint8_t coordx1;
			uint8_t coordy0;
			uint8_t coordy1;
		};
		
		uint8_t u8DataArray[XMEGA_SHORT_SERIAL_NUM_LEN];
	};
} xmega_shortened_serial_number_t;

/**
 * \brief Get the shortened XMEGA device serial number.
 *
 * This function gets the shortened version XMEGA device serial number.
 * Shortened version of complete serial number is comprised of 7 bytes
 * excluding lotnum0 - lotnum3 bytes.
 *
 * \Note Functions arguments (pointers) must be properly provided. No checks against NULL pointers are made.
 *
 * \param a_pInCompleteSerialNum	Pointer to the structure holding complete device serial number (11 bytes).
 * \param a_pOutShortenedSerialNum	Pointer to the structure holding shortened device serial number (7 bytes).
 *
 * \retval none.
 */
// void xmega_get_shortened_serial_num(nvm_device_serial *a_pInCompleteSerialNum, xmega_shortened_serial_number_t *a_pOutShortenedSerialNum);

#endif /* UTILS_H_ */