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

/**
 *  \brief Function calculates CRC-16 value using XMEGA CRC peripheral.
 *         Calculation is performed on the data and data count provided as parameters.
 *
 *	\note It is CRC-16(CRC-CCITT) Polynomial x^16 + x^12 + x^5 + 1 --> 0x1021
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
 *  \return 16bit unsigned random value.
 */
uint16_t xmega_generate_sram_random_value(uint16_t a_u16Offset, uint16_t a_u16ElemCount);

/************************************************************************/
/* Bit manipulation                                                     */
/************************************************************************/

// clear a bit 
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

#endif /* UTILS_H_ */