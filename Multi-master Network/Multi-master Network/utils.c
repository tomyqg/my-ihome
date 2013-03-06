/*
 * utils.c
 *
 * Created: 2013-02-26 15:08:37
 *  Author: fidectom
 */ 

#include "utils.h"
#include "nvm_driver/nvm_driver.h"
#include <stddef.h>

// Configure XMEGA oscillator and clock source.
void xmega_set_cpu_clock_to_32MHz(void)
{
	uint8_t u8PrescalerConfig;
	uint8_t u8ClockControl;
	
	/*  Enable internal 32MHz ring oscillator. */
	OSC.CTRL |= OSC_RC32MEN_bm;
	
	/*  Wait until oscillator is ready. */
	while ((OSC.STATUS & OSC_RC32MRDY_bm) == 0);

	/*  Select Prescaler A divider as 4 and Prescaler B & C divider as (1,1) respectively.  */
	/*  Overall divide by 4 i.e. A*B*C  */
	u8PrescalerConfig = (uint8_t)(CLK_PSADIV_4_gc | CLK_PSBCDIV_1_1_gc);
	
	/* Disable register security for clock update */
	CCP = CCP_IOREG_gc;
	CLK.PSCTRL = u8PrescalerConfig;
	
	/*  Set the 32 MHz ring oscillator as the main clock source */
	u8ClockControl = ( CLK.CTRL & ~CLK_SCLKSEL_gm ) | CLK_SCLKSEL_RC32M_gc;

	/* Disable register security for clock update */
	CCP = CCP_IOREG_gc;
	CLK.CTRL = u8ClockControl;
}

// CRC-16(CRC-CCITT) Polynomial x^16 + x^12 + x^5 + 1 --> 0x1021
uint16_t xmega_calculate_checksum_crc16(uint8_t *a_pData, uint8_t a_u8Count)
{
	uint16_t crc16_checksum = 0;
	
	// Reset all 4 checksum register to 0x00
	CRC_CTRL |= CRC_RESET_RESET0_gc;
	
	// The CRC registers will be reset one peripheral clock cycle after the RESET[1] bit is set
	_NOP();
	
	// Configure XMEGA CRC peripheral to compute CRC-16 value
	CRC_CTRL &= ~CRC_CRC32_bm;
	
	// Set IO as a CRC source
	CRC_CTRL = (CRC_CTRL & (~CRC_SOURCE_gm)) | CRC_SOURCE_IO_gc;
	
	for (uint8_t u8Idx = 0; u8Idx < a_u8Count; u8Idx++)
	{
		CRC_DATAIN = a_pData[a_u8Count];
	}

	// Signal CRC complete
	CRC_STATUS |= CRC_BUSY_bm;
	
	// Check if the CRC module is busy
	while((CRC_STATUS & CRC_BUSY_bm) == CRC_BUSY_bm);
	
	crc16_checksum = ((uint16_t)CRC_CHECKSUM0 & 0x00FF);
	crc16_checksum |= (((uint16_t)CRC_CHECKSUM1 << 8) & 0xFF00);
	
	// Stop engine by disabling the source
	CRC_CTRL = (CRC_CTRL & (~CRC_SOURCE_gm)) | CRC_SOURCE_DISABLE_gc;
	
	return crc16_checksum;
};

uint16_t xmega_generate_adc_random_value(ADC_t * a_pADC, ADC_REFSEL_t a_VoltageReference, ADC_CH_MUXPOS_t a_InputMux)
{
	(a_pADC)->CTRLA			= ADC_ENABLE_bm;			// Enable ADC
	(a_pADC)->CTRLB			= ADC_RESOLUTION_12BIT_gc;	// 12bit right adjusted
	(a_pADC)->REFCTRL		= a_VoltageReference;		// Internal Voltage reference 1.0V
	(a_pADC)->PRESCALER		= ADC_PRESCALER_DIV64_gc;	// 8MHz/64 = 125kHz
	(a_pADC)->CALL			= nvm_production_signature_row_byte(offsetof(NVM_PROD_SIGNATURES_t, ADCACAL0));	// calibration byte 0
	(a_pADC)->CALH			= nvm_production_signature_row_byte(offsetof(NVM_PROD_SIGNATURES_t, ADCACAL1));	// calibration byte 1
	(a_pADC)->CH0.CTRL		= ADC_CH_INPUTMODE_SINGLEENDED_gc;	// single-ended input mode
	(a_pADC)->CH0.MUXCTRL	= a_InputMux;				// input multiplexer pin selection
	
	// Start conversion
	(a_pADC)->CH0.CTRL |= ADC_CH_START_bm;
	
	// Wait for conversion to finish
	while((a_pADC)->CH0.INTFLAGS != ADC_CH_CHIF_bm);
	
	// Return the conversion result
	return ((a_pADC)->CH0RES);
};

uint16_t xmega_generate_sram_random_value(uint16_t a_u16Offset, uint16_t a_u16ElemCount)
{
	//! Parameters not checked against boundary values.
	
	return(xmega_calculate_checksum_crc16((uint8_t *)(INTERNAL_SRAM_START + a_u16Offset), a_u16ElemCount));
};