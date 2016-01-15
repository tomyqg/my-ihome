# Introduction #

Add your content here.

# CRC calculation hardware engine #

## CRC-16 calculation using ASF ##
```
// CRC-16 checksum
uint16_t g_u32crc16_checksum;

// Calculate CRC-16 checksum for the data using ASF
g_u32crc16_checksum = crc_io_checksum((void *)&xmegaSerialNumber.u8DataArray[0], 11, CRC_16BIT);
printf("SNum crc-16: 0x%04x\n", g_u32crc16_checksum);
```

## CRC-16 calculation using AVR-GCC library ##
xmodem version

```
uint16_t crc = 0;
    for( uint8_t u8idx = 0; u8idx < 11; u8idx++)
	crc = _crc_xmodem_update(crc, xmegaSerialNumber.u8DataArray[u8idx]);

    printf("crc-16 xmodem: 0x%04x\n", crc); */
```

## CRC-16 calculation (CRC-CCITT) using XMEGA hardware CRC peripheral ##

```
// CRC-16(CRC-CCITT) Polynomial x^16 + x^12 + x^5 + 1 --> 0x1021
uint16_t xmega_calculate_checksum_crc16(uint8_t *a_pData, uint8_t count)
{
	uint8_t i = 0;
	uint16_t crc16_checksum = 0;
	
	// Reset all 4 checksum register to 0x00
	CRC_CTRL |= CRC_RESET_RESET0_gc;
	
	// The CRC registers will be reset one peripheral clock cycle after the RESET[1] bit is set
	_NOP();
	
	// Configure xmega CRC engine to CRC-16 computation
	CRC_CTRL &= ~CRC_CRC32_bm;
	
	// Set IO as a CRC source
	CRC_CTRL = (CRC_CTRL & (~CRC_SOURCE_gm)) | CRC_SOURCE_IO_gc;
	
	for(i = 0; i < count; i++)
	{
		CRC_DATAIN = a_pData[i];		
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
}

// Calculate CRC-16 (CRC-CCITT) using XMEGA hardware CRC peripheral
g_u16crc16_checksum = xmega_calculate_checksum_crc16(&xmegaSerialNumber.u8DataArray[0], 11);

printf("crc-16: 0x%04x\n", g_u16crc16_checksum);
```

# Generating random value #

## Using Analog-to-Digital peripheral ##

```
	/************************************************************************/
	/* ADC                                                                  */
	/************************************************************************/
	ADCA.CTRLA = ADC_ENABLE_bm;	// enable ADC
	ADCA.CTRLB = ADC_RESOLUTION_12BIT_gc;	// 12bit right adjusted
	ADCA.REFCTRL = ADC_REFSEL_VCC_gc; //ADC_REFSEL_INT1V_gc;	// Internal Voltage reference 1.0V
	ADCA.PRESCALER = ADC_PRESCALER_DIV64_gc;	// 8MHz/64 = 125kHz
	ADCA.CALL = nvm_production_signature_row_byte(offsetof(NVM_PROD_SIGNATURES_t, ADCACAL0) );
	ADCA.CALH = nvm_production_signature_row_byte(offsetof(NVM_PROD_SIGNATURES_t, ADCACAL1) );
	ADCA.CH0.CTRL = ADC_CH_INPUTMODE_SINGLEENDED_gc;	// single-ended input mode
	ADCA.CH0.MUXCTRL = ADC_CH_MUXPOS_PIN5_gc;	// input pin 5
	
	// Start conversion
	ADCA.CH0.CTRL |= ADC_CH_START_bm;
	// Wait for conversion to finish
	while(ADCA.CH0.INTFLAGS != ADC_CH_CHIF_bm);
	// Get the conversion result
	uint16_t u16ConvResult = ADCA.CH0RES;
	
	printf("ADC conversion result = %i\n", u16ConvResult);
```