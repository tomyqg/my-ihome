/*
 * usart_driver.h
 *
 * Created: 2013-01-24 15:18:16
 *  Author: fidectom
 */ 

#ifndef USART_DRIVER_H_
#define USART_DRIVER_H_

#include <avr/io.h>

#define xmega_set_usart_baudrate(_usart, _bselValue, _bScaleFactor)				\
(_usart)->BAUDCTRLA =(uint8_t)_bselValue;										\
(_usart)->BAUDCTRLB =(_bScaleFactor << USART_BSCALE0_bp)|(_bselValue >> 8)

// Function prototypes
void xmega_set_usart_format(USART_t *a_pUsart, USART_CHSIZE_t charSize, USART_PMODE_t parityMode, uint8_t twoStopBits);
void xmega_set_usart_mode(USART_t *a_pUsart, USART_CMODE_t usartmode);
void xmega_set_usart_rx_interrupt_level(USART_t *a_pUsart, enum USART_RXCINTLVL_enum level);
void xmega_set_usart_tx_interrupt_level(USART_t *a_pUsart, enum USART_TXCINTLVL_enum level);
void xmega_set_usart_dre_interrupt_level(USART_t *a_pUsart, enum USART_DREINTLVL_enum level);
void xmega_enable_usart_tx(USART_t *a_pUsart);
void xmega_disable_usart_tx(USART_t *a_pUsart);
void xmega_enable_usart_rx(USART_t *a_pUsart);
void xmega_disable_usart_rx(USART_t *a_pUsart);

#endif /* USART_DRIVER_H_ */