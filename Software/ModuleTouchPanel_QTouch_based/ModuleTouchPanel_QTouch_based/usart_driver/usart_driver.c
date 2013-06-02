/*
 * usart_driver.c
 *
 * Created: 2013-01-24 15:27:36
 *  Author: fidectom
 */ 

#include "usart_driver.h"

/**
 * \brief  Configure the USART frame format.
 *
 *  Sets the frame format, Frame Size, parity mode and number of stop bits.
 *
 *  \param a_pUsart Pointer to the USART module
 *  \param charSize The character size. Use USART_CHSIZE_t type.
 *  \param parityMode The parity Mode. Use USART_PMODE_t type.
 *  \param twoStopBits Enable two stop bit mode. Use bool type.
 */
void xmega_set_usart_format(USART_t *a_pUsart, USART_CHSIZE_t charSize, USART_PMODE_t parityMode, uint8_t twoStopBits)
{
	(a_pUsart)->CTRLC = (uint8_t)charSize | parityMode | (twoStopBits ? USART_SBMODE_bm : 0);
};

/**
 * \brief Set the mode the USART run in.
 *
 * Set the mode the USART run in. The default mode is asynchronous mode.
 *
 * \param a_pUsart Pointer to the USART module register section.
 * \param usartmode Selects the USART mode. Use USART_CMODE_t type.
 *
 * USART modes:
 * - 0x0        : Asynchronous mode.
 * - 0x1        : Synchronous mode.
 * - 0x2        : IrDA mode.
 * - 0x3        : Master SPI mode.
 */
void xmega_set_usart_mode(USART_t *a_pUsart, USART_CMODE_t usartmode)
{
	(a_pUsart)->CTRLC = ((a_pUsart)->CTRLC & (~USART_CMODE_gm)) | usartmode;
};

/**
 * \brief Set USART RXD interrupt level.
 *
 * Sets the interrupt level on RX Complete interrupt.
 *
 * \param a_pUsart Pointer to the USART module.
 * \param level Interrupt level of the RXD interrupt.
 */
void xmega_set_usart_rx_interrupt_level(USART_t *a_pUsart, enum USART_RXCINTLVL_enum level)
{
	(a_pUsart)->CTRLA = ((a_pUsart)->CTRLA & ~USART_RXCINTLVL_gm) |	(level << USART_RXCINTLVL_gp);
};

/**
 * \brief Set USART TXD interrupt level.
 *
 * Sets the interrupt level on TX Complete interrupt.
 *
 * \param a_pUsart Pointer to the USART module.
 * \param level Interrupt level of the TXD interrupt.
 */
void xmega_set_usart_tx_interrupt_level(USART_t *a_pUsart, enum USART_TXCINTLVL_enum level)
{
	(a_pUsart)->CTRLA = ((a_pUsart)->CTRLA & ~USART_TXCINTLVL_gm) | (level << USART_TXCINTLVL_gp);	
};

/**
 * \brief Set USART DRE interrupt level.
 *
 * Sets the interrupt level on Data Register interrupt.
 *
 * \param a_pUsart	Pointer to the USART module.
 * \param level		Interrupt level of the DRE interrupt.
 *					Use USART_DREINTLVL_t type.
 */
void xmega_set_usart_dre_interrupt_level(USART_t *a_pUsart, enum USART_DREINTLVL_enum level)
{
	(a_pUsart)->CTRLA = ((a_pUsart)->CTRLA & ~USART_DREINTLVL_gm) |	(level << USART_DREINTLVL_gp);
};

/**
 * \brief Enable USART transmitter.
 *
 * \param a_pUsart Pointer to the USART module.
 */
void xmega_enable_usart_tx(USART_t *a_pUsart)
{
	(a_pUsart)->CTRLB |= USART_TXEN_bm;
};

/**
 * \brief Disable USART transmitter.
 *
 * \param a_pUsart Pointer to the USART module.
 */
void xmega_disable_usart_tx(USART_t *a_pUsart)
{
	(a_pUsart)->CTRLB &= ~USART_TXEN_bm;
};

/**
 * \brief Enable USART receiver.
 *
 * \param a_pUsart Pointer to the USART module
 */
void xmega_enable_usart_rx(USART_t *a_pUsart)
{
	(a_pUsart)->CTRLB |= USART_RXEN_bm;
};

/**
 * \brief Disable USART receiver.
 *
 * \param a_pUsart Pointer to the USART module.
 */ 
void xmega_disable_usart_rx(USART_t *a_pUsart)
{
	(a_pUsart)->CTRLB &= ~USART_RXEN_bm;
};