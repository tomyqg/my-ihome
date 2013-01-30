/*
 * board_config.h
 *
 * Created: 2013-01-24 15:09:54
 *  Author: fidectom
 */ 


#ifndef BOARD_CONFIG_H_
#define BOARD_CONFIG_H_

#define TPASTE3( a, b, c)	a##b##c

/* USART dedicated to serial communication terminal */
#define USART_TERMINAL			USARTD1
#define USART_TERMINAL_PORT		PORTD
#define USART_TERMINAL_TXD_PIN	7
#define USART_TERMINAL_RXD_PIN	6

// Serial communication terminal USART configuration - [RS-232 <=> PC]
#define USART_TERMINAL_CHAR_LENGTH	USART_CHSIZE_8BIT_gc
#define USART_TERMINAL_PARITY		USART_PMODE_DISABLED_gc
#define USART_TERMINAL_STOP_BIT		0

/** \brief Timer used to measure busy line */
#define TIMER_BUSY_LINE TCD0

/** Period used to measure busy line */
#define TIMER_BUSY_LINE_PERIOD 31250

/** \note Timer resolution */
#define TIMER_BUSY_LINE_RESOLUTION 31250

/** \brief Timer used to measure busy line */
#define TIMER_NO_RESPONSE TCE0

/** Period used to measure busy line */
#define TIMER_NO_RESPONSE_PERIOD 31250

/** \note Timer resolution */
#define TIMER_NO_RESPONSE_RESOLUTION 31250

/** RS-485 Transceiver */
// +----------------------------------------------+
// | RS-485 control pin |        Function		  |
// |--------------------|-------------------------|
// | LOW				| Receiver output enabled |
// |--------------------|-------------------------|
// | HIGH				| Driver output enabled   |
// +----------------------------------------------+
#define RS485_DRIVER_PORT	PORTD
#define RS485_DRIVER_PIN	0

// Configure corresponding GPIO as output - writing one to pin
#define rs485_driver_gpio_initialize()	\
PORTD.DIRSET = PIN0_bm

/**
 * \brief Enable Receiver output in RS-485 chip
 *
 * This macro enables receiver (RO) output in RS-485 transceiver device.
 */
#define rs485_receiver_enable(void)	\
PORTD.OUTCLR = PIN0_bm

/**
 * \brief Enable Driver output in RS-485 chip
 *
 * This function enables driver (DE) output in RS-485 transceiver device.
 */
#define rs485_driver_enable(void)	\
PORTD.OUTSET = PIN0_bm

#endif /* BOARD_CONFIG_H_ */