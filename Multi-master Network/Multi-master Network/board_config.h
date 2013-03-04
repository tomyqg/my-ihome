/*
 * board_config.h
 *
 * Created: 2013-01-24 15:09:54
 *  Author: fidectom
 */ 


#ifndef BOARD_CONFIG_H_
#define BOARD_CONFIG_H_

#include "utils.h"
#include "smm_NetworkSM.h"

#define TPASTE3( a, b, c)	a##b##c

#define MY_DEVICE_TYPE	MMSN_RelayUnit

/* USART dedicated to serial communication terminal */
#define USART_TERMINAL			USARTD1
#define USART_TERMINAL_PORT		PORTD
#define USART_TERMINAL_TXD_PIN	7
#define USART_TERMINAL_RXD_PIN	6
#define USART_TERMINAL_TX_IO	PORTD, PIN7_bm
#define USART_TERMINAL_RX_IO	PORTD, PIN6_bm

// Serial communication terminal USART configuration - [RS-232 <=> PC]
#define USART_TERMINAL_CHAR_LENGTH	USART_CHSIZE_8BIT_gc
#define USART_TERMINAL_PARITY		USART_PMODE_DISABLED_gc
#define USART_TERMINAL_STOP_BIT		0

/* USART dedicated to serial communication bus */
#define USART_COMMUNICATION_BUS			USARTD1
#define USART_COMMUNICATION_BUS_PORT	PORTD
#define USART_COMMUNICATION_BUS_TXD_PIN	7
#define USART_COMMUNICATION_BUS_RXD_PIN	6
#define USART_COMMUNICATION_BUS_TX_IO	PORTD, PIN7_bm
#define USART_COMMUNICATION_BUS_RX_IO	PORTD, PIN6_bm

// Serial communication bus USART configuration - [RS-485]
#define USART_COMMUNICATION_BUS_CHAR_LENGTH	USART_CHSIZE_8BIT_gc
#define USART_COMMUNICATION_BUS_PARITY		USART_PMODE_DISABLED_gc
#define USART_COMMUNICATION_BUS_STOP_BIT	0

/* The board runs at internal 32MHz cpu clock. The peripherals at 8MHz.
 * Wakeup timeout occurs 1000 times per second (1000Hz - 1ms)
 * TargetTimerCount = (Input Frequency/Prescaler Value)/(Target Frequency) - 1
 * Prescaler = 1
 * Input frequency = 8MHz
 * Target frequency = 1000Hz (1ms)
 * TargetTimerCount = (8M/1)/(1000) - 1 = 8000-1 = 7999
 */

/** \brief Heartbeat / Initial back off timer */
#define TIMER_HEARTBEAT	TCC0

/** Period used to measure busy line */
#define TIMER_HEARTBEAT_PERIOD 7999

/** \brief Timer used to measure busy line */
#define TIMER_BUSY_LINE TCD0

/** Period used to measure busy line */
// Is calculated based on device serial number and then CRC-16
// #define TIMER_BUSY_LINE_PERIOD 31250

/** \brief Timer used to measure busy line */
#define TIMER_NO_RESPONSE TCE0

/** Period used to measure busy line */
#define TIMER_NO_RESPONSE_PERIOD 31250

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
#define RS485_CONTROL_IO	PORTD, PIN0_bm

// Configure corresponding GPIO as output - writing one to pin
#define rs485_driver_gpio_initialize()	\
PORT_DIRSET(RS485_CONTROL_IO)
// PORTD.DIRSET = PIN0_bm

/**
 * \brief Enable Receiver output in RS-485 chip
 *
 * This macro enables receiver (RO) output in RS-485 transceiver device.
 */
#define rs485_receiver_enable(void)	\
PORT_OUTCLR(RS485_CONTROL_IO)
// PORTD.OUTCLR = PIN0_bm

/**
 * \brief Enable Driver output in RS-485 chip
 *
 * This function enables driver (DE) output in RS-485 transceiver device.
 */
#define rs485_driver_enable(void)	\
PORT_OUTSET(RS485_CONTROL_IO)
// PORTD.OUTSET = PIN0_bm

#endif /* BOARD_CONFIG_H_ */