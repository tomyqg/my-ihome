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

#define MMSN_DEBUG

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
#define USART_COMMUNICATION_BUS			USARTD0
#define USART_COMMUNICATION_BUS_PORT	PORTD
#define USART_COMMUNICATION_BUS_TXD_PIN	3
#define USART_COMMUNICATION_BUS_RXD_PIN	2
#define USART_COMMUNICATION_BUS_TX_IO	PORTD, PIN3_bm
#define USART_COMMUNICATION_BUS_RX_IO	PORTD, PIN2_bm

// Serial communication bus USART configuration - [RS-485]
#define USART_COMMUNICATION_BUS_CHAR_LENGTH	USART_CHSIZE_8BIT_gc
#define USART_COMMUNICATION_BUS_PARITY		USART_PMODE_DISABLED_gc
#define USART_COMMUNICATION_BUS_STOP_BIT	0

/* The board runs at internal 32MHz cpu clock. The peripherals at 8MHz.
 * Wakeup timeout occurs 1000 times per second (1000Hz - 1ms)
 * TargetTimerCount = (Input Frequency/Prescaler Value)/(Target Frequency) - 1
 * Prescaler = 64
 * Input frequency = 8MHz
 * Target frequency = 1000Hz (1ms)
 * TargetTimerCount = (8M/64)/(1000) - 1 = 125 - 1 = 124
 */

/** \brief Heartbeat / Initial back off timer */
#define TIMER_HEARTBEAT	TCC0

/** Period used to measure busy line */
#define TIMER_HEARTBEAT_PERIOD 124

/** \brief Timer used to measure busy line */
#define TIMER_COLLISION_AVOIDANCE TCD0

/** Collision Avoidance timer (busy line detection) is calculated based on a given logical address in the network,
 *  or random value within the range <1..127>.
 *  Formula for calculating Collision Avoidance period with already assigned Logical Address:
 *		Tca = LogicalAddress * 65 + 4
 *  Formula for calculating Collision Avoidance period without Logical Address assigned
 *		Tca = xmega_generate_random_logical_network_address() * 65 + 4
 */
/** Timer-Counter value for measuring period of 0,52[ms]
 *  This period is time needed to transmit 1Byte over serial network
 *  configured as 19200bps. 1Byte is comprised of 10 bits (start, 8b data, stop).
 *  (1/125kHZ) = 8us - 1 clock tick
 *  (520us/8us) = 65
 */
#define TIMER_COLLISION_AVOIDANCE_520us_VALUE 65

/** Timer-Counter value for measuring period of 0,032[ms]
 *  This is the additional time for Collision Avoidance.
 *  (1/125kHZ) = 8us - 1 clock tick
 *  (32us/8us) = 4
 */
#define TIMER_COLLISION_AVOIDANCE_32us_VALUE 4

/** \brief Timer used to measure busy line */
#define TIMER_NO_RESPONSE TCE0

/** Period used to measure waiting for response
 *  Timer configuration: Clock = 8MHz, Prescaler = 64
 *  1 CPU clock = 8ns (f = 125kHz)
 *  Wait for response time = 20ms
 *  Period value = 2500 (20ms/8ns)
 */
#define TIMER_NO_RESPONSE_PERIOD 2500

/************************************************************************/
/* RS-485 PHY transceiver                                               */
/************************************************************************/
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