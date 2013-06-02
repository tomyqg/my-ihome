/*
 * tc_driver.h
 *
 * Created: 2013-01-30 18:08:55
 *  Author: fidectom
 */ 


#ifndef TC_DRIVER_H_
#define TC_DRIVER_H_

/* The CMD[1:0] bits in CTRLFSET are used to issue special commands to the TC */
/* Force Update
The "Force update" command is used to force an UPDATE condition. Buffer registers
are copied to their destination when the  UPDATE conditions are met. Issuing this
command forces an UPDATE condition regardless of the counter value. This can be
used to update both the period and compare registers at the exact same time. In input
capture mode, the "Force update" command only has effect on the PERBUF/PER
registers. */

/* The "Force restart" command clears the CNT[H:L] registers and the direction bit to its
reset value. */

/* The "Force hard reset" command puts all registers in that TC instance back to their
reset state. For safety reasons, the TC clock selection must be set to OFF for this
command to have any effect. */

/**
 * \brief Macro to configure timer/counter clock source
 *
 * \param _tc Pointer to TC module.
 * \param _clockSelect Clock source selection of TC_CLKSEL_t enum type
 * \note Configuring the clock also starts the timer.
 *       OFF value puts timer/counter in OFF state.
 */
#define xmega_tc_select_clock_source(_tc, _clockSelect)	\
(_tc)->CTRLA = ((_tc)->CTRLA & ~(TC0_CLKSEL_gm)) | _clockSelect

/**
 * \brief Macro to force RESTART the timer/counter.
 *
 * \param _tc Pointer to TC module.
 * \note  bit 3:2 - CMD[1:0] in CTRLFSET is responsible for setting the command.
 */
#define xmega_tc_restart(_tc)	\
(_tc)->CTRLFSET = TC_CMD_RESTART_gc

/**
 * \brief Macro to force hard RESET (ignored if T/C is not in OFF state) of the timer-counter.
 *
 * \param _tc Pointer to TC module.
 * \note  bit 3:2 - CMD[1:0] in CTRLFSET is responsible for setting the command.
 */
#define xmega_tc_reset(_tc)	\
(_tc)->CTRLFSET = TC_CMD_RESET_gc

/**
 * \brief Macro to force UPDATE of the timer/counter.
 *
 * \param _tc Pointer to TC module.
 * \note  bit 3:2 - CMD[1:0] in CTRLFSET is responsible for setting the command.
 */
#define xmega_tc_update(_tc)	\
(_tc)->CTRLFSET = TC_CMD_UPDATE_gc

/* Exemplary usage

// Turn OFF timer by setting clock source to OFF state
xmega_tc_select_clock_source(&TIMER_NO_RESPONSE, TC_CLKSEL_OFF_gc);

// Turn ON timer by setting desired clock source
xmega_tc_select_clock_source(&TIMER_NO_RESPONSE, TC_CLKSEL_DIV1024_gc);
*/

#endif /* TC_DRIVER_H_ */