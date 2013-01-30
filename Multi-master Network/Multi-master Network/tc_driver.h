/*
 * tc_driver.h
 *
 * Created: 2013-01-30 18:08:55
 *  Author: fidectom
 */ 


#ifndef TC_DRIVER_H_
#define TC_DRIVER_H_

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