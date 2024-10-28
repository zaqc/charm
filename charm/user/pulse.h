/*
 * pulse.h
 *
 *  Created on: 2024 Jul 7
 *      Author: zaqc
 */

#ifndef USER_PULSE_H_
#define USER_PULSE_H_

extern volatile uint32_t pulse_count;

void init_pulse_tmr(void);

#endif /* USER_PULSE_H_ */
