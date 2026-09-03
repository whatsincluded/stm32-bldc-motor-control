/*
 * timer.h
 *
 *  Created on: Aug 31, 2026
 *      Author: kdk78
 */

#ifndef INC_TIMER_H_
#define INC_TIMER_H_

#include "stm32g4xx.h"

void PWM_Init(void);
void PWM_Enable(void);
void PWM_Disable(void);
void PWM_SetDuty(uint16_t duty);
#endif /* INC_TIMER_H_ */
