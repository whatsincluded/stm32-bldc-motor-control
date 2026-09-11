/*
 * timer.h
 *
 *  Created on: Aug 31, 2026
 *      Author: kdk78
 */

#ifndef INC_TIMER_H_
#define INC_TIMER_H_

#include "stm32g4xx.h"

void TIM1_PWM_Init(void);
void TIM1_PWM_Enable(void);
void TIM1_PWM_Disable(void);
void TIM1_BreakInit(void);
#endif /* INC_TIMER_H_ */
