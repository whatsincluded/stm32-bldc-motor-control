#ifndef INC_COMMUTATION_H_
#define INC_COMMUTATION_H_

#include "stm32g4xx.h"
#include "timer.h"

void Commutation_Update(uint8_t hall);
void Commutation_Stop(void);
void Commutation_SetDuty(uint16_t duty_permille);
#endif /* INC_COMMUTATION_H_ */