#ifndef INC_COMMUTATION_H_
#define INC_COMMUTATION_H_

#include "stm32g4xx.h"
#include "timer.h"

void Commutation_Update(uint8_t hall);
void Commutation_Stop(void);
#endif /* INC_COMMUTATION_H_ */