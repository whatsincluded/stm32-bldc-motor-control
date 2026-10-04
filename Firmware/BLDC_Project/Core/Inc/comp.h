#ifndef INC_COMP_H_
#define INC_COMP_H_

#include "stm32g4xx.h"


void Comp_Init(void);
void Comp_Enable(void);
void Comp_ArmFaultEdgeCapture(void);
void Comp_ClearFaultEdgeCapture(void);
uint32_t Comp_GetFaultEdges(void);
#endif /* INC_COMP_H_ */
