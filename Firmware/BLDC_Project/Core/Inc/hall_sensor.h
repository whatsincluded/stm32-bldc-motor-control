#ifndef INC_HALL_SENSOR_H_
#define INC_HALL_SENSOR_H_

#include "stm32g4xx.h"

void HallSensor_Init(void);
uint8_t HallSensor_Read(void);
void HallSensor_EnableIRQ(void);

#endif /* INC_HALL_SENSOR_H_ */