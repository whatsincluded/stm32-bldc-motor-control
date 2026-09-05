#ifndef INC_HALL_SENSOR_H_
#define INC_HALL_SENSOR_H_

#include "stm32g4xx.h"
#include "commutation.h"
void HallSensor_Init(void);
uint8_t HallSensor_Read(void);

#endif /* INC_HALL_SENSOR_H_ */