#ifndef INC_MOTOR_CONTROL_H_
#define INC_MOTOR_CONTROL_H_

#include "stm32g4xx.h"
#include "timer.h"



typedef enum{
    MOTOR_STOPPED = 0,
    MOTOR_STARTING,
    MOTOR_RUNNING,
    MOTOR_FAULT
}MotorState;

typedef enum{
    MOTOR_FAULT_NONE =0U,
    MOTOR_FAULT_OVERCURRENT = (1U<<0),
    MOTOR_FAULT_HALL_INVALID = (1U<<1),
    MOTOR_FAULT_HALL_SEQUENCE = (1U<<2),
    MOTOR_FAULT_START_TIMEOUT = (1U<<3),
    MOTOR_FAULT_STALL = (1U<<4)
}MotorFault;


MotorState Motor_GetState(void);
uint32_t Motor_GetFaults(void);
void Motor_ProcessHall(uint8_t hall_state);

void Motor_Stop(void);
void Motor_Trip(MotorFault reason);


#endif /* INC_MOTOR_CONTROL_H_ */