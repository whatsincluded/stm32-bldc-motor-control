#include "motor_control.h"
#include "commutation.h"

static volatile MotorState motor_state = MOTOR_STOPPED;
static volatile uint32_t motor_fault_flags = MOTOR_FAULT_NONE;

static uint8_t previous_hall = 0U;
static const uint8_t next_hall[8]={
    0U, // 인덱스 0: 사용 X
    5U, // 1 다음 5
    3U, // 2 다음 3
    1U, // 3 다음 1
    6U, // 4 다음 6
    4U, // 5 다음 4
    2U, // 6 다음 2
    0U
};

MotorState Motor_GetState(void)
{
    return motor_state;
}

uint32_t Motor_GetFaults(void)
{
    return motor_fault_flags;
}

void Motor_Stop(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    TIM1_PWM_Disable();

    if(motor_state != MOTOR_FAULT)
    {
        motor_state = MOTOR_STOPPED;
    }
    
    __set_PRIMASK(primask);

    
}

void Motor_Trip(MotorFault reason)
{   
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    TIM1_PWM_Disable();
    motor_fault_flags |= (uint32_t) reason;
    motor_state = MOTOR_FAULT;

    __set_PRIMASK(primask);
}



void Motor_ProcessHall(uint8_t hall_state)
{
    if(motor_state == MOTOR_FAULT)
    {
        return;
    }
    
    // 0 or 7 인 경우 Fault
    if(hall_state == 0U || hall_state>=7U)
    {
        Motor_Trip(MOTOR_FAULT_HALL_INVALID);
        return;
    }

    // 첫 시작일 경우
    if(previous_hall == 0U)
    {
        previous_hall = hall_state;
        Commutation_Update(hall_state);
        return;
    }

    // 이전 hall_state과 현재 hall_state 같을 시 종료
    if(hall_state == previous_hall)
    {
        return;
    }

    //현재의 hall_state이 다음의 hall_state 예상과 다를 경우 Fault
    if(hall_state != next_hall[previous_hall])
    {
        Motor_Trip(MOTOR_FAULT_HALL_SEQUENCE);
        return;
    }

    previous_hall = hall_state;
    Commutation_Update(hall_state);
}