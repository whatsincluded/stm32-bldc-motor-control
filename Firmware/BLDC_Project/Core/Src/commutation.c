#include "commutation.h"
#include "motor_control.h"


#define COMMUTATION_OUTPUT_MASK (TIM_CCER_CC1E | TIM_CCER_CC1NE |TIM_CCER_CC2E | TIM_CCER_CC2NE | TIM_CCER_CC3E | TIM_CCER_CC3NE)

#define COMM_UH_VL (TIM_CCER_CC1E | TIM_CCER_CC2NE)
#define COMM_UH_WL (TIM_CCER_CC1E | TIM_CCER_CC3NE)
#define COMM_VH_WL (TIM_CCER_CC2E | TIM_CCER_CC3NE)
#define COMM_VH_UL (TIM_CCER_CC2E | TIM_CCER_CC1NE)
#define COMM_WH_UL (TIM_CCER_CC3E | TIM_CCER_CC1NE)
#define COMM_WH_VL (TIM_CCER_CC3E | TIM_CCER_CC2NE)


#define OC_MODE_FORCED_INACTIVE 4U
#define OC_MODE_FORCED_ACTIVE   5U
#define OC_MODE_PWM1            6U


#define DUTY_SCALE 1000U




static void Commutation_SetOutputs(uint32_t outputs);

/* MOE가 꺼진 상태에서 호출한다. 상단은 모두 OFF로 두고
 * 지정한 한 상의 하단만 선택한다(1=U, 2=V, 3=W).
 * 실제 출력 허용과 과전류 확인은 모터 상태 제어에서 수행한다. */
void Commutation_BootstrapPhase(uint8_t phase)
{
    uint32_t outputs = 0U;

    switch (phase)
    {
        case 1U: outputs = TIM_CCER_CC1NE; break;
        case 2U: outputs = TIM_CCER_CC2NE; break;
        case 3U: outputs = TIM_CCER_CC3NE; break;
        default: break; /* 잘못된 번호에서는 모든 출력을 끈다. */
    }

    Commutation_SetOutputs(outputs);
}

/* 세 상 모두 비활성화하고 COM 이벤트로 출력 선택을 반영한다. */
void Commutation_AllOff(void)
{
    Commutation_SetOutputs(0U);
}


void Commutation_Update(uint8_t hall)
{
    if(Motor_GetState() == MOTOR_FAULT)
    {
        return;
    }

    switch(hall)
    {
        case 0b001:
        {
            Commutation_SetOutputs(COMM_UH_VL);
            break;
        }
        case 0b010:
        {
            Commutation_SetOutputs(COMM_WH_UL);
            break;
        }
        case 0b011:
        {
            Commutation_SetOutputs(COMM_WH_VL);
            break;
        }
        case 0b100:
        {
            Commutation_SetOutputs(COMM_VH_WL);
            break;
        }
        case 0b101:
        {
            Commutation_SetOutputs(COMM_UH_WL);
            break;
        }
        case 0b110:
        {
            Commutation_SetOutputs(COMM_VH_UL);
            break;
        }
        default:
        {
            //fault
            Motor_Trip(MOTOR_FAULT_HALL_INVALID);
            break;
        }

    }
}

static void Commutation_SetOutputs(uint32_t outputs)
{
    uint32_t mode1 = OC_MODE_FORCED_INACTIVE;
    uint32_t mode2 = OC_MODE_FORCED_INACTIVE;
    uint32_t mode3 = OC_MODE_FORCED_INACTIVE;

    outputs &= COMMUTATION_OUTPUT_MASK;
    /* U상: 상단 PWM / 하단 ON / 양쪽 OFF */
    if((outputs & TIM_CCER_CC1E) != 0U)
    {
        mode1 = OC_MODE_PWM1;
    }
    else if((outputs & TIM_CCER_CC1NE)!= 0U)
    {
        mode1 = OC_MODE_FORCED_ACTIVE;
    }
    /*V상*/
    if((outputs & TIM_CCER_CC2E) != 0U)
    {
        mode2 = OC_MODE_PWM1;
    }
    else if((outputs & TIM_CCER_CC2NE) != 0U)
    {
        mode2 = OC_MODE_FORCED_ACTIVE;
    }

    /*W상*/
    if((outputs & TIM_CCER_CC3E) != 0U)
    {
        mode3 = OC_MODE_PWM1;
    }
    else if((outputs & TIM_CCER_CC3NE) != 0U)
    {
        mode3 = OC_MODE_FORCED_ACTIVE;
    }

    /*
     * CCPC=1:
     * OCxM과 CCxE/CCxNE는 COM 이벤트에서 함께 반영.
     * 준비 도중 인터럽트가 끼어들지 않도록 보호.
     */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    if(Motor_GetState() == MOTOR_FAULT)
    {
        __set_PRIMASK(primask);
        return;
    }

    uint32_t ccmr1 = TIM1->CCMR1;
    uint32_t ccmr2 = TIM1->CCMR2;
    uint32_t ccer = TIM1->CCER;

    ccmr1 &=~ (TIM_CCMR1_OC1M_Msk | TIM_CCMR1_OC2M_Msk);
    ccmr1 |= (mode1 << TIM_CCMR1_OC1M_Pos) | (mode2<<TIM_CCMR1_OC2M_Pos);

    ccmr2 &=~ (TIM_CCMR2_OC3M_Msk);
    ccmr2 |= (mode3 << TIM_CCMR2_OC3M_Pos);

    ccer &=~COMMUTATION_OUTPUT_MASK;
    ccer |= outputs;

    /* 다음 단계 준비 */
    TIM1->CCMR1 = ccmr1;
    TIM1->CCMR2 = ccmr2;
    TIM1->CCER  = ccer;

    /* 세 상의 모드와 출력 선택을 함께 반영 */
    TIM1->EGR = TIM_EGR_COMG;

    __set_PRIMASK(primask);

}




void Commutation_SetDuty(uint16_t duty_permille)
{
    if(duty_permille > DUTY_SCALE)
    {
        duty_permille = DUTY_SCALE;
    }

    uint32_t ccr = (uint16_t)(((uint32_t)duty_permille*TIM1->ARR)/DUTY_SCALE);

    uint32_t primask = __get_PRIMASK();
    __disable_irq();

     /* 기존 Update 차단 상태 저장 */
    uint32_t saved_udis = TIM1->CR1 & TIM_CR1_UDIS;

    /* CCR preload가 쓰기 도중 반영되지 않도록 차단 */
    TIM1->CR1 |= TIM_CR1_UDIS;

    /* 세 상 모두 동일한 PWM 듀티를 준비 */
    TIM1->CCR1 = ccr;
    TIM1->CCR2 = ccr;
    TIM1->CCR3 = ccr;

    if(saved_udis == 0U)
    {
        TIM1->CR1 &= ~TIM_CR1_UDIS;
    }
    __set_PRIMASK(primask);
}



