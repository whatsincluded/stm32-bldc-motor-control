#include "commutation.h"
#define COMMUTATION_OUTPUT_MASK (TIM_CCER_CC1E | TIM_CCER_CC1NE |TIM_CCER_CC2E | TIM_CCER_CC2NE | TIM_CCER_CC3E | TIM_CCER_CC3NE)

#define COMM_UH_VL (TIM_CCER_CC1E | TIM_CCER_CC2NE)
#define COMM_UH_WL (TIM_CCER_CC1E | TIM_CCER_CC3NE)
#define COMM_VH_WL (TIM_CCER_CC2E | TIM_CCER_CC3NE)
#define COMM_VH_UL (TIM_CCER_CC2E | TIM_CCER_CC1NE)
#define COMM_WH_UL (TIM_CCER_CC3E | TIM_CCER_CC1NE)
#define COMM_WH_VL (TIM_CCER_CC3E | TIM_CCER_CC2NE)


#define DUTY_SCALE 1000U
static uint16_t target_ccr = 0U;


static void Commutation_SetOutputs(uint32_t outputs);
static void Commutation_ConfigureCCR(uint32_t outputs);

void Commutation_Update(uint8_t hall)
{
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
            Commutation_Stop();
            break;
        }

    }
}

static void Commutation_SetOutputs(uint32_t outputs)
{
    Commutation_ConfigureCCR(outputs);
    uint32_t ccer = TIM1->CCER;
    ccer &=~COMMUTATION_OUTPUT_MASK;
    ccer |= outputs & COMMUTATION_OUTPUT_MASK;
    TIM1->CCER = ccer;
    TIM1->EGR = TIM_EGR_COMG;
}

static void Commutation_ConfigureCCR(uint32_t outputs)
{
    TIM1->CCR1 = 0U;
    TIM1->CCR2 = 0U;
    TIM1->CCR3 = 0U;

    if((outputs & TIM_CCER_CC1E) != 0U)
    {
        TIM1->CCR1 = target_ccr;
    }
    else if((outputs & TIM_CCER_CC2E) != 0U)
    {
        TIM1->CCR2 = target_ccr;
    }
    else if((outputs & TIM_CCER_CC3E) != 0U)
    {
        TIM1->CCR3 = target_ccr;
    }
}


void Commutation_Stop(void)
{
    PWM_Disable();
}

void Commutation_SetDuty(uint16_t duty_permille)
{
    if(duty_permille > DUTY_SCALE)
    {
        duty_permille = DUTY_SCALE;
    }

    target_ccr = (uint16_t)(((uint32_t)duty_permille*TIM1->ARR)/DUTY_SCALE);
}


    