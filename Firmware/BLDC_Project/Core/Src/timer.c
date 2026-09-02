/*
 * timer.c
 *
 *  Created on: Aug 31, 2026
 *      Author: kdk78
 */


#include "timer.h"

#define TIM1_CLOCK_HZ 170000000UL
#define PWM_FREQUENCY_HZ 20000UL
#define PWM_PRESCALER 0U
#define PWM_ARR_VALUE (TIM1_CLOCK_HZ/(2UL * PWM_FREQUENCY_HZ*(PWM_PRESCALER + 1UL)))
#define PWM_DEADTIME_NS 500U
#define TIM1_DTG_VALUE 85U
//#define TIM1_DTG_VALUE ((PWM_DEADTIME_NS * TIM1_CLOCK_HZ + 999999999UL) / 1000000000UL)


void PWM_Init(void)
{   /*
    CH1 → PA8  CH1N → PC13
    CH2 → PA9  CH2N → PA12
    CH3 → PA10 CH3N → PB15
    */
    // PA, PB, PC Enable
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;

    // TIM1 Enable
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

    // PA8,9,10,12 PB15 PC13 AF MODE 설정

    // 해당 핀번호 초기화
    GPIOA->MODER &= ~(GPIO_MODER_MODE8_Msk | GPIO_MODER_MODE9_Msk |
                     GPIO_MODER_MODE10_Msk | GPIO_MODER_MODE12_Msk);
    GPIOB->MODER &= ~GPIO_MODER_MODE15_Msk;
    GPIOC->MODER &= ~GPIO_MODER_MODE13_Msk;

    // 해당 핀 번호에 AF MODE 설정
    GPIOA->MODER |= (GPIO_MODER_MODE8_1 | GPIO_MODER_MODE9_1 |
                    GPIO_MODER_MODE10_1 | GPIO_MODER_MODE12_1);
    GPIOB->MODER |= GPIO_MODER_MODE15_1;
    GPIOC->MODER |= GPIO_MODER_MODE13_1;
    
    // 해당 핀 번호 AFR 초기화
    GPIOA->AFR[1] &= ~(GPIO_AFRH_AFSEL8_Msk | GPIO_AFRH_AFSEL9_Msk |
                    GPIO_AFRH_AFSEL10_Msk | GPIO_AFRH_AFSEL12_Msk);

    GPIOB->AFR[1] &=~ GPIO_AFRH_AFSEL15_Msk;
    GPIOC->AFR[1] &=~ GPIO_AFRH_AFSEL13_Msk;

    // PA -> AF6 / PB PC-> AF4
    // Datasheet의 TIM1의 채널에 해당하는 AF번호 확인 후 입력함
    GPIOA->AFR[1] |= (0x6UL<<GPIO_AFRH_AFSEL8_Pos | 0x6UL<<GPIO_AFRH_AFSEL9_Pos |
                    0x6UL<<GPIO_AFRH_AFSEL10_Pos | 0x6UL<<GPIO_AFRH_AFSEL12_Pos);
    GPIOB->AFR[1] |= 0x4UL<<GPIO_AFRH_AFSEL15_Pos;
    GPIOC->AFR[1] |= 0x4UL<<GPIO_AFRH_AFSEL13_Pos;

    // GPIO Speed 설정 -> High Speed(10)으로 설정
    GPIOA->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED8_Msk | GPIO_OSPEEDR_OSPEED9_Msk |
                        GPIO_OSPEEDR_OSPEED10_Msk | GPIO_OSPEEDR_OSPEED12_Msk);
    GPIOB->OSPEEDR &=~ GPIO_OSPEEDR_OSPEED15_Msk;
    GPIOC->OSPEEDR &=~ GPIO_OSPEEDR_OSPEED13_Msk;

    GPIOA->OSPEEDR |= (GPIO_OSPEEDR_OSPEED8_1 | GPIO_OSPEEDR_OSPEED9_1 |
                      GPIO_OSPEEDR_OSPEED10_1 | GPIO_OSPEEDR_OSPEED12_1 );
    GPIOB->OSPEEDR |= GPIO_OSPEEDR_OSPEED15_1;
    GPIOC->OSPEEDR |= GPIO_OSPEEDR_OSPEED13_1;

    // TIM1 설정 -> f = 20kHz
    // PSC = 0, ARR = 4250, center aligend, pwm mode 1 
    TIM1->PSC = PWM_PRESCALER;
    TIM1->ARR = PWM_ARR_VALUE;

    TIM1->RCR = 0U;
    
    /*PWM MODE 1 설정*/
    TIM1->CCMR1 &= ~(TIM_CCMR1_OC1M_Msk | TIM_CCMR1_OC2M_Msk);
    TIM1->CCMR2 &= ~TIM_CCMR2_OC3M_Msk;

    TIM1->CCMR1 |= (0x6U<< TIM_CCMR1_OC1M_Pos | 0x6U<<TIM_CCMR1_OC2M_Pos);
    TIM1->CCMR2 |= 0x6U<<TIM_CCMR2_OC3M_Pos;

    /* ARR/CCR preload enable */
    TIM1->CR1 |= TIM_CR1_ARPE;  //ARR preload enable
    TIM1->CCMR1 |= TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE; // CCR preload enable
    TIM1->CCMR2 |= TIM_CCMR2_OC3PE;

    /* All PWM idle states = LOW */
    TIM1->CR2 &= ~ (TIM_CR2_OIS1  | TIM_CR2_OIS1N |
                    TIM_CR2_OIS2  | TIM_CR2_OIS2N |
                    TIM_CR2_OIS3  | TIM_CR2_OIS3N);
    
    /* CKD = DIV1: dead-time clock tDTS = TIM1 clock period 
       데드 타임이 바뀌지 않도록 그냥 도움 주는? 그런 역할 */
    TIM1->CR1 &= ~TIM_CR1_CKD_Msk;
    // 데드타임 설정 500ns
    TIM1->BDTR &=~ TIM_BDTR_DTG_Msk;
    TIM1->BDTR |= (TIM1_DTG_VALUE << TIM_BDTR_DTG_Pos);
    /* Drive inactive/disabled PWM outputs to their idle state */
    TIM1->BDTR |= TIM_BDTR_OSSI | TIM_BDTR_OSSR;

    /* 모든 출력은 비활성화 상태로 시작 */
    TIM1->CCER &= ~(TIM_CCER_CC1E  | TIM_CCER_CC1NE |
                    TIM_CCER_CC2E  | TIM_CCER_CC2NE |
                    TIM_CCER_CC3E  | TIM_CCER_CC3NE);

    TIM1->CCR1 = 0U;
    TIM1->CCR2 = 0U;
    TIM1->CCR3 = 0U;

    // Center Aligned Mode 설정 -> 1 FOC 할때는 3으로 변경 예정
    TIM1->CR1 &= ~ TIM_CR1_CMS_Msk;
    TIM1->CR1 |= TIM_CR1_CMS_0; // mode 1
    
    /* PSC/ARR/CCR preload 반영 */
    TIM1->EGR = TIM_EGR_UG;
    TIM1->SR &= ~TIM_SR_UIF;

    /* 6-step commutation preload enable */
    TIM1->CR2 |= TIM_CR2_CCPC;

    TIM1->CR1 |= TIM_CR1_CEN;
}


void PWM_Enable(void)
{
    TIM1->BDTR |= TIM_BDTR_MOE;
}

void PWM_Disable(void)
{
    TIM1->BDTR &=~ TIM_BDTR_MOE;
}

void PWM_SetDuty(uint16_t duty)
{
    if(duty > PWM_ARR_VALUE)
    {
        duty = PWM_ARR_VALUE;
    }

    TIM1->CCR1 = duty;
    TIM1->CCR2 = duty;
    TIM1->CCR3 = duty;
    
}