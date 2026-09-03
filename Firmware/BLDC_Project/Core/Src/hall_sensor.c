#include "hall_sensor.h"



void HallSensor_Init(void)
{
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    // PB6 PB7 PB8 -> 홀 센서 EXTI로 사용 

    // PB6,7,8 input mode(00)으로 설정
    GPIOB->MODER &= ~(GPIO_MODER_MODE6_Msk | GPIO_MODER_MODE7_Msk | GPIO_MODER_MODE8_Msk);
    // 보드에 10k 저항 존재하므로 Pull up 저항 없이 진행
    GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD6_Msk | GPIO_PUPDR_PUPD7_Msk | GPIO_PUPDR_PUPD8_Msk);

    // PB6,7,8 을 EXTI 와 연결
    SYSCFG->EXTICR[1] &=~(SYSCFG_EXTICR2_EXTI6_Msk | SYSCFG_EXTICR2_EXTI7_Msk);
    SYSCFG->EXTICR[2] &=~ SYSCFG_EXTICR3_EXTI8_Msk;
    SYSCFG->EXTICR[1] |= (SYSCFG_EXTICR2_EXTI6_PB | SYSCFG_EXTICR2_EXTI7_PB);
    SYSCFG->EXTICR[2] |= SYSCFG_EXTICR3_EXTI8_PB;

    /* 상승·하강 에지 모두 감지 */
    EXTI->RTSR1 |= (EXTI_RTSR1_RT6 | EXTI_RTSR1_RT7 | EXTI_RTSR1_RT8);
    EXTI->FTSR1 |= (EXTI_FTSR1_FT6 | EXTI_FTSR1_FT7 | EXTI_FTSR1_FT8);

    EXTI->IMR1 |= (EXTI_IMR1_IM6 | EXTI_IMR1_IM7 | EXTI_IMR1_IM8);

    NVIC_EnableIRQ(EXTI9_5_IRQn);
}

uint8_t HallSensor_Read(void)
{
    uint32_t gpio = GPIOB->IDR;
    uint8_t hallA = (gpio >> 6) & 1U;
    uint8_t hallB = (gpio >> 7) & 1U;
    uint8_t hallC = (gpio >> 8) & 1U;

    return (uint8_t) ((hallA << 2) | (hallB << 1) | (hallC << 0));

}





void EXTI9_5_IRQHandler(void)
{  
    uint32_t pending = EXTI->PR1 &(EXTI_PR1_PIF6 | EXTI_PR1_PIF7 | EXTI_PR1_PIF8);

    if(pending == 0U)
    {
        return;
    }
        
    EXTI->PR1 = pending;

    uint8_t hall = HallSensor_Read();

    Commutation_Update(hall);

}
