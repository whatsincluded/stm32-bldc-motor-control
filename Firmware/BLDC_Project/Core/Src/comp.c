#include "comp.h"

/* COMP1/2/4의 EXTI 상승 에지를 기록하는 선: 각각 21/22/30번.
 * TIM1 Break는 그대로 하드웨어에서 처리하고, 이 경로는 진단에만 사용한다. */
#define COMP_FAULT_EXTI_MASK (EXTI_PR1_PIF21 | EXTI_PR1_PIF22 | EXTI_PR1_PIF30)

void Comp_ArmFaultEdgeCapture(void)
{
    /* EXTI 요청은 허용해야 PR1에 순간 상승 에지가 남는다.
     * NVIC 인터럽트는 켜지 않아 기록 처리로 Break 차단을 늦추지 않는다. */
    NVIC_DisableIRQ(COMP1_2_3_IRQn);
    NVIC_DisableIRQ(COMP4_5_6_IRQn);
    EXTI->IMR1 |= COMP_FAULT_EXTI_MASK;
    EXTI->FTSR1 &= ~COMP_FAULT_EXTI_MASK;
    EXTI->RTSR1 |= COMP_FAULT_EXTI_MASK;
    Comp_ClearFaultEdgeCapture();
}

void Comp_ClearFaultEdgeCapture(void)
{
    /* EXTI_PR1은 1을 써서 해당 대기 비트만 지운다. */
    EXTI->PR1 = COMP_FAULT_EXTI_MASK;
    NVIC_ClearPendingIRQ(COMP1_2_3_IRQn);
    NVIC_ClearPendingIRQ(COMP4_5_6_IRQn);
}

uint32_t Comp_GetFaultEdges(void)
{
    uint32_t pending = EXTI->PR1;
    return (((pending & EXTI_PR1_PIF21) != 0U) ? 1U : 0U) |
           (((pending & EXTI_PR1_PIF22) != 0U) ? 2U : 0U) |
           (((pending & EXTI_PR1_PIF30) != 0U) ? 4U : 0U);
}

void Comp_Init(void)
{
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    // PA1, PA7, PB0 Analog Mode 설정
    GPIOA->MODER &= ~(GPIO_MODER_MODE1_Msk | GPIO_MODER_MODE7_Msk);
    GPIOA->MODER |= GPIO_MODER_MODE1 | GPIO_MODER_MODE7;
    GPIOB->MODER &= ~GPIO_MODER_MODE0_Msk;
    GPIOB->MODER |= GPIO_MODER_MODE0;

    GPIOA->PUPDR &=~(GPIO_PUPDR_PUPD1_Msk | GPIO_PUPDR_PUPD7_Msk);
    GPIOB->PUPDR &=~ GPIO_PUPDR_PUPD0_Msk;

    COMP1->CSR &=~ COMP_CSR_INPSEL;
    COMP2->CSR &=~ COMP_CSR_INPSEL;
    COMP4->CSR &=~ COMP_CSR_INPSEL;

    /* (-) 입력 선택 필드 초기화 */
    COMP1->CSR &=~COMP_CSR_INMSEL_Msk;
    COMP2->CSR &=~COMP_CSR_INMSEL_Msk;
    COMP4->CSR &=~COMP_CSR_INMSEL_Msk;

    /* INMSEL = 100: 내부 DAC3 연결 */
    COMP1->CSR |= (4U << COMP_CSR_INMSEL_Pos);
    COMP2->CSR |= (4U << COMP_CSR_INMSEL_Pos);
    COMP4->CSR |= (4U << COMP_CSR_INMSEL_Pos);

    /* 비반전 출력: (+) 입력 > (-) 입력이면 HIGH */
    COMP1->CSR &=~COMP_CSR_POLARITY_Msk;
    COMP2->CSR &=~COMP_CSR_POLARITY_Msk;
    COMP4->CSR &=~COMP_CSR_POLARITY_Msk;

    /* 정적 특성 검증용: hysteresis와 blanking 없음 */
    COMP1->CSR &= ~(COMP_CSR_HYST_Msk | COMP_CSR_BLANKING_Msk);
    COMP2->CSR &= ~(COMP_CSR_HYST_Msk | COMP_CSR_BLANKING_Msk);
    COMP4->CSR &= ~(COMP_CSR_HYST_Msk | COMP_CSR_BLANKING_Msk);


}

void Comp_Enable(void)
{
    COMP1->CSR |= COMP_CSR_EN;
    COMP2->CSR |= COMP_CSR_EN;
    COMP4->CSR |= COMP_CSR_EN;
}
