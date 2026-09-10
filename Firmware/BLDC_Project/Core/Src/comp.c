#include "comp.h"


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