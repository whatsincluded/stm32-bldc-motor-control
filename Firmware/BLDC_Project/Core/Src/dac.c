#include "dac.h"



void DAC_Init(void)
{
    RCC->AHB2ENR |= RCC_AHB2ENR_DAC3EN;
    // CH1,2 초기화
    DAC3->CR &= ~(DAC_CR_EN1 | DAC_CR_EN2 |
                DAC_CR_CEN1 | DAC_CR_CEN2);
    // 파형 생성 X
    DAC3->CR &= ~(DAC_CR_WAVE1_Msk | DAC_CR_WAVE2_Msk);
    // DAC 모드 설정 011 -> DAC channel is connected to on chip peripherals with Buffer disabled
    DAC3->MCR &= ~(DAC_MCR_MODE1_Msk | DAC_MCR_MODE2_Msk);
    DAC3->MCR |= (0x3U<<DAC_MCR_MODE1_Pos) | (0x3U<<DAC_MCR_MODE2_Pos);
    

    DAC3->CR &= ~(DAC_CR_TEN1_Msk | DAC_CR_TEN2_Msk);

    DAC3->DHR12R1 = 1;
    DAC3->DHR12R2 = 1;

    DAC3->CR |= DAC_CR_EN1 | DAC_CR_EN2;

}