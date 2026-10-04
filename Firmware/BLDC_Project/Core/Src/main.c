/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "hall_sensor.h"
#include "timer.h"
#include "motor_control.h"
#include "commutation.h"
#include "comp.h"
#include "dac.h"

void SystemClock_Config(void);
static void Analog_Wait10us(void);

volatile uint8_t fault_clear_request = 0U;
volatile uint8_t motor_start_request = 0U;
/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();

  /* CPU cycle counter 활성화 */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  /* PWM 출력은 계속 OFF */
  TIM1_PWM_Init();
  TIM1_PWM_Disable();

  /* 보호 설정 중 Break ISR 실행 방지 */
  NVIC_DisableIRQ(TIM1_BRK_TIM15_IRQn);
  TIM1->DIER &= ~TIM_DIER_BIE;

  /* COMP 설정만 수행 */
  Comp_Init();

  /* 현재 DAC_Init()은 채널 활성화까지 수행함 */
  DAC_Init();
  Analog_Wait10us();

  /* 기준 전압이 준비된 뒤 COMP 활성화 */
  Comp_Enable();
  Analog_Wait10us();

  /* COMP 초기 안정화 후 상승 에지 기록을 시작한다.
   * COMP 인터럽트는 사용하지 않고 기존 TIM1 Break 차단을 유지한다. */
  Comp_ArmFaultEdgeCapture();

  /* 내부 과전류 신호를 TIM1 Break에 연결 */
  TIM1_BreakInit();

  /* Break가 Hall IRQ보다 높은 우선순위 */
  NVIC_SetPriority(TIM1_BRK_TIM15_IRQn, 0U);
  NVIC_SetPriority(EXTI9_5_IRQn, 1U);

  /* 이미 발생한 BIF는 지우지 않음 */
  TIM1->DIER |= TIM_DIER_BIE;
  NVIC_EnableIRQ(TIM1_BRK_TIM15_IRQn);

  /* Hall 초기 처리 후 인터럽트 활성화 */
  HallSensor_Init();
  /* 첫 구동 시험용 듀티 5%. 모터 상전류를 5%로 제한한다는 뜻은 아니다. */
  Commutation_SetDuty(50U);
  HallSensor_EnableIRQ();
    

  while (1)
  {
    Motor_Update();
    if (motor_start_request != 0U)
    {
        motor_start_request = 0U;
        Motor_Start();
    }

    if(fault_clear_request != 0U)
    {
      fault_clear_request = 0U;
      Motor_ClearFault();
    }
  }

}


static void Analog_Wait10us(void)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = SystemCoreClock / 100000U;

    while ((uint32_t)(DWT->CYCCNT - start) < ticks) {
    }
}


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
