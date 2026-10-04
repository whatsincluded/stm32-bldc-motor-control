#include "motor_control.h"
#include "commutation.h"
#include "hall_sensor.h"
#include "stm32g4xx_hal.h"
#include "comp.h"



#define MOTOR_START_TIMEOUT_MS 1000U
#define MOTOR_STALL_TIMEOUT_MS 500U
#define MOTOR_TEST_MAX_RUN_MS 2000U

/* 1ms 틱을 사용하는 초기 시험 설정이며 실측 확정값은 아니다.
 * 각 상을 2틱 켜고 상 사이에 2틱 동안 모두 끈다.
 * 실제 대기 시간은 틱 경계와 메인 루프 지연에 영향을 받는다.
 * 충전 시간은 실제 부트스트랩 전압을 확인한 뒤 조정한다. */
#define MOTOR_BOOTSTRAP_CHARGE_MS 2U
#define MOTOR_BOOTSTRAP_GAP_MS 2U
#define MOTOR_START_GAP_MS 2U

static volatile MotorState motor_state = MOTOR_STOPPED;
static volatile uint32_t motor_fault_flags = MOTOR_FAULT_NONE;

/* 마지막으로 진행한 진단 단계: 출력 제어에는 사용하지 않는다.
 * 0: 초기화/Fault 해제 후 아직 시작하지 않음, 1: 시작 검사 및 충전 출력 준비,
 * 2: 해당 상 충전 출력 활성화 시점/충전 중, 3: 상 사이 또는 충전 후 출력 OFF 대기,
 * 4: 정류 출력 준비/활성화 검사, 5: 정류 출력 활성화 검사 통과 후.
 * 정지해도 마지막 단계를 보존한다. 정상 정지 후 발생한 Fault에도 이 값이
 * 남을 수 있으므로 motor_fault_state와 함께 해석한다.
 * 하드웨어 사건의 정확한 발생 시점이나 원인을 증명하는 값은 아니다. */
static volatile uint8_t motor_diag_stage = 0U;

/* Live Expressions에서 확인할 첫 Fault 기록.
 * COMP는 bit0=COMP1, bit1=COMP2, bit2=COMP4이며 1은 검출 당시 HIGH다.
 * 출력 차단 후 순차적으로 읽으므로 짧은 COMP 펄스는 이미 사라질 수 있다.
 * 해제 후에도 기록을 보존하고, 다음 Fault가 발생하면 새 기록으로 교체한다. */
volatile uint8_t motor_fault_stage = 0U;
/* 충전 중 Fault가 난 상. 1=U, 2=V, 3=W, 0=충전과 무관함.
 * 출력 OFF 대기 중의 값은 직전에 충전한 상을 나타낸다. */
volatile uint8_t motor_fault_bootstrap_phase = 0U;
volatile uint32_t motor_fault_comp = 0U;
/* bit0=COMP1, bit1=COMP2, bit2=COMP4의 상승 에지 기록.
 * 현재 COMP 출력이 이미 LOW여도 이전 펄스는 EXTI에 남을 수 있다. */
volatile uint32_t motor_fault_comp_edges = 0U;
volatile uint32_t motor_fault_tim_sr = 0U;
volatile MotorState motor_fault_state = MOTOR_STOPPED;

/* 시작 요청이 함수에 도달했는지와 이미 FAULT여서 거부됐는지 확인한다.
 * 거부된 호출도 횟수에 포함하며, 상태를 변경하기 전에 진입 값을 저장한다.
 * 마지막 호출 기록은 다음 호출 또는 MCU 초기화 전까지 유지한다.
 * Live Expressions는 여러 값을 동시에 읽지 않으므로 호출 완료 후 확인한다. */
volatile uint32_t motor_start_count = 0U;
volatile MotorState motor_start_entry_state = MOTOR_STOPPED;
volatile uint32_t motor_start_entry_faults = MOTOR_FAULT_NONE;

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

static volatile uint32_t start_tick = 0U;
static volatile uint32_t last_hall_tick = 0U;
static uint32_t startup_phase_tick = 0U;
static uint8_t bootstrap_phase = 0U; /* 1=U, 2=V, 3=W */
static uint8_t bootstrap_output_on = 0U;

/* 호출자는 짧은 임계 구역에서 상태 변경과 함께 검사한다.
 * 하드웨어 Break는 인터럽트 마스킹 중에도 출력을 차단한다. */
static uint8_t Motor_CheckStartProtection(void)
{
    if (motor_fault_flags != MOTOR_FAULT_NONE)
    {
        TIM1_PWM_Disable();
        motor_state = MOTOR_FAULT;
        return 0U;
    }

    if (((TIM1->SR & TIM_SR_BIF) != 0U) ||
        (((COMP1->CSR | COMP2->CSR | COMP4->CSR) & COMP_CSR_VALUE) != 0U))
    {
        Motor_Trip(MOTOR_FAULT_OVERCURRENT);
        return 0U;
    }

    if (((TIM1->BDTR & TIM_BDTR_BKE) == 0U) ||
        ((TIM1->DIER & TIM_DIER_BIE) == 0U) ||
        ((TIM1->BDTR & TIM_BDTR_AOE) != 0U))
    {
        Motor_Stop();
        return 0U;
    }
    return 1U;
}

/* 충전 출력과 정상 정류 출력 모두 같은 보호 검사를 거쳐 허용한다. */
static uint8_t Motor_EnableChecked(void)
{
    if (Motor_CheckStartProtection() == 0U)
    {
        return 0U;
    }
    /* 충전 전 검사에서 거부된 경우와 충전 출력을 허용한 경우를 구분한다. */
    if (motor_state == MOTOR_BOOTSTRAP)
    {
        motor_diag_stage = 2U;
    }
    TIM1_PWM_Enable();
    if (Motor_CheckStartProtection() == 0U)
    {
        return 0U;
    }
    if ((TIM1->BDTR & TIM_BDTR_MOE) == 0U)
    {
        Motor_Stop();
        return 0U;
    }
    return 1U;
}


static void Motor_ResetTiming(void)
{
    uint32_t now = HAL_GetTick();

    start_tick = now;
    last_hall_tick = now;
}

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
        /* MOE 차단을 감지한 뒤 대기 중인 Break ISR이 실행될 수 있다.
         * 이때 최초 진행 단계를 잃지 않도록 진단 단계는 지우지 않는다. */
    }
    
    __set_PRIMASK(primask);
}


void Motor_Start(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    /* 조기 반환 경로도 기록해야 시작 요청 거부 여부를 구분할 수 있다. */
    motor_start_entry_state = motor_state;
    motor_start_entry_faults = motor_fault_flags;
    motor_start_count++;

    /*정지 상태에서만 시작 가능*/
    if(motor_state != MOTOR_STOPPED)
    {
        goto exit;
    }
    
    TIM1_PWM_Disable();

    /* 기록된 fault가 있으면 시작 금지 */
    if(motor_fault_flags != MOTOR_FAULT_NONE)
    {
        motor_state = MOTOR_FAULT;
        goto exit;
    }
    motor_diag_stage = 1U;
    /* 보호 기능이 활성화된 상태인지 확인 */
    if(((TIM1->BDTR & TIM_BDTR_BKE)== 0U) || ((TIM1->DIER & TIM_DIER_BIE)==0U)||((TIM1->BDTR & TIM_BDTR_AOE) != 0U))
    {
        goto exit;
    }

    /* 기존 Break 기록 또는 현재 과전류 신호 확인 */
    if(((TIM1->SR & TIM_SR_BIF) != 0U) || (((COMP1->CSR | COMP2->CSR | COMP4->CSR) & COMP_CSR_VALUE) != 0U))
    {
        Motor_Trip(MOTOR_FAULT_OVERCURRENT);
        goto exit;
    }

    uint8_t initial_hall = HallSensor_Read();

    if((initial_hall == 0U) || (initial_hall == 7U))
    {
        Motor_Trip(MOTOR_FAULT_HALL_INVALID);
        goto exit;
    }

    motor_state = MOTOR_BOOTSTRAP;

    /* 아직 정류를 시작하지 않는다. 상단 3개 OFF / U상 하단만 ON으로
     * 순차 충전을 시작한다. Hall 인터럽트는 이 상태에서 출력을 변경하지 않는다. */

    previous_hall = 0U;
    bootstrap_phase = 1U;
    bootstrap_output_on = 0U;
    Commutation_BootstrapPhase(bootstrap_phase);

    if(motor_state == MOTOR_FAULT)
    {
        goto exit;
    }

    /*
     * PWM OFF 상태에서 CCR preload를 실제로 반영.
     * 현재 TIM1 설정(UDIS=0)을 전제로 한다.
     */
    TIM1->EGR = TIM_EGR_UG;

    /* 출력 준비 중 또는 활성화 직후 발생한 Break까지 공통 함수에서 확인한다. */
    if (Motor_EnableChecked() == 0U)
    {
        goto exit;
    }
    bootstrap_output_on = 1U;
    startup_phase_tick = HAL_GetTick();
    exit:
        __set_PRIMASK(primask);
}


void Motor_Trip(MotorFault reason)
{   
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    TIM1_PWM_Disable();
    /* 기존과 동일하게 출력을 먼저 차단한다. 기록 때문에 차단을 늦추지 않는다.
     * 동일 Fault의 추가 호출은 최초 기록을 덮어쓰지 않는다. */
    if (motor_fault_flags == MOTOR_FAULT_NONE)
    {
        motor_fault_tim_sr = TIM1->SR;
        motor_fault_comp =
            (((COMP1->CSR & COMP_CSR_VALUE) != 0U) ? 1U : 0U) |
            (((COMP2->CSR & COMP_CSR_VALUE) != 0U) ? 2U : 0U) |
            (((COMP4->CSR & COMP_CSR_VALUE) != 0U) ? 4U : 0U);
        motor_fault_comp_edges = Comp_GetFaultEdges();
        motor_fault_state = motor_state;
        motor_fault_stage = motor_diag_stage;
        motor_fault_bootstrap_phase =
            ((motor_diag_stage == 2U) || (motor_diag_stage == 3U))
                ? bootstrap_phase : 0U;
    }
    motor_fault_flags |= (uint32_t) reason;
    motor_state = MOTOR_FAULT;

    __set_PRIMASK(primask);
}



void Motor_ProcessHall(uint8_t hall_state)
{
    if((motor_state != MOTOR_STARTING) && (motor_state != MOTOR_RUNNING))
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

    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    /* 보호 인터럽트에서 발생한 FAULT를 덮어쓰지 않음 */
    if ((motor_state == MOTOR_STARTING) ||
        (motor_state == MOTOR_RUNNING))
    {
        previous_hall = hall_state;
        Commutation_Update(hall_state);

        if (motor_state != MOTOR_FAULT)
        {
            last_hall_tick = HAL_GetTick();

            if (motor_state == MOTOR_STARTING)
            {
                motor_state = MOTOR_RUNNING;
            }
        }
    }

    __set_PRIMASK(primask);
}


void Motor_Update(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    uint32_t now = HAL_GetTick();

    /* 매 호출에서는 상태만 확인하고 즉시 반환하므로 대기 중에도
     * 메인 루프와 인터럽트가 실행된다. 디버거 정지 중에는 진행되지 않는다. */
    if ((motor_state == MOTOR_BOOTSTRAP) ||
        (motor_state == MOTOR_START_WAIT))
    {
        if (Motor_CheckStartProtection() == 0U)
        {
            goto update_exit;
        }

        if (motor_state == MOTOR_BOOTSTRAP)
        {
            if (bootstrap_output_on != 0U)
            {
                if ((TIM1->BDTR & TIM_BDTR_MOE) == 0U)
                {
                    Motor_Stop();
                }
                else if ((uint32_t)(now - startup_phase_tick) >= MOTOR_BOOTSTRAP_CHARGE_MS)
                {
                    /* 다음 상을 켜기 전에 현재 상을 완전히 끈다. */
                    TIM1_PWM_Disable();
                    Commutation_AllOff();
                    bootstrap_output_on = 0U;
                    startup_phase_tick = HAL_GetTick();
                    motor_diag_stage = 3U;

                    if (bootstrap_phase == 3U)
                    {
                        motor_state = MOTOR_START_WAIT;
                    }
                }
            }
            else if ((uint32_t)(now - startup_phase_tick) >= MOTOR_BOOTSTRAP_GAP_MS)
            {
                /* 출력 OFF 간격 후 다음 한 상의 하단만 켠다. */
                bootstrap_phase++;
                Commutation_BootstrapPhase(bootstrap_phase);
                if (Motor_EnableChecked() != 0U)
                {
                    bootstrap_output_on = 1U;
                    startup_phase_tick = HAL_GetTick();
                }
            }
        }
        else if ((uint32_t)(now - startup_phase_tick) >= MOTOR_START_GAP_MS)
        {
            /* 충전 중 위치가 바뀌었을 수 있으므로 Hall을 다시 읽는다. */
            uint8_t hall = HallSensor_Read();
            if ((hall == 0U) || (hall == 7U))
            {
                Motor_Trip(MOTOR_FAULT_HALL_INVALID);
                goto update_exit;
            }

            TIM1_PWM_Disable();
            previous_hall = hall;
            motor_state = MOTOR_STARTING;
            motor_diag_stage = 4U;
            Commutation_Update(hall);
            TIM1->EGR = TIM_EGR_UG;

            /* 시작 timeout과 2초 제한은 충전이 아닌 실제 정류 시작 기준이다. */
            Motor_ResetTiming();
            if (Motor_EnableChecked() != 0U)
            {
                motor_diag_stage = 5U;
            }
        }
        goto update_exit;
    }

    if(motor_state == MOTOR_STARTING)
    {
        if((uint32_t)(now - start_tick) >= MOTOR_START_TIMEOUT_MS)
        {
            Motor_Trip(MOTOR_FAULT_START_TIMEOUT);
        }
    }
    else if(motor_state == MOTOR_RUNNING)
    {
        if((uint32_t)(now - last_hall_tick) >= MOTOR_STALL_TIMEOUT_MS)
        {
            Motor_Trip(MOTOR_FAULT_STALL);
        }
    }

    /* 첫 구동 시험: Hall 변화가 계속되어도 정류 시작 2초 후 출력을 끈다.
     * 메인 루프와 SysTick이 실행 중이어야 하므로 구동 중 디버거로 멈추지 않는다.
     * 위에서 발생한 Fault 상태와 원인은 그대로 보존한다.
     */
    if ((motor_state == MOTOR_STARTING) ||
        (motor_state == MOTOR_RUNNING))
    {
        if ((uint32_t)(now - start_tick) >= MOTOR_TEST_MAX_RUN_MS)
        {
            Motor_Stop();
        }
    }

update_exit:
    __set_PRIMASK(primask);
}

uint8_t Motor_ClearFault(void)
{
    uint8_t cleared = 0U;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    /* FAULT 상태에서만 해제 처리 */
    if(motor_state != MOTOR_FAULT)
    {
        goto exit;
    }

    TIM1_PWM_Disable();

    /* 하드웨어 Break 활성화, 자동 재시작 금지 확인 */
    if(((TIM1->BDTR & TIM_BDTR_BKE) == 0U) || ((TIM1->BDTR & TIM_BDTR_AOE) != 0U))
    {
        goto exit;
    }

    /* 현재 과전류 신호가 있으면 해제 금지 */
    if(((COMP1->CSR | COMP2->CSR | COMP4->CSR) & COMP_CSR_VALUE) != 0U)
    {
        goto exit;
    }

    uint8_t current_hall = HallSensor_Read();

    if((current_hall == 0U) || (current_hall >= 7U))
    {
        goto exit;
    }

    /* 이전 Break 기록만 제거 */
    TIM1->SR = (uint32_t)~TIM_SR_BIF;

    /* 해제 도중 Break가 다시 발생했는지 확인 */
    if (((TIM1->SR & TIM_SR_BIF) != 0U) ||
        (((COMP1->CSR | COMP2->CSR | COMP4->CSR)
          & COMP_CSR_VALUE) != 0U))
    {
        goto exit;
    }

    /* Fault가 확실히 해제되기 전, 이전 COMP 에지 기록만 정리한다.
     * 정리 중 다시 Break가 걸리면 FAULT 상태를 유지한다. */
    Comp_ClearFaultEdgeCapture();
    if (((TIM1->SR & TIM_SR_BIF) != 0U) ||
        (((COMP1->CSR | COMP2->CSR | COMP4->CSR)
          & COMP_CSR_VALUE) != 0U))
    {
        goto exit;
    }

    previous_hall = 0U;
    start_tick = 0U;
    last_hall_tick = 0U;
    startup_phase_tick = 0U;
    bootstrap_phase = 0U;
    bootstrap_output_on = 0U;

    motor_fault_flags = MOTOR_FAULT_NONE;
    motor_state = MOTOR_STOPPED;

    /* 저장한 Fault 기록은 보존하고 현재 진행 단계만 초기화한다. */
    motor_diag_stage = 0U;

    /* 저장된 motor_fault_comp_edges 값은 다음 Fault까지 유지한다. */

    /* Break ISR에서 껐던 인터럽트 복구 */
    TIM1->DIER |= TIM_DIER_BIE;

    cleared = 1U;

    exit:
        __set_PRIMASK(primask);
        return cleared;
}
