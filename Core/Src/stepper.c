#include "stepper.h"
#include "cmsis_os.h"
#include "gpio.h"
#include "tim.h"
#include "encoderTask.h"
#include <math.h>

/*
 * 기존 하드웨어 테스트에서 사용한 논리 레벨.
 * 실제 TB6600 배선 방식에 따라 반대로 바뀔 수 있다.
 */
#define STEPPER_DIR_CW_LEVEL         GPIO_PIN_RESET //실험을 통해 반전입력인걸 알음.
#define STEPPER_DIR_CCW_LEVEL        GPIO_PIN_SET

#define STEPPER_ENABLE_LEVEL         GPIO_PIN_RESET
#define STEPPER_DISABLE_LEVEL        GPIO_PIN_SET

/*
 * 1.8도 스텝모터:
 * 360 / 1.8 = 200 full steps/rev
 *
 * TB6600 마이크로스텝 설정:
 * 1/8 microstep
 *
 * 200 × 8 = 1600 pulse/rev
 */
#define STEPPER_FULL_STEPS_PER_REV   200U
#define STEPPER_MICROSTEP_DIVISION   8U
#define STEPPER_GEAR_RATIO           40U
#define STEPPER_PULSES_PER_OUTPUT_REV		 (STEPPER_FULL_STEPS_PER_REV * STEPPER_MICROSTEP_DIVISION * STEPPER_GEAR_RATIO)

/*
 * TIM3 설정
 *
 * Timer clock = 90 MHz
 * PSC = 89
 * Counter clock = 1 MHz
 *
 * ARR/CCR preload를 사용하여 다음 주기 경계에서 함께 갱신한다.
 * 속도 단위는 출력축 RPM이 아니라 STEP pulse/s이다.
 */
#define STEPPER_TIMER_CHANNEL        TIM_CHANNEL_1
#define STEPPER_TIMER_COUNTER_HZ     1000000U
#define STEPPER_START_SPEED_HZ       200.0f
#define STEPPER_MAX_SPEED_HZ         1000.0f
#define STEPPER_ACCEL_HZ_PER_SEC     1000.0f

static volatile uint32_t stepCount = 0U;
static volatile uint32_t targetStepCount = 0U;
static volatile uint8_t stepperBusy = 0U;
volatile StepperTrace_t debugStepperTrace = {0};
volatile uint8_t stepperAngleLimitHit = 0U;
extern volatile uint8_t motorCommLostRequest;
extern volatile uint8_t motorEstopRequest;

/* pulseIndex는 0부터 시작. 짧은 이동은 자동으로 삼각형 프로파일이 된다. */
static uint32_t Stepper_ProfilePeriod(uint32_t pulseIndex, uint32_t total)
{
    uint32_t remaining = total - 1U - pulseIndex;
    uint32_t rampSteps = (pulseIndex < remaining) ? pulseIndex : remaining;
    float speed = sqrtf(STEPPER_START_SPEED_HZ * STEPPER_START_SPEED_HZ +
                        2.0f * STEPPER_ACCEL_HZ_PER_SEC * (float)rampSteps);
    uint32_t period;

    if (speed > STEPPER_MAX_SPEED_HZ)
    {
        speed = STEPPER_MAX_SPEED_HZ;
    }
    period = (uint32_t)((float)STEPPER_TIMER_COUNTER_HZ / speed + 0.5f);
    /* TIM3는 16비트 타이머. CCR는 항상 ARR보다 작게 유지한다. */
    if (period > 65536U) period = 65536U;
    if (period < 2U) period = 2U;
    return period;
}

static void Stepper_SetPeriod(uint32_t period)
{
    __HAL_TIM_SET_AUTORELOAD(&htim3, period - 1U);
    __HAL_TIM_SET_COMPARE(&htim3, STEPPER_TIMER_CHANNEL, period / 2U);
}

static uint8_t Stepper_IsEstopActive(void)
{
    if (motorEstopRequest != 0U)
    {
        return 1U;
    }

    /*
     * 현재 실제 ESTOP 구성은 HIGH = 눌림.
     * CLEAR_ERROR에서도 동일한 기준을 사용 중.
     */
    if (HAL_GPIO_ReadPin(ESTOP_GPIO_Port,
                         ESTOP_Pin) == GPIO_PIN_RESET)
    {
        return 1U;
    }

    return 0U;
}

void Stepper_EmergencyDisableFromISR(void)
{
    Stepper_Disable();
}

void Stepper_EmergencyStop(void)
{
    if (stepperBusy != 0U) {
        debugStepperTrace.stopTick = HAL_GetTick();
        debugStepperTrace.stopReason = 2U;
    }
    (void)HAL_TIM_PWM_Stop_IT(&htim3,
                              STEPPER_TIMER_CHANNEL);

    __HAL_TIM_SET_COMPARE(&htim3,
                          STEPPER_TIMER_CHANNEL,
                          0U);

    Stepper_Disable();

    stepCount = 0U;
    targetStepCount = 0U;
    stepperBusy = 0U;
}


static void Stepper_SetDirection(StepperDirection_t direction)
{
    GPIO_PinState gpioLevel;

    if (direction == STEPPER_DIRECTION_CW)
    {
        gpioLevel = STEPPER_DIR_CW_LEVEL;
    }
    else
    {
        gpioLevel = STEPPER_DIR_CCW_LEVEL;
    }

    HAL_GPIO_WritePin(DIR_GPIO_Port,
                      DIR_Pin,
                      gpioLevel);
}

void Stepper_Enable(void)
{
    HAL_GPIO_WritePin(ENA_GPIO_Port, ENA_Pin, STEPPER_ENABLE_LEVEL);
}

void Stepper_Disable(void)
{
    HAL_GPIO_WritePin(ENA_GPIO_Port, ENA_Pin, STEPPER_DISABLE_LEVEL);
}

uint32_t Stepper_AngleToSteps(float angle_deg)
{
    float calculatedSteps;

    if (!isfinite(angle_deg) || angle_deg <= 0.0f)
    {
        return 0U;
    }

    calculatedSteps =
        (angle_deg * (float)STEPPER_PULSES_PER_OUTPUT_REV) /
        360.0f;

    if (!isfinite(calculatedSteps) || calculatedSteps >= 4294967296.0f)
    {
        return 0U;
    }
    /* 가장 가까운 정수 펄스 수로 반올림 */
    return (uint32_t)(calculatedSteps + 0.5f);
}


void Stepper_HandlePulseFinished(void)
{
    if (stepperBusy == 0U)
    {
        return;
    }

    /*
     * Heartbeat timeout 발생 시
     * 다음 STEP PWM 인터럽트에서 즉시 정지
     */
    if ((motorCommLostRequest != 0U) || (Stepper_IsEstopActive() != 0U))
    {
        Stepper_EmergencyStop();
        return;
    }

    /* Check the latest raw measurement on every pulse. Falling motion is
     * permitted so a new command can return from above the limit. */
    if (debugStepperTrace.directionSign > 0 && encoderStatus == AS5600_OK &&
        currentEncoderAngleDeg >= THETA1_MEASURED_STOP_DEG)
    {
        stepperAngleLimitHit = 1U;
        Stepper_EmergencyStop();
        debugStepperTrace.stopReason = 4U;
        return;
    }

    stepCount++;

    if (stepCount >= targetStepCount)
    {
        (void)HAL_TIM_PWM_Stop_IT(
            &htim3,
            STEPPER_TIMER_CHANNEL
        );

        __HAL_TIM_SET_COMPARE(
            &htim3,
            STEPPER_TIMER_CHANNEL,
            0U
        );

        Stepper_Disable();

        debugStepperTrace.stopTick = HAL_GetTick();
        debugStepperTrace.stopReason = 1U;
        stepCount = 0U;
        targetStepCount = 0U;
        stepperBusy = 0U;
        return;
    }

    /* CC1 콜백은 현재 펄스의 HIGH 구간 종료 시 호출된다.
     * 다음 펄스의 주기를 preload에 기록하며 CNT/UG는 건드리지 않는다.
     */
    uint32_t period = Stepper_ProfilePeriod(stepCount, targetStepCount);
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    /* 계산 중 ESTOP이 선점하여 정지했으면 타이머를 다시 갱신하지 않는다. */
    if (stepperBusy != 0U)
    {
        Stepper_SetPeriod(period);
    }
    __set_PRIMASK(primask);
}

StepperStatus_t Stepper_MoveRelative(float angle_deg)
{
    uint32_t requestedSteps;
    StepperDirection_t direction;
    HAL_StatusTypeDef halStatus;

    if (stepperBusy != 0U)
    {
        return STEPPER_ERROR_BUSY;
    }
    if (Stepper_IsEstopActive() != 0U)
    {
        Stepper_EmergencyStop();
        return STEPPER_ERROR_ESTOP;
    }

    if (!isfinite(angle_deg) || angle_deg == 0.0f)
    {
        return STEPPER_ERROR_INVALID_ANGLE;
    }

    if (angle_deg > 0.0f)
    {
        if (encoderStatus == AS5600_OK &&
            currentEncoderAngleDeg >= THETA1_MEASURED_STOP_DEG)
        {
            stepperAngleLimitHit = 1U;
            return STEPPER_ERROR_ANGLE_LIMIT;
        }
        direction = STEPPER_DIRECTION_CW;
    }
    else
    {
        direction = STEPPER_DIRECTION_CCW;

        angle_deg = -angle_deg;
    }

    requestedSteps = Stepper_AngleToSteps(angle_deg);

    if (requestedSteps == 0U){
        return STEPPER_ERROR_INVALID_ANGLE;
    }

    debugStepperTrace.moveId++;
    debugStepperTrace.directionSign = (direction == STEPPER_DIRECTION_CW) ? 1 : -1;
    debugStepperTrace.requestedSteps = requestedSteps;
    debugStepperTrace.startTick = HAL_GetTick();
    debugStepperTrace.stopTick = 0U;
    debugStepperTrace.stopReason = 0U;
    Stepper_SetDirection(direction);


    Stepper_Enable();

    if (Stepper_IsEstopActive() != 0U)
    {
        Stepper_EmergencyStop();
        return STEPPER_ERROR_ESTOP;
    }

    stepCount = 0U;
    targetStepCount = requestedSteps;
    stepperBusy = 1U;

    /* 정지 상태에서 첫 주기를 실제 레지스터에 반영한다.
     * 이전 이동의 ARR/CCR나 인터럽트 플래그가 남지 않도록 초기화한다.
     */
    SET_BIT(htim3.Instance->CR1, TIM_CR1_ARPE);
    SET_BIT(htim3.Instance->CCMR1, TIM_CCMR1_OC1PE);
    Stepper_SetPeriod(Stepper_ProfilePeriod(0U, requestedSteps));
    htim3.Instance->EGR = TIM_EGR_UG;
    __HAL_TIM_SET_COUNTER(&htim3, 0U);
    __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE | TIM_FLAG_CC1);

    halStatus = HAL_TIM_PWM_Start_IT(&htim3, STEPPER_TIMER_CHANNEL);

    /*
     * Enable 이후 ~ PWM Start 사이에
     * ESTOP이 발생한 경우까지 최종 방어.
     */
    if (Stepper_IsEstopActive() != 0U)
    {
        Stepper_EmergencyStop();
        return STEPPER_ERROR_ESTOP;
    }

    if (halStatus != HAL_OK)
    {
        debugStepperTrace.stopTick = HAL_GetTick();
        debugStepperTrace.stopReason = 3U;
    	Stepper_Disable();

    	__HAL_TIM_SET_COMPARE(&htim3, STEPPER_TIMER_CHANNEL, 0U);

        stepCount = 0U;
        targetStepCount = 0U;
        stepperBusy = 0U;

        return STEPPER_ERROR_HAL;
    }

    return STEPPER_OK;
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if ((htim->Instance == TIM3) &&
        (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1))
    {
        Stepper_HandlePulseFinished();
    }
}

void Stepper_Init(void)
{
    Stepper_Disable();

    stepCount = 0U;
    targetStepCount = 0U;
    stepperBusy = 0U;
}

uint8_t Stepper_IsBusy(void)
{
    return stepperBusy;
}

uint32_t Stepper_GetCurrentStepCount(void)
{
    return stepCount;
}

uint32_t Stepper_GetTargetStepCount(void)
{
    return targetStepCount;
}
