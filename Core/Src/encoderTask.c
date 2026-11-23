/*
 * encoderTask.c
 *
 *  Created on: 2026. 8. 4.
 *      Author: wowns
 */

#include "encoderTask.h"

#include "cmsis_os.h"
#include "as5600.h"
#include "i2c.h"


#define ENCODER_TASK_PERIOD_MS    20U
#define ENCODER_MOVING_AVERAGE_SAMPLES 3U

#define THETA1_ZERO_OFFSET_DEG    13.44f //수평 자세의 엔코더 절대각

#define ENCODER_I2C_RECOVERY_ERROR_COUNT    3U
#define ENCODER_I2C_RECOVERY_DELAY_MS       20U
#define ENCODER_I2C_RECOVERY_RETRY_MS       250U

volatile float currentEncoderAbsoluteAngleDeg = 0.0f;
volatile float currentEncoderAngleDeg = 0.0f;
/* Latest three-sample average of the zero-calibrated angle for motor control. */
volatile float filteredEncoderAngleDeg = 0.0f;
volatile uint8_t encoderMovingAverageReady = 0U;
volatile uint8_t encoderMovingAverageCount = 0U;

volatile AS5600_Status_t encoderStatus = AS5600_ERROR_I2C;
volatile uint32_t encoderReadCount = 0U;
volatile uint32_t encoderErrorCount = 0U;

volatile uint32_t encoderHalError = HAL_I2C_ERROR_NONE;
volatile HAL_I2C_StateTypeDef encoderI2cState;

volatile uint32_t encoderRecoveryCount = 0U;
/* Success requires a subsequent sensor read, not merely HAL initialization. */
volatile uint32_t encoderRecoverySuccessCount = 0U;
volatile uint32_t encoderRecoveryFailureCount = 0U;
/* 0=none, 1=read restored, 2=SCL low, 3=SDA low, 4=HAL init/deinit,
 * 5=read still failed, 6=waiting for verification. */
volatile uint32_t encoderRecoveryResult = 0U;
volatile uint32_t encoderRecoveryLines = 0U; /* PB8/PB9 IDR masked by 0x300 */
static uint8_t encoderRecoveryPending = 0U;

static uint32_t encoderConsecutiveErrorCount = 0U;
static float encoderAngleWindow[ENCODER_MOVING_AVERAGE_SAMPLES];
static uint8_t encoderAngleWindowIndex = 0U;

static float Encoder_WrapRelativeAngle(float angleDeg)
{
    while (angleDeg > 180.0f) angleDeg -= 360.0f;
    while (angleDeg < -180.0f) angleDeg += 360.0f;
    return angleDeg;
}

static void Encoder_ResetMovingAverage(void)
{
    encoderAngleWindowIndex = 0U;
    encoderMovingAverageCount = 0U;
    encoderMovingAverageReady = 0U;
}

static void Encoder_UpdateMovingAverage(float relativeAngleDeg)
{
    float sum = 0.0f;
    uint8_t i;

    encoderAngleWindow[encoderAngleWindowIndex] = relativeAngleDeg;
    encoderAngleWindowIndex =
        (uint8_t)((encoderAngleWindowIndex + 1U) % ENCODER_MOVING_AVERAGE_SAMPLES);

    if (encoderMovingAverageCount < ENCODER_MOVING_AVERAGE_SAMPLES)
    {
        encoderMovingAverageCount++;
    }
    if (encoderMovingAverageCount != ENCODER_MOVING_AVERAGE_SAMPLES)
    {
        return;
    }

    /* Average shortest angular differences to avoid the +/-180 deg wrap. */
    for (i = 0U; i < ENCODER_MOVING_AVERAGE_SAMPLES; i++)
    {
        sum += Encoder_WrapRelativeAngle(
            encoderAngleWindow[i] - relativeAngleDeg);
    }

    filteredEncoderAngleDeg = Encoder_WrapRelativeAngle(
        relativeAngleDeg + sum / (float)ENCODER_MOVING_AVERAGE_SAMPLES);
    encoderMovingAverageReady = 1U;
}


static uint8_t Encoder_WaitLineHigh(uint16_t pin)
{
    /* Bounded wait for line release/clock stretching; never drive HIGH. */
    for (uint8_t wait = 0U; wait < 10U; wait++)
    {
        if (HAL_GPIO_ReadPin(GPIOB, pin) == GPIO_PIN_SET) return 1U;
        osDelay(1U);
    }
    return HAL_GPIO_ReadPin(GPIOB, pin) == GPIO_PIN_SET;
}

static uint8_t Encoder_RecoverI2C(void)
{
    GPIO_InitTypeDef gpio = {0};
    uint8_t cleared = 1U;
    encoderRecoveryCount++;
    encoderRecoveryPending = 0U;
    encoderRecoveryResult = 6U;
    Encoder_ResetMovingAverage();
    if (HAL_I2C_DeInit(&hi2c1) != HAL_OK)
    {
        encoderRecoveryResult = 4U;
        encoderRecoveryFailureCount++;
        return 0U;
    }

    /* I2C1 is used only by this task. GPIO open-drain releases the lines
     * through their existing external pull-ups; it never drives them HIGH.
     * NXP UM10204 3.1.16: at most nine clocks to clear SDA held LOW. */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_SET);
    gpio.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    gpio.Mode = GPIO_MODE_OUTPUT_OD;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &gpio);
    if (!Encoder_WaitLineHigh(GPIO_PIN_8))
    {
        encoderRecoveryResult = 2U;
        cleared = 0U;
    }
    for (uint8_t pulse = 0U; cleared && pulse < 9U &&
         HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == GPIO_PIN_RESET; pulse++)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
        osDelay(1U);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
        if (!Encoder_WaitLineHigh(GPIO_PIN_8))
        {
            encoderRecoveryResult = 2U;
            cleared = 0U;
            break;
        }
        osDelay(1U);
    }
    if (cleared)
    {
        /* STOP: lower SCL first, lower SDA, release SCL, then release SDA. */
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
        osDelay(1U);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
        osDelay(1U);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
        if (!Encoder_WaitLineHigh(GPIO_PIN_8))
        {
            encoderRecoveryResult = 2U;
            cleared = 0U;
        }
        osDelay(1U);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
        if (cleared && !Encoder_WaitLineHigh(GPIO_PIN_9))
        {
            encoderRecoveryResult = 3U;
            cleared = 0U;
        }
    }
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_SET);
    encoderRecoveryLines = GPIOB->IDR & (GPIO_PIN_8 | GPIO_PIN_9);

    /* Reset the STM peripheral as well. HAL init restores AF4 GPIO via MSP.
     * Avoid MX_I2C1_Init's fatal Error_Handler on a failed recovery. */
    __HAL_RCC_I2C1_CLK_ENABLE();
    __HAL_RCC_I2C1_FORCE_RESET();
    __HAL_RCC_I2C1_RELEASE_RESET();
    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    {
        encoderRecoveryResult = 4U;
        cleared = 0U;
    }
    osDelay(ENCODER_I2C_RECOVERY_DELAY_MS);
    if (!cleared)
    {
        encoderRecoveryFailureCount++;
        return 0U;
    }
    encoderRecoveryPending = 1U;
    return 1U; /* Verified by the next ordinary sensor read. */
}

static float Encoder_CalculateRelativeAngle(float absoluteAngleDeg)
{
    float relativeAngleDeg;

    relativeAngleDeg =
        absoluteAngleDeg - THETA1_ZERO_OFFSET_DEG;

    if (relativeAngleDeg > 180.0f)
    {
        relativeAngleDeg -= 360.0f;
    }
    else if (relativeAngleDeg < -180.0f)
    {
        relativeAngleDeg += 360.0f;
    }
    return relativeAngleDeg;
}

void StartEncoderTask(void *argument)
{
    float absoluteAngleDeg;
    AS5600_Status_t status;

    (void)argument;

    for (;;)
    {
        status = AS5600_ReadAngleDeg(&absoluteAngleDeg);

        encoderStatus = status;

        if (status == AS5600_OK)
        {
            if (encoderRecoveryPending)
            {
                encoderRecoveryPending = 0U;
                encoderRecoverySuccessCount++;
                encoderRecoveryResult = 1U;
            }
            currentEncoderAbsoluteAngleDeg = absoluteAngleDeg;

            currentEncoderAngleDeg =
                Encoder_CalculateRelativeAngle(absoluteAngleDeg);

            Encoder_UpdateMovingAverage(currentEncoderAngleDeg);
            encoderReadCount++;

            /*
             * 정상 읽기 성공 시
             * 연속 오류 횟수 초기화
             */
            encoderConsecutiveErrorCount = 0U;

            encoderHalError = HAL_I2C_ERROR_NONE;
            encoderI2cState = HAL_I2C_GetState(&hi2c1);
        }
        else
        {
            if (encoderRecoveryPending)
            {
                encoderRecoveryPending = 0U;
                encoderRecoveryFailureCount++;
                encoderRecoveryResult = 5U;
            }
            encoderErrorCount++;
            encoderConsecutiveErrorCount++;
            /* Do not average measurements separated by a failed read. */
            Encoder_ResetMovingAverage();

            encoderHalError =
                HAL_I2C_GetError(&hi2c1);

            encoderI2cState =
                HAL_I2C_GetState(&hi2c1);

            /*
             * 연속 3회 I2C 읽기 실패 시 자동 복구
             */
            if (encoderConsecutiveErrorCount >=
                ENCODER_I2C_RECOVERY_ERROR_COUNT)
            {
                if (!Encoder_RecoverI2C())
                    osDelay(ENCODER_I2C_RECOVERY_RETRY_MS);

                /*
                 * 재초기화 후 다시 3회 연속 실패해야
                 * 다음 recovery를 수행하도록 초기화
                 */
                encoderConsecutiveErrorCount = 0U;
            }
        }

        osDelay(ENCODER_TASK_PERIOD_MS);
    }
}
