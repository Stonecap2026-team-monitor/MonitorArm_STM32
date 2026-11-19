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

#define THETA1_ZERO_OFFSET_DEG    343.47f //측정 후 병경

#define ENCODER_I2C_RECOVERY_ERROR_COUNT    3U
#define ENCODER_I2C_RECOVERY_DELAY_MS       20U

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


static uint8_t Encoder_RecoverI2C(void)
{

    if (HAL_I2C_DeInit(&hi2c1) != HAL_OK)
    {
        return 0U;
    }

    osDelay(ENCODER_I2C_RECOVERY_DELAY_MS);

    MX_I2C1_Init();

    osDelay(ENCODER_I2C_RECOVERY_DELAY_MS);

    encoderRecoveryCount++;

    return 1U;
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
                (void)Encoder_RecoverI2C();

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
