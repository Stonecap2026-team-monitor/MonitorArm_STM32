/*
 * servo.c
 *
 *  Created on: 2026. 8. 4.
 *      Author: wowns
 */


#include "servo.h"
#include "cmsis_os.h"
#include "tim.h"

#define SERVO_MIN_ANGLE_DEG      (-135.0f)
#define SERVO_MAX_ANGLE_DEG      135.0f

#define SERVO_MIN_PULSE_US       500U
#define SERVO_NEUTRAL_PULSE_US   1500U
#define SERVO_MAX_PULSE_US       2500U

#define SERVO_THETA2_ZERO_OFFSET_DEG (-16.0f)
#define SERVO_THETA3_ZERO_OFFSET_DEG (-16.0f)

#define SERVO_THETA2_CHANNEL     TIM_CHANNEL_1
#define SERVO_THETA3_CHANNEL     TIM_CHANNEL_2

#define SERVO_UPDATE_PERIOD_MS       20U
#define SERVO_SPEED_DEG_PER_SEC      20.0f //서보 속도 조저
#define SERVO_STEP_DEG               (SERVO_SPEED_DEG_PER_SEC * ((float)SERVO_UPDATE_PERIOD_MS / 1000.0f))

extern volatile uint8_t motorEstopRequest;
extern volatile uint8_t motorCommLostRequest;

static float servoTheta2CurrentDeg = 0.0f;
static float servoTheta3CurrentDeg = 0.0f;




static float Servo_MoveToward(float current_deg,
                              float target_deg)
{
    if (current_deg < target_deg)
    {
        current_deg += SERVO_STEP_DEG;

        if (current_deg > target_deg)
        {
            current_deg = target_deg;
        }
    }
    else if (current_deg > target_deg)
    {
        current_deg -= SERVO_STEP_DEG;

        if (current_deg < target_deg)
        {
            current_deg = target_deg;
        }
    }

    return current_deg;
}

uint16_t Servo_AngleToPulse(float angle_deg)
{
    float real_angle_deg;
	float pulse_us;

    if (angle_deg <= SERVO_MIN_ANGLE_DEG)
    {
        return SERVO_MIN_PULSE_US;
    }

    if (angle_deg >= SERVO_MAX_ANGLE_DEG)
    {
        return SERVO_MAX_PULSE_US;
    }

    real_angle_deg = 135.0f + angle_deg;

    pulse_us = 500.0f + (real_angle_deg / 270.0f)*2000.0f;

    return (uint16_t)(pulse_us + 0.5f);
}

ServoStatus_t Servo_Init(void)
{
    HAL_StatusTypeDef status;

    __HAL_TIM_SET_COMPARE(&htim4,
                          SERVO_THETA2_CHANNEL,
                          Servo_AngleToPulse(SERVO_THETA2_ZERO_OFFSET_DEG));

    __HAL_TIM_SET_COMPARE(&htim4,
                          SERVO_THETA3_CHANNEL,
                          Servo_AngleToPulse(SERVO_THETA3_ZERO_OFFSET_DEG));

    status =
        HAL_TIM_PWM_Start(&htim4,
                          SERVO_THETA2_CHANNEL);

    if (status != HAL_OK)
    {
        return SERVO_ERROR_HAL;
    }

    status =
        HAL_TIM_PWM_Start(&htim4,
                          SERVO_THETA3_CHANNEL);

    if (status != HAL_OK)
    {
        (void)HAL_TIM_PWM_Stop(&htim4,
                               SERVO_THETA2_CHANNEL);

        return SERVO_ERROR_HAL;
    }

    return SERVO_OK;
}

ServoStatus_t Servo_SetAngle(ServoChannel_t channel,
                             float angle_deg)
{
    uint16_t pulse_us;
    uint32_t timer_channel;

    if ((angle_deg < SERVO_MIN_ANGLE_DEG) ||
        (angle_deg > SERVO_MAX_ANGLE_DEG))
    {
        return SERVO_ERROR_INVALID_ANGLE;
    }

    if (channel == SERVO_CHANNEL_THETA2)
    {
        timer_channel = SERVO_THETA2_CHANNEL;
    }
    else if (channel == SERVO_CHANNEL_THETA3)
    {
        timer_channel = SERVO_THETA3_CHANNEL;
    }
    else
    {
        return SERVO_ERROR_INVALID_CHANNEL;
    }

    float corrected_angle = angle_deg;

    if (channel == SERVO_CHANNEL_THETA2)
    {
        corrected_angle = -angle_deg
                        + SERVO_THETA2_ZERO_OFFSET_DEG;
    }
    else if (channel == SERVO_CHANNEL_THETA3)
    {
        corrected_angle = angle_deg
                        + SERVO_THETA3_ZERO_OFFSET_DEG;
    }

    if ((corrected_angle < SERVO_MIN_ANGLE_DEG) ||
        (corrected_angle > SERVO_MAX_ANGLE_DEG))
    {
        return SERVO_ERROR_INVALID_ANGLE;
    }

    pulse_us = Servo_AngleToPulse(corrected_angle);

    __HAL_TIM_SET_COMPARE(&htim4,
                          timer_channel,
                          pulse_us);

    if (channel == SERVO_CHANNEL_THETA2)
    {
        servoTheta2CurrentDeg = angle_deg;
    }
    else
    {
        servoTheta3CurrentDeg = angle_deg;
    }

    return SERVO_OK;
}

ServoStatus_t Servo_MoveSmooth(float theta2_deg,
                               float theta3_deg)
{
    ServoStatus_t status;
    float nextTheta2Deg;
    float nextTheta3Deg;

    if ((theta2_deg < SERVO_MIN_ANGLE_DEG) ||
        (theta2_deg > SERVO_MAX_ANGLE_DEG) ||
        (theta3_deg < SERVO_MIN_ANGLE_DEG) ||
        (theta3_deg > SERVO_MAX_ANGLE_DEG))
    {
        return SERVO_ERROR_INVALID_ANGLE;
    }

    while ((servoTheta2CurrentDeg != theta2_deg) ||
           (servoTheta3CurrentDeg != theta3_deg))
    {
        /* ESTOP 최우선 */
        if (motorEstopRequest != 0U)
        {
            return SERVO_ERROR_ESTOP;
        }

        /* Heartbeat timeout이 이미 검출된 경우 */
        if (motorCommLostRequest != 0U)
        {
            return SERVO_ERROR_COMM_LOST;
        }

        nextTheta2Deg =
            Servo_MoveToward(
                servoTheta2CurrentDeg,
                theta2_deg
            );

        nextTheta3Deg =
            Servo_MoveToward(
                servoTheta3CurrentDeg,
                theta3_deg
            );

        status =
            Servo_SetAngle(
                SERVO_CHANNEL_THETA2,
                nextTheta2Deg
            );

        if (status != SERVO_OK)
        {
            return status;
        }

        status =
            Servo_SetAngle(
                SERVO_CHANNEL_THETA3,
                nextTheta3Deg
            );

        if (status != SERVO_OK)
        {
            return status;
        }

        osDelay(SERVO_UPDATE_PERIOD_MS);
    }

    return SERVO_OK;
}

void Servo_GetCurrentAngles(float *theta2_deg,
                            float *theta3_deg)
{
    if (theta2_deg != NULL)
    {
        *theta2_deg = servoTheta2CurrentDeg;
    }

    if (theta3_deg != NULL)
    {
        *theta3_deg = servoTheta3CurrentDeg;
    }
}
