/*
 * motorTask.c
 *
 *  Created on: 2026. 8. 2.
 *      Author: wowns
 */


#include "motorTask.h"
#include "stepper.h"
#include "cmsis_os.h"
#include "motorCommand.h"
#include "servo.h"
#include "encoderTask.h"
#include "communicationTask.h"
#include "main.h"

extern osMessageQueueId_t motorCommandQueueHandle;
extern osMessageQueueId_t motorStatusQueueHandle;


#define THETA1_MIN_X10   (-450)
#define THETA1_MAX_X10    900

#define THETA2_MIN_X10   (-1050)
#define THETA2_MAX_X10   1050

#define THETA3_MIN_X10   (-1050)
#define THETA3_MAX_X10   1050

#define MOTOR_INIT_THETA2_DEG    0.0f
#define MOTOR_INIT_THETA3_DEG    0.0f

#define THETA1_POSITION_TOLERANCE_DEG       0.4f //목포값
#define THETA1_MAX_CORRECTION_DEG           5.0f //최대 보정 각도

#define THETA1_SETTLING_TIME_MS             150U //노이즈 떄문에 모터 끄고 150ms기다린 다음 시작
#define THETA1_MAX_CORRECTION_COUNT 		3U

#define THETA1_ENCODER_STABLE_SAMPLES       5U //5개 측정해서 평균
#define THETA1_ENCODER_STABLE_TOLERANCE_DEG 0.3f
#define THETA1_ENCODER_TIMEOUT_MS            1000U
#define MOTOR_TASK_QUEUE_WAIT_MS    10U
static float commandedTheta1Deg = 0.0f; // 마지막으로 명령한 theta1 목표각
static float commandedTheta2Deg = 0.0f;
static float commandedTheta3Deg = 0.0f;

static float homeTheta1Deg = 0.0f;
static float homeTheta2Deg = 0.0f;
static float homeTheta3Deg = 0.0f;

static uint8_t homeValid = 0U;
static void Motor_ReportError(MotorError_t error);
static void Motor_EnterSafeStop(MotorError_t error);
static uint8_t Motor_HandleEstopRequest(void);
static uint8_t Motor_HandleCommLostRequest(void);
/* 디버깅용 변수 */
volatile int16_t debugTheta1 = 0;
volatile int16_t debugTheta2 = 0;
volatile int16_t debugTheta3 = 0;
volatile float debugStableTheta1Deg = 0.0f;
volatile float debugStepperMoveAngleDeg = 0.0f;
volatile float debugStableTheta1AfterMoveDeg = 0.0f;
volatile float debugTheta1PositionErrorDeg = 0.0f;
volatile uint8_t debugTheta1WithinTolerance = 0U;
volatile uint32_t debugTheta1CorrectionCount = 0U;
volatile uint8_t motorCommLostRequest = 0U;
volatile uint8_t motorEstopRequest = 0U;

volatile MotorState_t motorState = MOTOR_STATE_IDLE;

volatile StepperStatus_t debugStepperStatus = STEPPER_OK;
volatile ServoStatus_t debugServo2Status = SERVO_OK;
volatile ServoStatus_t debugServo3Status = SERVO_OK;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == ESTOP_Pin)
    {
        /* STEP PWM까지 즉시 정지 */
        Stepper_EmergencyStop();

        /* MotorTask에서 ERROR latch 처리 */
        motorEstopRequest = 1U;
    }
}


static float Motor_AbsFloat(float value)
{
    if (value < 0.0f)
    {
        return -value;
    }

    return value;
}

static uint8_t Motor_GetStableTheta1Angle(float *stableAngleDeg)
{
    uint32_t startTick;
    uint32_t lastReadCount;
    uint32_t stableCount;

    float currentAngleDeg;
    float previousAngleDeg;
    float angleSum;

    if (stableAngleDeg == NULL)
    {
        return 0U;
    }

    startTick = osKernelGetTickCount();
    lastReadCount = encoderReadCount;

    stableCount = 0U;
    previousAngleDeg = 0.0f;
    angleSum = 0.0f;

    while ((osKernelGetTickCount() - startTick) <
           THETA1_ENCODER_TIMEOUT_MS)
    {
        //정상 측정시 시작
        if (encoderReadCount != lastReadCount)
        {
            lastReadCount = encoderReadCount;

            if (encoderStatus != AS5600_OK)
            {
                stableCount = 0U;
                angleSum = 0.0f;

                osDelay(1U);
                continue;
            }

            currentAngleDeg = currentEncoderAngleDeg;

            if (stableCount == 0U)
            {
                previousAngleDeg = currentAngleDeg;
                angleSum = currentAngleDeg;
                stableCount = 1U;
            }
            else
            {
                if (Motor_AbsFloat(
                        currentAngleDeg - previousAngleDeg) <=
                    THETA1_ENCODER_STABLE_TOLERANCE_DEG)
                {
                    angleSum += currentAngleDeg;
                    stableCount++;

                    previousAngleDeg = currentAngleDeg;

                    if (stableCount >=
                        THETA1_ENCODER_STABLE_SAMPLES)
                    {
                        *stableAngleDeg =
                            angleSum /
                            (float)THETA1_ENCODER_STABLE_SAMPLES;

                        return 1U;
                    }
                }
                else
                {
                    //값이 이상하면 다시.
                    previousAngleDeg = currentAngleDeg;
                    angleSum = currentAngleDeg;
                    stableCount = 1U;
                }
            }
        }

        osDelay(1U);
    }

    return 0U;
}

static uint8_t Motor_CorrectTheta1Position(float targetDeg)
{
    float stableTheta1Deg;
    debugTheta1WithinTolerance = 0U;
    debugTheta1CorrectionCount = 0U;

    while (Stepper_IsBusy() != 0U)
    {
        osDelay(1U);
    }

    if (Motor_HandleEstopRequest() != 0U)
    {
        return 0U;
    }


    if (Motor_HandleCommLostRequest() != 0U)
    {
        return 0U;
    }

    Stepper_Disable();

    osDelay(THETA1_SETTLING_TIME_MS);

    /*
     * 이동 후 안정된 실제 위치 측정
     */
    if (Motor_GetStableTheta1Angle(&stableTheta1Deg) == 0U)
    {
    	Motor_EnterSafeStop(MOTOR_ERROR_ENCODER);
        return 0U;
    }

    debugStableTheta1AfterMoveDeg = stableTheta1Deg;

    debugTheta1PositionErrorDeg =
        targetDeg - stableTheta1Deg;

    /*
     * 허용오차에 들어올 때까지 보정
     */
    while (Motor_AbsFloat(debugTheta1PositionErrorDeg) >
           THETA1_POSITION_TOLERANCE_DEG)
    {
    	if (debugTheta1CorrectionCount >= THETA1_MAX_CORRECTION_COUNT)
    	{
    	    Stepper_Disable();

    	    Motor_EnterSafeStop(
    	        MOTOR_ERROR_ENCODER
    	    );

    	    return 0U;
    	}
    	if (Motor_HandleEstopRequest() != 0U)
    	    {
    	        return 0U;
    	    }

		if (Motor_HandleCommLostRequest() != 0U)
		{
			return 0U;
		}
        /*
         * 비정상적으로 큰 잔여오차는
         * 센서 오류 가능성이 있으므로 따라가지 않음
         */
        if (Motor_AbsFloat(debugTheta1PositionErrorDeg) >
            THETA1_MAX_CORRECTION_DEG)
        {
        	Motor_EnterSafeStop(MOTOR_ERROR_ENCODER);
            return 0U;
        }

        if (Stepper_IsBusy() != 0U)
        {
            debugStepperStatus = STEPPER_ERROR_BUSY;
            Motor_ReportError(MOTOR_ERROR_STEPPER_BUSY);
            return 0U;
        }

        debugStepperMoveAngleDeg =
            debugTheta1PositionErrorDeg;

        debugStepperStatus =
            Stepper_MoveRelative(
                debugTheta1PositionErrorDeg);

        if (debugStepperStatus == STEPPER_ERROR_ESTOP)
        {
            motorEstopRequest = 0U;

            Motor_EnterSafeStop(
                MOTOR_ERROR_ESTOP
            );

            return 0U;
        }

        if (debugStepperStatus != STEPPER_OK)
        {
        	Motor_EnterSafeStop(MOTOR_ERROR_STEPPER);
            return 0U;
        }

        motorState = MOTOR_STATE_MOVING;

        while (Stepper_IsBusy() != 0U)
        {
            osDelay(1U);
        }

        if (Motor_HandleEstopRequest() != 0U)
        {
            return 0U;
        }

        if (Motor_HandleCommLostRequest() != 0U)
        {
            return 0U;
        }

        Stepper_Disable();

        osDelay(THETA1_SETTLING_TIME_MS);

        if (Motor_GetStableTheta1Angle(
                &stableTheta1Deg) == 0U)
        {
        	Motor_EnterSafeStop(MOTOR_ERROR_ENCODER);
            return 0U;
        }

        debugStableTheta1AfterMoveDeg =
            stableTheta1Deg;

        debugTheta1PositionErrorDeg =
            targetDeg - stableTheta1Deg;

        debugTheta1CorrectionCount++;
    }
    if (Motor_HandleEstopRequest() != 0U)
    {
        return 0U;
    }

    if (Motor_HandleCommLostRequest() != 0U)
    {
        return 0U;
    }

    debugTheta1WithinTolerance = 1U;

    return 1U;
}


static uint8_t Motor_IsEncoderReady(void)
{
    if (encoderStatus != AS5600_OK)
    {
        return 0U;
    }

    return 1U;
}

static uint8_t Motor_Init(void)
{
    Stepper_Init();

    if (Servo_Init() != SERVO_OK)
    {
        return 0U;
    }

    if (Servo_SetAngle(SERVO_CHANNEL_THETA2, MOTOR_INIT_THETA2_DEG) != SERVO_OK)
    {
        return 0U;
    }

    if (Servo_SetAngle(SERVO_CHANNEL_THETA3, MOTOR_INIT_THETA3_DEG) != SERVO_OK)
    {
        return 0U;
    }

    commandedTheta1Deg = currentEncoderAngleDeg;
    commandedTheta2Deg = MOTOR_INIT_THETA2_DEG;
    commandedTheta3Deg = MOTOR_INIT_THETA3_DEG;

    return 1U;
}

static uint8_t Motor_IsAngleInRange(MotorAxis_t axis, float angleDeg)
{
    switch (axis)
    {
        case MOTOR_AXIS_THETA1:
            return (angleDeg >= ((float)THETA1_MIN_X10 / 10.0f)) && (angleDeg <= ((float)THETA1_MAX_X10 / 10.0f));

        case MOTOR_AXIS_THETA2:
            return (angleDeg >= ((float)THETA2_MIN_X10 / 10.0f)) && (angleDeg <= ((float)THETA2_MAX_X10 / 10.0f));

        case MOTOR_AXIS_THETA3:
            return (angleDeg >= ((float)THETA3_MIN_X10 / 10.0f)) && (angleDeg <= ((float)THETA3_MAX_X10 / 10.0f));

        default:
            return 0U;
    }
}

static void Motor_ReportError(MotorError_t error)
{
    MotorStatusMessage_t message;

    message.status = MOTOR_STATUS_ERROR;
    message.error = error;
    message.command_type = 0U;

    (void)osMessageQueuePut(
        motorStatusQueueHandle,
        &message,
        0U,
        0U
    );
}

static void Motor_EnterSafeStop(MotorError_t error)
{
    MotorStatusMessage_t message;

    /*
     * Theta1 즉시 안전 정지
     */
    Stepper_EmergencyStop();

    /*
     * 시스템을 ERROR 상태로 고정
     */
    motorState = MOTOR_STATE_ERROR;

    /*
     * Jetson으로 에러 보고
     */
    message.status = MOTOR_STATUS_ERROR;
    message.error = error;
    message.command_type = 0U;

    (void)osMessageQueuePut(
        motorStatusQueueHandle,
        &message,
        0U,
        0U
    );
}

static uint8_t Motor_HandleEstopRequest(void)
{
    if (motorEstopRequest == 0U)
    {
        return 0U;
    }

    motorEstopRequest = 0U;

    Motor_EnterSafeStop(
        MOTOR_ERROR_ESTOP
    );

    return 1U;
}

static uint8_t Motor_HandleCommLostRequest(void)
{
    if (motorCommLostRequest == 0U)
    {
        return 0U;
    }

    motorCommLostRequest = 0U;

    Motor_EnterSafeStop(
        MOTOR_ERROR_COMM_LOST
    );

    return 1U;
}


static void Motor_ReportCommandDone(uint8_t commandType)
{
    MotorStatusMessage_t message;

    message.status = MOTOR_STATUS_COMMAND_DONE;
    message.error = MOTOR_ERROR_NONE;
    message.command_type = commandType;

    if (osMessageQueuePut(motorStatusQueueHandle, &message, 0U, 0U) != osOK){
        motorState = MOTOR_STATE_ERROR;
    }
}

static uint8_t Motor_CheckTarget(const MotorCommand_t *command)
{
    if (command == NULL)
    {
        return 0U;
    }

    if ((command->theta1_x10 < THETA1_MIN_X10) || (command->theta1_x10 > THETA1_MAX_X10))
    {
        return 0U;
    }

    if ((command->theta2_x10 < THETA2_MIN_X10) || (command->theta2_x10 > THETA2_MAX_X10))
    {
        return 0U;
    }

    if ((command->theta3_x10 < THETA3_MIN_X10) || (command->theta3_x10 > THETA3_MAX_X10))
    {
        return 0U;
    }

    return 1U;
}
void Motor_GetCommandedAngles(float *theta1Deg,
                              float *theta2Deg,
                              float *theta3Deg)
{
    if (theta1Deg != NULL)
    {
        *theta1Deg = commandedTheta1Deg;
    }

    if (theta2Deg != NULL)
    {
        *theta2Deg = commandedTheta2Deg;
    }

    if (theta3Deg != NULL)
    {
        *theta3Deg = commandedTheta3Deg;
    }
}
void StartMotorTask(void *argument)
{
    MotorCommand_t command;
    float theta1Deg;
    float theta2Deg;
    float theta3Deg;
    float theta1MoveDeg;
    (void)argument;

    motorState = MOTOR_STATE_INIT;

    if (Motor_Init() == 0U)
    {
    	Motor_EnterSafeStop(MOTOR_ERROR_INIT);
    }
    else
    {
    	motorState = MOTOR_STATE_IDLE;
    }

    for (;;)
    {
    	osStatus_t queueStatus;

    	queueStatus = osMessageQueueGet(
    	    motorCommandQueueHandle,
    	    &command,
    	    NULL,
    	    MOTOR_TASK_QUEUE_WAIT_MS
    	);

    	if (motorEstopRequest != 0U)
    	{
    	    motorEstopRequest = 0U;

    	    Motor_EnterSafeStop(
    	        MOTOR_ERROR_ESTOP
    	    );

    	    continue;
    	}

    	if (motorCommLostRequest != 0U)
    	{
    	    motorCommLostRequest = 0U;

    	    Motor_EnterSafeStop(
    	        MOTOR_ERROR_COMM_LOST
    	    );

    	    continue;
    	}

    	if (queueStatus == osErrorTimeout)
    	{
    	    continue;
    	}

    	if (queueStatus != osOK)
    	{
    	    Motor_EnterSafeStop(
    	        MOTOR_ERROR_QUEUE
    	    );

    	    continue;
    	}

        if ((motorState == MOTOR_STATE_ERROR) &&
            (command.type != MOTOR_COMMAND_CLEAR_ERROR))
        {
            Motor_ReportError(MOTOR_ERROR_SYSTEM_LOCKED);
            continue;
        }

        switch(command.type){
			case MOTOR_COMMAND_SET_TARGET:
			{
				float stableTheta1Deg;
				ServoStatus_t servoSmoothStatus;
				if (Motor_IsEncoderReady() == 0U){
					Motor_EnterSafeStop(MOTOR_ERROR_ENCODER);
				    break;
				}

				if(Motor_CheckTarget(&command) == 0U){
					Motor_ReportError(MOTOR_ERROR_INVALID_TARGET);
					break;
				}

				if (Stepper_IsBusy() != 0U){
				    debugStepperStatus = STEPPER_ERROR_BUSY;
				    Motor_ReportError(MOTOR_ERROR_STEPPER_BUSY);
					break;
				}

				debugTheta1 = command.theta1_x10;
				debugTheta2 = command.theta2_x10;
				debugTheta3 = command.theta3_x10;

				theta2Deg = (float)command.theta2_x10 / 10.0f;
				theta3Deg = (float)command.theta3_x10 / 10.0f;
				theta1Deg = (float)command.theta1_x10 / 10.0f;

				//제거
				if (Motor_GetStableTheta1Angle(&stableTheta1Deg) == 0U)
				{
					Motor_EnterSafeStop(MOTOR_ERROR_ENCODER);
					break;
				}

				debugStableTheta1Deg = stableTheta1Deg;
				//제거

				theta1MoveDeg = theta1Deg - stableTheta1Deg;

				debugStepperMoveAngleDeg = theta1MoveDeg;

				if(theta1MoveDeg != 0.0f){

					debugStepperStatus = Stepper_MoveRelative(theta1MoveDeg);

					if (debugStepperStatus == STEPPER_ERROR_ESTOP)
					{
						motorEstopRequest = 0U;

						Motor_EnterSafeStop(
							MOTOR_ERROR_ESTOP
						);

						break;
					}


					if(debugStepperStatus != STEPPER_OK){
						Motor_EnterSafeStop(MOTOR_ERROR_STEPPER);
						break;
					}

					motorState = MOTOR_STATE_MOVING;
				}

				else{
					debugStepperStatus = STEPPER_OK;
				}

				servoSmoothStatus =
				    Servo_MoveSmooth(theta2Deg, theta3Deg);

				if (servoSmoothStatus == SERVO_ERROR_ESTOP)
				{
				    /*
				     * ESTOP 요청은 여기서 소비.
				     */
				    motorEstopRequest = 0U;

				    Motor_EnterSafeStop(
				        MOTOR_ERROR_ESTOP
				    );

				    break;
				}

				/* 추가 */
				if (servoSmoothStatus == SERVO_ERROR_COMM_LOST)
				{
				    motorCommLostRequest = 0U;

				    Motor_EnterSafeStop(
				        MOTOR_ERROR_COMM_LOST
				    );

				    break;
				}

				if (servoSmoothStatus != SERVO_OK)
				{
					Stepper_Disable();
					Motor_EnterSafeStop(MOTOR_ERROR_SERVO2);
					break;
				}

				commandedTheta2Deg = theta2Deg;
				commandedTheta3Deg = theta3Deg;

				if (Motor_CorrectTheta1Position(theta1Deg) == 0U)
				{
				    break;
				}

				commandedTheta1Deg = theta1Deg;
				motorState = MOTOR_STATE_IDLE;

				Motor_ReportCommandDone((uint8_t)command.type);

				break;
			}

			case MOTOR_COMMAND_SET_HOME:
			{
				float stableTheta1Deg;

				if (Motor_IsEncoderReady() == 0U)
				{
					Motor_EnterSafeStop(MOTOR_ERROR_ENCODER);
					break;
				}

				if (Motor_GetStableTheta1Angle(&stableTheta1Deg) == 0U)
				{
					Motor_EnterSafeStop(MOTOR_ERROR_ENCODER);
					break;
				}

				homeTheta1Deg = stableTheta1Deg;
				homeTheta2Deg = commandedTheta2Deg;
				homeTheta3Deg = commandedTheta3Deg;

				homeValid = 1U;

				motorState = MOTOR_STATE_IDLE;

				Motor_ReportCommandDone((uint8_t)command.type);

				break;
			}

			case MOTOR_COMMAND_MOVE_HOME:
			{
				float stableTheta1Deg;
				float theta1MoveDeg;
				ServoStatus_t servoSmoothStatus;

				if (Motor_IsEncoderReady() == 0U)
				{
					Motor_EnterSafeStop(MOTOR_ERROR_ENCODER);
					break;
				}

				if (homeValid == 0U)
				{
					Motor_ReportError(MOTOR_ERROR_HOME_NOT_SET);
					break;
				}

				if (Stepper_IsBusy() != 0U)
				{
					debugStepperStatus = STEPPER_ERROR_BUSY;
					Motor_ReportError(MOTOR_ERROR_STEPPER_BUSY);
					break;
				}

				/* 이동 전 안정된 실제 Theta1 측정 */
				if (Motor_GetStableTheta1Angle(&stableTheta1Deg) == 0U)
				{
					Motor_EnterSafeStop(MOTOR_ERROR_ENCODER);
					break;
				}

				debugStableTheta1Deg = stableTheta1Deg;

				theta1MoveDeg =
					homeTheta1Deg - stableTheta1Deg;

				debugStepperMoveAngleDeg = theta1MoveDeg;

				if (theta1MoveDeg != 0.0f)
				{
					debugStepperStatus =
						Stepper_MoveRelative(theta1MoveDeg);

					if (debugStepperStatus == STEPPER_ERROR_ESTOP)
					{
						motorEstopRequest = 0U;

						Motor_EnterSafeStop(
							MOTOR_ERROR_ESTOP
						);

						break;
					}

					if (debugStepperStatus != STEPPER_OK)
					{
						Motor_EnterSafeStop(MOTOR_ERROR_STEPPER);
						break;
					}

					motorState = MOTOR_STATE_MOVING;
				}
				else
				{
					debugStepperStatus = STEPPER_OK;
				}

				/* Theta2, Theta3는 제한 속도로 HOME 이동 */
				servoSmoothStatus =
					Servo_MoveSmooth(homeTheta2Deg,
									 homeTheta3Deg);

				if (servoSmoothStatus == SERVO_ERROR_ESTOP)
				{
				    motorEstopRequest = 0U;

				    Motor_EnterSafeStop(
				        MOTOR_ERROR_ESTOP
				    );

				    break;
				}

				if (servoSmoothStatus == SERVO_ERROR_COMM_LOST)
				{
				    motorCommLostRequest = 0U;

				    Motor_EnterSafeStop(
				        MOTOR_ERROR_COMM_LOST
				    );

				    break;
				}

				if (servoSmoothStatus != SERVO_OK)
				{
					Stepper_Disable();
					Motor_EnterSafeStop(MOTOR_ERROR_SERVO2);
					break;
				}

				/*
				 * 서보는 실제로 HOME 명령에 도달했으므로
				 * 여기서 commanded 값을 갱신한다.
				 */
				commandedTheta2Deg = homeTheta2Deg;
				commandedTheta3Deg = homeTheta3Deg;

				if (Motor_CorrectTheta1Position(homeTheta1Deg) == 0U)
				{
				    break;
				}

				commandedTheta1Deg = homeTheta1Deg;

				motorState = MOTOR_STATE_IDLE;

				Motor_ReportCommandDone((uint8_t)command.type);

				break;
			}

			case MOTOR_COMMAND_JOG:
			{
				float deltaDeg;
				float targetDeg;
				uint8_t jogCommandSucceeded = 0U;

				deltaDeg = (float)command.delta_x10 / 10.0f;

				if (deltaDeg == 0.0f){
				        motorState = MOTOR_STATE_IDLE;
				        Motor_ReportCommandDone((uint8_t)command.type);
				        break;
				}

				switch (command.axis){
					case MOTOR_AXIS_THETA1:
					{
						float stableTheta1Deg;
						float theta1MoveDeg;

						if (Motor_IsEncoderReady() == 0U)
						{
							Motor_EnterSafeStop(MOTOR_ERROR_ENCODER);
							break;
						}

						if (Stepper_IsBusy() != 0U)
						{
							debugStepperStatus = STEPPER_ERROR_BUSY;
							Motor_ReportError(MOTOR_ERROR_STEPPER_BUSY);
							break;
						}

						/*
						 * JOG 기준 위치도 단일 측정값이 아니라
						 * 안정된 실제 위치를 사용
						 */
						if (Motor_GetStableTheta1Angle(
								&stableTheta1Deg) == 0U)
						{
							Motor_EnterSafeStop(MOTOR_ERROR_ENCODER);
							break;
						}

						debugStableTheta1Deg = stableTheta1Deg;

						/*
						 * JOG는 현재 실제 위치에서
						 * delta만큼 이동하는 상대 이동
						 */
						targetDeg =
							stableTheta1Deg + deltaDeg;

						if (Motor_IsAngleInRange(
								command.axis,
								targetDeg) == 0U)
						{
							Motor_ReportError(MOTOR_ERROR_ANGLE_LIMIT);
							break;
						}

						theta1MoveDeg =
							targetDeg - stableTheta1Deg;

						debugStepperMoveAngleDeg =
							theta1MoveDeg;

						debugStepperStatus =
							Stepper_MoveRelative(theta1MoveDeg);

						if (debugStepperStatus == STEPPER_ERROR_ESTOP)
						{
						    motorEstopRequest = 0U;

						    Motor_EnterSafeStop(
						        MOTOR_ERROR_ESTOP
						    );

						    break;
						}

						if (debugStepperStatus != STEPPER_OK)
						{
							Motor_EnterSafeStop(MOTOR_ERROR_STEPPER);
							break;
						}

						motorState = MOTOR_STATE_MOVING;

						if (Motor_CorrectTheta1Position(targetDeg) == 0U)
						{
						    break;
						}

						commandedTheta1Deg = targetDeg;
						motorState = MOTOR_STATE_IDLE;
						jogCommandSucceeded = 1U;

						break;
					}
					case MOTOR_AXIS_THETA2:
					{
						targetDeg = commandedTheta2Deg + deltaDeg;

						if (Motor_IsAngleInRange(command.axis, targetDeg) == 0U){
							Motor_ReportError(MOTOR_ERROR_ANGLE_LIMIT);
							break;
						}

						debugServo2Status =
						    Servo_MoveSmooth(
						        targetDeg,
						        commandedTheta3Deg
						    );

						if (debugServo2Status == SERVO_ERROR_ESTOP)
						{
						    motorEstopRequest = 0U;

						    Motor_EnterSafeStop(
						        MOTOR_ERROR_ESTOP
						    );

						    break;
						}

						if (debugServo2Status == SERVO_ERROR_COMM_LOST)
						{
						    motorCommLostRequest = 0U;

						    Motor_EnterSafeStop(
						        MOTOR_ERROR_COMM_LOST
						    );

						    break;
						}

						if (debugServo2Status != SERVO_OK)
						{
						    Motor_EnterSafeStop(
						        MOTOR_ERROR_SERVO2
						    );

						    break;
						}

						commandedTheta2Deg = targetDeg;
						motorState = MOTOR_STATE_IDLE;
						jogCommandSucceeded = 1U;
						break;
					}

					case MOTOR_AXIS_THETA3:
					{

						targetDeg = commandedTheta3Deg + deltaDeg;

						if (Motor_IsAngleInRange(command.axis, targetDeg) == 0U){
							Motor_ReportError(MOTOR_ERROR_ANGLE_LIMIT);
							break;
						}

						debugServo3Status = Servo_MoveSmooth(commandedTheta2Deg, targetDeg);

						if (debugServo3Status == SERVO_ERROR_ESTOP)
						{
						    motorEstopRequest = 0U;

						    Motor_EnterSafeStop(
						        MOTOR_ERROR_ESTOP
						    );

						    break;
						}

						if (debugServo3Status == SERVO_ERROR_COMM_LOST)
						{
						    motorCommLostRequest = 0U;

						    Motor_EnterSafeStop(
						        MOTOR_ERROR_COMM_LOST
						    );

						    break;
						}

						if(debugServo3Status != SERVO_OK){
							Motor_EnterSafeStop(MOTOR_ERROR_SERVO3);
							break;
						}

						commandedTheta3Deg = targetDeg;
						motorState = MOTOR_STATE_IDLE;
						jogCommandSucceeded = 1U;
						break;
					}


					default:
						Motor_ReportError(MOTOR_ERROR_INVALID_AXIS);
						break;
				}

				if (jogCommandSucceeded != 0U)
				{
				    Motor_ReportCommandDone(
				        (uint8_t)command.type
				    );
				}

				break;
			}

			case MOTOR_COMMAND_CLEAR_ERROR:
			{
				float stableTheta1Deg;
				float currentTheta2Deg;
				float currentTheta3Deg;
			    /*
			     * ESTOP 버튼이 아직 눌려 있으면
			     * ERROR 해제 금지
			     *
			     * 현재 ESTOP은 Rising Edge로 동작했고,
			     * 실제 버튼을 눌렀을 때 인터럽트가 발생했으므로
			     * 눌린 상태를 HIGH로 사용.
			     */
			    if (HAL_GPIO_ReadPin(ESTOP_GPIO_Port, ESTOP_Pin) == GPIO_PIN_SET)
			    {
			        Motor_ReportError(MOTOR_ERROR_ESTOP);
			        break;
			    }

			    /*
			     * 통신이 아직 복구되지 않았다면
			     * ERROR 해제 금지
			     */
			    if (communicationLost != 0U)
			    {
			        Motor_ReportError(MOTOR_ERROR_COMM_LOST);
			        break;
			    }

			    if (Motor_IsEncoderReady() == 0U)
			    {
			        Motor_ReportError(MOTOR_ERROR_ENCODER);
			        break;
			    }

			    if (Stepper_IsBusy() != 0U)
			    {
			        Motor_ReportError(MOTOR_ERROR_STEPPER_BUSY);
			        break;
			    }

			    Stepper_EmergencyStop();

			    if (Motor_GetStableTheta1Angle(&stableTheta1Deg) == 0U)
			    {
			        Motor_ReportError(MOTOR_ERROR_ENCODER);
			        break;
			    }

			    Servo_GetCurrentAngles(
			        &currentTheta2Deg,
			        &currentTheta3Deg
			    );

			    commandedTheta1Deg = stableTheta1Deg;
			    commandedTheta2Deg = currentTheta2Deg;
			    commandedTheta3Deg = currentTheta3Deg;

			    motorState = MOTOR_STATE_IDLE;

			    Motor_ReportCommandDone(
			        (uint8_t)command.type
			    );

			    break;
			}
			default:
				Motor_ReportError(MOTOR_ERROR_INVALID_COMMAND);

				break;
        }
    }
}
