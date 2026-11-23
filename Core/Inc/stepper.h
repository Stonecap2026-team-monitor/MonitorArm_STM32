#ifndef INC_STEPPER_H_
#define INC_STEPPER_H_

#include <stdint.h>

/* Diagnostic timestamps use HAL_GetTick (ms). No change to motor timing. */
typedef struct {
    uint32_t moveId, startTick, stopTick, requestedSteps;
    int32_t directionSign;
    uint32_t stopReason; /* 0: not stopped, 1: completed, 2: emergency, 3: HAL error */
} StepperTrace_t;
extern volatile StepperTrace_t debugStepperTrace;

typedef enum
{
    STEPPER_OK = 0,
    STEPPER_ERROR_BUSY,
    STEPPER_ERROR_INVALID_ANGLE,
    STEPPER_ERROR_INVALID_DIRECTION,
    STEPPER_ERROR_HAL,
	STEPPER_ERROR_ESTOP
} StepperStatus_t;

typedef enum
{
    STEPPER_DIRECTION_CCW = 0,
    STEPPER_DIRECTION_CW
} StepperDirection_t;

//TB6600 활성화 및 비활성화
void Stepper_Enable(void);
void Stepper_Disable(void);

// 모터축 기준 각도를 STEP 펄스 개수로 변환
uint32_t Stepper_AngleToSteps(float angle_deg);

// 고정 주파수로 지정 각도만큼 비동기 이동 시작
StepperStatus_t Stepper_MoveRelative(float angle_deg);

// TIM PWM 완료 콜백에서 호출
void Stepper_HandlePulseFinished(void);

//상태 확인
uint8_t Stepper_IsBusy(void);
uint32_t Stepper_GetCurrentStepCount(void);
uint32_t Stepper_GetTargetStepCount(void);

void Stepper_EmergencyStop(void);
void Stepper_Init(void);
void Stepper_EmergencyDisableFromISR(void);
#endif /* INC_STEPPER_H_ */
