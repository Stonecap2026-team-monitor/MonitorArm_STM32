/*
 * as5600.h
 *
 *  Created on: 2026. 8. 4.
 *      Author: wowns
 */

#ifndef INC_AS5600_H_
#define INC_AS5600_H_

#include <stdint.h>
#include "stepper.h"


typedef enum{
	AS5600_OK = 0U,
	AS5600_ERROR_NULL_POINTER,
	AS5600_ERROR_I2C
}AS5600_Status_t;

/* Frozen first failure and latest failure, captured before bus recovery.
 * Inspect in the debugger; valid is published last. Reset clears both. */
typedef struct {
    uint32_t valid, tick, elapsedMs, halStatus, halError;
    uint32_t entryState, entryLock, state, lock;
    uint32_t cr1, sr2, sr1, lines, remainingBytes;
    uint32_t startFailureStage; /* 0: other failure, 1: START, 2: repeated START */
    uint32_t readStartTick, busyAtReadStart, busyAtFailure;
    uint32_t dirOutput, enableOutput;
    StepperTrace_t move;
} AS5600_I2CFault_t;
extern volatile AS5600_I2CFault_t debugI2CFirstFault;
extern volatile AS5600_I2CFault_t debugI2CLastFault;
extern volatile AS5600_I2CFault_t debugI2CFirstFaultAfterMove;
extern volatile uint32_t debugI2CFailureCount;


AS5600_Status_t AS5600_ReadRawAngle(uint16_t *raw_angle);

AS5600_Status_t AS5600_ReadAngleDeg(float *angle_deg);

#endif /* INC_AS5600_H_ */
