/*
 * encoderTask.h
 *
 *  Created on: 2026. 8. 4.
 *      Author: wowns
 */

#ifndef INC_ENCODERTASK_H_
#define INC_ENCODERTASK_H_
#include "as5600.h"

void StartEncoderTask(void *argument);

extern volatile float currentEncoderAngleDeg;
extern volatile float filteredEncoderAngleDeg;
extern volatile uint8_t encoderMovingAverageReady;
extern volatile AS5600_Status_t encoderStatus;
extern volatile uint32_t encoderReadCount;
#endif /* INC_ENCODERTASK_H_ */
