/*
 * as5600.c
 *
 *  Created on: 2026. 8. 4.
 *      Author: wowns
 */

#include "as5600.h"

#include <stddef.h>

#include "i2c.h"
#include "main.h"

#define AS5600_I2C_ADDRESS (0x36U << 1U)

#define AS5600_RAW_ANGLE_REG     0x0CU

#define AS5600_RAW_ANGLE_MASK    0x0FFFU

#define AS5600_I2C_TIMEOUT_MS    100U
volatile uint16_t debugAS5600RawAngle = 0U;
volatile AS5600_I2CFault_t debugI2CFirstFault = {0};
volatile AS5600_I2CFault_t debugI2CLastFault = {0};
volatile AS5600_I2CFault_t debugI2CFirstFaultAfterMove = {0};
volatile uint32_t debugI2CFailureCount = 0U;
/* Diagnostic marker in the local HAL I2C driver. */
extern volatile uint32_t debugI2CStartFailureStage;
AS5600_Status_t AS5600_ReadRawAngle(uint16_t *raw_angle){
	uint8_t rx_Data[2];
	HAL_StatusTypeDef hal_status;
	uint32_t entryTick, entryState, entryLock;
	uint32_t entryBusy;

	if(raw_angle == NULL){
		return AS5600_ERROR_NULL_POINTER;
	}

	entryTick = HAL_GetTick();
	entryState = (uint32_t)hi2c1.State;
	entryLock = (uint32_t)hi2c1.Lock;
	entryBusy = Stepper_IsBusy();
	debugI2CStartFailureStage = 0U;
	hal_status = HAL_I2C_Mem_Read(&hi2c1, AS5600_I2C_ADDRESS, AS5600_RAW_ANGLE_REG, I2C_MEMADD_SIZE_8BIT,
			rx_Data, sizeof(rx_Data), AS5600_I2C_TIMEOUT_MS);

	if(hal_status != HAL_OK){
		AS5600_I2CFault_t fault = {0};
		fault.tick = HAL_GetTick();
		fault.elapsedMs = fault.tick - entryTick;
		fault.halStatus = (uint32_t)hal_status;
		fault.halError = hi2c1.ErrorCode;
		fault.entryState = entryState;
		fault.entryLock = entryLock;
		fault.state = (uint32_t)hi2c1.State;
		fault.lock = (uint32_t)hi2c1.Lock;
		fault.cr1 = hi2c1.Instance->CR1;
		/* SR2 before SR1 avoids the SR1-then-SR2 ADDR-clear sequence.
		 * Read only after HAL returns failure, never during a good read. */
		fault.sr2 = hi2c1.Instance->SR2;
		fault.sr1 = hi2c1.Instance->SR1;
		fault.lines = GPIOB->IDR & (GPIO_PIN_8 | GPIO_PIN_9);
		fault.remainingBytes = hi2c1.XferCount;
		fault.startFailureStage = debugI2CStartFailureStage;
		fault.readStartTick = entryTick;
		fault.busyAtReadStart = entryBusy;
		fault.busyAtFailure = Stepper_IsBusy();
		fault.move = debugStepperTrace;
		fault.dirOutput = (DIR_GPIO_Port->ODR & DIR_Pin) != 0U;
		fault.enableOutput = (ENA_GPIO_Port->ODR & ENA_Pin) != 0U;
		debugI2CFailureCount++;
		debugI2CLastFault.valid = 0U;
		debugI2CLastFault = fault;
		debugI2CLastFault.valid = 1U;
		/* Preserve the first failure associated with each newly attempted move.
		 * Read move.moveId: a new move with no failure leaves the old record. */
		if (debugI2CFirstFaultAfterMove.valid == 0U ||
		    debugI2CFirstFaultAfterMove.move.moveId != fault.move.moveId) {
			debugI2CFirstFaultAfterMove.valid = 0U;
			debugI2CFirstFaultAfterMove = fault;
			debugI2CFirstFaultAfterMove.valid = 1U;
		}
		if (debugI2CFirstFault.valid == 0U) {
			debugI2CFirstFault = fault;
			debugI2CFirstFault.valid = 1U;
		}
		return AS5600_ERROR_I2C;
	}

	*raw_angle = (((uint16_t)rx_Data[0]<<8U)|((uint16_t)rx_Data[1])) & AS5600_RAW_ANGLE_MASK;
	debugAS5600RawAngle = *raw_angle;
	return AS5600_OK;
}

AS5600_Status_t AS5600_ReadAngleDeg(float *angle_deg){
	uint16_t raw_angle;
	AS5600_Status_t status;

	if(angle_deg == NULL){
		return AS5600_ERROR_NULL_POINTER;
	}

	status = AS5600_ReadRawAngle(&raw_angle);

	if(status != AS5600_OK){
		return status;
	}

	*angle_deg = ((float)raw_angle * 360.0f)/ 4096.0f;

	return AS5600_OK;
}

