/*
 * motorCommand.h
 *
 *  Created on: 2026. 8. 2.
 *      Author: wowns
 */

#ifndef INC_MOTORCOMMAND_H_
#define INC_MOTORCOMMAND_H_

#include <stdint.h>

typedef enum
{
    MOTOR_COMMAND_SET_TARGET = 0,
    MOTOR_COMMAND_SET_HOME,
    MOTOR_COMMAND_MOVE_HOME,
	MOTOR_COMMAND_JOG,
	MOTOR_COMMAND_CLEAR_ERROR,
    MOTOR_COMMAND_THETA1_TEST,
    MOTOR_COMMAND_MOVE_INTERMEDIATE,
    MOTOR_COMMAND_MOVE_FINAL
} MotorCommandType_t;


typedef enum
{
    MOTOR_AXIS_THETA1 = 1U,
    MOTOR_AXIS_THETA2 = 2U,
    MOTOR_AXIS_THETA3 = 3U
} MotorAxis_t;

typedef struct
{
    MotorCommandType_t type;

    int16_t theta1_x10;
    int16_t theta2_x10;
    int16_t theta3_x10;

    MotorAxis_t axis;
    int16_t delta_x10;
} MotorCommand_t;

#endif /* INC_MOTORCOMMAND_H_ */
