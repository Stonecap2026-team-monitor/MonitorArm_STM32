/*
 * protocol.h
 *
 *  Created on: 2026. 8. 1.
 *      Author: wowns
 */

#ifndef INC_PROTOCOL_H_
#define INC_PROTOCOL_H_
/*
 * Frame format
 *
 * +-------+--------+--------+----------+----------+------+
 * | START | MSG_ID | LENGTH |   DATA   | CHECKSUM | END  |
 * +-------+--------+--------+----------+----------+------+
 * | 1Byte | 1Byte  | 1Byte  | N Bytes  | 1Byte    | 1Byte|
 * +-------+--------+--------+----------+----------+------+
 */
#include <stdint.h>

#define PROTOCOL_START_BYTE 0xAAU
#define PROTOCOL_END_BYTE 0x55U

#define PROTOCOL_MAX_DATA_LENGTH 16U

#define PROTOCOL_OVERHEAD_LENGTH 5U

#define PROTOCOL_MIN_FRAME_LENGTH PROTOCOL_OVERHEAD_LENGTH

#define PROTOCOL_MAX_FRAME_LENGTH (PROTOCOL_MAX_DATA_LENGTH+PROTOCOL_MIN_FRAME_LENGTH)

/* Jetson -> STM32 message IDs */
typedef enum
{
    PROTOCOL_MSG_HEARTBEAT  = 0x01U,
    PROTOCOL_MSG_SET_TARGET = 0x10U,
    PROTOCOL_MSG_SET_HOME   = 0x11U,
    PROTOCOL_MSG_MOVE_HOME  = 0x12U,
	PROTOCOL_MSG_JOG		= 0x13U,
	PROTOCOL_MSG_CLEAR_ERROR= 0x14U,
    PROTOCOL_MSG_THETA1_TEST = 0x19U,
    PROTOCOL_MSG_MOVE_INTERMEDIATE = 0x1AU,
    PROTOCOL_MSG_MOVE_FINAL = 0x1BU
} ProtocolRxMessageId_t;

/* STM32 -> Jetson message IDs */
typedef enum
{
    PROTOCOL_MSG_ACK       				= 0x80U,
    PROTOCOL_MSG_STATUS    				= 0x81U,
	PROTOCOL_MSG_COMMAND_DONE  			= 0x82U,
    PROTOCOL_MSG_ERROR     				= 0x83U,
	PROTOCOL_MSG_CURRENT_ANGLE  		= 0x84U,
	PROTOCOL_MSG_CURRENT_COMMAND_ANGLES = 0x85U,
	PROTOCOL_MSG_READY	   				= 0x86U
} ProtocolTxMessageId_t;

/* Protocol function return values */
typedef enum
{
    PROTOCOL_OK = 0,
    PROTOCOL_ERROR_NULL_POINTER,
    PROTOCOL_ERROR_DATA_LENGTH,
    PROTOCOL_ERROR_FRAME_LENGTH,
    PROTOCOL_ERROR_START_BYTE,
    PROTOCOL_ERROR_END_BYTE,
    PROTOCOL_ERROR_CHECKSUM
} ProtocolStatus_t;

typedef enum
{
    PROTOCOL_RX_WAIT_START = 0,
    PROTOCOL_RX_READ_MSG_ID,
    PROTOCOL_RX_READ_LENGTH,
    PROTOCOL_RX_READ_DATA,
    PROTOCOL_RX_READ_CHECKSUM,
    PROTOCOL_RX_READ_END
} ProtocolRxState_t;

/* Decoded protocol message */
typedef struct
{
    uint8_t msg_id;
    uint8_t data_length;
    uint8_t data[PROTOCOL_MAX_DATA_LENGTH];
} ProtocolMessage_t;

typedef struct
{
    ProtocolRxState_t state;
    uint8_t msg_id;
    uint8_t data_length;
    uint8_t data_index;
    uint8_t data[PROTOCOL_MAX_DATA_LENGTH];
    uint8_t checksum;
    uint8_t frame_complete;
} ProtocolRxParser_t;

void Protocol_RxParserInit(ProtocolRxParser_t *parser);

void Protocol_RxProcessByte(ProtocolRxParser_t *parser, uint8_t rx_byte);

void Protocol_RxParserReset(ProtocolRxParser_t *parser);

/*XOR checksum msg_id, data_Length, data*/
uint8_t Protocol_CalculateChecksum(uint8_t msg_id,
                                   uint8_t data_length,
                                   const uint8_t *data);

/*Build Frame*/
ProtocolStatus_t Protocol_BuildFrame(uint8_t msg_id,
                                     const uint8_t *data,
                                     uint8_t data_length,
                                     uint8_t *frame,
                                     uint16_t frame_capacity,
                                     uint16_t *frame_length);
/*decode FRAME*/
ProtocolStatus_t Protocol_RxDecodeMessage(const ProtocolRxParser_t *parser,
										  ProtocolMessage_t *message);
#endif /* INC_PROTOCOL_H_ */
