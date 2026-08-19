/*
 * uart_dma.h
 *
 *  Created on: 2026. 8. 12.
 *      Author: wowns
 */

#ifndef INC_UART_DMA_H_
#define INC_UART_DMA_H_

#include "main.h"
#include "cmsis_os.h"
#include "protocol.h"

#define UART_DMA_RX_BUFFER_SIZE  64U
#define UART_DMA_TX_BUFFER_SIZE  PROTOCOL_MAX_FRAME_LENGTH

HAL_StatusTypeDef UartDma_RecoverReceive(void);
uint8_t UartDma_IsReceiveRecoveryRequested(void);

extern volatile uint32_t debugUartRxErrorCount;
extern volatile uint32_t debugUartRxLastError;
extern volatile uint32_t debugUartRxRecoveryCount;

typedef struct
{
    uint16_t length;
    uint8_t data[UART_DMA_RX_BUFFER_SIZE];
} UartDmaRxChunk_t;

typedef struct
{
    uint16_t length;
    uint8_t data[UART_DMA_TX_BUFFER_SIZE];
} UartDmaTxFrame_t;

HAL_StatusTypeDef UartDma_StartReceive(void);
HAL_StatusTypeDef UartDma_QueueTransmit(const uint8_t *data, uint16_t length);

void UartDma_ProcessTransmit(void);

#endif /* INC_UART_DMA_H_ */
