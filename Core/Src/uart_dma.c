/*
 * uart_dma.c
 *
 *  Created on: 2026. 8. 12.
 *      Author: wowns
 */

#include "uart_dma.h"

#include <string.h>
#include "usart.h"

extern osMessageQueueId_t uartRxQueueHandle;
extern osMessageQueueId_t uartTxQueueHandle;


/*
 * ============================================================
 * Communication UART Selection
 * ============================================================
 *
 * Jetson : &huart1
 * PC     : &huart2
 *
 * 사용할 UART를 바꿀 때 이 한 줄만 변경하면 된다.
 */
static UART_HandleTypeDef * const communicationUart = &huart1;



static uint8_t uart_dma_rx_buffer[UART_DMA_RX_BUFFER_SIZE];

static UartDmaTxFrame_t uart_tx_active_frame;

static volatile uint8_t uart_tx_busy = 0U;
static volatile uint8_t uart_rx_recovery_requested = 0U;

/* UART RX recovery debug */
volatile uint32_t debugUartRxErrorCount = 0U;
volatile uint32_t debugUartRxLastError = HAL_UART_ERROR_NONE;
volatile uint32_t debugUartRxRecoveryCount = 0U;

void HAL_UART_ErrorCallback(
    UART_HandleTypeDef *huart
)
{
    if (huart != communicationUart)
    {
        return;
    }

    /*
     * ISR에서는 blocking HAL 함수나
     * ReceiveToIdle 재시작을 직접 수행하지 않는다.
     * 복구 요청만 기록하고 CommunicationTask에서 처리한다.
     */
    debugUartRxLastError =
        HAL_UART_GetError(huart);

    debugUartRxErrorCount++;

    uart_rx_recovery_requested = 1U;
}
HAL_StatusTypeDef UartDma_StartReceive(void)
{
    HAL_StatusTypeDef status;

    status = HAL_UARTEx_ReceiveToIdle_DMA(
        communicationUart,
        uart_dma_rx_buffer,
        UART_DMA_RX_BUFFER_SIZE
    );

    if ((status == HAL_OK) &&
        (communicationUart->hdmarx != NULL))
    {
        /*
         * Half Transfer interrupt는 사용하지 않음.
         * UART IDLE 또는 DMA 완료 이벤트만 사용.
         */
        __HAL_DMA_DISABLE_IT(
            communicationUart->hdmarx,
            DMA_IT_HT
        );
    }

    return status;
}



void HAL_UARTEx_RxEventCallback(
    UART_HandleTypeDef *huart,
    uint16_t Size)
{
    UartDmaRxChunk_t rxChunk;

    /*
     * 현재 Communication UART에서 발생한 이벤트만 처리
     */
    if (huart != communicationUart)
    {
        return;
    }

    if ((Size > 0U) &&
        (Size <= UART_DMA_RX_BUFFER_SIZE))
    {
        rxChunk.length = Size;

        memcpy(
            rxChunk.data,
            uart_dma_rx_buffer,
            Size
        );

        /*
         * ISR에서 호출되므로 timeout은 반드시 0
         */
        (void)osMessageQueuePut(
            uartRxQueueHandle,
            &rxChunk,
            0U,
            0U
        );
    }

    /*
     * RX DMA Normal Mode이므로 다음 수신 재시작
     */
    if (UartDma_StartReceive() != HAL_OK)
    {
    	uart_rx_recovery_requested = 1U;
    }
}



HAL_StatusTypeDef UartDma_QueueTransmit(
    const uint8_t *data,
    uint16_t length)
{
    UartDmaTxFrame_t txFrame;

    if ((data == NULL) ||
        (length == 0U) ||
        (length > UART_DMA_TX_BUFFER_SIZE))
    {
        return HAL_ERROR;
    }

    txFrame.length = length;

    memcpy(
        txFrame.data,
        data,
        length
    );

    if (osMessageQueuePut(
            uartTxQueueHandle,
            &txFrame,
            0U,
            0U) != osOK)
    {
        return HAL_BUSY;
    }

    return HAL_OK;
}


void UartDma_ProcessTransmit(void)
{
    if (uart_tx_busy != 0U)
    {
        return;
    }

    if (osMessageQueueGet(
            uartTxQueueHandle,
            &uart_tx_active_frame,
            NULL,
            0U) != osOK)
    {
        return;
    }

    uart_tx_busy = 1U;

    if (HAL_UART_Transmit_DMA(
            communicationUart,
            uart_tx_active_frame.data,
            uart_tx_active_frame.length) != HAL_OK)
    {
        uart_tx_busy = 0U;
    }
}

uint8_t UartDma_IsReceiveRecoveryRequested(void)
{
    return uart_rx_recovery_requested;
}


HAL_StatusTypeDef UartDma_RecoverReceive(void)
{
    HAL_StatusTypeDef status;

    if (uart_rx_recovery_requested == 0U)
    {
        return HAL_OK;
    }

    uart_rx_recovery_requested = 0U;

    /*
     * 진행 중이거나 HAL에 남아 있는 RX 상태를
     * RX 방향만 안전하게 정리한다.
     */
    (void)HAL_UART_AbortReceive(
        communicationUart
    );

    /*
     * STM32F4의 PE/FE/NE/ORE 플래그는
     * SR 읽기 + DR 읽기 순서로 같이 정리된다.
     */
    __HAL_UART_CLEAR_OREFLAG(
        communicationUart
    );

    communicationUart->ErrorCode =
        HAL_UART_ERROR_NONE;

    status = UartDma_StartReceive();

    if (status == HAL_OK)
    {
        debugUartRxRecoveryCount++;
    }
    else
    {
        /*
         * 다음 CommunicationTask 반복에서 재시도
         */
        uart_rx_recovery_requested = 1U;
    }

    return status;
}

void HAL_UART_TxCpltCallback(
    UART_HandleTypeDef *huart)
{
    if (huart == communicationUart)
    {
        uart_tx_busy = 0U;
    }
}
