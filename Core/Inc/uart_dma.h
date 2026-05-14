/**
 * uart_dma.h
 * UART Telemetry driver using DMA2 Stream 7 (non-blocking).
 */
#ifndef UART_DMA_H
#define UART_DMA_H

#include "std_types.h"

#define UART_TX_BUF_SIZE    128u

extern volatile u8  UART_TxBuf[UART_TX_BUF_SIZE];
extern volatile u8  UART_DMA_Busy;

void UART_DMA_Init(void);
void UART_DMA_Transmit(u8 len);
void DMA2_Stream7_IRQHandler(void);

#endif /* UART_DMA_H */