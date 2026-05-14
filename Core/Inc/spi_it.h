/**
 * spi_it.h
 * Interrupt-driven SPI1 handle type and API.
 * Single source of truth for SPI_HandleTypeDef.
 */
#ifndef SPI_IT_H
#define SPI_IT_H

#include "std_types.h"
#include "stm32f401ve.h"
#include "shared.h"

#define SPI_STATE_READY 0u
#define SPI_STATE_BUSY  1u

/* ─────────────────────────────────────────────────────────────────────────
 * SPI Handle
 * Tracks the state of one non-blocking SPI transfer.
 * ───────────────────────────────────────────────────────────────────────── */
typedef struct {
    volatile u8  *pTxBuf;       /* Pointer to TX data                   */
    volatile u8  *pRxBuf;       /* Pointer to RX destination            */
    u8            Len;          /* Total bytes in transfer              */
    u8            TxIdx;        /* Next TX byte index                   */
    u8            RxIdx;        /* Next RX byte index                   */
    volatile u8   RxComplete;   /* 1 = all bytes received               */
    volatile u8   State;        /* SPI_STATE_READY / SPI_STATE_BUSY     */
    volatile u8   RxBuffer[IPC_PACKET_SIZE]; /* Raw bytes from last RX  */
} SPI_HandleTypeDef;

extern SPI_HandleTypeDef SPI1_Handle;

/* API */
void SPI_MasterTransfer_IT(SPI_HandleTypeDef *h,
                            volatile u8 *pTx,
                            volatile u8 *pRx,
                            u8 len);
void SPI_SlavePreload(SPI_HandleTypeDef *h, volatile u8 *pTx);

/* IRQ Handler (defined in spi_it.c) */
void SPI1_IRQHandler(void);

#endif /* SPI_IT_H */
