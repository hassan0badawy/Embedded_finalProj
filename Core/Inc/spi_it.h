#ifndef SPI_IT_H
#define SPI_IT_H

#include "std_types.h"
#include "stm32f401ve.h"

#define SPI_STATE_READY 0u
#define SPI_STATE_BUSY  1u

/* ─────────────────────────────────────────
 * SPI_Handle_t
 * ─────────────────────────────────────────
 * Holds all state for one interrupt-driven
 * SPI transfer. Both Master and Slave use
 * this struct via the global SPI1_Handle.
 *
 * FIX: Added pTxBuffer, TxCount, RxBuffer
 * fields required by SPI_SlavePreload() in
 * spi.c to set up the remaining 7 bytes
 * after the first byte is preloaded into DR.
 * ───────────────────────────────────────── */
typedef struct {
    SPI_RegDef_t    *Instance;

    /* RX path */
    volatile u8     *pRxBuffer;         /* Pointer to RX destination buffer  */
    u8               RxCount;           /* Total bytes expected to receive    */
    volatile u8      RxComplete;        /* 1 = all bytes received (set by ISR)*/

    /* TX path (used by Slave preload and Master IT transfer) */
    volatile u8     *pTxBuffer;         /* Pointer to TX source buffer        */
    u8               TxCount;           /* Remaining bytes to transmit        */

    /* Raw RX storage for Slave (8-byte internal buffer) */
    volatile u8      RxBuffer[8];       /* ISR fills this during Slave RX     */

    u8               State;             /* SPI_STATE_READY or SPI_STATE_BUSY  */
} SPI_Handle_t;

extern SPI_Handle_t SPI1_Handle;

/* ─────────────────────────────────────────
 * API
 * ───────────────────────────────────────── */
void SPI_MasterInit(SPI_Handle_t *h);
void SPI_SlaveInit(SPI_Handle_t *h);
void SPI_TransmitReceive_IT(SPI_Handle_t *h, volatile u8 *pTx, volatile u8 *pRx, u8 len);

/*
 * SPI_SlavePreload()
 * ──────────────────
 * SLAVE ONLY. Preloads the first byte of pTxBuffer
 * into SPI1->DR immediately, and stores the remaining
 * 7 bytes in pHandle so the ISR can clock them out
 * byte-by-byte as the Master drives SCK.
 *
 * Must be called BEFORE Master pulls CS low.
 */
void SPI_SlavePreload(SPI_Handle_t *h, volatile u8 *pTxBuffer);

/* Chip-select control */
void SPI_CS_Enable(void);
void SPI_CS_Disable(void);

#endif /* SPI_IT_H */
