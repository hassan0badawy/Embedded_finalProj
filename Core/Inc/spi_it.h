#ifndef SPI_IT_H
#define SPI_IT_H

#include "std_types.h"
#include "stm32f401ve.h"

#define SPI_STATE_READY 0u
#define SPI_STATE_BUSY  1u

typedef struct {
    SPI_RegDef_t *Instance;
    volatile u8  *pRxBuffer;
    u8            RxCount;
    volatile u8   RxComplete;
    u8            State;
} SPI_Handle_t;

extern SPI_Handle_t SPI1_Handle;

/* Minimal IT-driven SPI API expected by ipc.c */
void SPI_MasterInit(SPI_Handle_t *h);
void SPI_SlaveInit(SPI_Handle_t *h);
void SPI_TransmitReceive_IT(SPI_Handle_t *h, volatile u8 *pTx, volatile u8 *pRx, u8 len);
void SPI_SlavePreload(SPI_Handle_t *h, volatile u8 *pTxBuffer);

/* Chip-select control (PA4) */
void SPI_CS_Enable(void);
void SPI_CS_Disable(void);

#endif /* SPI_IT_H */
