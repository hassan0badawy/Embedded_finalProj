#ifndef SPI_H
#define SPI_H

#include "std_types.h"
#include "stm32f401ve.h"
#include "Bit_Math.h"

/* ─────────────────────────────────────────
 * SPI CONFIGURATION
 * ─────────────────────────────────────────
 * SPI1 pins (all on Port A, Alternate Function 5):
 *   PA4 → NSS  (CS)
 *   PA5 → SCK
 *   PA6 → MISO
 *   PA7 → MOSI
 * ───────────────────────────────────────── */

/* Baud rate: fPCLK2/4 → 16MHz/4 = 4.0MHz
 * BR[2:0] = 001 for divide by 4              */
#define SPI_BAUD_DIV4       (0x1UL << SPI_CR1_BR0)

/* ─────────────────────────────────────────
 * SPI PACKET SIZE
 * ───────────────────────────────────────── */
#define SPI_PACKET_SIZE     8u  /* 8 bytes per frame */

/* ─────────────────────────────────────────
 * SPI DRIVER STATE
 * ───────────────────────────────────────── */
typedef enum {
    SPI_STATE_READY    = 0,  /* Idle, ready to transfer  */
    SPI_STATE_BUSY_TX  = 1,  /* Transmitting             */
    SPI_STATE_BUSY_RX  = 2,  /* Receiving                */
    SPI_STATE_BUSY     = 3,  /* Full duplex in progress  */
    SPI_STATE_ERROR    = 4   /* Error occurred           */
} SPI_State_t;

/* ─────────────────────────────────────────
 * SPI HANDLE STRUCT
 * Holds all runtime state for one SPI bus
 * ───────────────────────────────────────── */
typedef struct {
    SPI_RegDef_t  *Instance;            /* Pointer to SPI1/2/3 registers     */
    SPI_State_t    State;               /* Current driver state               */

    volatile u8   *pTxBuffer;           /* Pointer to TX data buffer          */
    volatile u8   *pRxBuffer;           /* Pointer to RX data buffer          */
    volatile u8    TxCount;             /* Bytes remaining to transmit        */
    volatile u8    RxCount;             /* Bytes remaining to receive         */

    volatile u8    TxComplete;          /* Flag: 1 when TX done               */
    volatile u8    RxComplete;          /* Flag: 1 when RX done               */
} SPI_Handle_t;

/* ─────────────────────────────────────────
 * GLOBAL SPI HANDLE (used by ISR)
 * ───────────────────────────────────────── */
extern SPI_Handle_t SPI1_Handle;

/* ─────────────────────────────────────────
 * FUNCTION PROTOTYPES
 * ───────────────────────────────────────── */

/* Initialize SPI1 as Master
 * - Full duplex, 8-bit, MSB first
 * - Software NSS, CPOL=0, CPHA=0
 * - Enables RXNE interrupt               */
void SPI_MasterInit(SPI_Handle_t *hSpi);

/* Initialize SPI1 as Slave
 * - Full duplex, 8-bit, MSB first
 * - Hardware NSS, CPOL=0, CPHA=0
 * - Enables RXNE + TXE interrupts
 * - Pre-loads DR before first transfer   */
void SPI_SlaveInit(SPI_Handle_t *hSpi);

/* Start a non-blocking full-duplex transfer
 * Loads TX buffer and enables interrupts
 * Returns immediately — completion via TxComplete/RxComplete flags */
void SPI_TransmitReceive_IT(SPI_Handle_t *hSpi,
                             volatile u8 *pTxData,
                             volatile u8 *pRxData,
                             u8 Size);

/* Pre-load slave TX register with first byte
 * Must be called before Master drives CS low */
void SPI_SlavePreload(SPI_Handle_t *hSpi, volatile u8 *pTxData);

/* Pull CS low to start a Master transfer */
void SPI_CS_Enable(void);

/* Pull CS high to end a Master transfer  */
void SPI_CS_Disable(void);

/* SPI1 Interrupt Handler — called from vector table */
void SPI1_IRQHandler(void);

#endif /* SPI_H */
