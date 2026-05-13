#ifndef UART_DMA_H
#define UART_DMA_H

#include "std_types.h"
#include "stm32f401ve.h"
#include "Bit_Math.h"

/* ─────────────────────────────────────────
 * UART + DMA HARDWARE MAPPING
 * ─────────────────────────────────────────
 * UART    : USART2 (APB1)
 * TX Pin  : PA2 → Alternate Function 7 (USART2_TX)
 * RX Pin  : PA3 → Alternate Function 7 (USART2_RX)
 *
 * Baud    : 115200
 * fAPB1   : 42 MHz (84MHz / 2)
 * BRR     : 42000000 / 115200 = 364.58
 *           → Mantissa = 364 (0x16C)
 *           → Fraction = 0.58 × 16 = ~9 (0x9)
 *           → BRR = 0x16C9
 *
 * DMA     : DMA1, Stream 6, Channel 4 (USART2_TX)
 * ───────────────────────────────────────── */
#define USART2_BASE_ADDR    0x40004400UL
#define DMA1_BASE_ADDR      0x40026000UL

#define UART_BRR_VALUE      0x16C9u     /* 115200 baud @ 42MHz APB1 */
#define UART_TX_BUF_SIZE    128u        /* Telemetry string max size */

/* ─────────────────────────────────────────
 * USART2 REGISTER STRUCT
 * ───────────────────────────────────────── */
typedef struct {
    volatile u32 SR;        /* 0x00  Status register             */
    volatile u32 DR;        /* 0x04  Data register               */
    volatile u32 BRR;       /* 0x08  Baud rate register          */
    volatile u32 CR1;       /* 0x0C  Control register 1          */
    volatile u32 CR2;       /* 0x10  Control register 2          */
    volatile u32 CR3;       /* 0x14  Control register 3          */
    volatile u32 GTPR;      /* 0x18  Guard time & prescaler      */
} USART_RegDef_t;

#define USART2              ((USART_RegDef_t *) USART2_BASE_ADDR)

/* ─────────────────────────────────────────
 * USART CR1 BIT POSITIONS
 * ───────────────────────────────────────── */
#define USART_CR1_UE        13  /* USART enable                      */
#define USART_CR1_TE        3   /* Transmitter enable                */
#define USART_CR1_RE        2   /* Receiver enable                   */

/* ─────────────────────────────────────────
 * USART CR3 BIT POSITIONS
 * ───────────────────────────────────────── */
#define USART_CR3_DMAT      7   /* DMA enable transmitter            */

/* ─────────────────────────────────────────
 * USART SR BIT POSITIONS
 * ───────────────────────────────────────── */
#define USART_SR_TC         6   /* Transmission complete             */
#define USART_SR_TXE        7   /* TX data register empty            */

/* ─────────────────────────────────────────
 * DMA1 REGISTER STRUCTS
 * ─────────────────────────────────────────
 * DMA1 has 8 streams (Stream 0–7).
 * Each stream has its own register set.
 * Interrupt status is in LISR/HISR/LIFCR/HIFCR.
 * ───────────────────────────────────────── */
typedef struct {
    volatile u32 CR;        /* 0x00  Stream config & control     */
    volatile u32 NDTR;      /* 0x04  Number of data items        */
    volatile u32 PAR;       /* 0x08  Peripheral address          */
    volatile u32 M0AR;      /* 0x0C  Memory 0 address            */
    volatile u32 M1AR;      /* 0x10  Memory 1 address (not used) */
    volatile u32 FCR;       /* 0x14  FIFO control register       */
} DMA_Stream_RegDef_t;

typedef struct {
    volatile u32 LISR;      /* 0x00  Low interrupt status        */
    volatile u32 HISR;      /* 0x04  High interrupt status       */
    volatile u32 LIFCR;     /* 0x08  Low interrupt flag clear    */
    volatile u32 HIFCR;     /* 0x0C  High interrupt flag clear   */
    DMA_Stream_RegDef_t Stream[8]; /* Streams 0–7               */
} DMA_RegDef_t;

#define DMA1                ((DMA_RegDef_t *) DMA1_BASE_ADDR)
/* USART2_TX → DMA1 Stream 6 */
#define DMA1_S6             (&DMA1->Stream[6])

/* ─────────────────────────────────────────
 * DMA STREAM CR BIT POSITIONS
 * ───────────────────────────────────────── */
#define DMA_CR_EN           0   /* Stream enable                     */
#define DMA_CR_DMEIE        1   /* Direct mode error IRQ enable      */
#define DMA_CR_TEIE         2   /* Transfer error IRQ enable         */
#define DMA_CR_HTIE         3   /* Half-transfer IRQ enable          */
#define DMA_CR_TCIE         4   /* Transfer complete IRQ enable      */
#define DMA_CR_PFCTRL       5   /* Peripheral flow control           */
#define DMA_CR_DIR0         6   /* Data direction bit 0              */
#define DMA_CR_DIR1         7   /* Data direction bit 1              */
#define DMA_CR_CIRC         8   /* Circular mode                     */
#define DMA_CR_PINC         9   /* Peripheral increment mode         */
#define DMA_CR_MINC         10  /* Memory increment mode             */
#define DMA_CR_PSIZE0       11  /* Peripheral data size bit 0        */
#define DMA_CR_MSIZE0       13  /* Memory data size bit 0            */
#define DMA_CR_CHSEL0       25  /* Channel select bit 0              */

/* Direction: Memory → Peripheral (DIR = 01) */
#define DMA_DIR_MEM_TO_PERIPH   (0x1UL << DMA_CR_DIR0)

/* Channel 4 select: CHSEL[2:0] = 100 */
#define DMA_CH4_SELECT          (0x4UL << DMA_CR_CHSEL0)

/* ─────────────────────────────────────────
 * DMA1 HISR / HIFCR bits for Stream 6
 * Streams 4–7 are in the HIGH status register.
 * Stream 6 bits start at offset 16 in HISR.
 * ───────────────────────────────────────── */
#define DMA_S6_TCIF_BIT     21  /* Transfer complete flag  (HISR)    */
#define DMA_S6_HTIF_BIT     20  /* Half transfer flag      (HISR)    */
#define DMA_S6_TEIF_BIT     19  /* Transfer error flag     (HISR)    */
#define DMA_S6_DMEIF_BIT    18  /* Direct mode error flag  (HISR)    */
#define DMA_S6_FEIF_BIT     16  /* FIFO error flag         (HISR)    */

/* ─────────────────────────────────────────
 * IRQ NUMBERS FOR UART + DMA
 * ───────────────────────────────────────── */
#define IRQ_DMA1_STREAM6    17  /* DMA1 Stream6 global interrupt     */
#define IRQ_USART2          38  /* USART2 global interrupt           */

/* ─────────────────────────────────────────
 * GLOBAL TX BUFFER (filled by System_Logger)
 * ───────────────────────────────────────── */
extern volatile u8  UART_TxBuf[UART_TX_BUF_SIZE];
extern volatile u8  UART_DMA_Busy;   /* 1 = DMA transfer in progress */

/* ─────────────────────────────────────────
 * FUNCTION PROTOTYPES
 * ───────────────────────────────────────── */

/*
 * UART_DMA_Init()
 * ───────────────
 * Initializes USART2 (PA2/PA3) at 115200 baud
 * and configures DMA1 Stream6 Ch4 for TX.
 * Call once in main() at startup.
 */
void UART_DMA_Init(void);

/*
 * UART_DMA_Transmit()
 * ────────────────────
 * Starts a non-blocking DMA transfer of 'len'
 * bytes from UART_TxBuf to USART2->DR.
 * Returns immediately. Sets UART_DMA_Busy=1.
 * DMA1_Stream6_IRQHandler clears it when done.
 *
 * Parameters:
 *   len → number of bytes to transmit
 */
void UART_DMA_Transmit(u8 len);

/* DMA1 Stream6 IRQ handler — clears TC flag */
void DMA1_Stream6_IRQHandler(void);

#endif /* UART_DMA_H */