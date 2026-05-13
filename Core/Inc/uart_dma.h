#ifndef UART_DMA_H
#define UART_DMA_H

#include "std_types.h"
#include "stm32f401ve.h"
#include "Bit_Math.h"

/* ─────────────────────────────────────────
 * UART + DMA HARDWARE MAPPING
 * ─────────────────────────────────────────
 * UART    : USART1 (APB2)
 * TX Pin  : PA9 → Alternate Function 7 (USART1_TX)
 * RX Pin  : PA10 → Alternate Function 7 (USART1_RX)
 *
 * Baud    : 115200
 * fAPB2   : 84 MHz
 * BRR     : 84000000 / 115200 = 729.16
 *           → Mantissa = 729 (0x2D9)
 *           → Fraction = 0.16 × 16 = ~3 (0x3)
 *           → BRR = 0x2D93
 *
 * DMA     : DMA2, Stream 7, Channel 4 (USART1_TX)
 * ───────────────────────────────────────── */
#define USART1_BASE_ADDR    0x40011000UL
#define DMA2_BASE_ADDR      0x40026400UL

#define UART_BRR_VALUE      0x2D93u     /* 115200 baud @ 84MHz APB2 */
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

#define USART1              ((USART_RegDef_t *) USART1_BASE_ADDR)

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
 * DMA2 REGISTER STRUCTS
 * ─────────────────────────────────────────
 * DMA2 has 8 streams (Stream 0–7).
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

#define DMA2                ((DMA_RegDef_t *) DMA2_BASE_ADDR)
/* USART1_TX → DMA2 Stream 7 */
#define DMA2_S7             (&DMA2->Stream[7])

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
 * DMA2 HISR / HIFCR bits for Stream 7
 * Streams 4–7 are in the HIGH status register.
 * Stream 7 bits start at offset 22 in HISR.
 * ───────────────────────────────────────── */
#define DMA_S7_TCIF_BIT     27  /* Transfer complete flag  (HISR)    */
#define DMA_S7_HTIF_BIT     26  /* Half transfer flag      (HISR)    */
#define DMA_S7_TEIF_BIT     25  /* Transfer error flag     (HISR)    */
#define DMA_S7_DMEIF_BIT    24  /* Direct mode error flag  (HISR)    */
#define DMA_S7_FEIF_BIT     22  /* FIFO error flag         (HISR)    */

/* ─────────────────────────────────────────
 * IRQ NUMBERS FOR UART + DMA
 * ───────────────────────────────────────── */
#define IRQ_DMA2_STREAM7    70  /* DMA2 Stream7 global interrupt     */
#define IRQ_USART1          37  /* USART1 global interrupt           */

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
 * Initializes USART1 (PA9/PA10) at 115200 baud
 * and configures DMA2 Stream7 Ch4 for TX.
 * Call once in main() at startup.
 */
void UART_DMA_Init(void);

/*
 * UART_DMA_Transmit()
 * ────────────────────
 * Starts a non-blocking DMA transfer of 'len'
 * bytes from UART_TxBuf to USART1->DR.
 * Returns immediately. Sets UART_DMA_Busy=1.
 * DMA2_Stream7_IRQHandler clears it when done.
 *
 * Parameters:
 *   len → number of bytes to transmit
 */
void UART_DMA_Transmit(u8 len);

/* DMA2 Stream7 IRQ handler — clears TC flag */
void DMA2_Stream7_IRQHandler(void);

#endif /* UART_DMA_H */