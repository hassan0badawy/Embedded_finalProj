#include "uart_dma.h"
#include "shared.h"
#include "Bit_Math.h"

/* ─────────────────────────────────────────
 * GLOBAL BUFFERS & FLAGS
 * ───────────────────────────────────────── */
volatile u8 UART_TxBuf[UART_TX_BUF_SIZE];
volatile u8 UART_DMA_Busy = 0u;

/* ─────────────────────────────────────────
 * UART_DMA_Init()
 * ─────────────────────────────────────────
 * Full initialization of USART2 + DMA1 Stream6.
 *
 * Clock sources:
 *   USART2 → APB1 @ 42MHz
 *   DMA1   → AHB1
 *   GPIOA  → AHB1
 * ───────────────────────────────────────── */
void UART_DMA_Init(void)
{
    /* ── Step 1: Enable clocks ── */

    /* GPIOA clock (AHB1 bit 0) — shared with SPI/PWM, safe to re-set */
    SET_BIT(RCC->AHB1ENR, 0);

    /* DMA1 clock (AHB1 bit 21) */
    SET_BIT(RCC->AHB1ENR, 21);

    /* USART2 clock (APB1 bit 17) */
    SET_BIT(RCC->APB1ENR, 17);

    /* ── Step 2: Configure PA2 (TX) and PA3 (RX) → AF7 ── */
    volatile u32 *GPIOA_MODER   = (volatile u32 *)(0x40020000UL + 0x00);
    volatile u32 *GPIOA_OSPEEDR = (volatile u32 *)(0x40020000UL + 0x08);
    volatile u32 *GPIOA_AFRL   = (volatile u32 *)(0x40020000UL + 0x20);

    /* PA2 and PA3 → MODER = 10 (Alternate Function) */
    *GPIOA_MODER &= ~((0x3UL << 4) | (0x3UL << 6));
    *GPIOA_MODER |=  ((0x2UL << 4) | (0x2UL << 6));

    /* PA2 and PA3 → OSPEEDR = 10 (High speed) */
    *GPIOA_OSPEEDR &= ~((0x3UL << 4) | (0x3UL << 6));
    *GPIOA_OSPEEDR |=  ((0x2UL << 4) | (0x2UL << 6));

    /* PA2 → AFRL bits [11:8] = 0111 (AF7 = USART2_TX)
     * PA3 → AFRL bits [15:12] = 0111 (AF7 = USART2_RX) */
    *GPIOA_AFRL &= ~((0xFUL << 8) | (0xFUL << 12));
    *GPIOA_AFRL |=  ((0x7UL << 8) | (0x7UL << 12));

    /* ── Step 3: Configure USART2 ── */

    /* Disable USART before configuration */
    CLEAR_BIT(USART2->CR1, USART_CR1_UE);

    /* Set baud rate: 115200 @ 42MHz APB1
     * BRR = 0x16C9 (mantissa=0x16C, fraction=9) */
    USART2->BRR = UART_BRR_VALUE;

    /* CR1: Enable TX, RX (no parity, 8-bit data, 1 stop bit = defaults) */
    USART2->CR1 = 0u;
    SET_BIT(USART2->CR1, USART_CR1_TE);  /* Transmitter enable */
    SET_BIT(USART2->CR1, USART_CR1_RE);  /* Receiver enable    */

    /* CR3: Enable DMA for transmitter
     * DMAT=1 → USART generates a DMA request whenever TXE=1 */
    USART2->CR3 = 0u;
    SET_BIT(USART2->CR3, USART_CR3_DMAT);

    /* Enable USART2 */
    SET_BIT(USART2->CR1, USART_CR1_UE);

    /* ── Step 4: Configure DMA1 Stream6 for USART2_TX ──
     *
     * Mapping (from STM32F401 reference manual, Table 27):
     *   USART2_TX → DMA1 Stream 6, Channel 4
     *
     * Transfer setup:
     *   Direction    : Memory → Peripheral
     *   Memory       : UART_TxBuf (byte array, increment)
     *   Peripheral   : &USART2->DR (fixed address)
     *   Data width   : 8-bit both sides
     *   Mode         : Normal (not circular)
     *   Priority     : High
     *   FIFO         : Direct mode (FIFO disabled)
     */

    /* First: disable stream before configuring */
    CLEAR_BIT(DMA1_S6->CR, DMA_CR_EN);

    /* Wait until stream is actually disabled */
    while (READ_BIT(DMA1_S6->CR, DMA_CR_EN));

    /* Clear all interrupt flags for Stream 6 in HIFCR */
    DMA1->HIFCR = (0x3FUL << 16);  /* Bits [21:16] cover all S6 flags */

    /* Set peripheral address = USART2 data register */
    DMA1_S6->PAR = (u32)(&USART2->DR);

    /* Set memory address = our TX buffer */
    DMA1_S6->M0AR = (u32)(UART_TxBuf);

    /* CR register:
     *   CHSEL [27:25] = 100  → Channel 4
     *   PL    [17:16] = 10   → High priority
     *   MSIZE [14:13] = 00   → 8-bit memory
     *   PSIZE [12:11] = 00   → 8-bit peripheral
     *   MINC  [10]    = 1    → Memory pointer increments
     *   PINC  [9]     = 0    → Peripheral address fixed
     *   CIRC  [8]     = 0    → Normal mode (not circular)
     *   DIR   [7:6]   = 01   → Memory to Peripheral
     *   TCIE  [4]     = 1    → Transfer complete interrupt
     *   TEIE  [2]     = 1    → Transfer error interrupt
     */
    DMA1_S6->CR = 0u;
    DMA1_S6->CR |= DMA_CH4_SELECT;             /* Channel 4            */
    DMA1_S6->CR |= (0x2UL << 16);              /* Priority: High       */
    SET_BIT(DMA1_S6->CR, DMA_CR_MINC);         /* Memory increment     */
    DMA1_S6->CR |= DMA_DIR_MEM_TO_PERIPH;      /* Mem → Periph         */
    SET_BIT(DMA1_S6->CR, DMA_CR_TCIE);         /* TC interrupt enable  */
    SET_BIT(DMA1_S6->CR, DMA_CR_TEIE);         /* TE interrupt enable  */

    /* FCR: Direct mode (FIFO disabled, DMDIS=0 by default) */
    DMA1_S6->FCR = 0u;

    /* ── Step 5: Enable DMA1 Stream6 NVIC interrupt ── */
    /* Priority 3 — lower than EXTI safety (0) and SPI (1/2) */
    NVIC_SET_PRIORITY(IRQ_DMA1_STREAM6, 3);
    NVIC_ENABLE_IRQ(IRQ_DMA1_STREAM6);
}

/* ─────────────────────────────────────────
 * UART_DMA_Transmit()
 * ─────────────────────────────────────────
 * Starts a non-blocking DMA transfer of 'len'
 * bytes from UART_TxBuf to USART2->DR.
 *
 * The CPU is completely free during transfer.
 * DMA hardware reads bytes from UART_TxBuf and
 * feeds them to USART2->DR automatically,
 * gated by USART2's TXE DMA request signal.
 *
 * Guard: if DMA is still busy from a previous
 * transfer, this call is silently dropped to
 * prevent buffer corruption.
 * ───────────────────────────────────────── */
void UART_DMA_Transmit(u8 len)
{
    /* Guard: don't restart if DMA is still busy */
    if (UART_DMA_Busy) { return; }
    if (len == 0u)     { return; }

    /* Mark as busy — cleared by ISR on transfer complete */
    UART_DMA_Busy = 1u;

    /* Disable stream before reconfiguring NDTR */
    CLEAR_BIT(DMA1_S6->CR, DMA_CR_EN);
    while (READ_BIT(DMA1_S6->CR, DMA_CR_EN));

    /* Clear Stream6 interrupt flags in HIFCR */
    DMA1->HIFCR = (0x3FUL << 16);

    /* Set number of bytes to transfer */
    DMA1_S6->NDTR = (u32)len;

    /* Reload memory address (resets DMA pointer to buffer start) */
    DMA1_S6->M0AR = (u32)(UART_TxBuf);

    /* Enable stream → DMA starts transferring when USART2 asserts TXE */
    SET_BIT(DMA1_S6->CR, DMA_CR_EN);
}

/* ─────────────────────────────────────────
 * DMA1_Stream6_IRQHandler()
 * ─────────────────────────────────────────
 * Fires when DMA finishes sending all bytes.
 *
 * Actions:
 *   1. Clear the transfer-complete flag
 *   2. Disable the DMA stream
 *   3. Release the busy flag for next send
 * ───────────────────────────────────────── */
void DMA1_Stream6_IRQHandler(void)
{
    /* Check Transfer Complete flag for Stream 6 (HISR bit 21) */
    if (READ_BIT(DMA1->HISR, DMA_S6_TCIF_BIT))
    {
        /* Clear TC flag by writing to HIFCR */
        SET_BIT(DMA1->HIFCR, DMA_S6_TCIF_BIT);

        /* Disable DMA stream */
        CLEAR_BIT(DMA1_S6->CR, DMA_CR_EN);

        /* Release busy flag → System_Logger can transmit again */
        UART_DMA_Busy = 0u;
    }

    /* Check Transfer Error flag */
    if (READ_BIT(DMA1->HISR, DMA_S6_TEIF_BIT))
    {
        /* Clear error flag */
        SET_BIT(DMA1->HIFCR, DMA_S6_TEIF_BIT);

        /* Recover: disable stream and release lock */
        CLEAR_BIT(DMA1_S6->CR, DMA_CR_EN);
        UART_DMA_Busy = 0u;
    }
}