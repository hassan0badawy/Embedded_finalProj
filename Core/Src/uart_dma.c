#include "../Inc/uart_dma.h"
#include "../Inc/shared.h"
#include "../Inc/Elevator.h"   /* Enter_Critical / Exit_Critical, GSS */
#include "../Inc/Bit_Math.h"

/* ─────────────────────────────────────────
 * GLOBAL BUFFERS & FLAGS
 * ───────────────────────────────────────── */
volatile u8 UART_TxBuf[UART_TX_BUF_SIZE];
volatile u8 UART_DMA_Busy = 0u;

/* ─────────────────────────────────────────
 * UART_DMA_Init()
 * Initialises USART1 @ 115200 baud (PA9/PA10)
 * and DMA2 Stream7 Ch4 for non-blocking TX.
 * ───────────────────────────────────────── */
void UART_DMA_Init(void)
{
    /* Step 1: Enable clocks */
    SET_BIT(RCC->AHB1ENR, 0);   /* GPIOA */
    SET_BIT(RCC->AHB1ENR, 22);  /* DMA2  */
    SET_BIT(RCC->APB2ENR, 4);   /* USART1 */

    /* Step 2: PA9 (TX), PA10 (RX) → AF7 */
    volatile u32 *GPIOA_MODER   = (volatile u32 *)(0x40020000UL + 0x00);
    volatile u32 *GPIOA_OSPEEDR = (volatile u32 *)(0x40020000UL + 0x08);
    volatile u32 *GPIOA_AFRH   = (volatile u32 *)(0x40020000UL + 0x24);

    *GPIOA_MODER   &= ~((0x3UL << 18) | (0x3UL << 20));
    *GPIOA_MODER   |=  ((0x2UL << 18) | (0x2UL << 20));  /* AF mode */
    *GPIOA_OSPEEDR &= ~((0x3UL << 18) | (0x3UL << 20));
    *GPIOA_OSPEEDR |=  ((0x2UL << 18) | (0x2UL << 20));  /* High speed */
    *GPIOA_AFRH    &= ~((0xFUL << 4)  | (0xFUL << 8));
    *GPIOA_AFRH    |=  ((0x7UL << 4)  | (0x7UL << 8));   /* AF7 = USART1 */

    /* Step 3: Configure USART1 */
    CLEAR_BIT(USART1->CR1, USART_CR1_UE);
    USART1->BRR  = UART_BRR_VALUE;   /* 115200 @ 16MHz */
    USART1->CR1  = 0u;
    SET_BIT(USART1->CR1, USART_CR1_TE);
    SET_BIT(USART1->CR1, USART_CR1_RE);
    USART1->CR3  = 0u;
    SET_BIT(USART1->CR3, USART_CR3_DMAT);  /* DMA for TX */
    SET_BIT(USART1->CR1, USART_CR1_UE);

    /* Step 4: Configure DMA2 Stream7 Channel4 (USART1_TX) */
    CLEAR_BIT(DMA2_S7->CR, DMA_CR_EN);
    while (READ_BIT(DMA2_S7->CR, DMA_CR_EN));

    DMA2->HIFCR    = (0x3FUL << 22);         /* Clear all S7 flags */
    DMA2_S7->PAR   = (u32)(&USART1->DR);
    DMA2_S7->M0AR  = (u32)(UART_TxBuf);
    DMA2_S7->CR    = 0u;
    DMA2_S7->CR   |= DMA_CH4_SELECT;         /* Channel 4 */
    DMA2_S7->CR   |= (0x2UL << 16);          /* High priority */
    SET_BIT(DMA2_S7->CR, DMA_CR_MINC);       /* Memory increment */
    DMA2_S7->CR   |= DMA_DIR_MEM_TO_PERIPH;  /* Mem → Periph */
    SET_BIT(DMA2_S7->CR, DMA_CR_TCIE);       /* TC interrupt */
    SET_BIT(DMA2_S7->CR, DMA_CR_TEIE);       /* TE interrupt */
    DMA2_S7->FCR   = 0u;                     /* Direct mode */

    /* Step 5: Enable DMA2 Stream7 IRQ */
    NVIC_SET_PRIORITY(IRQ_DMA2_STREAM7, 3);
    NVIC_ENABLE_IRQ(IRQ_DMA2_STREAM7);
}

/* ─────────────────────────────────────────
 * UART_DMA_Transmit()
 * Non-blocking: starts DMA then returns.
 * CPU is completely free during transfer.
 * Silently dropped if a transfer is active.
 * ───────────────────────────────────────── */
void UART_DMA_Transmit(u8 len)
{
    if (UART_DMA_Busy) { return; }
    if (len == 0u)     { return; }

    UART_DMA_Busy = 1u;

    CLEAR_BIT(DMA2_S7->CR, DMA_CR_EN);
    while (READ_BIT(DMA2_S7->CR, DMA_CR_EN));

    DMA2->HIFCR    = (0x3FUL << 22);
    DMA2_S7->NDTR  = (u32)len;
    DMA2_S7->M0AR  = (u32)(UART_TxBuf);

    SET_BIT(DMA2_S7->CR, DMA_CR_EN);
}

/* ─────────────────────────────────────────
 * DMA2_Stream7_IRQHandler()
 * Fires when DMA finishes all bytes.
 * Clears flags and releases the busy lock.
 * ───────────────────────────────────────── */
void DMA2_Stream7_IRQHandler(void)
{
    if (READ_BIT(DMA2->HISR, DMA_S7_TCIF_BIT))
    {
        SET_BIT(DMA2->HIFCR, DMA_S7_TCIF_BIT);
        CLEAR_BIT(DMA2_S7->CR, DMA_CR_EN);
        UART_DMA_Busy = 0u;
    }
    if (READ_BIT(DMA2->HISR, DMA_S7_TEIF_BIT))
    {
        SET_BIT(DMA2->HIFCR, DMA_S7_TEIF_BIT);
        CLEAR_BIT(DMA2_S7->CR, DMA_CR_EN);
        UART_DMA_Busy = 0u;
    }
}

/* ─────────────────────────────────────────
 * System_Logger()
 * ─────────────────────────────────────────
 * MEMBER D DELIVERABLE — DMA Telemetry (+5 bonus)
 *
 * Called from main loop when GSS.telem_flag==1
 * (set by TIM6 every 500ms).
 *
 * Steps:
 *   1. Atomically snapshot all relevant GSS fields
 *   2. Format telemetry string into UART_TxBuf
 *   3. Kick off non-blocking DMA transfer
 *   4. Clear telem_flag
 *
 * Output format:
 *   "ELV|FL:%u|ST:%u|SP:%u|DIR:%u|EM:%u|CF:%u|"
 *   "SL_FL:%u|SL_ST:%u\r\n"
 *
 * Non-blocking: returns immediately after
 * DMA starts. CPU overhead ≈ 0 during TX.
 *
 * FIX: This function was declared in Elevator.h
 * and called in main.c but had NO implementation
 * in any source file. Added here in uart_dma.c
 * since this is the module that owns UART + DMA.
 * ───────────────────────────────────────── */
void System_Logger(void)
{
    /* Guard: skip if DMA is still busy with previous frame */
    if (UART_DMA_Busy) { return; }

    /* ── Step 1: Atomic snapshot of GSS ──
     * All fields read inside one critical section so we
     * never mix fields from two different FSM states.  */
    u8 snap_pos, snap_state, snap_speed, snap_dir;
    u8 snap_em,  snap_cf,    snap_door;
    u8 snap_sl_pos, snap_sl_state;

    u32 pm = Enter_Critical();
    snap_pos      = GSS.position;
    snap_state    = GSS.fsm_state;
    snap_speed    = GSS.speed;
    snap_dir      = GSS.direction;
    snap_em       = GSS.emergency;
    snap_cf       = GSS.comm_fault;
    snap_door     = GSS.door_open;
    snap_sl_pos   = GSS.slave_position;
    snap_sl_state = GSS.slave_fsm_state;
    GSS.telem_flag = 0u;   /* Clear flag — still inside critical section */
    Exit_Critical(pm);

    /* ── Step 2: Format telemetry string ──
     * Avoid stdlib sprintf to keep stack small.
     * Manual formatting into UART_TxBuf.
     * Format: "ELV|FL:x|ST:x|SP:xxx|DIR:x|EM:x|CF:x|DO:x|SL_FL:x|SL_ST:x\r\n"
     */
    volatile u8 *p = UART_TxBuf;
    u8 len = 0u;

    /* Helper macro: append a string literal */
    #define PUTS(s) do { \
        const char *_s = (s); \
        while (*_s && len < (UART_TX_BUF_SIZE - 3u)) { \
            p[len++] = (u8)(*_s++); \
        } \
    } while(0)

    /* Helper macro: append a single decimal byte (0–255) */
    #define PUTU8(v) do { \
        u8 _v = (v); \
        if (_v >= 100u) { p[len++] = (u8)('0' + _v / 100u); _v %= 100u; } \
        if (_v >= 10u)  { p[len++] = (u8)('0' + _v / 10u);  _v %= 10u; } \
        p[len++] = (u8)('0' + _v); \
    } while(0)

    PUTS("ELV|FL:");   PUTU8(snap_pos + 1u);  /* Floor 1–4 for readability */
    PUTS("|ST:");      PUTU8(snap_state);
    PUTS("|SP:");      PUTU8(snap_speed);
    PUTS("|DIR:");     PUTU8(snap_dir);
    PUTS("|EM:");      PUTU8(snap_em);
    PUTS("|CF:");      PUTU8(snap_cf);
    PUTS("|DO:");      PUTU8(snap_door);
    PUTS("|SL_FL:");   PUTU8(snap_sl_pos + 1u);
    PUTS("|SL_ST:");   PUTU8(snap_sl_state);
    PUTS("\r\n");

    #undef PUTS
    #undef PUTU8

    /* ── Step 3: Kick off non-blocking DMA transfer ── */
    UART_DMA_Transmit(len);
}
