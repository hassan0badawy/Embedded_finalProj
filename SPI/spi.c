#include "spi.h"
#include "shared.h"    /* RCC registers */
#include "Bit_Math.h"

/* ─────────────────────────────────────────
 * GLOBAL SPI HANDLE
 * ───────────────────────────────────────── */
SPI_Handle_t SPI1_Handle;

/* ─────────────────────────────────────────
 * STATIC HELPERS
 * ───────────────────────────────────────── */

/* Configure PA4(NSS), PA5(SCK), PA6(MISO), PA7(MOSI)
 * to Alternate Function 5 (SPI1)
 *
 * MODER  bits [2n+1:2n] = 10 → Alternate function
 * AFRL   bits [4n+3:4n] = 0101 → AF5
 * OSPEEDR bits          = 11 → Very high speed        */
static void SPI1_GPIO_Init(void)
{
    /* Enable GPIOA clock */
    SET_BIT(RCC->AHB1ENR, 0);   /* bit 0 = GPIOAEN */

    /* We need GPIOA register struct — defined here locally
     * since GPIO driver belongs to another team member    */
    volatile u32 *GPIOA_MODER   = (volatile u32 *)(0x40020000UL + 0x00);
    volatile u32 *GPIOA_OSPEEDR = (volatile u32 *)(0x40020000UL + 0x08);
    volatile u32 *GPIOA_PUPDR   = (volatile u32 *)(0x40020000UL + 0x0C);
    volatile u32 *GPIOA_AFRL    = (volatile u32 *)(0x40020000UL + 0x20);

    /* ── PA4 (NSS/CS) — for Master: GPIO output
     *                  for Slave:  AF5 hardware NSS
     * We configure PA4 as AF5 for both and let
     * SSM/SSOE bits decide who controls it            */

    /* Set PA4, PA5, PA6, PA7 to Alternate Function mode (MODER = 10) */
    /* Clear bits first */
    *GPIOA_MODER &= ~( (0x3UL << 8)  |  /* PA4 */
                       (0x3UL << 10) |  /* PA5 */
                       (0x3UL << 12) |  /* PA6 */
                       (0x3UL << 14) ); /* PA7 */
    /* Set to AF mode (10) */
    *GPIOA_MODER |=  ( (0x2UL << 8)  |  /* PA4 */
                       (0x2UL << 10) |  /* PA5 */
                       (0x2UL << 12) |  /* PA6 */
                       (0x2UL << 14) ); /* PA7 */

    /* Set speed to Very High (11) for all 4 pins */
    *GPIOA_OSPEEDR &= ~( (0x3UL << 8)  |
                         (0x3UL << 10) |
                         (0x3UL << 12) |
                         (0x3UL << 14) );
    *GPIOA_OSPEEDR |=  ( (0x3UL << 8)  |
                         (0x3UL << 10) |
                         (0x3UL << 12) |
                         (0x3UL << 14) );

    /* No pull-up/pull-down (PUPDR = 00) — already default */
    *GPIOA_PUPDR &= ~( (0x3UL << 8)  |
                       (0x3UL << 10) |
                       (0x3UL << 12) |
                       (0x3UL << 14) );

    /* Set Alternate Function 5 (AF5 = 0101) for PA4–PA7
     * AFRL register: each pin gets 4 bits
     * PA4 → bits [19:16], PA5 → [23:20], PA6 → [27:24], PA7 → [31:28] */
    *GPIOA_AFRL &= ~( (0xFUL << 16) |   /* PA4 */
                      (0xFUL << 20) |   /* PA5 */
                      (0xFUL << 24) |   /* PA6 */
                      (0xFUL << 28) );  /* PA7 */
    *GPIOA_AFRL |=  ( (0x5UL << 16) |   /* AF5 on PA4 */
                      (0x5UL << 20) |   /* AF5 on PA5 */
                      (0x5UL << 24) |   /* AF5 on PA6 */
                      (0x5UL << 28) );  /* AF5 on PA7 */
}

/* ─────────────────────────────────────────
 * SPI MASTER INIT
 * ─────────────────────────────────────────
 * Mode: Full-Duplex Master
 * CPOL=0, CPHA=0 (Mode 0)
 * 8-bit data, MSB first
 * Software NSS (SSM=1, SSI=1)
 * Baud = fPCLK2/16 ≈ 5.25MHz
 * Interrupts: RXNE only
 *   (TX triggered per-transfer in TransmitReceive_IT)
 * ───────────────────────────────────────── */
void SPI_MasterInit(SPI_Handle_t *hSpi)
{
    /* 1. Configure GPIO pins for SPI1 */
    SPI1_GPIO_Init();

    /* 2. Configure PA4 as plain GPIO Output for manual CS control */
    volatile u32 *GPIOA_MODER = (volatile u32 *)(0x40020000UL + 0x00);
    *GPIOA_MODER &= ~(0x3UL << 8);   /* Clear PA4 mode */
    *GPIOA_MODER |=  (0x1UL << 8);   /* PA4 = GPIO Output */

    /* Start CS high (inactive) */
    SPI_CS_Disable();

    /* 3. Enable SPI1 clock on APB2 */
    SET_BIT(RCC->APB2ENR, 12);   /* bit 12 = SPI1EN */

    /* 4. Make sure SPI is disabled before configuring */
    CLEAR_BIT(hSpi->Instance->CR1, SPI_CR1_SPE);

    /* 5. Configure CR1 */
    hSpi->Instance->CR1 = 0;  /* Clear all first */

    /* CPOL=0, CPHA=0 → Mode 0 (bits 0,1 stay 0) */
    /* Baud rate: BR[2:0] = 011 → fPCLK/16        */
    hSpi->Instance->CR1 |= SPI_BAUD_DIV16;
    /* Master mode */
    SET_BIT(hSpi->Instance->CR1, SPI_CR1_MSTR);
    /* Software slave management — we control CS manually */
    SET_BIT(hSpi->Instance->CR1, SPI_CR1_SSM);
    SET_BIT(hSpi->Instance->CR1, SPI_CR1_SSI);
    /* MSB first (LSBFIRST=0, already 0) */
    /* 8-bit data frame (DFF=0, already 0) */
    /* Full duplex (RXONLY=0, BIDIMODE=0, already 0) */

    /* 6. Configure CR2 — no TXEIE yet, enabled per-transfer */
    hSpi->Instance->CR2 = 0;

    /* 7. Enable NVIC for SPI1 (IRQ 35), priority 2 */
    NVIC_SET_PRIORITY(IRQ_SPI1, 2);
    NVIC_ENABLE_IRQ(IRQ_SPI1);

    /* 8. Init handle state */
    hSpi->State      = SPI_STATE_READY;
    hSpi->TxComplete = 0;
    hSpi->RxComplete = 0;
    hSpi->TxCount    = 0;
    hSpi->RxCount    = 0;

    /* 9. Enable SPI1 */
    SET_BIT(hSpi->Instance->CR1, SPI_CR1_SPE);
}

/* ─────────────────────────────────────────
 * SPI SLAVE INIT
 * ─────────────────────────────────────────
 * Mode: Full-Duplex Slave
 * CPOL=0, CPHA=0 (must match Master)
 * 8-bit data, MSB first
 * Hardware NSS (SSM=0) — PA4 controlled by Master
 * Interrupts: RXNE + TXEIE
 * ───────────────────────────────────────── */
void SPI_SlaveInit(SPI_Handle_t *hSpi)
{
    /* 1. Configure GPIO pins for SPI1 */
    SPI1_GPIO_Init();
    /* PA4 stays as AF5 for hardware NSS on Slave */

    /* 2. Enable SPI1 clock on APB2 */
    SET_BIT(RCC->APB2ENR, 12);

    /* 3. Disable SPI before configuring */
    CLEAR_BIT(hSpi->Instance->CR1, SPI_CR1_SPE);

    /* 4. Configure CR1 */
    hSpi->Instance->CR1 = 0;
    /* CPOL=0, CPHA=0 → Mode 0 */
    /* Baud rate bits ignored on Slave (clock comes from Master) */
    /* MSTR=0 → Slave mode (already 0) */
    /* SSM=0  → Hardware NSS (already 0) */
    /* MSB first, 8-bit, full duplex — all already 0 */

    /* 5. Configure CR2 */
    hSpi->Instance->CR2 = 0;
    /* Enable RXNE interrupt — fires when a byte is received */
    SET_BIT(hSpi->Instance->CR2, SPI_CR2_RXNEIE);
    /* Enable TXEIE interrupt — fires when TX buffer is empty
     * This is how we preload the next byte before Master clocks it */
    SET_BIT(hSpi->Instance->CR2, SPI_CR2_TXEIE);
    /* Enable error interrupt (overrun detection) */
    SET_BIT(hSpi->Instance->CR2, SPI_CR2_ERRIE);

    /* 6. Enable NVIC for SPI1 (IRQ 35), priority 1
     * Slave priority slightly higher than Master
     * so it can preload before Master starts      */
    NVIC_SET_PRIORITY(IRQ_SPI1, 1);
    NVIC_ENABLE_IRQ(IRQ_SPI1);

    /* 7. Init handle state */
    hSpi->State      = SPI_STATE_READY;
    hSpi->TxComplete = 0;
    hSpi->RxComplete = 0;
    hSpi->TxCount    = 0;
    hSpi->RxCount    = 0;

    /* 8. Enable SPI1 */
    SET_BIT(hSpi->Instance->CR1, SPI_CR1_SPE);
}

/* ─────────────────────────────────────────
 * SPI SLAVE PRELOAD
 * ─────────────────────────────────────────
 * The Slave MUST write its first status byte
 * into DR BEFORE the Master drives CS low.
 * If it doesn't, the Master will clock out
 * 0xFF (empty register) instead of real data.
 *
 * Call this in your idle loop or after every
 * completed transfer to always have fresh data
 * ready for the next Master request.
 * ───────────────────────────────────────── */
void SPI_SlavePreload(SPI_Handle_t *hSpi, volatile u8 *pTxData)
{
    /* Wait until TX buffer is empty before preloading */
    while (!READ_BIT(hSpi->Instance->SR, SPI_SR_TXE));

    /* Write first byte of the packet into DR
     * This sits in the TX shift register waiting
     * for the Master to start the clock          */
    hSpi->Instance->DR = pTxData[0];

    /* Store the rest of the buffer for the ISR to send */
    hSpi->pTxBuffer = pTxData;
    hSpi->TxCount   = SPI_PACKET_SIZE - 1u;  /* First byte already loaded */
}

/* ─────────────────────────────────────────
 * NON-BLOCKING TRANSMIT/RECEIVE
 * ─────────────────────────────────────────
 * Sets up buffers and enables interrupts.
 * Returns immediately.
 * The ISR handles byte-by-byte TX and RX.
 * Completion: check hSpi->TxComplete and
 *             hSpi->RxComplete flags.
 * ───────────────────────────────────────── */
void SPI_TransmitReceive_IT(SPI_Handle_t *hSpi,
                             volatile u8 *pTxData,
                             volatile u8 *pRxData,
                             u8 Size)
{
    /* Guard: don't start if already busy */
    if (hSpi->State != SPI_STATE_READY) return;

    ENTER_CRITICAL();

    /* Load transfer parameters into handle */
    hSpi->pTxBuffer  = pTxData;
    hSpi->pRxBuffer  = pRxData;
    hSpi->TxCount    = Size;
    hSpi->RxCount    = Size;
    hSpi->TxComplete = 0;
    hSpi->RxComplete = 0;
    hSpi->State      = SPI_STATE_BUSY;

    /* Enable RXNE interrupt (receive) */
    SET_BIT(hSpi->Instance->CR2, SPI_CR2_RXNEIE);
    /* Enable TXE interrupt (transmit) */
    SET_BIT(hSpi->Instance->CR2, SPI_CR2_TXEIE);

    EXIT_CRITICAL();

    /* For Master: pull CS low to start the transfer */
    /* For Slave: CS is controlled by Master hardware */
}

/* ─────────────────────────────────────────
 * CS CONTROL (MASTER ONLY)
 * ───────────────────────────────────────── */
void SPI_CS_Enable(void)
{
    /* PA4 low → CS active → Slave starts listening */
    volatile u32 *GPIOA_ODR = (volatile u32 *)(0x40020000UL + 0x14);
    CLEAR_BIT(*GPIOA_ODR, 4);
}

void SPI_CS_Disable(void)
{
    /* PA4 high → CS inactive → transfer complete */
    volatile u32 *GPIOA_ODR = (volatile u32 *)(0x40020000UL + 0x14);
    SET_BIT(*GPIOA_ODR, 4);
}

/* ─────────────────────────────────────────
 * SPI1 INTERRUPT HANDLER
 * ─────────────────────────────────────────
 * Handles 3 events:
 *   TXE  → TX buffer empty  → load next byte
 *   RXNE → RX buffer full   → read received byte
 *   OVR  → Overrun error    → clear and recover
 *
 * This runs for BOTH Master and Slave.
 * The handle's TxCount/RxCount track progress.
 * ───────────────────────────────────────── */
void SPI1_IRQHandler(void)
{
    SPI_Handle_t *hSpi = &SPI1_Handle;
    u32 sr = hSpi->Instance->SR;
    u32 cr2 = hSpi->Instance->CR2;

    /* ── TXE: Transmit buffer empty ── */
    if (READ_BIT(sr, SPI_SR_TXE) && READ_BIT(cr2, SPI_CR2_TXEIE))
    {
        if (hSpi->TxCount > 0u)
        {
            /* Load next byte — index based on how many sent so far */
            u8 idx = SPI_PACKET_SIZE - hSpi->TxCount;
            hSpi->Instance->DR = hSpi->pTxBuffer[idx];
            hSpi->TxCount--;
        }
        else
        {
            /* All bytes sent — disable TXE interrupt */
            CLEAR_BIT(hSpi->Instance->CR2, SPI_CR2_TXEIE);
            hSpi->TxComplete = 1u;

            /* Master: disable CS after last byte is shifted out
             * Wait for BSY to clear first                        */
            if (READ_BIT(hSpi->Instance->CR1, SPI_CR1_MSTR))
            {
                while (READ_BIT(hSpi->Instance->SR, SPI_SR_BSY));
                SPI_CS_Disable();
            }
        }
    }

    /* ── RXNE: Receive buffer not empty ── */
    if (READ_BIT(sr, SPI_SR_RXNE) && READ_BIT(cr2, SPI_CR2_RXNEIE))
    {
        if (hSpi->RxCount > 0u)
        {
            u8 idx = SPI_PACKET_SIZE - hSpi->RxCount;
            hSpi->pRxBuffer[idx] = (u8)(hSpi->Instance->DR);
            hSpi->RxCount--;

            if (hSpi->RxCount == 0u)
            {
                /* All bytes received — disable RXNE interrupt */
                CLEAR_BIT(hSpi->Instance->CR2, SPI_CR2_RXNEIE);
                hSpi->RxComplete = 1u;
                hSpi->State      = SPI_STATE_READY;

                /* Slave: preload DR for the NEXT transfer immediately
                 * so it's always ready before Master drives CS low   */
                if (!READ_BIT(hSpi->Instance->CR1, SPI_CR1_MSTR))
                {
                    /* Slave preloads first byte of next TX packet
                     * The IPC layer will update pTxBuffer before this */
                    if (hSpi->pTxBuffer != NULL)
                    {
                        hSpi->Instance->DR = hSpi->pTxBuffer[0];
                        hSpi->TxCount = SPI_PACKET_SIZE - 1u;
                    }
                }
            }
        }
    }

    /* ── OVR: Overrun error — clear it ── */
    if (READ_BIT(sr, SPI_SR_OVR))
    {
        /* Clear OVR: read DR then SR */
        volatile u8 dummy = (u8)hSpi->Instance->DR;
        dummy = (u8)hSpi->Instance->SR;
        (void)dummy;
        hSpi->State = SPI_STATE_READY;
    }
}
