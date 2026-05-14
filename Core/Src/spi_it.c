#include "spi_it.h"
#include "stm32f401ve.h"
#include "Bit_Math.h"
#include "ipc.h"
#include "shared.h"
#include "../Inc/Gpio.h"

/* Simple interrupt-driven SPI1 implementation to satisfy ipc.c expectations.
 * This is intentionally minimal: it tracks a TX/RX buffer and uses SPI1
 * TXE/RXNE interrupts to shift bytes. CS (PA4) is driven using GPIOA.
 */

SPI_HandleTypeDef SPI1_Handle;

static volatile u8 *g_txBuf = 0;
static u8 g_txLen = 0;
static u8 g_txIdx = 0;

static volatile u8 *g_rxBuf = 0;
static u8 g_rxLen = 0;
static u8 g_rxIdx = 0;

void SPI_CS_Enable(void)
{
    /* Pull PA4 low (BSRR upper half to reset) */
    Gpio_WritePin(GPIO_A, 4, 0);
}

void SPI_CS_Disable(void)
{
    /* Pull PA4 high (BSRR lower half to set) */
    Gpio_WritePin(GPIO_A, 4, 1);
}

void SPI_MasterInit(SPI_HandleTypeDef *h)
{
    h->Instance = SPI1;
    h->State = SPI_STATE_READY;
    h->pRxBuffer = 0;
    h->RxCount = 0;
    h->RxComplete = 0;

    /* Mark SPI as master in CR1 so ipc.c can detect role */
    SET_BIT(SPI1->CR1, SPI_CR1_MSTR);

    /* Enable SPI1 IRQ in NVIC and clear interrupt enables in CR2 */
    NVIC_ENABLE_IRQ(IRQ_SPI1);
    CLEAR_BIT(SPI1->CR2, SPI_CR2_RXNEIE);
    CLEAR_BIT(SPI1->CR2, SPI_CR2_TXEIE);
}

void SPI_SlaveInit(SPI_HandleTypeDef *h)
{
    h->Instance = SPI1;
    h->State = SPI_STATE_READY;
    h->pRxBuffer = 0;
    h->RxCount = 0;
    h->RxComplete = 0;

    /* Clear master bit to indicate slave role */
    CLEAR_BIT(SPI1->CR1, SPI_CR1_MSTR);

    /* Ensure interrupts are disabled until transfer starts */
    NVIC_ENABLE_IRQ(IRQ_SPI1);
    CLEAR_BIT(SPI1->CR2, SPI_CR2_RXNEIE);
    CLEAR_BIT(SPI1->CR2, SPI_CR2_TXEIE);
}

void SPI_SlavePreload(SPI_HandleTypeDef *h, volatile u8 *pTxBuffer)
{
    (void)h;
    /* Preload first byte into DR so Master will read it when clocked */
    u8 first = pTxBuffer[0];
    /* Wait until TXE then write */
    while (!READ_BIT(SPI1->SR, SPI_SR_TXE)) {}
    SPI1->DR = (u32)first;
}

void SPI_TransmitReceive_IT(SPI_HandleTypeDef *h, volatile u8 *pTx, volatile u8 *pRx, u8 len)
{
    if (h->State != SPI_STATE_READY) return;

    /* Setup transfer state */
    g_txBuf = pTx;
    g_txLen = len;
    g_txIdx = 0;

    g_rxBuf = pRx;
    g_rxLen = len;
    g_rxIdx = 0;

    h->pRxBuffer = pRx;
    h->RxCount = len;
    h->RxComplete = 0;
    h->State = SPI_STATE_BUSY;

    /* Enable RXNE and TXE interrupts */
    SET_BIT(SPI1->CR2, SPI_CR2_RXNEIE);
    SET_BIT(SPI1->CR2, SPI_CR2_TXEIE);

    /* Kickstart transfer: if TXE, write first byte */
    if (READ_BIT(SPI1->SR, SPI_SR_TXE))
    {
        SPI1->DR = (u32)g_txBuf[g_txIdx++];
    }

    /* NVIC is already enabled in init; ISR will finish and clear CS */
}

/* SPI1 global IRQ handler — called from vector table */
void SPI1_IRQHandler(void)
{
    u32 sr = SPI1->SR;

    /* RXNE: read received byte */
    if (READ_BIT(sr, SPI_SR_RXNE))
    {
        u8 val = (u8)(SPI1->DR & 0xFFu);
        if (g_rxBuf && (g_rxIdx < g_rxLen))
        {
            g_rxBuf[g_rxIdx++] = val;
        }

        /* If we've received the expected number of bytes, finish */
        if (g_rxIdx >= g_rxLen)
        {
            /* Disable interrupts */
            CLEAR_BIT(SPI1->CR2, SPI_CR2_RXNEIE);
            CLEAR_BIT(SPI1->CR2, SPI_CR2_TXEIE);

            /* Mark handle complete */
            SPI1_Handle.RxComplete = 1u;
            SPI1_Handle.State = SPI_STATE_READY;

            /* If master, release CS */
            if (READ_BIT(SPI1->CR1, SPI_CR1_MSTR))
            {
                SPI_CS_Disable();
            }
        }
    }

    /* TXE: transmit next byte if available */
    if (READ_BIT(sr, SPI_SR_TXE))
    {
        if (g_txBuf && (g_txIdx < g_txLen))
        {
            SPI1->DR = (u32)g_txBuf[g_txIdx++];
        }
        else
        {
            /* Nothing left to send — disable TXE interrupt */
            CLEAR_BIT(SPI1->CR2, SPI_CR2_TXEIE);
        }
    }
}
