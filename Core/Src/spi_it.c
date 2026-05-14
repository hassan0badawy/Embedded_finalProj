/**
 * spi_it.c
 * Interrupt-driven non-blocking SPI1 implementation for IPC.
 */
#include "spi_it.h"
#include "Spi.h"
#include "Bit_Math.h"

void SPI_MasterTransfer_IT(SPI_HandleTypeDef *h,
                           volatile u8 *pTx,
                           volatile u8 *pRx,
                           u8 len)
{
    if (h->State != SPI_STATE_READY) return;

    h->pTxBuf = pTx;
    h->pRxBuf = pRx;
    h->Len = len;
    h->TxIdx = 0;
    h->RxIdx = 0;
    h->RxComplete = 0;
    h->State = SPI_STATE_BUSY;

    /* Assert CS before starting transfer */
    SPI1_CS_Assert();

    /* Enable TXE and RXNE interrupts */
    SPI1->CR2 |= (1u << SPI_CR2_TXEIE) | (1u << SPI_CR2_RXNEIE);

    /* Kickstart transmission if TXE is ready */
    if (SPI1->SR & (1u << SPI_SR_TXE)) {
        SPI1->DR = h->pTxBuf[h->TxIdx++];
    }
}

void SPI_SlavePreload(SPI_HandleTypeDef *h, volatile u8 *pTx)
{
    /* Preload the first byte into the Data Register NOW */
    u8 first_byte = pTx[0];

    h->pTxBuf = pTx;
    h->pRxBuf = h->RxBuffer;
    h->Len = IPC_PACKET_SIZE;
    h->TxIdx = 1;  /* We already preloaded byte 0 */
    h->RxIdx = 0;
    h->RxComplete = 0;
    h->State = SPI_STATE_BUSY;

    /* Write the first byte */
    SPI1->DR = first_byte;

    /* Wait for CS to go low before enabling TXE interrupts */
    /* Only enable RXNE interrupt initially */
    SPI1->CR2 |= (1u << SPI_CR2_RXNEIE);
}

void SPI1_IRQHandler(void)
{
    SPI_HandleTypeDef *h = &SPI1_Handle;
    u32 sr = SPI1->SR;

    /* RXNE: Received a byte */
    if (sr & (1u << SPI_SR_RXNE)) {
        u8 received = (u8)SPI1->DR;
        if (h->RxIdx < h->Len) {
            h->pRxBuf[h->RxIdx++] = received;
        }

        /* If we have received all bytes */
        if (h->RxIdx >= h->Len) {
            /* Disable both TX and RX interrupts */
            SPI1->CR2 &= ~((1u << SPI_CR2_TXEIE) | (1u << SPI_CR2_RXNEIE));
            h->State = SPI_STATE_READY;
            h->RxComplete = 1;

            /* If Master, release CS */
            if (SPI1->CR1 & (1u << SPI_CR1_MSTR)) {
                SPI1_CS_Release();
            }
        }
    }

    /* TXE: Transmit buffer is empty and ready for next byte */
    if (sr & (1u << SPI_SR_TXE)) {
        if (h->TxIdx < h->Len) {
            SPI1->DR = h->pTxBuf[h->TxIdx++];
        } else {
            /* Nothing left to send, disable TXE interrupt */
            SPI1->CR2 &= ~(1u << SPI_CR2_TXEIE);
        }
    }
}
