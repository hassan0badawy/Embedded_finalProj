#include "stm32f401ve.h"
#include "../Inc/Gpio.h"
#include "../Inc/spi_it.h"
#include "../Inc/ipc.h"
#include "../Inc/Bit_Math.h"

#include "../Inc/shared.h"

void Spi1_Init(uint8 MasterSlave, uint8 ClkPol, uint8 ClkPhase) {
    /* 1. SPI Pins on Port B (SCK: B3, MISO: B4, MOSI: B5) */
    Gpio_Init(GPIO_B, 3, GPIO_AF, GPIO_PUSH_PULL);
    Gpio_Init(GPIO_B, 4, GPIO_AF, GPIO_PUSH_PULL);
    Gpio_Init(GPIO_B, 5, GPIO_AF, GPIO_PUSH_PULL);
    Gpio_SetAF(GPIO_B, 3, GPIO_AF5);
    Gpio_SetAF(GPIO_B, 4, GPIO_AF5);
    Gpio_SetAF(GPIO_B, 5, GPIO_AF5);

    /* 2. PC13 as Manual Chip Select (Avoids B12 and A4 conflicts) */
    Gpio_Init(IPC_CS_PORT, IPC_CS_PIN, GPIO_OUTPUT, GPIO_PUSH_PULL);
    Spi1_CS_Release(); // High = Deselected

    /* 3. SPI Hardware Configuration */
    SPI1->CR1 |= (1 << SPI_CR1_SSM) | (1 << SPI_CR1_SSI);
    
    SPI1->CR1 &= ~(1 << SPI_CR1_MSTR);
    SPI1->CR1 |= (MasterSlave << SPI_CR1_MSTR);

    SPI1->CR1 &= ~(1 << SPI_CR1_CPOL);
    SPI1->CR1 |= (ClkPol << SPI_CR1_CPOL);
    SPI1->CR1 &= ~(1 << SPI_CR1_CPHA);
    SPI1->CR1 |= (ClkPhase << SPI_CR1_CPHA);

    // Baud rate 4MHz
    SPI1->CR1 &= ~(0x7 << SPI_CR1_BR0);
    SPI1->CR1 |= (0x1 << SPI_CR1_BR0);

    /* 4. Interrupts & NVIC (Required for 30% IPC Reliability) */
    SPI1->CR2 |= (1 << SPI_CR2_RXNEIE); 
    NVIC_SET_PRIORITY(IRQ_SPI1, 1); // High Priority 
    NVIC_ENABLE_IRQ(IRQ_SPI1);

    SPI1->CR1 |= (1 << SPI_CR1_SPE);
}

void Spi1_CS_Select(void) {
    Gpio_WritePin(IPC_CS_PORT, IPC_CS_PIN, 0); 
}

void Spi1_CS_Release(void) {
    Gpio_WritePin(IPC_CS_PORT, IPC_CS_PIN, 1);
}

/* ─────────────────────────────────────────
 * SPI_SlavePreload()
 * ─────────────────────────────────────────
 * SLAVE ONLY: Pre-load first byte into SPI DR
 * BEFORE Master initiates the transfer.
 *
 * Non-blocking Slave architecture:
 *   1. Slave calls SPI_SlavePreload() in main loop
 *   2. First byte goes into SPI->DR immediately
 *   3. ISR handles remaining bytes via interrupt
 *   4. When Master drives CS low, Slave is READY
 * ───────────────────────────────────────── */
void SPI_SlavePreload(SPI_HandleTypeDef *pHandle, volatile u8 *pBuf)
{
    if (!pBuf) return;
    
    /* 1. Preload first byte into DR NOW (before Master clock starts) */
    SPI1->DR = pBuf[0];
    
    /* 2. Set up handle for ISR to handle remaining 7 bytes */
    pHandle->pTxBuffer = (volatile u8 *)(&pBuf[1]);  /* Start from byte 1 */
    pHandle->TxCount   = (IPC_PACKET_SIZE - 1u);    /* 7 bytes remaining */
    pHandle->pRxBuffer = pHandle->RxBuffer;         /* RX buffer for incoming data */
    pHandle->RxCount   = IPC_PACKET_SIZE;           /* Expect all 8 bytes */
    
    /* 3. Ensure SPI RX interrupt is enabled for byte-by-byte handling */
    SET_BIT(SPI1->CR2, SPI_CR2_RXNEIE);
}

/* ISR to handle non-blocking SPI data exchange [cite: 17, 41] */
void SPI1_IRQHandler(void) {
    if (SPI1->SR & (1 << SPI_SR_RXNE)) {
        uint8 data = (uint8)SPI1->DR;
        
        /* Protect shared state using Critical Section [cite: 29, 42] */
        uint32 status = Enter_Critical();
        // Forward 'data' to your IPC buffer logic here
        Exit_Critical(status);
    }
}