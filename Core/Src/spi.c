#include "../Inc/stm32f401ve.h"
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
 * SPI_SlavePreload and IRQHandler are now
 * implemented in spi_it.c to support the
 * interrupt-driven IPC architecture.
 * ───────────────────────────────────────── */
