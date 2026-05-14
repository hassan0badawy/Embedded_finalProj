#include "stm32f401xe.h"
#include "Gpio.h"
#include "Spi.h"
#include "shared.h"

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
    SPI1->CR1 |= (1 << SPI_CR1_SSM_Pos) | (1 << SPI_CR1_SSI_Pos);
    
    SPI1->CR1 &= ~(1 << SPI_CR1_MSTR_Pos);
    SPI1->CR1 |= (MasterSlave << SPI_CR1_MSTR_Pos);

    SPI1->CR1 &= ~(1 << SPI_CR1_CPOL_Pos);
    SPI1->CR1 |= (ClkPol << SPI_CR1_CPOL_Pos);
    SPI1->CR1 &= ~(1 << SPI_CR1_CPHA_Pos);
    SPI1->CR1 |= (ClkPhase << SPI_CR1_CPHA_Pos);

    // Baud rate 4MHz
    SPI1->CR1 &= ~(0x7 << SPI_CR1_BR_Pos);
    SPI1->CR1 |= (0x1 << SPI_CR1_BR_Pos);

    /* 4. Interrupts & NVIC (Required for 30% IPC Reliability) */
    SPI1->CR2 |= (1 << SPI_CR2_RXNEIE_Pos); 
    NVIC_SetPriority(SPI1_IRQn, 1); // High Priority 
    NVIC_EnableIRQ(SPI1_IRQn);

    SPI1->CR1 |= (1 << SPI_CR1_SPE_Pos);
}

void Spi1_CS_Select(void) {
    Gpio_WritePin(IPC_CS_PORT, IPC_CS_PIN, 0); 
}

void Spi1_CS_Release(void) {
    Gpio_WritePin(IPC_CS_PORT, IPC_CS_PIN, 1);
}

/* ISR to handle non-blocking SPI data exchange [cite: 17, 41] */
void SPI1_IRQHandler(void) {
    if (SPI1->SR & (1 << SPI_SR_RXNE_Pos)) {
        uint8 data = (uint8)SPI1->DR;
        
        /* Protect shared state using Critical Section [cite: 29, 42] */
        uint32 status = Enter_Critical();
        // Forward 'data' to your IPC buffer logic here
        Exit_Critical(status);
    }
}