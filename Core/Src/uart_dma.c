/**
 * uart_dma.c
 * Configures USART1 and DMA2 Stream 7 for non-blocking transmission.
 */
#include "uart_dma.h"
#include "stm32f401ve.h"
#include "RCC.h"
#include "../INC/gpio.h"
#include "nvic.h"
#include "Bit_Math.h"

volatile u8 UART_TxBuf[UART_TX_BUF_SIZE];
volatile u8 UART_DMA_Busy = 0;

void UART_DMA_Init(void)
{
    /* 1. Enable clocks */
    RCC_EnableClock(RCC_USART1);
    RCC_EnableClock(RCC_DMA2);

    /* 2. Configure PA9 (TX) and PA10 (RX) as AF7 */
    Gpio_Init(GPIOA, 9, GPIO_AF, GPIO_PUSH_PULL);
    Gpio_Init(GPIOA, 10, GPIO_AF, GPIO_PUSH_PULL);
    Gpio_SetAF(GPIOA, 9, GPIO_AF7);
    Gpio_SetAF(GPIOA, 10, GPIO_AF7);

    /* 3. USART1 Configuration */
    USART1->CR1 = 0;
    
    /* Baud rate = 115200 @ 16MHz APB2
     * BRR = 16,000,000 / 115200 = 138.88 -> 0x008B 
     */
    USART1->BRR = 0x008Bu;

    /* Enable TX, RX, and USART */
    USART1->CR1 |= (1u << USART_CR1_TE) | (1u << USART_CR1_RE);
    
    /* Enable DMA Transmitter */
    USART1->CR3 |= (1u << USART_CR3_DMAT);
    
    USART1->CR1 |= (1u << USART_CR1_UE);

    /* 4. DMA2 Stream 7 Configuration (USART1_TX is Ch4) */
    DMA2_Stream7->CR = 0;
    
    /* Wait for stream to be disabled */
    while (DMA2_Stream7->CR & (1u << DMA_CR_EN));

    /* Channel 4 selection */
    DMA2_Stream7->CR |= (4u << DMA_CR_CHSEL_SHIFT);
    
    /* Memory to Peripheral */
    DMA2_Stream7->CR |= DMA_CR_DIR_MEM2PER;
    
    /* Memory increment mode */
    DMA2_Stream7->CR |= (1u << DMA_CR_MINC);
    
    /* Enable Transfer Complete Interrupt */
    DMA2_Stream7->CR |= (1u << DMA_CR_TCIE);

    /* Set Peripheral Address to USART1 DR */
    DMA2_Stream7->PAR = (u32)&USART1->DR;

    /* 5. Enable DMA2 Stream7 IRQ in NVIC */
    NVIC_SET_PRIORITY(IRQ_DMA2_STREAM7, 5u);
    NVIC_ENABLE_IRQ(IRQ_DMA2_STREAM7);
}

void UART_DMA_Transmit(u8 len)
{
    if (UART_DMA_Busy) return;
    if (len == 0 || len > UART_TX_BUF_SIZE) return;

    UART_DMA_Busy = 1;

    /* Disable stream before configuration */
    DMA2_Stream7->CR &= ~(1u << DMA_CR_EN);
    while (DMA2_Stream7->CR & (1u << DMA_CR_EN));

    /* Clear all interrupt flags for Stream 7 */
    DMA2->HIFCR = DMA2_HIFCR_CTCIF7 | DMA2_HIFCR_CTEIF7 | 
                  DMA2_HIFCR_CDMEIF7 | DMA2_HIFCR_CFEIF7;

    /* Set memory address and transfer size */
    DMA2_Stream7->M0AR = (u32)UART_TxBuf;
    DMA2_Stream7->NDTR = len;

    /* Enable stream */
    DMA2_Stream7->CR |= (1u << DMA_CR_EN);
}

void DMA2_Stream7_IRQHandler(void)
{
    /* Check Transfer Complete flag */
    if (DMA2->HISR & DMA2_HISR_TCIF7) {
        /* Clear flag */
        DMA2->HIFCR = DMA2_HIFCR_CTCIF7;
        UART_DMA_Busy = 0;
    }
}
