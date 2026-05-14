/**
 * RCC.c
 * Clock initialization for STM32F401VE.
 *
 * System Clock: HSI = 16 MHz
 * AHB  Prescaler: /1 → HCLK  = 16 MHz
 * APB1 Prescaler: /1 → PCLK1 = 16 MHz  (TIM2, TIM6, SPI2)
 * APB2 Prescaler: /1 → PCLK2 = 16 MHz  (TIM1, SPI1, USART1)
 */
#include "RCC.h"
#include "stm32f401ve.h"

void RCC_InitSystemClock(void)
{
    /* 1. Enable HSI */
    RCC->CR |= (1u << 0);                   /* HSION */
    while (!(RCC->CR & (1u << 1)));         /* Wait HSIRDY */

    /* 2. Flash latency = 0 WS (correct for 16MHz) */
    FLASH_ACR &= ~(0x7u << 0);

    /* 3. AHB prescaler = /1 (HPRE = 0000) */
    RCC->CFGR &= ~(0xFu << 4);

    /* 4. APB1 prescaler = /1 (PPRE1 = 000) */
    RCC->CFGR &= ~(0x7u << 10);

    /* 5. APB2 prescaler = /1 (PPRE2 = 000) */
    RCC->CFGR &= ~(0x7u << 13);

    /* 6. Select HSI as system clock (SW = 00) */
    RCC->CFGR &= ~(0x3u << 0);

    /* 7. Wait until HSI selected (SWS = 00) */
    while ((RCC->CFGR & (0x3u << 2)) != 0u);
}

void RCC_EnableClock(RCC_Peripheral_t periph)
{
    u32 bit = (u32)periph;

    if (bit < 32u)
    {
        /* AHB1 peripheral */
        RCC->AHB1ENR |= (1u << bit);
    }
    else if (bit < 200u)
    {
        /* APB1 peripheral (enum value = 100 + bit position) */
        RCC->APB1ENR |= (1u << (bit - 100u));
    }
    else
    {
        /* APB2 peripheral (enum value = 200 + bit position) */
        RCC->APB2ENR |= (1u << (bit - 200u));
    }

    /* Short settling delay (clock domain crossing) */
    (void)RCC->APB2ENR;
}
