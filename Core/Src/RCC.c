#include "RCC.h"
#include "shared.h"

/* ── Flash Register Definitions for Latency ── */
#define FLASH_BASE_ADDR    0x40023C00UL
#define FLASH_ACR          (*(volatile u32*)(FLASH_BASE_ADDR + 0x00))

void RCC_InitSystemClock(void)
{
    /* 1. Enable HSI Oscillator */
    RCC->CR |= (1u << 0); /* HSION */

    /* 2. Wait until HSI is ready */
    while (!(RCC->CR & (1u << 1))); /* HSIRDY */

    /* 3. Configure Flash Latency (0 WS for 16MHz) */
    FLASH_ACR &= ~(0x7u << 0);
    FLASH_ACR |=  (0x0u << 0);

    /* 4. Configure Bus Prescalers: APB2 = AHB/4, APB1 = AHB/4 */
    RCC->CFGR &= ~(0xFu << 4);  /* AHB  = /1 */
    RCC->CFGR &= ~(0x7u << 10); 
    RCC->CFGR |=  (0x5u << 10); /* PPRE1: div 4 */
    RCC->CFGR &= ~(0x7u << 13); 
    RCC->CFGR |=  (0x5u << 13); /* PPRE2: div 4 */

    /* 5. Switch System Clock to HSI */
    RCC->CFGR &= ~(0x3u << 0);  /* SW = 00 (HSI) */
    
    /* 6. Wait until HSI is used as system clock */
    while ((RCC->CFGR & (0x3u << 2)) != (0x0u << 2)); /* SWS */
}

void RCC_EnableClock(Peripheral_t peripheral)
{
    switch (peripheral)
    {
        /* AHB1 Peripherals (GPIOs) */
        case RCC_GPIOA: RCC->AHB1ENR |= (1u << 0); break;
        case RCC_GPIOB: RCC->AHB1ENR |= (1u << 1); break;
        case RCC_GPIOC: RCC->AHB1ENR |= (1u << 2); break;
        case RCC_GPIOD: RCC->AHB1ENR |= (1u << 3); break;
        case RCC_GPIOE: RCC->AHB1ENR |= (1u << 4); break;

        /* APB2 Peripherals (ADC) */
        case RCC_ADC1:  RCC->APB2ENR |= (1u << 8); break;

        /* APB1 Peripherals (Timers) */
        case RCC_TIM2:  RCC->APB1ENR |= (1u << 0); break;
        case RCC_TIM3:  RCC->APB1ENR |= (1u << 1); break;
        case RCC_TIM4:  RCC->APB1ENR |= (1u << 2); break;
        case RCC_TIM5:  RCC->APB1ENR |= (1u << 3); break;

        default: break;
    }
    
    /* Short delay for clock stabilization */
    for (volatile int i = 0; i < 100; i++);
}
