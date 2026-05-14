/**
 * RCC.h
 * Clock management for STM32F401VE.
 * System clock: HSI 16MHz, all bus prescalers = /1.
 */
#ifndef RCC_H
#define RCC_H

#include "std_types.h"

typedef enum {
    /* AHB1 */
    RCC_GPIOA   = 0,
    RCC_GPIOB   = 1,
    RCC_GPIOC   = 2,
    RCC_GPIOD   = 3,
    RCC_DMA2    = 22,
    /* APB1 */
    RCC_TIM2    = 100,
    RCC_TIM3    = 101,
    RCC_TIM4    = 102,
    RCC_TIM5    = 103,
    RCC_TIM6    = 104,
    /* APB2 */
    RCC_TIM1    = 200,
    RCC_USART1  = 204,
    RCC_SPI1    = 212,
    RCC_SYSCFG  = 214
} RCC_Peripheral_t;

/**
 * Initialize system clock:
 *   HSI = 16MHz, AHB=/1, APB1=/1, APB2=/1
 *   Flash latency = 0 WS
 */
void RCC_InitSystemClock(void);

/**
 * Enable the peripheral clock.
 */
void RCC_EnableClock(RCC_Peripheral_t periph);

#endif /* RCC_H */
