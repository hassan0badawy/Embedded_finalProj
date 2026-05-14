#ifndef RCC_H
#define RCC_H

#include "std_types.h"

/**
 * Peripheral Enum for Clock Management
 */
typedef enum {
    RCC_GPIOA,
    RCC_GPIOB,
    RCC_GPIOC,
    RCC_GPIOD,
    RCC_GPIOE,
    RCC_ADC1,
    RCC_TIM2,
    RCC_TIM3,
    RCC_TIM4,
    RCC_TIM5
} Peripheral_t;

/**
 * Initialize the system clock to 16MHz using HSI
 */
void RCC_InitSystemClock(void);

/**
 * Enable the clock for a specific peripheral
 */
void RCC_EnableClock(Peripheral_t peripheral);

#endif /* RCC_H */
