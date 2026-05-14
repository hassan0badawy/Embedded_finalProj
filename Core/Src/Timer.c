#include "Timer.h"
#include "Timer_Private.h"
#include "../Bit_Math.h"
#include "../NVIC.h"
#include "../RCC/RCC.h"

#define TIM_SR_UIF    (1u << 0)
#define TIM_DIER_UIE  (1u << 0)
#define TIM_CR1_CEN   (1u << 0)

#define TIM2          ((TimerType *)TIM2_BASE_ADDR)

static volatile uint32_t tick_ms = 0;

void TIM2_IRQHandler(void)
{
    if (TIM2->SR & TIM_SR_UIF) {
        TIM2->SR &= ~TIM_SR_UIF;
        tick_ms++;
    }
}

void Timer_Init(void)
{
    /* TIM2 clock enable is handled in main.c via Rcc_Enable(RCC_TIM2) */
    RCC_EnableClock(RCC_TIM2);
    
    /* Timer Clock = 8MHz (16MHz/4 * 2) */
    /* PSC = 16-1 -> Timer runs at 500kHz (8MHz/16) */
    TIM2->PSC = 15U;
    
    /* ARR = 499 (for exactly 1ms at 8MHz clock with PSC=16) */
    /* 500,000 / 500 = 1,000 Hz */
    TIM2->ARR = 499;
    
    /* Force update to load PSC and ARR into shadow registers */
    TIM2->EGR |= 0x01; 
    /* Clear the UIF flag that the EGR just set so we don't fire an IRQ immediately */
    TIM2->SR &= ~TIM_SR_UIF;

    TIM2->DIER |= TIM_DIER_UIE;
    TIM2->CR1  |= TIM_CR1_CEN;

    /* Enable TIM2 IRQ in NVIC (IRQ 28) */
    NVIC_ENABLE_IRQ(28);
}

void delay_ms(uint32_t ms)
{
    uint32_t start = tick_ms;
    while ((tick_ms - start) < ms)
    {
        /* Wait For Interrupt: Tells Proteus the CPU is idle.
           This significantly speeds up simulation by skipping cycles. */
        __asm volatile ("wfi");
    }
}

void delay_us(uint32_t us)
{
    /* At 16MHz, roughly 4 cycles per loop */
    volatile uint32_t count = us * 4; 
    while (count--);
}

uint32_t Timer_GetMs(void)
{
    return tick_ms;
}
