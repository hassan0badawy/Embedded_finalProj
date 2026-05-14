/**
 * Timer.c
 * TIM2 configuration for 1ms system tick and 50ms IPC sync.
 *
 * Hardware Math:
 *   Clock: APB1 = 16 MHz
 *   Target Period: 1 ms
 *   PSC = 15   → Timer tick = 16,000,000 / 16 = 1,000,000 Hz (1us)
 *   ARR = 999  → Period = 1000 us = 1 ms ✓
 */
#include "Timer.h"
#include "stm32f401ve.h"
#include "RCC.h"
#include "shared.h"
#include "nvic.h"

static volatile uint32_t tick_ms = 0;

void TIM2_IRQHandler(void)
{
    if (TIM2->SR & (1u << TIM_SR_UIF)) {
        /* Clear update interrupt flag */
        TIM2->SR &= ~(1u << TIM_SR_UIF);
        
        tick_ms++;
        
        /* 50ms IPC Sync trigger */
        if ((tick_ms % 50u) == 0u) {
            GSS.ipc_tick_flag = 1u;
        }
        
        /* 500ms Telemetry and Door Tick trigger */
        if ((tick_ms % 500u) == 0u) {
            GSS.telem_flag = 1u;
            GSS.telem_tick = 1u;
        }
    }
}

void Timer_Init(void)
{
    /* Enable TIM2 clock */
    RCC_EnableClock(RCC_TIM2);
    
    /* Configure for 1ms interrupts */
    TIM2->PSC = 15u;
    TIM2->ARR = 999u;
    
    /* Force update to load shadow registers */
    TIM2->EGR |= (1u << TIM_EGR_UG);
    
    /* Clear flag before enabling interrupt */
    TIM2->SR &= ~(1u << TIM_SR_UIF);
    
    /* Enable update interrupt */
    TIM2->DIER |= (1u << TIM_DIER_UIE);
    
    /* Enable TIM2 IRQ in NVIC (Priority 3) */
    NVIC_SET_PRIORITY(IRQ_TIM2, 3u);
    NVIC_ENABLE_IRQ(IRQ_TIM2);
    
    /* Start timer */
    TIM2->CR1 |= (1u << TIM_CR1_CEN);
}

void delay_ms(uint32_t ms)
{
    uint32_t start = tick_ms;
    while ((tick_ms - start) < ms)
    {
        /* Wait For Interrupt: saves power / speeds up Proteus simulation */
        __asm volatile ("wfi");
    }
}

void delay_us(uint32_t us)
{
    /* Simple loop delay for microsecond resolution at 16MHz */
    volatile uint32_t count = us * 4u; 
    while (count--) {
        __asm volatile ("nop");
    }
}

uint32_t Timer_GetMs(void)
{
    return tick_ms;
}
