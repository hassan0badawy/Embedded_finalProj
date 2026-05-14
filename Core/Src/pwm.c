/**
 * Pwm.c
 * PWM driver using TIM1 Channel 1 (PA8, AF1).
 *
 * Hardware math (16MHz HSI, APB2 = 16MHz, TIM1 input clock = 16MHz):
 *   Target frequency: 10 kHz
 *   PSC = 0   → Timer clock = 16MHz / 1 = 16MHz
 *   ARR = 1599 → Frequency = 16,000,000 / (0+1) / (1599+1) = 10,000 Hz ✓
 *
 * Duty cycles:
 *   Stop:  CCR = 0    (0%)
 *   Slow:  CCR = 320  (20%)
 *   Full:  CCR = 1599 (100%)
 */
#include "Pwm.h"
#include "stm32f401ve.h"
#include "RCC.h"
#include "Gpio.h"
#include "Bit_Math.h"

#define PWM_PSC         0u
#define PWM_ARR         1599u
#define PWM_DUTY_STOP   0u
#define PWM_DUTY_SLOW   320u
#define PWM_DUTY_FULL   1599u

void Pwm_Init(void)
{
    /* 1. Enable TIM1 clock on APB2 */
    RCC_EnableClock(RCC_TIM1);

    /* 2. Configure PA8 as TIM1_CH1 (AF1, Push-Pull) */
    Gpio_Init(GPIO_A, 8, GPIO_AF, GPIO_PUSH_PULL);
    Gpio_SetAF(GPIO_A, 8, GPIO_AF1);

    /* 3. Time-base: PSC=0, ARR=1599 → 10kHz */
    TIM1->CR1  = 0;
    TIM1->PSC  = PWM_PSC;
    TIM1->ARR  = PWM_ARR;
    TIM1->RCR  = 0;
    TIM1->CNT  = 0;

    /* 4. OC1 → PWM Mode 1 + preload enable on CH1
     *    CCMR1 bits[6:4] = OC1M = 110 (PWM1)
     *    CCMR1 bit[3]    = OC1PE = 1  (preload)
     */
    TIM1->CCMR1 = TIM_CCMR_OC1M_PWM1 | TIM_CCMR_OC1PE;

    /* 5. Enable CH1 output (CC1E) */
    SET_BIT(TIM1->CCER, TIM_CCER_CC1E);

    /* 6. Set initial duty = 0 */
    TIM1->CCR1 = 0u;

    /* 7. Auto-reload preload + force update to load shadow registers */
    SET_BIT(TIM1->CR1, TIM_CR1_ARPE);
    SET_BIT(TIM1->EGR, TIM_EGR_UG);
    TIM1->SR = 0;

    /* 8. TIM1 is an Advanced Timer — must set MOE (Main Output Enable) */
    TIM1->BDTR |= (1u << TIM_BDTR_MOE);

    /* 9. Start counter */
    SET_BIT(TIM1->CR1, TIM_CR1_CEN);
}

void Pwm_SetDuty(u8 duty_percent)
{
    u32 ccr;

    if (duty_percent == 0u) {
        ccr = PWM_DUTY_STOP;
    } else if (duty_percent >= 100u) {
        ccr = PWM_DUTY_FULL;
    } else {
        ccr = ((u32)duty_percent * PWM_ARR) / 100u;
    }

    TIM1->CCR1 = ccr;
}

void Pwm_Stop(void)
{
    TIM1->CCR1 = 0u;
}
