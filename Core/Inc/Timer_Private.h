#ifndef TIMER_PRIVATE_H
#define TIMER_PRIVATE_H

#include "std_types.h"

#define TIM1_BASE_ADDR    0x40010000UL
#define TIM2_BASE_ADDR    0x40000000UL
#define TIM3_BASE_ADDR    0x40000400UL
#define TIM4_BASE_ADDR    0x40000800UL
#define TIM5_BASE_ADDR    0x40000C00UL

typedef struct {
    volatile u32 CR1;
    volatile u32 CR2;
    volatile u32 SMCR;
    volatile u32 DIER;
    volatile u32 SR;
    volatile u32 EGR;
    volatile u32 CCMR1;
    volatile u32 CCMR2;
    volatile u32 CCER;
    volatile u32 CNT;
    volatile u32 PSC;
    volatile u32 ARR;
    u32 Reserved0;
    volatile u32 CCR1;
    volatile u32 CCR2;
    volatile u32 CCR3;
    volatile u32 CCR4;
    volatile u32 BDTR;
} TimerType;

/* Register Bit Definitions */
#define CR1_CEN       0
#define CR1_OPM       3
#define SR_UIF        0
#define EGR_UG        0
#define DIER_UIE      0

/* CCMR Definitions */
#define CCMR_OC_TOGGLE 0x3u

/* Timer IDs */
#define TIMER1        0
#define TIMER2        1
#define TIMER3        2
#define TIMER4        3
#define TIMER5        4

#endif
