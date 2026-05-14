#ifndef TIMER_H
#define TIMER_H

#include "std_types.h"
#include "Timer_Private.h"

/* Core Functions */
void Timer_Init(void);
void delay_ms(uint32_t ms);
void delay_us(uint32_t us);

/* System Time Helper */
uint32_t Timer_GetMs(void);

#endif
