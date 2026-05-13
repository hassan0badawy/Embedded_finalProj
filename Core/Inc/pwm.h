#ifndef PWM_H
#define PWM_H

#include "std_types.h"
#include "stm32f401ve.h"

/* 
 * TIM1 on PA8 (AF1) 
 * Clock = 84MHz. 
 * Target = 10kHz.
 * Formula: Freq = Clock / ((PSC+1)*(ARR+1))
 * 10,000 = 84,000,000 / ((83+1)*(99+1)) = 10,000 Hz
 */
#define PWM_TIM             TIM1
#define PWM_PSC             83u
#define PWM_ARR             99u

#define PWM_DUTY_STOP       0u
#define PWM_DUTY_SLOW       20u
#define PWM_DUTY_FULL       99u

void PWM_Init(void);
void PWM_SetDuty(u8 duty);

#endif /* PWM_H */