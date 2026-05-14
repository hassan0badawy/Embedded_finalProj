/**
 * Pwm.h
 * PWM driver interface — TIM1 CH1 (PA8, AF1).
 * Simplified API: no TimerId/Channel arguments since we use TIM1 only.
 */
#ifndef PWM_H
#define PWM_H

#include "std_types.h"

/* Motor duty cycle levels (%) */
#define PWM_DUTY_STOP   0u
#define PWM_DUTY_SLOW   20u
#define PWM_DUTY_FULL   100u

void Pwm_Init(void);
void Pwm_SetDuty(u8 duty_percent);
void Pwm_Stop(void);

#endif /* PWM_H */
