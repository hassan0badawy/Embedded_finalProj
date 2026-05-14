/**
 * Elevator.h
 * FSM, GPIO, EXTI, and Telemetry (TIM6) management.
 */
#ifndef ELEVATOR_H
#define ELEVATOR_H

#include "std_types.h"
#include "stm32f401ve.h"
#include "shared.h"
#include "Pwm.h"

#define DOOR_OPEN_TICKS     6u      /* ~3s at 500ms tick */

void Elevator_Init(void);
void Elevator_Update(void);
void System_Logger(void);
void Poll_Inputs(void);

/* IRQ Handlers for EXTI and TIM6 */
void EXTI0_IRQHandler(void);
void EXTI1_IRQHandler(void);
void EXTI2_IRQHandler(void);
void EXTI3_IRQHandler(void);
void EXTI9_5_IRQHandler(void);
void EXTI15_10_IRQHandler(void);
void TIM6_DAC_IRQHandler(void);

#endif /* ELEVATOR_H */
