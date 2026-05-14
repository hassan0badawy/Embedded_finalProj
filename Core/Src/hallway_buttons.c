#include "../Inc/gpio.h"
#include "../Inc/Gpio_Private.h"
#include "../Inc/stm32f401ve.h"
#include "../Inc/dispatcher.h"

/* Hallway Buttons wiring (handled by EXTI in Elevator.c):
 * PB6, PB7, PB8, PB9, PB10, PB12 — inputs with pull-up
 * This module only configures GPIO pull-ups; EXTI mapping
 * and ISRs are centralized in Elevator.c to avoid duplicates.
 */

void HallwayButtons_Init(void)
{
    /* Use the existing Gpio_Init for consistency */
    Gpio_Init(GPIO_B, 6,  GPIO_INPUT, GPIO_PULL_UP);
    Gpio_Init(GPIO_B, 7,  GPIO_INPUT, GPIO_PULL_UP);
    Gpio_Init(GPIO_B, 8,  GPIO_INPUT, GPIO_PULL_UP);
    Gpio_Init(GPIO_B, 9,  GPIO_INPUT, GPIO_PULL_UP);
    Gpio_Init(GPIO_B, 10, GPIO_INPUT, GPIO_PULL_UP);
    Gpio_Init(GPIO_B, 12, GPIO_INPUT, GPIO_PULL_UP);
}
