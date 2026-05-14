#include "../Inc/gpio.h"
#include "../Inc/stm32f401ve.h"
#include "../Inc/dispatcher.h"

/* Hallway Buttons wiring (handled by EXTI in Elevator.c):
 * PB6, PB7, PB8, PB9, PB10, PB12 — inputs with pull-up
 * This module only configures GPIO pull-ups; EXTI mapping
 * and ISRs are centralized in Elevator.c to avoid duplicates.
 */

void HallwayButtons_Init(void)
{
    /* Enable GPIOB clock */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

    /* Configure PB6,PB7,PB8,PB9,PB10,PB12 as inputs with pull-up */
    /* Clear MODER bits (inputs) and set PUPDR = 01 (pull-up) */
    GPIOB->MODER &= ~((0x3UL << (6*2)) | (0x3UL << (7*2)) |
                      (0x3UL << (8*2)) | (0x3UL << (9*2)) |
                      (0x3UL << (10*2))| (0x3UL << (12*2)));

    GPIOB->PUPDR &= ~((0x3UL << (6*2)) | (0x3UL << (7*2)) |
                      (0x3UL << (8*2)) | (0x3UL << (9*2)) |
                      (0x3UL << (10*2))| (0x3UL << (12*2)));

    GPIOB->PUPDR |=  ((0x1UL << (6*2)) | (0x1UL << (7*2)) |
                      (0x1UL << (8*2)) | (0x1UL << (9*2)) |
                      (0x1UL << (10*2))| (0x1UL << (12*2)));
}
