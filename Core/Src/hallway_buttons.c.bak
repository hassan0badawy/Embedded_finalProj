#include "gpio.h"
#include "stm32f401ve.h"
#include "dispatcher.h"
#include "shared.h" /* For RCC access */

/* 
 * Hallway Buttons (Master MCU)
 * PB0: U1, PB1: D2, PB2: U2, PB3: D3, PB4: U3, PB5: D4
 */

void HallwayButtons_Init(void) {
    /* 1. Enable GPIOB Clock */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

    /* 2. Configure PB0 to PB5 as Input with Pull-up */
    for (u8 i = 0; i <= 5; i++) {
        GPIOB->MODER &= ~(0x3u << (i * 2));     /* 00: Input Mode */
        GPIOB->PUPDR &= ~(0x3u << (i * 2));
        GPIOB->PUPDR |=  (0x1u << (i * 2));     /* 01: Pull-up    */
    }

    /* 3. Enable SYSCFG Clock (for EXTI mapping) */
    RCC->APB2ENR |= (1u << 14); /* SYSCFG clock enable */

    /* 4. Map PB0-PB5 to EXTI Lines */
    /* SYSCFG_EXTICR1 maps EXTI0-3, EXTICR2 maps EXTI4-7 */
    /* Each EXTICR has 4 bits per line. 0001 = Port B */
    u32 *pEXTICR = (u32 *)(SYSCFG_BASE + 0x08); /* SYSCFG_EXTICR1 */
    pEXTICR[0] = 0x1111; /* EXTI0, 1, 2, 3 -> Port B */
    pEXTICR[1] &= ~0xFF; 
    pEXTICR[1] |= 0x11;   /* EXTI4, 5 -> Port B       */

    /* 5. Configure EXTI Triggers (Falling Edge for Button Press) */
    EXTI->IMR  |= 0x3Fu;  /* Unmask EXTI0 to EXTI5 */
    EXTI->FTSR |= 0x3Fu;  /* Falling edge trigger  */
    EXTI->RTSR &= ~0x3Fu; /* No rising edge        */

    /* 6. Enable Interrupts in NVIC */
    NVIC_ENABLE_IRQ(IRQ_EXTI0);
    NVIC_ENABLE_IRQ(IRQ_EXTI1);
    NVIC_ENABLE_IRQ(IRQ_EXTI2);
    NVIC_ENABLE_IRQ(IRQ_EXTI3);
    NVIC_ENABLE_IRQ(IRQ_EXTI4);
    NVIC_ENABLE_IRQ(IRQ_EXTI9_5);
}

/* ─────────────────────────────────────────
 * INTERRUPT SERVICE ROUTINES (ISRs)
 * ───────────────────────────────────────── */

void EXTI0_IRQHandler(void) {
    if (EXTI->PR & (1u << 0)) {
        Dispatcher_RegisterCall(1, DIR_UP); /* U1 */
        EXTI->PR = (1u << 0); /* Clear pending bit */
    }
}

void EXTI1_IRQHandler(void) {
    if (EXTI->PR & (1u << 1)) {
        Dispatcher_RegisterCall(2, DIR_DOWN); /* D2 */
        EXTI->PR = (1u << 1);
    }
}

void EXTI2_IRQHandler(void) {
    if (EXTI->PR & (1u << 2)) {
        Dispatcher_RegisterCall(2, DIR_UP); /* U2 */
        EXTI->PR = (1u << 2);
    }
}

void EXTI3_IRQHandler(void) {
    if (EXTI->PR & (1u << 3)) {
        Dispatcher_RegisterCall(3, DIR_DOWN); /* D3 */
        EXTI->PR = (1u << 3);
    }
}

void EXTI4_IRQHandler(void) {
    if (EXTI->PR & (1u << 4)) {
        Dispatcher_RegisterCall(3, DIR_UP); /* U3 */
        EXTI->PR = (1u << 4);
    }
}

void EXTI9_5_IRQHandler(void) {
    if (EXTI->PR & (1u << 5)) {
        Dispatcher_RegisterCall(4, DIR_DOWN); /* D4 */
        EXTI->PR = (1u << 5);
    }
}
