#ifndef ELEVATOR_H
#define ELEVATOR_H

#include "std_types.h"
#include "stm32f401ve.h"
#include "Bit_Math.h"
#include "ipc.h"        /* ElevatorState_t, IPC_Frame_t */
#include "pwm.h"
#include "uart_dma.h"
#include "shared.h"     /* GlobalSharedState_t, GSS — SINGLE unified struct */

/* ─────────────────────────────────────────
 * SYSTEM CONFIGURATION
 * ───────────────────────────────────────── */
#define NUM_FLOORS          4u      /* Floors: 0 = F1, 1 = F2, 2 = F3, 3 = F4 */
#define DOOR_OPEN_TICKS     6u      /* ~3s at 500ms tick: doors stay open       */

/* ─────────────────────────────────────────
 * PIN DEFINITIONS — EXTI INPUTS
 * ─────────────────────────────────────────
 * FINAL EXTI MAPPING (no pin number overlap):
 *   EXTI0  → PD0  (Emergency Stop)   NVIC priority 0
 *   EXTI1  → PC1  (Floor Sensor F2)  NVIC priority 2
 *   EXTI2  → PC2  (Floor Sensor F3)  NVIC priority 2
 *   EXTI3  → PC3  (Floor Sensor F4)  NVIC priority 2
 *   EXTI4  → PA4  (Cabin Button F3)  NVIC priority 3
 *   EXTI5  → PA5  (Cabin Button F4)  NVIC priority 3  ─┐ EXTI9_5 IRQ
 *   EXTI6  → PB6  (Hall Button U1)   NVIC priority 3   |
 *   EXTI7  → PB7  (Hall Button D2)   NVIC priority 3   |
 *   EXTI8  → PB8  (Hall Button U2)   NVIC priority 3   |
 *   EXTI9  → PB9  (Hall Button D3)   NVIC priority 3  ─┘
 *   EXTI10 → PB10 (Hall Button U3)   NVIC priority 3  ─┐ EXTI15_10 IRQ
 *   EXTI12 → PB12 (Hall Button D4)   NVIC priority 3  ─┘
 *
 *   POLLED (EXTI conflicts resolved):
 *   PC0  (Floor Sensor F1) — polled in main loop (EXTI0 taken by E-Stop)
 *   PA0  (Cabin Button F1) — polled in main loop (EXTI0 taken by E-Stop)
 *   PA1  (Cabin Button F2) — polled in main loop (EXTI1 used by PC1)
 *   PA2  (Cabin Button F3) — polled in main loop (EXTI2 used by PC2)
 *   PA3  (Cabin Button F4) — polled in main loop (EXTI3 used by PC3)
 * ───────────────────────────────────────── */

/* ─────────────────────────────────────────
 * SYSCFG BASE — for EXTICR configuration
 * ───────────────────────────────────────── */
#define SYSCFG_BASE_ADDR    0x40013800UL

typedef struct {
    volatile u32 MEMRMP;
    volatile u32 PMC;
    volatile u32 EXTICR[4];
    u32          RESERVED[2];
    volatile u32 CMPCR;
} SYSCFG_RegDef_t;

#define SYSCFG              ((SYSCFG_RegDef_t *) SYSCFG_BASE_ADDR)

#define SYSCFG_PORT_A       0x0u
#define SYSCFG_PORT_B       0x1u
#define SYSCFG_PORT_C       0x2u
#define SYSCFG_PORT_D       0x3u

#define RCC_APB2ENR_SYSCFGEN (1u << 14)

/* ─────────────────────────────────────────
 * GPIO BASE ADDRESSES (for EXTI config)
 * ───────────────────────────────────────── */
#define GPIOA_BASE_ADDR     0x40020000UL
#define GPIOB_BASE_ADDR     0x40020400UL
#define GPIOC_BASE_ADDR     0x40020800UL
#define GPIOD_BASE_ADDR     0x40020C00UL

typedef struct {
    volatile u32 MODER;
    volatile u32 OTYPER;
    volatile u32 OSPEEDR;
    volatile u32 PUPDR;
    volatile u32 IDR;
    volatile u32 ODR;
    volatile u32 BSRR;
    volatile u32 LCKR;
    volatile u32 AFRL;
    volatile u32 AFRH;
} GPIO_RegDef_t;

#define GPIOA               ((GPIO_RegDef_t *) GPIOA_BASE_ADDR)
#define GPIOB               ((GPIO_RegDef_t *) GPIOB_BASE_ADDR)
#define GPIOC               ((GPIO_RegDef_t *) GPIOC_BASE_ADDR)
#define GPIOD               ((GPIO_RegDef_t *) GPIOD_BASE_ADDR)

/* ─────────────────────────────────────────
 * BASIC TIMER TIM6 — 500ms telemetry tick
 * TIM6 @ APB1 (16MHz):
 *   PSC = 15999 → 1kHz tick
 *   ARR = 499   → 500ms period
 * ───────────────────────────────────────── */
#define TIM6_BASE_ADDR      0x40001000UL

typedef struct {
    volatile u32 CR1;
    volatile u32 CR2;
    u32          RESERVED0;
    volatile u32 DIER;
    volatile u32 SR;
    volatile u32 EGR;
    u32          RESERVED1[3];
    volatile u32 CNT;
    volatile u32 PSC;
    volatile u32 ARR;
} TIM6_RegDef_t;

#define TIM6                ((TIM6_RegDef_t *) TIM6_BASE_ADDR)
#define TIM6_PSC_VALUE      15999u
#define TIM6_ARR_VALUE      499u
#define TIM6_IRQn           54
#define RCC_APB1ENR_TIM6EN  (1u << 4)

/* ─────────────────────────────────────────
 * CONCURRENCY MACROS
 * Pattern (nestable critical sections):
 *   u32 primask = Enter_Critical();
 *   ... critical section ...
 *   Exit_Critical(primask);
 * ───────────────────────────────────────── */
#define Enter_Critical()    ({ u32 _pm = __get_PRIMASK(); __disable_irq(); _pm; })
#define Exit_Critical(_pm)  (__set_PRIMASK(_pm))

/* ─────────────────────────────────────────
 * GLOBAL SHARED STATE INSTANCE
 * Defined once in Elevator.c, declared here.
 * ALL modules must use this single instance.
 * DO NOT define a second GlobalSharedState or
 * GlobalSharedState_t variable anywhere else.
 * ───────────────────────────────────────── */
extern volatile GlobalSharedState_t GSS;

/* ─────────────────────────────────────────
 * FUNCTION PROTOTYPES
 * ───────────────────────────────────────── */
void Elevator_Init(void);
void Elevator_Update(void);
void System_Logger(void);
void EXTI_Callback(u8 exti_line);

void EXTI_Init(void);
void TIM6_Init(void);

void EXTI0_IRQHandler(void);
void EXTI1_IRQHandler(void);
void EXTI2_IRQHandler(void);
void EXTI3_IRQHandler(void);
void EXTI4_IRQHandler(void);
void EXTI9_5_IRQHandler(void);
void EXTI15_10_IRQHandler(void);
void TIM6_DAC_IRQHandler(void);

u8 ELV_Sprintf(volatile u8 *buf, u8 bufSize, const char *fmt, ...);

#endif /* ELEVATOR_H */
