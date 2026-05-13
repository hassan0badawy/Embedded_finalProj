#ifndef ELEVATOR_H
#define ELEVATOR_H

#include "std_types.h"
#include "stm32f401ve.h"
#include "Bit_Math.h"
#include "ipc.h"        /* ElevatorState_t, IPC_Frame_t */
#include "pwm.h"
#include "uart_dma.h"

/* ─────────────────────────────────────────
 * SYSTEM CONFIGURATION
 * ───────────────────────────────────────── */
#define NUM_FLOORS          4u      /* Floors: 0 = F1, 1 = F2, 2 = F3, 3 = F4 */
#define DOOR_OPEN_TICKS     6u      /* ~3s at 500ms tick: doors stay open       */

/* ─────────────────────────────────────────
 * PIN DEFINITIONS — EXTI INPUTS
 * ─────────────────────────────────────────
 * Rule: each EXTI line maps to ONE port.
 * EXTI0–EXTI15 can each be assigned to a
 * GPIO port via SYSCFG_EXTICR registers.
 *
 * Emergency Stop : PD0  → EXTI0  (highest priority)
 * Floor Sensors  : PC0–PC3 → EXTI0–3 (BUT PD0 uses EXTI0)
 *   → Floor sensors use PC0=EXTI0? NO — conflict!
 *   → Solution: E-Stop on PD0 takes EXTI0.
 *               Floor sensors PC1,PC2,PC3 on EXTI1,2,3.
 *               PC0 floor sensor re-mapped to TIM6 polling
 *               OR use a free EXTI line (EXTI5 via PC5).
 *               Here we map PC0 → EXTI0 with lower priority,
 *               but since PD0 is E-Stop priority 0, we assign
 *               EXTI0 to PD (E-Stop) via SYSCFG.
 *               Floor sensor for F1 detected via PC1 instead.
 *
 * Cabin Buttons  : PA0,PA1,PA4,PA5 → EXTI0,1,4,5
 *   → PA0 conflicts with PD0 on EXTI0 → use PA0 as F1 cabin btn
 *     but SYSCFG assigns EXTI0 to port D (E-Stop priority wins).
 *   → Safe mapping with no overlap:
 *
 * FINAL EXTI MAPPING (matching prompt, no pin number overlap):
 *   EXTI0  → PD0  (Emergency Stop)   NVIC priority 0
 *   EXTI1  → PC1  (Floor Sensor F2)  NVIC priority 2
 *   EXTI2  → PC2  (Floor Sensor F3)  NVIC priority 2
 *   EXTI3  → PC3  (Floor Sensor F4)  NVIC priority 2
 *   EXTI4  → PA4  (Cabin Button F3)  NVIC priority 3
 *   EXTI5  → PA5  (Cabin Button F4)  NVIC priority 3  ─┐ EXTI9_5 IRQ
 *   EXTI6  → PB6  (Hall Button)      NVIC priority 3   |
 *   EXTI7  → PB7  (Hall Button)      NVIC priority 3   |
 *   EXTI8  → PB8  (Hall Button)      NVIC priority 3   |
 *   EXTI9  → PB9  (Hall Button)      NVIC priority 3  ─┘
 *   EXTI10 → PB10 (Hall Button)      NVIC priority 3  ─┐ EXTI15_10 IRQ
 *   EXTI12 → PB12 (Hall Button)      NVIC priority 3  ─┘
 *
 *   POLLING (no EXTI conflict):
 *   PC0 (Floor Sensor F1) — polled in main loop
 *   PA0 (Cabin Button F1) — polled in main loop
 *   PA1 (Cabin Button F2) — EXTI1 shared with PC1 floor sensor?
 *     → Different ports on same EXTI line is NOT allowed.
 *     → PA1 polled OR use EXTI1 exclusively for PA1:
 *       Floor sensor F2 (PC1) polled instead.
 *
 * NOTE: In a real project, pin assignments would be resolved
 * at board-design time. This driver uses the EXTI lines that
 * are unambiguously specified in the prompt and notes conflicts.
 * ───────────────────────────────────────── */

/* ─────────────────────────────────────────
 * SYSCFG BASE — for EXTICR configuration
 * ───────────────────────────────────────── */
#define SYSCFG_BASE_ADDR    0x40013800UL

typedef struct {
    volatile u32 MEMRMP;        /* 0x00  Memory remap                */
    volatile u32 PMC;           /* 0x04  Peripheral mode config      */
    volatile u32 EXTICR[4];     /* 0x08  EXTI config registers 1–4  */
    u32          RESERVED[2];
    volatile u32 CMPCR;         /* 0x20  Compensation cell control   */
} SYSCFG_RegDef_t;

#define SYSCFG              ((SYSCFG_RegDef_t *) SYSCFG_BASE_ADDR)

/* SYSCFG port codes for EXTICR */
#define SYSCFG_PORT_A       0x0u
#define SYSCFG_PORT_B       0x1u
#define SYSCFG_PORT_C       0x2u
#define SYSCFG_PORT_D       0x3u

/* RCC APB2 bit for SYSCFG */
#define RCC_APB2ENR_SYSCFGEN (1u << 14)

/* ─────────────────────────────────────────
 * GPIO BASE ADDRESSES (for EXTI config)
 * ───────────────────────────────────────── */
#define GPIOA_BASE_ADDR     0x40020000UL
#define GPIOB_BASE_ADDR     0x40020400UL
#define GPIOC_BASE_ADDR     0x40020800UL
#define GPIOD_BASE_ADDR     0x40020C00UL

/* Generic GPIO register layout (MODER only needed here) */
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
 * ─────────────────────────────────────────
 * TIM6 is on APB1 (16MHz).
 * We want a 500ms interrupt.
 *
 * Formula: T = (PSC+1)(ARR+1) / fCLK
 * 0.5s = (PSC+1)(ARR+1) / 16000000
 * (PSC+1)(ARR+1) = 8000000
 * Choose: PSC = 15999 → divides by 16000 → 1kHz tick
 *         ARR = 499   → 500 ticks → 500ms
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
#define TIM6_PSC_VALUE      15999u  /* 16MHz / 16000 = 1kHz tick */
#define TIM6_ARR_VALUE      499u    /* 1kHz / 500    = 2Hz = 500ms */
#define TIM6_IRQn           54      /* TIM6 and DAC IRQ number */
#define RCC_APB1ENR_TIM6EN  (1u << 4)

/* ─────────────────────────────────────────
 * CONCURRENCY MACROS (Member D deliverable)
 * ─────────────────────────────────────────
 * Uses PRIMASK register:
 *   __get_PRIMASK() → returns current IRQ mask
 *   __disable_irq() → sets PRIMASK=1 (masks all IRQs)
 *   __set_PRIMASK() → restores saved mask
 *
 * Pattern (safe, nestable):
 *   u32 primask = Enter_Critical();
 *   ... critical section ...
 *   Exit_Critical(primask);
 *
 * This is safer than bare CPSID/CPSIE because
 * it preserves the previous mask state, allowing
 * nested critical sections without accidentally
 * re-enabling interrupts too early.
 * ───────────────────────────────────────── */
#define Enter_Critical()    ({ u32 _pm = __get_PRIMASK(); __disable_irq(); _pm; })
#define Exit_Critical(_pm)  (__set_PRIMASK(_pm))

/* ─────────────────────────────────────────
 * GLOBAL SHARED STATE STRUCT (Member D)
 * ─────────────────────────────────────────
 * __attribute__((packed)) ensures no padding
 * bytes are inserted — critical for DMA and
 * SPI frame alignment.
 *
 * volatile: all fields may be written by ISRs
 * and read by the main loop — must never be
 * cached in a register by the compiler.
 * ───────────────────────────────────────── */
typedef struct __attribute__((packed)) {
    volatile u8  position;          /* Current floor (0–3)               */
    volatile u8  target;            /* Target floor  (0–3)               */
    volatile u8  direction;         /* 0=none, 1=up, 2=down              */
    volatile u8  speed;             /* PWM_DUTY_STOP/SLOW/FULL           */
    volatile u8  fsm_state;         /* ElevatorState_t value             */
    volatile u8  emergency;         /* 1 = emergency stop active         */
    volatile u8  door_open;         /* 1 = doors currently open          */
    volatile u8  comm_fault;        /* 1 = IPC link lost                 */
    volatile u8  telem_flag;        /* 1 = TIM6 fired, send telemetry    */
    volatile u8  telem_tick;        /* 1 = TIM6 500ms tick (one-shot)    */
    volatile u8  floor_request[NUM_FLOORS]; /* Pending floor requests    */
} GlobalSharedState;

/* ─────────────────────────────────────────
 * GLOBAL SHARED STATE INSTANCE
 * ───────────────────────────────────────── */
extern volatile GlobalSharedState GSS;

/* ─────────────────────────────────────────
 * FUNCTION PROTOTYPES — DELIVERABLES
 * ───────────────────────────────────────── */

/*
 * Elevator_Init()
 * ───────────────
 * Initializes all hardware:
 *   PWM, UART+DMA, EXTI, TIM6, IPC/SPI
 * Resets GlobalSharedState.
 * Call once at startup before main loop.
 */
void Elevator_Init(void);

/*
 * Elevator_Update()  ← MEMBER A DELIVERABLE
 * ────────────────────────────────────────────
 * FSM tick function. Call from main loop.
 *
 * Reads GlobalSharedState flags set by ISRs,
 * transitions FSM states, ramps PWM speed,
 * and updates IPC TxFrame for SPI dispatch.
 *
 * States:
 *   IDLE        → wait for floor_request[]
 *   MOVING_UP   → set speed, check floor sensors
 *   MOVING_DOWN → set speed, check floor sensors
 *   DOORS_OPEN  → count down DOOR_OPEN_TICKS then IDLE
 *   EMERGENCY   → kill motor, halt all motion
 */
void Elevator_Update(void);

/*
 * System_Logger()  ← MEMBER D DELIVERABLE
 * ─────────────────────────────────────────
 * Formats a telemetry string into UART_TxBuf
 * and triggers a DMA transfer.
 *
 * Called from main loop when GSS.telem_flag==1.
 * String format:
 *   "ELV|FL:%d|ST:%d|SP:%d|DIR:%d|EM:%d\r\n"
 *
 * Non-blocking: returns immediately after
 * kicking off DMA. No HAL_Delay() used anywhere.
 */
void System_Logger(void);

/*
 * EXTI_Callback()
 * ────────────────
 * Centralized EXTI dispatcher.
 * Called by all EXTI IRQ handlers with the
 * pending bit number that triggered them.
 *
 * Handles:
 *   line 0       → Emergency Stop (PD0)
 *   lines 1,2,3  → Floor Sensors (PC1,PC2,PC3)
 *   lines 4,5    → Cabin Buttons (PA4,PA5)
 *   lines 6–10,12→ Hallway Buttons (PB6–PB12)
 */
void EXTI_Callback(u8 exti_line);

/* ─────────────────────────────────────────
 * PERIPHERAL INIT SUB-FUNCTIONS
 * ───────────────────────────────────────── */
void EXTI_Init(void);       /* Configure all EXTI lines + NVIC            */
void TIM6_Init(void);       /* Configure 500ms basic timer                */

/* ─────────────────────────────────────────
 * IRQ HANDLERS (defined in elevator.c)
 * ───────────────────────────────────────── */
void EXTI0_IRQHandler(void);
void EXTI1_IRQHandler(void);
void EXTI2_IRQHandler(void);
void EXTI3_IRQHandler(void);
void EXTI4_IRQHandler(void);
void EXTI9_5_IRQHandler(void);
void EXTI15_10_IRQHandler(void);
void TIM6_DAC_IRQHandler(void);

/* ─────────────────────────────────────────
 * STRING HELPER (no stdlib dependency)
 * ───────────────────────────────────────── */
u8 ELV_Sprintf(volatile u8 *buf, u8 bufSize,
               const char *fmt, ...);

#endif /* ELEVATOR_H */