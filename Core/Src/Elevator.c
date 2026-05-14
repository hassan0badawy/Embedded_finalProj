#include "../Inc/Elevator.h"
#include "../Inc/shared.h"
#include "../Inc/dispatcher.h"
#include "../Inc/Bit_Math.h"
#include "../Inc/Pwm.h"
#include "../Inc/Timer.h"
#include "../Inc/gpio.h"
#include <stdarg.h>

/* ─────────────────────────────────────────
 * GLOBAL SHARED STATE — SINGLE DEFINITION
 * ALL modules (dispatcher.c, ipc.c, main.c,
 * uart_dma.c) use this via extern in shared.h.
 * DO NOT define GSS anywhere else.
 * ───────────────────────────────────────── */
volatile GlobalSharedState_t GSS;

/* ─────────────────────────────────────────
 * PWM CONFIGURATION (10kHz via Timer2 CH1)
 * 16MHz / (PSC+1=16) = 1MHz tick
 * 1MHz  / (ARR+1=100) = 10kHz PWM period
 * ───────────────────────────────────────── */
#define ELV_PWM_TIMER      TIMER1   /* TIM1 CH1 on PA8 — avoids PA0 cabin button conflict */
#define ELV_PWM_CH         1
#define ELV_PWM_PSC        15u
#define ELV_PWM_ARR        99u

/* ─────────────────────────────────────────
 * PWM wrapper — bridges FSM to Pwm driver
 * ───────────────────────────────────────── */
void PWM_SetDuty(u8 duty_percent)
{
    Pwm_SetDutyPercent(ELV_PWM_TIMER, ELV_PWM_CH, duty_percent);
}

/* ─────────────────────────────────────────
 * INTERNAL FSM HELPERS
 * ───────────────────────────────────────── */
static u8 HasPendingRequest(void)
{
    u8 i;
    for (i = 0u; i < NUM_FLOORS; i++)
    {
        if (GSS.floor_request[i]) { return 1u; }
    }
    return 0u;
}

static u8 FindNearestUp(void)
{
    u8 i;
    for (i = GSS.position + 1u; i < NUM_FLOORS; i++)
    {
        if (GSS.floor_request[i]) { return i; }
    }
    return 0xFFu;
}

static u8 FindNearestDown(void)
{
    u8 i;
    if (GSS.position == 0u) { return 0xFFu; }
    i = GSS.position - 1u;
    do {
        if (GSS.floor_request[i]) { return i; }
        if (i == 0u) { break; }
        i--;
    } while (1);
    return 0xFFu;
}

/* ─────────────────────────────────────────
 * EXTI_Init()
 * Configures all EXTI lines per the final
 * pin mapping (see Elevator.h pin table).
 * ───────────────────────────────────────── */
void EXTI_Init(void)
{
    /* Enable SYSCFG clock */
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    /* ── Map EXTI sources ── */

    /* EXTI0 → Port D (PD0 = Emergency Stop) */
    SYSCFG->EXTICR[0] &= ~(0xFu << 0);
    SYSCFG->EXTICR[0] |=  (SYSCFG_PORT_D << 0);

    /* EXTI1 → Port C (PC1 = Floor Sensor F2) */
    SYSCFG->EXTICR[0] &= ~(0xFu << 4);
    SYSCFG->EXTICR[0] |=  (SYSCFG_PORT_C << 4);

    /* EXTI2 → Port C (PC2 = Floor Sensor F3) */
    SYSCFG->EXTICR[0] &= ~(0xFu << 8);
    SYSCFG->EXTICR[0] |=  (SYSCFG_PORT_C << 8);

    /* EXTI3 → Port C (PC3 = Floor Sensor F4) */
    SYSCFG->EXTICR[0] &= ~(0xFu << 12);
    SYSCFG->EXTICR[0] |=  (SYSCFG_PORT_C << 12);

    /* EXTI6 → Port B, EXTI7 → Port B (PB6=U1, PB7=D2) */
    SYSCFG->EXTICR[1] &= ~((0xFu << 8) | (0xFu << 12));
    SYSCFG->EXTICR[1] |=  ((SYSCFG_PORT_B << 8) | (SYSCFG_PORT_B << 12));

    /* EXTI8 → Port B, EXTI9 → Port B (PB8=U2, PB9=D3) */
    SYSCFG->EXTICR[2] &= ~((0xFu << 0) | (0xFu << 4));
    SYSCFG->EXTICR[2] |=  ((SYSCFG_PORT_B << 0) | (SYSCFG_PORT_B << 4));

    /* EXTI10 → Port B, EXTI12 → Port B (PB10=U3, PB12=D4) */
    SYSCFG->EXTICR[2] &= ~(0xFu << 8);
    SYSCFG->EXTICR[2] |=  (SYSCFG_PORT_B << 8);
    SYSCFG->EXTICR[3] &= ~(0xFu << 0);
    SYSCFG->EXTICR[3] |=  (SYSCFG_PORT_B << 0);

    /* ── Unmask lines and set falling-edge triggers ── */
    u32 lines = (1u << 0)  | (1u << 1)  | (1u << 2)  | (1u << 3)  |
                (1u << 6)  | (1u << 7)  | (1u << 8)  | (1u << 9)  |
                (1u << 10) | (1u << 12);

    EXTI->IMR  |= lines;
    EXTI->FTSR |= lines;
    EXTI->RTSR &= ~lines;  /* No rising edge */

    /* ── NVIC priorities and enable ── */
    NVIC_SET_PRIORITY(IRQ_EXTI0,    0);  /* Emergency Stop — highest */
    NVIC_SET_PRIORITY(IRQ_EXTI1,    2);
    NVIC_SET_PRIORITY(IRQ_EXTI2,    2);
    NVIC_SET_PRIORITY(IRQ_EXTI3,    2);
    NVIC_SET_PRIORITY(IRQ_EXTI4,    3);
    NVIC_SET_PRIORITY(IRQ_EXTI9_5,  3);
    NVIC_SET_PRIORITY(40,           3);  /* EXTI15_10 */

    NVIC_ENABLE_IRQ(IRQ_EXTI0);
    NVIC_ENABLE_IRQ(IRQ_EXTI1);
    NVIC_ENABLE_IRQ(IRQ_EXTI2);
    NVIC_ENABLE_IRQ(IRQ_EXTI3);
    NVIC_ENABLE_IRQ(IRQ_EXTI4);
    NVIC_ENABLE_IRQ(IRQ_EXTI9_5);
    NVIC_ENABLE_IRQ(40);
}

/* ─────────────────────────────────────────
 * TIM6_Init()
 * 500ms basic timer for telemetry tick.
 * TIM6 @ APB1 16MHz:
 *   PSC=15999 → 1kHz, ARR=499 → 500ms
 * ───────────────────────────────────────── */
void TIM6_Init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;

    TIM6->CR1  = 0u;
    TIM6->PSC  = TIM6_PSC_VALUE;
    TIM6->ARR  = TIM6_ARR_VALUE;
    TIM6->CNT  = 0u;
    TIM6->DIER = (1u << 0);   /* UIE: Update Interrupt Enable */
    TIM6->EGR  = (1u << 0);   /* UG:  Force load PSC/ARR */
    TIM6->SR   = 0u;

    NVIC_SET_PRIORITY(TIM6_IRQn, 4);  /* Lowest priority */
    NVIC_ENABLE_IRQ(TIM6_IRQn);

    TIM6->CR1 |= (1u << 0);   /* CEN: Counter Enable */
}

/* ─────────────────────────────────────────
 * Elevator_Init()
 * ───────────────────────────────────────── */
void Elevator_Init(void)
{
    u8 i;
    u32 pm;

    /* Enable GPIO clocks */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN |
                    RCC_AHB1ENR_GPIOCEN  | RCC_AHB1ENR_GPIODEN;
                    
    /* Enable TIM1 clock (APB2) */
    RCC->APB2ENR |= (1u << 0);

    /* Clear GSS atomically */
    pm = Enter_Critical();
    GSS.position  = 0u;
    GSS.target    = 0u;
    GSS.direction = 0u;
    GSS.speed     = 0u;
    GSS.fsm_state = (u8)ELV_IDLE;
    GSS.emergency = 0u;
    GSS.door_open = 0u;
    GSS.comm_fault = 0u;
    GSS.telem_flag = 0u;
    GSS.telem_tick = 0u;
    for (i = 0u; i < NUM_FLOORS; i++) { GSS.floor_request[i] = 0u; }
    GSS.slave_position  = 0u;
    GSS.slave_fsm_state = 0u;
    GSS.slave_target    = 0u;
    GSS.slave_speed     = 0u;
    GSS.slave_flags     = 0u;
    GSS.last_valid_rx_tick = 0u;
    Exit_Critical(pm);

    /* Configure PA8 as TIM1 CH1 PWM output (AF1)
     * PA8 is free — no conflict with PA0 cabin button. */
    Gpio_Init(GPIO_A, 8, GPIO_AF, GPIO_PUSH_PULL);
    Gpio_SetAF(GPIO_A, 8, GPIO_AF1);   /* AF1 = TIM1_CH1 on PA8 */

    /* Initialise peripherals */
    Pwm_Init(ELV_PWM_TIMER, ELV_PWM_CH, ELV_PWM_PSC, ELV_PWM_ARR);
    Pwm_Start(ELV_PWM_TIMER, ELV_PWM_CH);
    PWM_SetDuty(PWM_DUTY_STOP);

    UART_DMA_Init();
    EXTI_Init();
    TIM6_Init();
}

/* ─────────────────────────────────────────
 * Elevator_Update()
 * FSM tick — call from main loop at full speed.
 * ───────────────────────────────────────── */
void Elevator_Update(void)
{
    static u8 door_tick_count = 0u;
    u8 up_target;
    u8 dn_target;
    u32 primask;

    /* ── Emergency stop — highest priority, overrides all states ── */
    if (GSS.emergency)
    {
        PWM_SetDuty(PWM_DUTY_STOP);
        primask = Enter_Critical();
        GSS.fsm_state   = (u8)ELV_EMERGENCY;
        GSS.speed       = PWM_DUTY_STOP;
        GSS.direction   = 0u;
        GSS.door_open   = 0u;
        door_tick_count = 0u;
        u8 i;
        for (i = 0u; i < NUM_FLOORS; i++) { GSS.floor_request[i] = 0u; }
        Exit_Critical(primask);
        return;
    }

    switch ((ElevatorState_t)GSS.fsm_state)
    {
        /* ── IDLE ── */
        case ELV_IDLE:
        {
            PWM_SetDuty(PWM_DUTY_STOP);
            primask = Enter_Critical();
            GSS.speed     = PWM_DUTY_STOP;
            GSS.direction = 0u;
            Exit_Critical(primask);

            if (!HasPendingRequest()) { break; }

            up_target = FindNearestUp();
            dn_target = FindNearestDown();

            u8 speed_snap = PWM_DUTY_STOP;
            primask = Enter_Critical();
            if (up_target != 0xFFu)
            {
                GSS.target    = up_target;
                GSS.direction = 1u;
                GSS.fsm_state = (u8)ELV_MOVING_UP;
                GSS.speed     = PWM_DUTY_FULL;
                speed_snap    = PWM_DUTY_FULL;
            }
            else if (dn_target != 0xFFu)
            {
                GSS.target    = dn_target;
                GSS.direction = 2u;
                GSS.fsm_state = (u8)ELV_MOVING_DOWN;
                GSS.speed     = PWM_DUTY_FULL;
                speed_snap    = PWM_DUTY_FULL;
            }
            Exit_Critical(primask);
            PWM_SetDuty(speed_snap);   /* Use snapshot — not volatile GSS.speed */
            break;
        }

        /* ── MOVING UP ── */
        case ELV_MOVING_UP:
        {
            /* Ramp down 1 floor before target */
            if ((GSS.target > GSS.position) && (GSS.target - GSS.position) <= 1)
            {
                if (GSS.speed != PWM_DUTY_SLOW)
                {
                    primask = Enter_Critical();
                    GSS.speed = PWM_DUTY_SLOW;
                    Exit_Critical(primask);
                    PWM_SetDuty(PWM_DUTY_SLOW);
                }
            }
            else if (GSS.speed != PWM_DUTY_FULL)
            {
                primask = Enter_Critical();
                GSS.speed = PWM_DUTY_FULL;
                Exit_Critical(primask);
                PWM_SetDuty(PWM_DUTY_FULL);
            }

            if (GSS.position == GSS.target)
            {
                primask = Enter_Critical();
                GSS.speed                       = PWM_DUTY_STOP;
                GSS.floor_request[GSS.position] = 0u;
                GSS.door_open                   = 1u;
                GSS.fsm_state                   = (u8)ELV_DOORS_OPEN;
                door_tick_count                 = 0u;
                Exit_Critical(primask);
                PWM_SetDuty(PWM_DUTY_STOP);
            }
            break;
        }

        /* ── MOVING DOWN ── */
        case ELV_MOVING_DOWN:
        {
            /* Ramp down 1 floor before target */
            if ((GSS.position > GSS.target) && (GSS.position - GSS.target) <= 1)
            {
                if (GSS.speed != PWM_DUTY_SLOW)
                {
                    primask = Enter_Critical();
                    GSS.speed = PWM_DUTY_SLOW;
                    Exit_Critical(primask);
                    PWM_SetDuty(PWM_DUTY_SLOW);
                }
            }
            else if (GSS.speed != PWM_DUTY_FULL)
            {
                primask = Enter_Critical();
                GSS.speed = PWM_DUTY_FULL;
                Exit_Critical(primask);
                PWM_SetDuty(PWM_DUTY_FULL);
            }

            if (GSS.position == GSS.target)
            {
                primask = Enter_Critical();
                GSS.speed                       = PWM_DUTY_STOP;
                GSS.floor_request[GSS.position] = 0u;
                GSS.door_open                   = 1u;
                GSS.fsm_state                   = (u8)ELV_DOORS_OPEN;
                door_tick_count                 = 0u;
                Exit_Critical(primask);
                PWM_SetDuty(PWM_DUTY_STOP);
            }
            break;
        }

        /* ── DOORS OPEN ── */
        case ELV_DOORS_OPEN:
        {
            PWM_SetDuty(PWM_DUTY_STOP);

            /* Count 500ms ticks via TIM6 telem_tick flag */
            if (GSS.telem_tick)
            {
                door_tick_count++;
                primask = Enter_Critical();
                GSS.telem_tick = 0u;
                Exit_Critical(primask);
            }

            if (door_tick_count >= DOOR_OPEN_TICKS)   /* 6 × 500ms = 3s */
            {
                primask = Enter_Critical();
                GSS.door_open   = 0u;
                GSS.fsm_state   = (u8)ELV_IDLE;
                GSS.direction   = 0u;
                door_tick_count = 0u;
                Exit_Critical(primask);
            }
            break;
        }

        /* ── EMERGENCY (persistent until reset) ── */
        case ELV_EMERGENCY:
        {
            PWM_SetDuty(PWM_DUTY_STOP);
            break;
        }

        default:
            primask = Enter_Critical();
            GSS.fsm_state = (u8)ELV_IDLE;
            Exit_Critical(primask);
            PWM_SetDuty(PWM_DUTY_STOP);
            break;
    }
}

/* ─────────────────────────────────────────
 * EXTI_Callback()
 * Centralized handler called by all EXTI ISRs.
 * ───────────────────────────────────────── */
void EXTI_Callback(u8 exti_line)
{
    u32 pm;
    switch (exti_line)
    {
        case 0:   /* PD0 — Emergency Stop (priority 0) */
            pm = Enter_Critical();
            GSS.emergency = 1u;
            Exit_Critical(pm);
            break;

        case 1:   /* PC1 — Floor Sensor F2 (floor index 1) */
            pm = Enter_Critical();
            GSS.position = 1u;
            Exit_Critical(pm);
            break;

        case 2:   /* PC2 — Floor Sensor F3 */
            pm = Enter_Critical();
            GSS.position = 2u;
            Exit_Critical(pm);
            break;

        case 3:   /* PC3 — Floor Sensor F4 */
            pm = Enter_Critical();
            GSS.position = 3u;
            Exit_Critical(pm);
            break;

        case 6:   /* PB6 — Hallway Button U1 (floor 0, UP) */
            Dispatcher_RegisterCall(0u, DIR_UP);
            break;

        case 7:   /* PB7 — Hallway Button D2 (floor 1, DOWN) */
            Dispatcher_RegisterCall(1u, DIR_DOWN);
            break;

        case 8:   /* PB8 — Hallway Button U2 (floor 1, UP) */
            Dispatcher_RegisterCall(1u, DIR_UP);
            break;

        case 9:   /* PB9 — Hallway Button D3 (floor 2, DOWN) */
            Dispatcher_RegisterCall(2u, DIR_DOWN);
            break;

        case 10:  /* PB10 — Hallway Button U3 (floor 2, UP) */
            Dispatcher_RegisterCall(2u, DIR_UP);
            break;

        case 12:  /* PB12 — Hallway Button D4 (floor 3, DOWN) */
            Dispatcher_RegisterCall(3u, DIR_DOWN);
            break;

        default:
            break;
    }
}

/* ── EXTI ISR Handlers — clear pending bit then dispatch ── */

void EXTI0_IRQHandler(void)
{
    if (READ_BIT(EXTI->PR, 0)) { EXTI->PR = (1u << 0); EXTI_Callback(0); }
}
void EXTI1_IRQHandler(void)
{
    if (READ_BIT(EXTI->PR, 1)) { EXTI->PR = (1u << 1); EXTI_Callback(1); }
}
void EXTI2_IRQHandler(void)
{
    if (READ_BIT(EXTI->PR, 2)) { EXTI->PR = (1u << 2); EXTI_Callback(2); }
}
void EXTI3_IRQHandler(void)
{
    if (READ_BIT(EXTI->PR, 3)) { EXTI->PR = (1u << 3); EXTI_Callback(3); }
}
void EXTI4_IRQHandler(void)
{
    if (READ_BIT(EXTI->PR, 4)) { EXTI->PR = (1u << 4); EXTI_Callback(4); }
}
void EXTI9_5_IRQHandler(void)
{
    u8 line;
    for (line = 5u; line <= 9u; line++)
    {
        if (READ_BIT(EXTI->PR, line))
        {
            EXTI->PR = (1u << line);
            EXTI_Callback(line);
        }
    }
}
void EXTI15_10_IRQHandler(void)
{
    u8 line;
    for (line = 10u; line <= 15u; line++)
    {
        if (READ_BIT(EXTI->PR, line))
        {
            EXTI->PR = (1u << line);
            EXTI_Callback(line);
        }
    }
}

/* ─────────────────────────────────────────
 * TIM6_DAC_IRQHandler()
 * Fires every 500ms. Sets telem_flag and
 * telem_tick for the main loop and door timer.
 * ───────────────────────────────────────── */
void TIM6_DAC_IRQHandler(void)
{
    if (READ_BIT(TIM6->SR, 0))
    {
        TIM6->SR = 0u;   /* Clear UIF */

        u32 pm = Enter_Critical();
        GSS.telem_flag = 1u;   /* Signal System_Logger() to run */
        GSS.telem_tick = 1u;   /* Signal door counter increment */
        Exit_Critical(pm);
    }
}