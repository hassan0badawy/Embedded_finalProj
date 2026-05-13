#include "Elevator.h"
#include "shared.h"
#include "Bit_Math.h"
#include <stdarg.h>     /* va_list for ELV_Sprintf */

/* ─────────────────────────────────────────
 * GLOBAL SHARED STATE
 * ───────────────────────────────────────── */
volatile GlobalSharedState GSS;

/* ─────────────────────────────────────────
 * INTERNAL HELPERS
 * ───────────────────────────────────────── */

/* Returns 1 if any floor_request[] is pending */
static u8 HasPendingRequest(void)
{
    u8 i;
    for (i = 0u; i < NUM_FLOORS; i++)
    {
        if (GSS.floor_request[i]) { return 1u; }
    }
    return 0u;
}

/* Find the nearest pending floor above current position.
 * Returns 0xFF if none found.                            */
static u8 FindNearestUp(void)
{
    u8 i;
    for (i = GSS.position + 1u; i < NUM_FLOORS; i++)
    {
        if (GSS.floor_request[i]) { return i; }
    }
    return 0xFFu;
}

/* Find the nearest pending floor below current position.
 * Returns 0xFF if none found.                            */
static u8 FindNearestDown(void)
{
    u8 i;
    /* Walk downward from position-1 to 0 */
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
 * Elevator_Init()
 * ─────────────────────────────────────────
 * Brings up all subsystems in correct order:
 *   1. Clear shared state
 *   2. PWM (motor output)
 *   3. UART + DMA (telemetry)
 *   4. EXTI (sensors, buttons, e-stop)
 *   5. TIM6 (500ms telemetry tick)
 *   6. IPC/SPI (inter-MCU link)
 * ───────────────────────────────────────── */
void Elevator_Init(void)
{
    u8 i;
    u32 primask;

    /* ── 1. Clear all shared state fields ── */
    primask = Enter_Critical();

    GSS.position      = 0u;
    GSS.target        = 0u;
    GSS.direction     = 0u;
    GSS.speed         = PWM_DUTY_STOP;
    GSS.fsm_state     = (u8)ELV_IDLE;
    GSS.emergency     = 0u;
    GSS.door_open     = 0u;
    GSS.comm_fault    = 0u;
    GSS.telem_flag    = 0u;

    for (i = 0u; i < NUM_FLOORS; i++)
    {
        GSS.floor_request[i] = 0u;
    }

    Exit_Critical(primask);

    /* ── 2. PWM motor output (TIM1 CH1, PA8) ── */
    PWM_Init();

    /* ── 3. Telemetry UART + DMA ── */
    UART_DMA_Init();

    /* ── 4. EXTI interrupt lines ── */
    EXTI_Init();

    /* ── 5. Basic timer TIM6 for 500ms telemetry flag ── */
    TIM6_Init();

    /* ── 6. IPC/SPI layer (this MCU = Master) ── */
    IPC_Init(1u);  /* 1 = Master */
}

/* ─────────────────────────────────────────
 * TIM6_Init()
 * ─────────────────────────────────────────
 * Configures TIM6 basic timer for 500ms
 * periodic interrupt.
 *
 * TIM6 is the simplest STM32 timer:
 *   - No PWM channels, no capture, no DMA
 *   - Just: PSC → ARR → UIF flag → IRQ
 * ───────────────────────────────────────── */
void TIM6_Init(void)
{
    /* Enable TIM6 clock on APB1 (bit 4) */
    SET_BIT(RCC->APB1ENR, 4);

    /* Disable timer before configuring */
    CLEAR_BIT(TIM6->CR1, 0);

    /* Set prescaler: 16MHz / (15999+1) = 1000Hz = 1kHz */
    TIM6->PSC = TIM6_PSC_VALUE;

    /* Set auto-reload: 1kHz / (499+1) = 2Hz = 500ms */
    TIM6->ARR = TIM6_ARR_VALUE;

    /* Enable auto-reload preload (ARPE=1, bit 7 in CR1) */
    SET_BIT(TIM6->CR1, 7);

    /* Enable update interrupt (UIE = bit 0 in DIER) */
    SET_BIT(TIM6->DIER, 0);

    /* Enable NVIC for TIM6 (IRQ 54), priority 4 (lowest) */
    NVIC_SET_PRIORITY(TIM6_IRQn, 4);
    NVIC_ENABLE_IRQ(TIM6_IRQn);

    /* Generate update event to load PSC/ARR shadow registers */
    SET_BIT(TIM6->EGR, 0);   /* UG bit */

    /* Clear any pending update flag from the UG event above */
    CLEAR_BIT(TIM6->SR, 0);  /* UIF bit */

    /* Enable timer */
    SET_BIT(TIM6->CR1, 0);   /* CEN bit */
}

/* ─────────────────────────────────────────
 * EXTI_Init()
 * ─────────────────────────────────────────
 * Configures all EXTI lines for the elevator
 * input signals. All triggers are falling-edge
 * (button pressed = pull-down active).
 *
 * Pin pull configuration must be set in GPIO
 * PUPDR registers (pull-up so pin rests HIGH,
 * falls LOW when button pressed).
 * ───────────────────────────────────────── */
void EXTI_Init(void)
{
    /* ── Enable SYSCFG clock (APB2 bit 14) ── */
    SET_BIT(RCC->APB2ENR, 14);

    /* ── Enable GPIO clocks: A, B, C, D ── */
    SET_BIT(RCC->AHB1ENR, 0);   /* GPIOA */
    SET_BIT(RCC->AHB1ENR, 1);   /* GPIOB */
    SET_BIT(RCC->AHB1ENR, 2);   /* GPIOC */
    SET_BIT(RCC->AHB1ENR, 3);   /* GPIOD */

    /* ── Configure all input pins as Input + Pull-up ──
     *
     * MODER = 00 (Input mode — reset default)
     * PUPDR = 01 (Pull-up so pin reads 1 at rest,
     *             falls to 0 when button pressed)
     *
     * PD0  — E-Stop
     * PC1,PC2,PC3 — Floor sensors
     * PB6,PB7,PB8,PB9,PB10,PB12 — Hall buttons
     */

    /* PD0 — pull-up */
    GPIOD->PUPDR &= ~(0x3UL << 0);
    GPIOD->PUPDR |=  (0x1UL << 0);

    /* PC1, PC2, PC3 — pull-up */
    GPIOC->PUPDR &= ~((0x3UL << 2) | (0x3UL << 4) | (0x3UL << 6));
    GPIOC->PUPDR |=  ((0x1UL << 2) | (0x1UL << 4) | (0x1UL << 6));

    /* PB6,PB7,PB8,PB9,PB10,PB12 — pull-up */
    GPIOB->PUPDR &= ~((0x3UL << 12) | (0x3UL << 14) | (0x3UL << 16) |
                      (0x3UL << 18) | (0x3UL << 20) | (0x3UL << 24));
    GPIOB->PUPDR |=  ((0x1UL << 12) | (0x1UL << 14) | (0x1UL << 16) |
                      (0x1UL << 18) | (0x1UL << 20) | (0x1UL << 24));

    /* ── SYSCFG EXTICR — Route EXTI lines to GPIO ports ──
     *
     * EXTICR[0] controls EXTI0–EXTI3 (4 bits each)
     * EXTICR[1] controls EXTI4–EXTI7
     * EXTICR[2] controls EXTI8–EXTI11
     * EXTICR[3] controls EXTI12–EXTI15
     *
     *   EXTI0  → PD (0x3)  E-Stop
     *   EXTI1  → PC (0x2)  Floor sensor F2
     *   EXTI2  → PC (0x2)  Floor sensor F3
     *   EXTI3  → PC (0x2)  Floor sensor F4
     *   EXTI6  → PB (0x1)  Hall button
     *   EXTI7  → PB (0x1)  Hall button
     *   EXTI8  → PB (0x1)  Hall button
     *   EXTI9  → PB (0x1)  Hall button
     *   EXTI10 → PB (0x1)  Hall button
     *   EXTI12 → PB (0x1)  Hall button
     */

    /* EXTICR[0]: EXTI0=PD(3), EXTI1=PC(2), EXTI2=PC(2), EXTI3=PC(2) */
    SYSCFG->EXTICR[0]  = 0u;
    SYSCFG->EXTICR[0] |= (SYSCFG_PORT_D << 0);   /* EXTI0 bits [3:0]   */
    SYSCFG->EXTICR[0] |= (SYSCFG_PORT_C << 4);   /* EXTI1 bits [7:4]   */
    SYSCFG->EXTICR[0] |= (SYSCFG_PORT_C << 8);   /* EXTI2 bits [11:8]  */
    SYSCFG->EXTICR[0] |= (SYSCFG_PORT_C << 12);  /* EXTI3 bits [15:12] */

    /* EXTICR[1]: EXTI6=PB(1), EXTI7=PB(1) */
    SYSCFG->EXTICR[1]  = 0u;
    SYSCFG->EXTICR[1] |= (SYSCFG_PORT_B << 8);   /* EXTI6 bits [11:8]  */
    SYSCFG->EXTICR[1] |= (SYSCFG_PORT_B << 12);  /* EXTI7 bits [15:12] */

    /* EXTICR[2]: EXTI8=PB(1), EXTI9=PB(1), EXTI10=PB(1) */
    SYSCFG->EXTICR[2]  = 0u;
    SYSCFG->EXTICR[2] |= (SYSCFG_PORT_B << 0);   /* EXTI8  bits [3:0]  */
    SYSCFG->EXTICR[2] |= (SYSCFG_PORT_B << 4);   /* EXTI9  bits [7:4]  */
    SYSCFG->EXTICR[2] |= (SYSCFG_PORT_B << 8);   /* EXTI10 bits [11:8] */

    /* EXTICR[3]: EXTI12=PB(1) */
    SYSCFG->EXTICR[3]  = 0u;
    SYSCFG->EXTICR[3] |= (SYSCFG_PORT_B << 0);   /* EXTI12 bits [3:0]  */

    /* ── EXTI: Set falling-edge trigger for all lines ──
     * FTSR (Falling Trigger Selection Register):
     * Set bit N to enable falling edge on EXTI line N  */
    EXTI->FTSR |= (1UL << 0);    /* EXTI0  — E-Stop PD0       */
    EXTI->FTSR |= (1UL << 1);    /* EXTI1  — Floor sensor PC1  */
    EXTI->FTSR |= (1UL << 2);    /* EXTI2  — Floor sensor PC2  */
    EXTI->FTSR |= (1UL << 3);    /* EXTI3  — Floor sensor PC3  */
    EXTI->FTSR |= (1UL << 6);    /* EXTI6  — Hall button PB6   */
    EXTI->FTSR |= (1UL << 7);    /* EXTI7  — Hall button PB7   */
    EXTI->FTSR |= (1UL << 8);    /* EXTI8  — Hall button PB8   */
    EXTI->FTSR |= (1UL << 9);    /* EXTI9  — Hall button PB9   */
    EXTI->FTSR |= (1UL << 10);   /* EXTI10 — Hall button PB10  */
    EXTI->FTSR |= (1UL << 12);   /* EXTI12 — Hall button PB12  */

    /* ── EXTI: Enable interrupt mask for all lines ──
     * IMR (Interrupt Mask Register):
     * Set bit N to unmask (enable) EXTI line N         */
    EXTI->IMR |= (1UL << 0);
    EXTI->IMR |= (1UL << 1);
    EXTI->IMR |= (1UL << 2);
    EXTI->IMR |= (1UL << 3);
    EXTI->IMR |= (1UL << 6);
    EXTI->IMR |= (1UL << 7);
    EXTI->IMR |= (1UL << 8);
    EXTI->IMR |= (1UL << 9);
    EXTI->IMR |= (1UL << 10);
    EXTI->IMR |= (1UL << 12);

    /* ── NVIC: Enable and prioritize EXTI IRQs ── */

    /* EXTI0 — Emergency Stop — HIGHEST PRIORITY = 0 */
    NVIC_SET_PRIORITY(IRQ_EXTI0, 0);
    NVIC_ENABLE_IRQ(IRQ_EXTI0);

    /* EXTI1 — Floor Sensor F2 — priority 2 */
    NVIC_SET_PRIORITY(IRQ_EXTI1, 2);
    NVIC_ENABLE_IRQ(IRQ_EXTI1);

    /* EXTI2 — Floor Sensor F3 — priority 2 */
    NVIC_SET_PRIORITY(IRQ_EXTI2, 2);
    NVIC_ENABLE_IRQ(IRQ_EXTI2);

    /* EXTI3 — Floor Sensor F4 — priority 2 */
    NVIC_SET_PRIORITY(IRQ_EXTI3, 2);
    NVIC_ENABLE_IRQ(IRQ_EXTI3);

    /* EXTI9_5 — Hall PB6–PB9 — priority 3 */
    NVIC_SET_PRIORITY(IRQ_EXTI9_5, 3);
    NVIC_ENABLE_IRQ(IRQ_EXTI9_5);

    /* EXTI15_10 — Hall PB10, PB12 — priority 3 */
    NVIC_SET_PRIORITY(23 + 10, 3);  /* IRQ_EXTI15_10 = 40 */
    NVIC_ENABLE_IRQ(40);
}

/* ─────────────────────────────────────────
 * Elevator_Update()          ← MEMBER A
 * ─────────────────────────────────────────
 * Central FSM tick. Called from main loop.
 *
 * Design principles:
 *  - Reads volatile GSS flags atomically
 *  - Emergency check runs FIRST every tick
 *  - PWM ramping: SLOW when approaching floor,
 *    FULL during mid-travel
 *  - Door open state uses a static counter
 *    so no HAL_Delay() is ever needed
 *  - After each FSM transition, IPC TxFrame
 *    is updated so the Slave MCU stays in sync
 * ───────────────────────────────────────── */
void Elevator_Update(void)
{
    static u8 door_tick_count = 0u;
    u8 up_target;
    u8 dn_target;
    u32 primask;

    /* ══════════════════════════════════════
     * EMERGENCY CHECK — overrides all states
     * ══════════════════════════════════════ */
    if (GSS.emergency)
    {
        /* Kill motor immediately */
        PWM_SetDuty(PWM_DUTY_STOP);

        /* Transition to EMERGENCY regardless of current state */
        primask = Enter_Critical();
        GSS.fsm_state  = (u8)ELV_EMERGENCY;
        GSS.speed      = PWM_DUTY_STOP;
        GSS.direction  = 0u;
        GSS.door_open  = 0u;
        door_tick_count = 0u;

        /* Clear all pending requests — safety first */
        u8 i;
        for (i = 0u; i < NUM_FLOORS; i++)
        {
            GSS.floor_request[i] = 0u;
        }
        Exit_Critical(primask);

        /* Update IPC frame to broadcast emergency to Slave */
        IPC_Handle.TxFrame.fsm_state    = (u8)ELV_EMERGENCY;
        IPC_Handle.TxFrame.motor_speed  = 0u;
        
        primask = Enter_Critical();
        IPC_Handle.TxFrame.flags |= IPC_FLAG_EMERGENCY;
        Exit_Critical(primask);

        return;  /* Skip all other FSM logic this tick */
    }

    /* ══════════════════════════════════════
     * FSM STATE MACHINE
     * ══════════════════════════════════════ */
    switch ((ElevatorState_t)GSS.fsm_state)
    {
        /* ──────────────────────────────────
         * STATE: IDLE
         * Motor stopped. Waiting for a floor
         * request from cabin or hallway button.
         * ────────────────────────────────── */
        case ELV_IDLE:
        {
            PWM_SetDuty(PWM_DUTY_STOP);

            primask = Enter_Critical();
            GSS.speed     = PWM_DUTY_STOP;
            GSS.direction = 0u;
            Exit_Critical(primask);

            if (!HasPendingRequest()) { break; }  /* Nothing to do */

            /* Choose direction: prefer UP first (SCAN algorithm) */
            up_target = FindNearestUp();
            dn_target = FindNearestDown();

            primask = Enter_Critical();

            if (up_target != 0xFFu)
            {
                GSS.target    = up_target;
                GSS.direction = 1u;             /* Going up */
                GSS.fsm_state = (u8)ELV_MOVING_UP;
                GSS.speed     = PWM_DUTY_FULL;
            }
            else if (dn_target != 0xFFu)
            {
                GSS.target    = dn_target;
                GSS.direction = 2u;             /* Going down */
                GSS.fsm_state = (u8)ELV_MOVING_DOWN;
                GSS.speed     = PWM_DUTY_FULL;
            }

            Exit_Critical(primask);

            /* Apply PWM speed */
            PWM_SetDuty(GSS.speed);

            break;
        }

        /* ──────────────────────────────────
         * STATE: MOVING_UP
         * Motor running. Floor sensor ISR
         * updates GSS.position each floor.
         * Ramp down to SLOW within 1 floor
         * of target to allow smooth stop.
         * ────────────────────────────────── */
        case ELV_MOVING_UP:
        {
            /* Speed ramp: slow down when 1 floor away from target */
            if ((GSS.target > GSS.position) &&
                (GSS.target - GSS.position) <= 1u)
            {
                /* Approaching target — slow down */
                if (GSS.speed != PWM_DUTY_SLOW)
                {
                    primask = Enter_Critical();
                    GSS.speed = PWM_DUTY_SLOW;
                    Exit_Critical(primask);
                    PWM_SetDuty(PWM_DUTY_SLOW);
                }
            }
            else
            {
                /* Full speed mid-travel */
                if (GSS.speed != PWM_DUTY_FULL)
                {
                    primask = Enter_Critical();
                    GSS.speed = PWM_DUTY_FULL;
                    Exit_Critical(primask);
                    PWM_SetDuty(PWM_DUTY_FULL);
                }
            }

            /* Check if we have reached the target floor
             * (floor sensor ISR updates GSS.position)    */
            if (GSS.position == GSS.target)
            {
                /* Arrived — stop motor, open doors */
                PWM_SetDuty(PWM_DUTY_STOP);

                primask = Enter_Critical();
                GSS.speed                         = PWM_DUTY_STOP;
                GSS.floor_request[GSS.position]   = 0u;  /* Clear request */
                GSS.door_open                     = 1u;
                GSS.fsm_state                     = (u8)ELV_DOORS_OPEN;
                door_tick_count                   = 0u;
                Exit_Critical(primask);
            }

            break;
        }

        /* ──────────────────────────────────
         * STATE: MOVING_DOWN
         * Symmetric to MOVING_UP.
         * ────────────────────────────────── */
        case ELV_MOVING_DOWN:
        {
            /* Speed ramp: slow when 1 floor above target */
            if ((GSS.position > GSS.target) &&
                (GSS.position - GSS.target) <= 1u)
            {
                if (GSS.speed != PWM_DUTY_SLOW)
                {
                    primask = Enter_Critical();
                    GSS.speed = PWM_DUTY_SLOW;
                    Exit_Critical(primask);
                    PWM_SetDuty(PWM_DUTY_SLOW);
                }
            }
            else
            {
                if (GSS.speed != PWM_DUTY_FULL)
                {
                    primask = Enter_Critical();
                    GSS.speed = PWM_DUTY_FULL;
                    Exit_Critical(primask);
                    PWM_SetDuty(PWM_DUTY_FULL);
                }
            }

            /* Arrived at target? */
            if (GSS.position == GSS.target)
            {
                PWM_SetDuty(PWM_DUTY_STOP);

                primask = Enter_Critical();
                GSS.speed                         = PWM_DUTY_STOP;
                GSS.floor_request[GSS.position]   = 0u;
                GSS.door_open                     = 1u;
                GSS.fsm_state                     = (u8)ELV_DOORS_OPEN;
                door_tick_count                   = 0u;
                Exit_Critical(primask);
            }

            break;
        }

        /* ──────────────────────────────────
         * STATE: DOORS_OPEN
         * Motor stopped. Doors stay open for
         * DOOR_OPEN_TICKS × 500ms = 3 seconds.
         * Counter incremented by TIM6 ISR via
         * telem_flag (same 500ms cadence).
         * ────────────────────────────────── */
        case ELV_DOORS_OPEN:
        {
            PWM_SetDuty(PWM_DUTY_STOP);

            /* Use telem_flag as our 500ms tick source */
            if (GSS.telem_flag)
            {
                door_tick_count++;
            }

            if (door_tick_count >= DOOR_OPEN_TICKS)
            {
                /* Doors close — back to IDLE */
                primask = Enter_Critical();
                GSS.door_open   = 0u;
                GSS.fsm_state   = (u8)ELV_IDLE;
                GSS.direction   = 0u;
                door_tick_count = 0u;
                Exit_Critical(primask);
            }

            break;
        }

        /* ──────────────────────────────────
         * STATE: EMERGENCY
         * Latched until manually reset.
         * Motor is already stopped (set above).
         * ────────────────────────────────── */
        case ELV_EMERGENCY:
        {
            /* Stay here until GSS.emergency is cleared
             * by an external reset mechanism (e.g. power
             * cycle or a dedicated reset button ISR).    */
            PWM_SetDuty(PWM_DUTY_STOP);
            break;
        }

        default:
        {
            /* Unknown state — safe default */
            primask = Enter_Critical();
            GSS.fsm_state = (u8)ELV_IDLE;
            Exit_Critical(primask);
            PWM_SetDuty(PWM_DUTY_STOP);
            break;
        }
    }

    /* ── Update IPC Tx Frame after every FSM tick ── */
    primask = Enter_Critical();

    IPC_Handle.TxFrame.current_floor = GSS.position;
    IPC_Handle.TxFrame.fsm_state     = GSS.fsm_state;
    IPC_Handle.TxFrame.target_floor  = GSS.target;
    IPC_Handle.TxFrame.motor_speed   = GSS.speed;

    /* Rebuild flags byte */
    IPC_Handle.TxFrame.flags = 0u;
    if (GSS.emergency)   { IPC_Handle.TxFrame.flags |= IPC_FLAG_EMERGENCY; }
    if (GSS.door_open)   { IPC_Handle.TxFrame.flags |= IPC_FLAG_DOOR_OPEN; }
    if (GSS.direction == 1u) { IPC_Handle.TxFrame.flags |= IPC_FLAG_MOVING_UP; }
    if (GSS.direction == 2u) { IPC_Handle.TxFrame.flags |= IPC_FLAG_MOVING_DN; }

    Exit_Critical(primask);
}

/* ─────────────────────────────────────────
 * System_Logger()            ← MEMBER D
 * ─────────────────────────────────────────
 * Formats elevator telemetry into UART_TxBuf
 * and fires a DMA transfer.
 *
 * Called from main loop ONLY when:
 *   GSS.telem_flag == 1   (set by TIM6 ISR)
 *
 * The flag is cleared here so next call happens
 * after the next 500ms TIM6 period.
 *
 * Format (example):
 *   "ELV|FL:2|ST:1|SP:99|DIR:1|EM:0|CF:0\r\n"
 *    ^    ^    ^    ^     ^     ^    ^
 *    |    |    |    |     |     |    Comm Fault
 *    |    |    |    |     |     Emergency
 *    |    |    |    |     Direction (0/1/2)
 *    |    |    |    Speed (0/20/99)
 *    |    |    FSM state (0=IDLE..4=EMRG)
 *    |    Floor (0–3)
 *    Header
 * ───────────────────────────────────────── */
void System_Logger(void)
{
    u8  len;
    u32 primask;

    /* Guard: only run when TIM6 has fired the flag */
    if (!GSS.telem_flag) { return; }

    /* Guard: don't overwrite buffer mid-transfer */
    if (UART_DMA_Busy)   { return; }

    /* ── Snapshot shared state atomically ── */
    primask = Enter_Critical();

    u8 snap_floor  = GSS.position;
    u8 snap_state  = GSS.fsm_state;
    u8 snap_speed  = GSS.speed;
    u8 snap_dir    = GSS.direction;
    u8 snap_em     = GSS.emergency;
    u8 snap_cf     = GSS.comm_fault;

    /* Clear the telemetry flag now that we've consumed it */
    GSS.telem_flag = 0u;

    Exit_Critical(primask);

    /* ── Format telemetry string into TX buffer ──
     *
     * ELV_Sprintf is our lightweight formatter
     * (no heap, no stdlib printf dependency).
     * Writes directly into the DMA source buffer.
     */
    len = ELV_Sprintf(
        UART_TxBuf,
        UART_TX_BUF_SIZE,
        "ELV|FL:%d|ST:%d|SP:%d|DIR:%d|EM:%d|CF:%d\r\n",
        (u32)snap_floor,
        (u32)snap_state,
        (u32)snap_speed,
        (u32)snap_dir,
        (u32)snap_em,
        (u32)snap_cf
    );

    /* ── Kick off non-blocking DMA transfer ── */
    UART_DMA_Transmit(len);

    /* CPU returns immediately — DMA handles the rest */
}

/* ─────────────────────────────────────────
 * EXTI_Callback()
 * ─────────────────────────────────────────
 * Centralized EXTI dispatcher.
 * All EXTI IRQ handlers call this with the
 * EXTI line number that fired.
 *
 * Responsibilities:
 *   Line 0      → E-Stop: latch emergency, kill PWM
 *   Lines 1–3   → Floor sensors: update position
 *   Lines 6–12  → Hallway buttons: set floor_request
 *
 * No debounce hardware assumed — a 1ms software
 * debounce counter could be added per line if
 * needed (hook into SysTick or TIM6 flag).
 * ───────────────────────────────────────── */
void EXTI_Callback(u8 exti_line)
{
    switch (exti_line)
    {
        /* ── LINE 0: Emergency Stop (PD0) ── */
        case 0u:
        {
            /* Latch emergency flag — ISR runs at priority 0
             * so it will preempt any ongoing FSM work        */
            GSS.emergency = 1u;

            /* Kill PWM immediately from ISR context
             * (safe: PWM_SetDuty only writes CCR1)    */
            PWM_SetDuty(PWM_DUTY_STOP);

            /* Update IPC frame emergency flag */
            IPC_Handle.TxFrame.flags |= IPC_FLAG_EMERGENCY;

            break;
        }

        /* ── LINES 1–3: Floor Sensors (PC1, PC2, PC3) ──
         * Each sensor corresponds to a floor arrival.
         * Line 1 → F2 (floor index 1)
         * Line 2 → F3 (floor index 2)
         * Line 3 → F4 (floor index 3)
         *
         * F1 (PC0, floor index 0) is polled in main loop
         * since EXTI0 is taken by the E-Stop.
         */
        case 1u:
        {
            GSS.position = 1u;   /* Arrived at floor 2 */
            break;
        }
        case 2u:
        {
            GSS.position = 2u;   /* Arrived at floor 3 */
            break;
        }
        case 3u:
        {
            GSS.position = 3u;   /* Arrived at floor 4 */
            break;
        }

        /* ── LINES 6–9: Hallway Buttons (PB6–PB9) ──
         * PB6 → F1 up call
         * PB7 → F1 down call (or F2 up call)
         * PB8 → F2 up/down
         * PB9 → F3 up/down
         * Mapping: each button requests the corresponding floor.
         */
        case 6u:
        {
            GSS.floor_request[0u] = 1u;  /* Hall call → F1 */
            break;
        }
        case 7u:
        {
            GSS.floor_request[1u] = 1u;  /* Hall call → F2 */
            break;
        }
        case 8u:
        {
            GSS.floor_request[2u] = 1u;  /* Hall call → F3 */
            break;
        }
        case 9u:
        {
            GSS.floor_request[3u] = 1u;  /* Hall call → F4 */
            break;
        }

        /* ── LINES 10, 12: Hallway Buttons (PB10, PB12) ──
         * Additional hall call buttons (e.g. top/bottom landings).
         */
        case 10u:
        {
            GSS.floor_request[3u] = 1u;  /* Hall call → F4 top */
            break;
        }
        case 12u:
        {
            GSS.floor_request[0u] = 1u;  /* Hall call → F1 bottom */
            break;
        }

        default:
        {
            /* Unhandled EXTI line — ignore */
            break;
        }
    }
}

/* ─────────────────────────────────────────
 * ELV_Sprintf()
 * ─────────────────────────────────────────
 * Minimal integer-only formatter.
 * Supports %d (u32) only. No heap, no float.
 * Writes into a volatile u8 buffer.
 * Returns number of bytes written.
 * ───────────────────────────────────────── */
u8 ELV_Sprintf(volatile u8 *buf, u8 bufSize,
               const char *fmt, ...)
{
    va_list  args;
    u8       pos = 0u;
    u8       i;
    char     tmp[12];    /* Enough for u32 max (10 digits) */
    u8       tlen;
    u32      val;
    const char *s = fmt;

    va_start(args, fmt);

    while (*s != '\0' && pos < (bufSize - 1u))
    {
        if (*s != '%')
        {
            buf[pos++] = (u8)(*s);
            s++;
            continue;
        }

        s++;  /* Skip '%' */

        if (*s == 'd')
        {
            val  = va_arg(args, u32);
            tlen = 0u;

            /* Convert integer to ASCII string (reversed) */
            if (val == 0u)
            {
                tmp[tlen++] = '0';
            }
            else
            {
                u32 v = val;
                while (v > 0u && tlen < 11u)
                {
                    tmp[tlen++] = (char)('0' + (v % 10u));
                    v /= 10u;
                }
            }

            /* Reverse digits into buffer */
            for (i = tlen; i > 0u && pos < (bufSize - 1u); i--)
            {
                buf[pos++] = (u8)tmp[i - 1u];
            }
        }

        s++;
    }

    va_end(args);

    /* Null-terminate (not counted in return length) */
    buf[pos] = 0u;

    return pos;
}

/* ═══════════════════════════════════════════
 * IRQ HANDLERS
 * Each handler:
 *   1. Calls EXTI_Callback(line)
 *   2. Clears the EXTI pending bit (PR register)
 *      — MUST clear PR or IRQ fires again forever
 * ═══════════════════════════════════════════ */

/* EXTI0 — PD0 Emergency Stop — PRIORITY 0 */
void EXTI0_IRQHandler(void)
{
    EXTI_Callback(0u);
    SET_BIT(EXTI->PR, 0);   /* Clear pending bit for line 0 */
}

/* EXTI1 — PC1 Floor Sensor F2 */
void EXTI1_IRQHandler(void)
{
    EXTI_Callback(1u);
    SET_BIT(EXTI->PR, 1);
}

/* EXTI2 — PC2 Floor Sensor F3 */
void EXTI2_IRQHandler(void)
{
    EXTI_Callback(2u);
    SET_BIT(EXTI->PR, 2);
}

/* EXTI3 — PC3 Floor Sensor F4 */
void EXTI3_IRQHandler(void)
{
    EXTI_Callback(3u);
    SET_BIT(EXTI->PR, 3);
}

/* EXTI4 — PA4 Cabin Button F3 */
void EXTI4_IRQHandler(void)
{
    EXTI_Callback(4u);
    SET_BIT(EXTI->PR, 4);
}

/* EXTI9_5 — PA5 (cabin F4), PB6,PB7,PB8,PB9 (hall) */
void EXTI9_5_IRQHandler(void)
{
    /* Read pending register to find which lines fired */
    u32 pr = EXTI->PR;

    if (READ_BIT(pr, 5))  { EXTI_Callback(5u);  SET_BIT(EXTI->PR, 5);  }
    if (READ_BIT(pr, 6))  { EXTI_Callback(6u);  SET_BIT(EXTI->PR, 6);  }
    if (READ_BIT(pr, 7))  { EXTI_Callback(7u);  SET_BIT(EXTI->PR, 7);  }
    if (READ_BIT(pr, 8))  { EXTI_Callback(8u);  SET_BIT(EXTI->PR, 8);  }
    if (READ_BIT(pr, 9))  { EXTI_Callback(9u);  SET_BIT(EXTI->PR, 9);  }
}

/* EXTI15_10 — PB10, PB12 (hall buttons) */
void EXTI15_10_IRQHandler(void)
{
    u32 pr = EXTI->PR;

    if (READ_BIT(pr, 10)) { EXTI_Callback(10u); SET_BIT(EXTI->PR, 10); }
    if (READ_BIT(pr, 12)) { EXTI_Callback(12u); SET_BIT(EXTI->PR, 12); }
}

/* ─────────────────────────────────────────
 * TIM6_DAC_IRQHandler()
 * ─────────────────────────────────────────
 * Fires every 500ms.
 * Sets GSS.telem_flag = 1 so the main loop
 * knows to call System_Logger().
 *
 * Why not call System_Logger() from here?
 *   - ISR context is stack-constrained
 *   - DMA setup involves multiple register
 *     writes — better done in main loop
 *   - Keeps ISR minimal and fast
 *
 * Also updates IPC comm fault status from
 * the latest IPC_Handle state.
 * ───────────────────────────────────────── */
void TIM6_DAC_IRQHandler(void)
{
    /* Check update interrupt flag (UIF = bit 0 in SR) */
    if (READ_BIT(TIM6->SR, 0))
    {
        /* Clear the UIF flag — mandatory, or IRQ fires forever */
        CLEAR_BIT(TIM6->SR, 0);

        /* Signal main loop to transmit telemetry */
        GSS.telem_flag = 1u;

        /* Mirror IPC comm fault into shared state */
        GSS.comm_fault = IPC_Handle.CommFault;

        /* Advance IPC 50ms update cycle (10 × 50ms = 500ms) */
        /* IPC_Update() is called at 50ms rate from SysTick,
         * but if not using SysTick, call it here.
         * Uncomment if SysTick is not configured:
         * IPC_Update(); */
    }
}