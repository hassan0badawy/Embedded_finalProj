/*
 * ═════════════════════════════════════════════════════════════════════════════
 * MAIN LOOP — Dual-Elevator Master Controller (STM32F401VE)
 * ═════════════════════════════════════════════════════════════════════════════
 *
 * Architecture:
 *   1. Startup sequence (Elevator_Init, IPC_Init)
 *   2. Infinite loop:
 *      - Call Elevator_Update() → FSM tick, motor control, IPC sync
 *      - If telem_flag set → System_Logger() → Format & DMA telemetry
 *      - Poll unpinned floor sensor (PC0/F1) and cabin buttons (PA0/PA1)
 *
 * ISR Hierarchy (NVIC priorities, lower = higher precedence):
 *   Priority 0: EXTI0 (Emergency Stop)       ← Highest
 *   Priority 1: SPI1 (IPC Master/Slave rx)
 *   Priority 2: EXTI1–3 (Floor sensors)
 *   Priority 3: EXTI4–15, UART DMA, DMA
 *   Priority 4: TIM6 (500ms telemetry tick) ← Lowest
 *
 * CRITICAL SECTION USAGE:
 *   All accesses to GSS (GlobalSharedState) and IPC_Handle
 *   that happen in both ISR and main context must be wrapped:
 *   
 *       u32 primask = Enter_Critical();
 *       // Access shared state
 *       Exit_Critical(primask);
 *
 * NO HAL_Delay() anywhere — blocking delays cause race conditions
 * and missed interrupts. Use TIM6 telem_flag for time-based logic instead.
 *
 * ═════════════════════════════════════════════════════════════════════════════
 */

#include "../Inc/std_types.h"
#include "../Inc/Elevator.h"
#include "../Inc/uart_dma.h"
#include "../Inc/pwm.h"
#include "../Inc/ipc.h"
#include "../Inc/spi.h"
#include "../Inc/Bit_Math.h"
#include "../Inc/dispatcher.h"


/* ─────────────────────────────────────────────────────────────────────────────
 * POLLING HELPERS (for unpinned pins: PC0=F1 sensor, PA0/PA1=cabin buttons)
 * ─────────────────────────────────────────────────────────────────────────────
 */

/*
 * Poll_FloorSensorF1()
 * ─────────────────────
 * PC0 is the floor sensor for floor 1 (F1).
 * It's not assigned to an EXTI line because EXTI0 is taken by E-Stop (PD0).
 * Instead, we poll it here in the main loop.
 *
 * Logic:
 *   - Pin normally HIGH (pull-up), goes LOW when floor is reached
 *   - When we detect a falling edge (HIGH → LOW), set position to floor 0
 *   - Use a static debounce counter to avoid bouncing
 */
static volatile u8 poll_f1_last = 1u;      /* Track last sampled state */

static void Poll_FloorSensorF1(void)
{
    /* Read PC0 (bit 0 of IDR) */
    u8 pc0_now = (u8)READ_BIT(GPIOC->IDR, 0);

    /* Falling edge detection: transition from 1 to 0 */
    if (poll_f1_last && !pc0_now)
    {
        GSS.position = 0u;  /* Arrived at floor 1 */
    }

    poll_f1_last = pc0_now;
}

/*
 * Poll_CabinButtons()
 * ────────────────────────
 * PA0 = Cabin button F1 (floor 0)
 * PA1 = Cabin button F2 (floor 1)
 * PA2 = Cabin button F3 (floor 2)
 * PA3 = Cabin button F4 (floor 3)
 *
 * PA0 conflicts with EXTI0 on PD0 (E-Stop), so we poll PA0.
 * PA1 could conflict with EXTI1 on PC1 (Floor sensor), so we poll PA1.
 * PA2/PA3 could conflict with EXTI2/3 on PC2/PC3 (Floor sensors), so we poll them.
 */
static volatile u8 poll_pa0_last = 1u;
static volatile u8 poll_pa1_last = 1u;
static volatile u8 poll_pa2_last = 1u;
static volatile u8 poll_pa3_last = 1u;

static void Poll_CabinButtons(void)
{
    /* PA0 — Cabin button F1 (floor 0) */
    u8 pa0_now = (u8)READ_BIT(GPIOA->IDR, 0);
    if (poll_pa0_last && !pa0_now)
    {
        GSS.floor_request[0u] = 1u;
    }
    poll_pa0_last = pa0_now;

    /* PA1 — Cabin button F2 (floor 1) */
    u8 pa1_now = (u8)READ_BIT(GPIOA->IDR, 1);
    if (poll_pa1_last && !pa1_now)
    {
        GSS.floor_request[1u] = 1u;
    }
    poll_pa1_last = pa1_now;

    /* PA2 — Cabin button F3 (floor 2) */
    u8 pa2_now = (u8)READ_BIT(GPIOA->IDR, 2);
    if (poll_pa2_last && !pa2_now)
    {
        GSS.floor_request[2u] = 1u;
    }
    poll_pa2_last = pa2_now;

    /* PA3 — Cabin button F4 (floor 3) */
    u8 pa3_now = (u8)READ_BIT(GPIOA->IDR, 3);
    if (poll_pa3_last && !pa3_now)
    {
        GSS.floor_request[3u] = 1u;
    }
    poll_pa3_last = pa3_now;
}

/* ─────────────────────────────────────────────────────────────────────────────
 * main()
 * ─────────────────────────────────────────────────────────────────────────────
 * Entry point. Initializes all subsystems, then enters infinite loop.
 *
 * Startup Sequence:
 *   1. Elevator_Init()        → Clears GSS, enables PWM, UART+DMA, EXTI, TIM6
 *   2. IPC_Init(1)            → Initialize SPI as Master
 *   3. Infinite loop:
 *      - Elevator_Update()    → FSM, speed ramp, IPC TxFrame sync
 *      - System_Logger()      → Send telemetry via DMA (if telem_flag set)
 *      - Poll_FloorSensorF1() → Handle unpinned F1 sensor
 *      - Poll_CabinButtons()  → Handle unpinned cabin buttons
 *
 * ─────────────────────────────────────────────────────────────────────────────
 */
int main(void)
{
    /* ── INITIALIZATION PHASE ── */

    /*
     * Step 1: Bring up all elevator subsystems
     *   - Clear GlobalSharedState
     *   - Initialize PWM (TIM1, PA8, 10kHz)
     *   - Initialize UART2 + DMA1 Stream6 (telemetry)
     *   - Initialize EXTI (sensors, buttons, e-stop)
     *   - Initialize TIM6 (500ms interrupt tick)
     *   - Initialize IPC/SPI (Slave controller link)
     */
    Elevator_Init();
    Dispatcher_Init();

    /*
     * Step 2: Configure this MCU as SPI Master in IPC layer
     *   - Sets up Master TX/RX cycles
     *   - Enables SPI1 with DMA
     *   - Starts periodic SPI exchanges (if SysTick is configured)
     */
    /* IPC role determined by build definition `IS_MASTER` set in CMake */
#if defined(IS_MASTER) && (IS_MASTER == 1)
    IPC_Init(1u);  /* Master */
#else
    IPC_Init(0u);  /* Slave */
#endif

    /* Configure SysTick to drive IPC_Update() every 50ms */
    {
        /* Cortex-M SysTick registers */
        volatile uint32_t *SYST_CSR = (uint32_t *)0xE000E010UL;
        volatile uint32_t *SYST_RVR = (uint32_t *)0xE000E014UL;
        volatile uint32_t *SYST_CVR = (uint32_t *)0xE000E018UL;
        const uint32_t reload = 800000UL - 1UL; /* 16MHz * 0.05s - 1 */
        *SYST_RVR = reload;
        *SYST_CVR = 0UL;
        /* Enable SysTick: CLKSOURCE=Core, TICKINT=1, ENABLE=1 */
        *SYST_CSR = (1UL << 2) | (1UL << 1) | (1UL << 0);
    }

    /* ══════════════════════════════════════════════════════════════════════════
     * INFINITE MAIN LOOP
     * ══════════════════════════════════════════════════════════════════════════
     *
     * This loop runs at full CPU speed (16 MHz) with NO blocking calls.
     * All timing is managed by ISRs and non-blocking flags:
     *   - ISRs set flags: GSS.telem_flag, GSS.floor_request[]
     *   - Main loop reads flags, acts on them, clears them
     *   - ISRs (priority 0–2) can preempt main loop any time
     *   - Motor control is glitch-free because PWM changes are immediate
     */
    while (1)
    {
        /* ──────────────────────────────────────────────────────────────────
         * STEP 1: FSM Update
         * ──────────────────────────────────────────────────────────────────
         * Elevator_Update() is the core state machine:
         *   - Reads GSS flags from ISRs atomically
         *   - Executes FSM transitions (IDLE → MOVING_UP/DOWN → DOORS_OPEN)
         *   - Adjusts PWM speed (ramp down when approaching floor)
         *   - Handles emergency stop (highest ISR priority, runs first)
         *   - Updates IPC TxFrame for Slave sync
         *
         * Call frequency: As fast as possible (main loop speed ~ 1000+ Hz)
         *
         * Why not in an ISR?
         *   - FSM logic is complex (multiple branches, PWM updates)
         *   - Stack-constrained in ISR context
         *   - Better responsiveness if main loop can preempt FSM updates
         */
        Elevator_Update();
        Dispatcher_Update();

        /* ──────────────────────────────────────────────────────────────────
         * STEP 2: Telemetry Transmission (500ms cadence via TIM6)
         * ──────────────────────────────────────────────────────────────────
         * System_Logger() is called ONLY when TIM6 fires every 500ms:
         *   - Copies GSS snapshot atomically
         *   - Formats telemetry string: "ELV|FL:x|ST:x|SP:x|DIR:x|EM:x|CF:x\r\n"
         *   - Initiates DMA transfer (non-blocking)
         *   - Clears the telem_flag for next cycle
         *
         * Why check flag before calling?
         *   - Avoid wasting CPU formatting when no new data
         *   - Flag is set by TIM6_DAC_IRQHandler every 500ms
         *
         * Why call from main, not ISR?
         *   - DMA setup involves volatile register writes that are safer
         *     to do outside ISR context
         *   - Allows previous DMA transfer to complete before starting next
         */
        if (GSS.telem_flag)
        {
            System_Logger();
        }

        /* ──────────────────────────────────────────────────────────────────
         * STEP 3: Poll Unpinned Inputs (PC0, PA0, PA1)
         * ──────────────────────────────────────────────────────────────────
         * These pins couldn't be assigned to EXTI lines due to conflicts:
         *
         * PC0 = Floor Sensor F1
         *   - EXTI0 is taken by E-Stop (PD0)
         *   - Polled here for falling edge (floor reached)
         *
         * PA0 = Cabin Button F1
         *   - EXTI0 is taken by E-Stop (PD0)
         *   - Polled here for falling edge (button pressed)
         *
         * PA1 = Cabin Button F2
         *   - Could conflict with EXTI1/PC1 (Floor sensor)
         *   - Polled here for consistency
         *
         * Poll frequency: Main loop speed (~1000+ Hz)
         * Debounce: Built into Poll_* functions (static last-state tracking)
         */
        Poll_FloorSensorF1();
        Poll_CabinButtons();

        /* ──────────────────────────────────────────────────────────────────
         * STEP 4: IPC Periodic Update (Optional — if SysTick not configured)
         * ──────────────────────────────────────────────────────────────────
         * IPC_Update() should be called at 50ms intervals.
         * Normally this is done by SysTick handler.
         *
         * If SysTick is not available, uncomment the following and add your
         * own 50ms timer logic (e.g., using TIM6 counter or a free timer).
         *
         * Uncomment if SysTick is disabled:
              if (IPC_50ms_tick)  // Your 50ms flag
         *     {
         *         IPC_Update();
         *         IPC_50ms_tick = 0;
         *     }
         */
        /* IPC_Update(); */  // Disabled — SysTick handles this

        /* ──────────────────────────────────────────────────────────────────
         * End of main loop iteration
         * ──────────────────────────────────────────────────────────────────
         * The CPU repeats this loop at full speed until an ISR fires.
         * ISRs have priority 0–4 and can interrupt the main loop:
         *
         * When EXTI0 (E-Stop, priority 0) fires:
         *   - Current main loop instruction is paused
         *   - EXTI0_IRQHandler runs → calls EXTI_Callback(0) → latches emergency
         *   - Main loop resumes
         *   - Elevator_Update() detects emergency, kills motor, halts motion
         *
         * When TIM6 (priority 4) fires every 500ms:
         *   - Sets GSS.telem_flag = 1
         *   - Next main loop iteration sees flag and calls System_Logger()
         */
    }

    /* ──────────────────────────────────────────────────────────────────────
     * This point is never reached in normal operation.
     * The while(1) loop runs forever until power loss or reset.
     * ────────────────────────────────────────────────────────────────────── */
    return 0;
}

/* ─────────────────────────────────────────────────────────────────────────────
 * OPTIONAL: SysTick ISR (if SysTick is used for 50ms IPC ticks)
 * ─────────────────────────────────────────────────────────────────────────────
 *
 * If you're using SysTick for timing, configure it for 50ms (20Hz):
 *   STM32F401VE @ 16MHz SysTick clock
 *   50ms = 0.050s → Count = 16MHz × 0.050 = 800,000
 *
 * Uncomment and integrate if needed:
 *
 *   static u32 systick_count = 0;
 *   
 *   void SysTick_Handler(void)
 *   {
 *       // Simple 50ms ticker: call IPC_Update() each SysTick
 *   }
*/
void SysTick_Handler(void)
{
    IPC_Update();
}


