#include "../Inc/Elevator.h"
#include "../Inc/shared.h"
#include "../Inc/dispatcher.h"
#include "../Inc/Bit_Math.h"
#include "../Inc/Pwm.h"      /* تم إضافة الـ Driver الجديد */
#include "../Inc/Timer.h"    /* عشان الـ Macros بتاعة TIMER2 */
#include <stdarg.h> 

/* ─────────────────────────────────────────
 * GLOBAL SHARED STATE (UNIFIED definition)
 * ───────────────────────────────────────── */
volatile GlobalSharedState_t GSS;

/* ─────────────────────────────────────────
 * PWM CONFIGURATION (حسب حسابات الـ 10kHz)
 * ───────────────────────────────────────── */
#define ELV_PWM_TIMER      TIMER2    /* اخترنا Timer 2 لأنه مدعوم في الـ Driver بتاعك */
#define ELV_PWM_CH         1         /* Channel 1 (مثلاً PA0) */
#define ELV_PWM_PSC        15u       /* 16MHz / (15+1) = 1MHz */
#define ELV_PWM_ARR        99u       /* 1MHz / (99+1) = 10kHz */

/* ─────────────────────────────────────────
 * INTERNAL HELPERS
 * ───────────────────────────────────────── */

/* دالة الـ Wrapper لربط الـ FSM بالـ Driver الجديد */
void PWM_SetDuty(u8 duty_percent)
{
    /* بننادي الدالة اللي إنت كاتبها في الـ Pwm.c */
    Pwm_SetDutyPercent(ELV_PWM_TIMER, ELV_PWM_CH, duty_percent);
}

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
 * Elevator_Init()
 * ───────────────────────────────────────── */
void EXTI_Init(void) {
    // 1. Ensure SYSCFG Clock is ON
    RCC->APB2ENR |= (1 << 14);

    // 2. Map Hallway Buttons (PB6-PB12) to EXTI
    // SYSCFG_EXTICR2 (for pins 4-7) and EXTICR3 (for pins 8-11)
    // 0x0001 sets the source to Port B
    SYSCFG->EXTICR[1] |= 0x1100; // Lines 6, 7 -> Port B
    SYSCFG->EXTICR[2] |= 0x1111; // Lines 8, 9, 10, 11 -> Port B
    
    // 3. Map Emergency Stop (PD0)
    SYSCFG->EXTICR[0] &= ~0x000F;
    SYSCFG->EXTICR[0] |= 0x0003; // Line 0 -> Port D

    // 4. Unmask and set falling edge triggers
    EXTI->IMR |= (1 << 0) | (1 << 6) | (1 << 7) | (1 << 8) | (1 << 9) | (1 << 10) | (1 << 12);
    EXTI->FTSR |= (1 << 0) | (1 << 6) | (1 << 7) | (1 << 8) | (1 << 9) | (1 << 10) | (1 << 12);

    // 5. Enable in NVIC
    NVIC_ENABLE_IRQ(IRQ_EXTI0);
    NVIC_ENABLE_IRQ(IRQ_EXTI9_5);
    NVIC_ENABLE_IRQ(40); // EXTI15_10 is IRQ 40 on STM32F401
}
/* ─────────────────────────────────────────
 * Elevator_Update()
 * ───────────────────────────────────────── */
void Elevator_Update(void)
{
    static u8 door_tick_count = 0u;
    u8 up_target;
    u8 dn_target;
    u32 primask;

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

        IPC_Handle.TxFrame.fsm_state    = (u8)ELV_EMERGENCY;
        IPC_Handle.TxFrame.motor_speed  = 0u;
        
        primask = Enter_Critical();
        IPC_Handle.TxFrame.flags |= IPC_FLAG_EMERGENCY;
        Exit_Critical(primask);
        return;
    }

    switch ((ElevatorState_t)GSS.fsm_state)
    {
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

            u8 speed_to_set = PWM_DUTY_STOP;
            primask = Enter_Critical();
            if (up_target != 0xFFu)
            {
                GSS.target    = up_target;
                GSS.direction = 1u;
                GSS.fsm_state = (u8)ELV_MOVING_UP;
                GSS.speed     = PWM_DUTY_FULL;
                speed_to_set  = PWM_DUTY_FULL;
            }
            else if (dn_target != 0xFFu)
            {
                GSS.target    = dn_target;
                GSS.direction = 2u;
                GSS.fsm_state = (u8)ELV_MOVING_DOWN;
                GSS.speed     = PWM_DUTY_FULL;
                speed_to_set  = PWM_DUTY_FULL;
            }
            Exit_Critical(primask);
            PWM_SetDuty(speed_to_set);  /* Use snapshot, not volatile GSS.speed */
            break;
        }

        case ELV_MOVING_UP:
        {
            /* تعديل الـ Ramping باستخدام قيم الـ % */
            if ((GSS.target > GSS.position) && (GSS.target - GSS.position) <= 1u)
            {
                if (GSS.speed != PWM_DUTY_SLOW)
                {
                    primask = Enter_Critical();
                    GSS.speed = PWM_DUTY_SLOW;
                    Exit_Critical(primask);
                    PWM_SetDuty(PWM_DUTY_SLOW);  /* Constant, safe to use */
                }
            }
            else if (GSS.speed != PWM_DUTY_FULL)
            {
                primask = Enter_Critical();
                GSS.speed = PWM_DUTY_FULL;
                Exit_Critical(primask);
                PWM_SetDuty(PWM_DUTY_FULL);  /* Constant, safe to use */
            }

            if (GSS.position == GSS.target)
            {
                primask = Enter_Critical();
                GSS.speed = PWM_DUTY_STOP;
                GSS.floor_request[GSS.position] = 0u;
                GSS.door_open = 1u;
                GSS.fsm_state = (u8)ELV_DOORS_OPEN;
                door_tick_count = 0u;
                Exit_Critical(primask);
                PWM_SetDuty(PWM_DUTY_STOP);
            }
            break;
        }

        case ELV_MOVING_DOWN:
        {
            if ((GSS.position > GSS.target) && (GSS.position - GSS.target) <= 1u)
            {
                if (GSS.speed != PWM_DUTY_SLOW)
                {
                    primask = Enter_Critical();
                    GSS.speed = PWM_DUTY_SLOW;
                    Exit_Critical(primask);
                    PWM_SetDuty(PWM_DUTY_SLOW);  /* Constant, safe to use */
                }
            }
            else if (GSS.speed != PWM_DUTY_FULL)
            {
                primask = Enter_Critical();
                GSS.speed = PWM_DUTY_FULL;
                Exit_Critical(primask);
                PWM_SetDuty(PWM_DUTY_FULL);  /* Constant, safe to use */
            }

            if (GSS.position == GSS.target)
            {
                primask = Enter_Critical();
                GSS.speed = PWM_DUTY_STOP;
                GSS.floor_request[GSS.position] = 0u;
                GSS.door_open = 1u;
                GSS.fsm_state = (u8)ELV_DOORS_OPEN;
                door_tick_count = 0u;
                Exit_Critical(primask);
                PWM_SetDuty(PWM_DUTY_STOP);
            }
            break;
        }

        case ELV_DOORS_OPEN:
        {
            PWM_SetDuty(PWM_DUTY_STOP);
            if (GSS.telem_tick)
            {
                door_tick_count++;
                GSS.telem_tick = 0u;
            }
            if (door_tick_count >= DOOR_OPEN_TICKS)
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

    /* ── Update IPC Tx Frame ── */
    primask = Enter_Critical();
    IPC_Handle.TxFrame.current_floor = GSS.position;
    IPC_Handle.TxFrame.fsm_state     = GSS.fsm_state;
    IPC_Handle.TxFrame.target_floor  = GSS.target;
    IPC_Handle.TxFrame.motor_speed   = GSS.speed;
    IPC_Handle.TxFrame.reserved      = SystemState.master_state.reserved;
    IPC_Handle.TxFrame.flags = 0u;
    if (GSS.emergency)       { IPC_Handle.TxFrame.flags |= IPC_FLAG_EMERGENCY; }
    if (GSS.door_open)       { IPC_Handle.TxFrame.flags |= IPC_FLAG_DOOR_OPEN; }
    if (GSS.direction == 1u) { IPC_Handle.TxFrame.flags |= IPC_FLAG_MOVING_UP; }
    if (GSS.direction == 2u) { IPC_Handle.TxFrame.flags |= IPC_FLAG_MOVING_DN; }
    Exit_Critical(primask);
}

/* باقي الملف (System_Logger, EXTI_Callback, إلخ) يظل كما هو دون تغيير */