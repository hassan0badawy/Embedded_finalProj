#include "../Inc/shared.h"
#include "../Inc/elevator.h"
#include "../Inc/dispatcher.h"
#include "../Inc/ipc.h"

/* ─────────────────────────────────────────
 * INTERNAL DATA
 * ───────────────────────────────────────── */
static volatile HallwayCall_t HallwayQueue[TOTAL_HALLWAY_CALLS];

/* Provide the actual instance for the global system state */
GlobalSharedState_t SystemState;

/* ─────────────────────────────────────────
 * HELPERS
 * ───────────────────────────────────────── */
static s8 CalculateScore(u8 elv_floor, FSM_State_t elv_state, u8 call_floor, CallDirection_t call_dir) {
    s8 distance = (s8)elv_floor - (s8)call_floor;
    if (distance < 0) distance = -distance;

    /* Rule 1: Immediate / Idle Match */
    if (elv_state == STATE_IDLE) {
        return (s8)distance; /* Score is just distance (0 to 3) */
    }

    /* Rule 2: Perfect Directional Match (Moving towards in same direction) */
    if (elv_state == STATE_UP && call_dir == DIR_UP && elv_floor < call_floor) {
        return (s8)distance;
    }
    if (elv_state == STATE_DOWN && call_dir == DIR_DOWN && elv_floor > call_floor) {
        return (s8)distance;
    }

    /* Rule 3: Passed Match (Same direction but already passed) */
    if (elv_state == STATE_UP && call_dir == DIR_UP && elv_floor >= call_floor) {
        return (s8)(10 + distance); /* Penalty for having to finish path first */
    }
    if (elv_state == STATE_DOWN && call_dir == DIR_DOWN && elv_floor <= call_floor) {
        return (s8)(10 + distance);
    }

    /* Rule 4: Opposite Direction (Moving away or mismatched) */
    return (s8)(20 + distance); /* Highest penalty */
}

/* ─────────────────────────────────────────
 * CORE FUNCTIONS
 * ───────────────────────────────────────── */

void Dispatcher_Init(void) {
    for (u8 i = 0; i < TOTAL_HALLWAY_CALLS; i++) {
        HallwayQueue[i].is_active = 0;
        HallwayQueue[i].assigned_to = ELV_NONE;
        
        /* Initialize floor (1-4) and direction for the 6 fixed buttons */
        if (i == CALL_U1) { HallwayQueue[i].floor = 1; HallwayQueue[i].dir = DIR_UP;   }
        if (i == CALL_D2) { HallwayQueue[i].floor = 2; HallwayQueue[i].dir = DIR_DOWN; }
        if (i == CALL_U2) { HallwayQueue[i].floor = 2; HallwayQueue[i].dir = DIR_UP;   }
        if (i == CALL_D3) { HallwayQueue[i].floor = 3; HallwayQueue[i].dir = DIR_DOWN; }
        if (i == CALL_U3) { HallwayQueue[i].floor = 3; HallwayQueue[i].dir = DIR_UP;   }
        if (i == CALL_D4) { HallwayQueue[i].floor = 4; HallwayQueue[i].dir = DIR_DOWN; }
    }
}

void Dispatcher_RegisterCall(u8 floor, CallDirection_t direction) {
    for (u8 i = 0; i < TOTAL_HALLWAY_CALLS; i++) {
        if (HallwayQueue[i].floor == floor && HallwayQueue[i].dir == direction) {
            if (HallwayQueue[i].assigned_to == ELV_NONE) {
                HallwayQueue[i].is_active = 1;
            }
            break;
        }
    }
}

u8 Dispatcher_AssignTask(u8 call_floor, CallDirection_t call_dir) {
    /* 1. Comm Fault Handling: Master (Elv A) takes everything */
    if (SystemState.comm_fault) {
        return ELV_A;
    }

    /* 2. Get Current States */
    u8 elvA_floor = SystemState.master_state.current_floor;
    /* Map IPC fsm_state to internal FSM_State_t (treat DOORS_OPEN as IDLE)
     * Master/Slave send fsm values as in ipc.h (0..4). Convert to
     * Dispatcher FSM_State_t which uses: STATE_IDLE=0, STATE_UP=1,
     * STATE_DOWN=2, STATE_EMERGENCY=3. */
    FSM_State_t elvA_state;
    switch (SystemState.master_state.fsm_state)
    {
        case 1: elvA_state = STATE_UP; break;      /* MOVING_UP */
        case 2: elvA_state = STATE_DOWN; break;    /* MOVING_DOWN */
        case 3: elvA_state = STATE_IDLE; break;    /* DOORS_OPEN -> treat as IDLE */
        case 4: elvA_state = STATE_EMERGENCY; break;
        default: elvA_state = STATE_IDLE; break;
    }

    u8 elvB_floor = SystemState.slave_state.current_floor;
    FSM_State_t elvB_state;
    switch (SystemState.slave_state.fsm_state)
    {
        case 1: elvB_state = STATE_UP; break;
        case 2: elvB_state = STATE_DOWN; break;
        case 3: elvB_state = STATE_IDLE; break;
        case 4: elvB_state = STATE_EMERGENCY; break;
        default: elvB_state = STATE_IDLE; break;
    }

    /* 3. Calculate Scores (Lower is better) */
    s8 scoreA = CalculateScore(elvA_floor, elvA_state, call_floor, call_dir);
    s8 scoreB = CalculateScore(elvB_floor, elvB_state, call_floor, call_dir);

    /* 4. Comparison */
    if (scoreA <= scoreB) {
        return ELV_A;
    } else {
        return ELV_B;
    }
}

void Dispatcher_Update(void) {
    for (u8 i = 0; i < TOTAL_HALLWAY_CALLS; i++) {
        if (HallwayQueue[i].is_active && HallwayQueue[i].assigned_to == ELV_NONE) {
            u8 assigned = Dispatcher_AssignTask(HallwayQueue[i].floor, HallwayQueue[i].dir);
            HallwayQueue[i].assigned_to = assigned;
            
            /* Master handles updating its own targets or sending Slave's target via SPI */
            if (assigned == ELV_B) {
                /* Place the assigned target into master_state.reserved
                 * so the Elevator/IPC layer will forward it to the Slave
                 * inside the next 50ms IPC cycle. */
                SystemState.master_state.reserved = (u8)HallwayQueue[i].floor;
            }
        }
        
        /* Check if call is completed (elevator reached floor and doors open) */
        u8 f = HallwayQueue[i].floor;
        if (HallwayQueue[i].assigned_to == ELV_A) {
            /* Door open detected via flags bit */
            u8 master_door = (SystemState.master_state.flags & IPC_FLAG_DOOR_OPEN) ? 1u : 0u;
            if (SystemState.master_state.current_floor == f && master_door == 1) {
                HallwayQueue[i].is_active = 0;
                HallwayQueue[i].assigned_to = ELV_NONE;
            }
        } else if (HallwayQueue[i].assigned_to == ELV_B) {
            u8 slave_door = (SystemState.slave_state.flags & IPC_FLAG_DOOR_OPEN) ? 1u : 0u;
            if (SystemState.slave_state.current_floor == f && slave_door == 1) {
                HallwayQueue[i].is_active = 0;
                HallwayQueue[i].assigned_to = ELV_NONE;
            }
        }
    }
}
/* DISPATCHER_c */