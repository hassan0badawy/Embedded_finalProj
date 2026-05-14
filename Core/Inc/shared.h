#ifndef SHARED_H
#define SHARED_H

#include "std_types.h"

/* ── Hardware Specific Definitions (RCC) ── */
#define RCC_BASE_ADDR      0x40023800

typedef struct {
    volatile u32 CR;
    volatile u32 PLLCFGR;
    volatile u32 CFGR;
    volatile u32 CIR;
    volatile u32 AHB1RSTR;
    volatile u32 AHB2RSTR;
    volatile u32 AHB3RSTR;
    u32 Reserved0;
    volatile u32 APB1RSTR;
    volatile u32 APB2RSTR;
    u32 Reserved1[2];
    volatile u32 AHB1ENR;
    volatile u32 AHB2ENR;
    volatile u32 AHB3ENR;
    u32 Reserved2;
    volatile u32 APB1ENR;
    volatile u32 APB2ENR;
} RccType;

#define RCC                ((RccType *)RCC_BASE_ADDR)
#define RCC_AHB1ENR        (RCC->AHB1ENR)  /* Compatibility macro */
#define RCC_AHB1ENR_GPIOAEN (1u << 0)
#define RCC_AHB1ENR_GPIOBEN (1u << 1)
#define RCC_AHB1ENR_GPIOCEN (1u << 2)
#define RCC_AHB1ENR_GPIODEN (1u << 3)

typedef enum {
    LOCKED   = 0,
    UNLOCKED = 1,
    ALARM    = 2
} SystemState_t;

typedef enum {
    SEQ_IDLE     = 0,
    SEQ_CORRECT  = 1,
    SEQ_WRONG    = 2,
    SEQ_COMPLETE = 3
} SequenceState_t;

typedef struct {
    u8              has_input;  /* 1 if a key was pressed this cycle     */
    char            key;        /* '0'-'9', '*', '#', or NO_KEY          */
    SequenceState_t seq_state;  /* result of last sequence evaluation    */
    u8              lock_cmd;   /* 1 when '#' is pressed (lock request)  */
} InputEvent_t;

/* ── Global Elevator Shared State ── */

/* 
 * FSM States (matches IPC ElevatorState_t values from ipc.h)
 * IMPORTANT: These must remain in sync with ElevatorState_t
 * used in IPC communication (Byte 2 of SPI packet)
 * 
 * FSM_State_t is used internally by Dispatcher and Elevator
 * but the values must match ElevatorState_t for IPC compatibility
 */
typedef enum {
    STATE_IDLE      = 0,      /* Idle */
    STATE_UP        = 1,      /* Moving UP (matches ELV_MOVING_UP) */
    STATE_DOWN      = 2,      /* Moving DOWN (matches ELV_MOVING_DOWN) */
    STATE_DOOR_OPEN = 3,      /* Doors OPEN (matches ELV_DOORS_OPEN) */
    STATE_EMERGENCY = 4       /* Emergency (matches ELV_EMERGENCY) */
} FSM_State_t;

/* 
 * SPI Packet Definition (8-Byte Frame)
 */
typedef struct {
    u8 header;          /* Byte 0: 0xA5 */
    u8 current_floor;   /* Byte 1: where elevator is now (0..3) */
    u8 fsm_state;       /* Byte 2: ElevatorState_t (0..4) */
    u8 target_floor;    /* Byte 3: where elevator is going (0..3) */
    u8 motor_speed;     /* Byte 4: 0, 20, 100 (maps to PWM duty) */
    u8 flags;           /* Byte 5: bit-packed status (door, emg, etc.) */
    u8 reserved;        /* Byte 6: reserved (used for task payload) */
    u8 checksum;        /* Byte 7: XOR sum of bytes 0..6 */
} SPI_Packet_t;

/* ─────────────────────────────────────────
 * UNIFIED GLOBAL SHARED STATE
 * Single struct used by ALL modules: Elevator.c, Dispatcher.c, IPC.c, uart_dma.c
 * ───────────────────────────────────────── */
typedef struct __attribute__((packed)) {
    /* ─ Master Elevator Local State ─ */
    volatile u8  position;          /* Current floor (0–3) */
    volatile u8  target;            /* Target floor  (0–3) */
    volatile u8  direction;         /* 0=none, 1=up, 2=down */
    volatile u8  speed;             /* PWM duty: 0, 20, or 100 */
    volatile u8  fsm_state;         /* FSM_State_t value */
    volatile u8  emergency;         /* 1 = emergency stop active */
    volatile u8  door_open;         /* 1 = doors currently open */
    volatile u8  comm_fault;        /* 1 = IPC link lost (timeout) */
    volatile u8  telem_flag;        /* 1 = TIM6 fired, send telemetry */
    volatile u8  telem_tick;        /* 1 = TIM6 500ms tick */
    volatile u8  floor_request[4];  /* Pending cabin floor requests */
    
    /* ─ Slave Elevator State (from last SPI RX) ─ */
    volatile u8  slave_position;    /* Slave's current floor */
    volatile u8  slave_fsm_state;   /* Slave's FSM state */
    volatile u8  slave_target;      /* Slave's target floor */
    volatile u8  slave_speed;       /* Slave's PWM speed */
    volatile u8  slave_flags;       /* Slave's status flags */
    
    /* ─ IPC Management ─ */
    SPI_Packet_t last_rx_packet;    /* Last valid SPI packet received */
    volatile u32 last_valid_rx_tick;/* Tick of last good RX (for timeout) */
} GlobalSharedState_t;

/* ─────────────────────────────────────────
 * GLOBAL INSTANCE (defined in Elevator.c)
 * ───────────────────────────────────────── */
extern volatile GlobalSharedState_t GSS;

#endif /* SHARED_H */
