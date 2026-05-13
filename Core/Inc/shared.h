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
 * FSM States 
 * 0: Idle, 1: Up, 2: Down, 3: Emergency
 */
typedef enum {
    STATE_IDLE      = 0,
    STATE_UP        = 1,
    STATE_DOWN      = 2,
    STATE_EMERGENCY = 3
} FSM_State_t;

/* 
 * SPI Packet Definition (8-Byte Frame)
 */
typedef struct {
    u8 header;          /* Byte 0: 0xA5 */
    u8 state;           /* Byte 1: FSM_State_t (0-3) */
    u8 current_floor;   /* Byte 2: 1, 2, 3, or 4 */
    u8 target_floor;    /* Byte 3: 1, 2, 3, or 4 */
    u8 door_status;     /* Byte 4: 0: Closed, 1: Open */
    u8 internal_reqs;   /* Byte 5: Bitmask (Bit 0-3) */
    u8 reserved;        /* Byte 6: Reserved */
    u8 checksum;        /* Byte 7: XOR sum of 0-6 */
} SPI_Packet_t;

/* 
 * Global Shared State
 * Used by Member C (Dispatcher) and Member D (Telemetry)
 */
typedef struct {
    SPI_Packet_t master_state; /* Elevator A */
    SPI_Packet_t slave_state;  /* Elevator B */
    volatile u8 comm_fault;    /* 1 if SPI fails (>200ms) */
} GlobalSharedState_t;

extern GlobalSharedState_t SystemState;

#endif /* SHARED_H */
