#ifndef SHARED_TYPES_H
#define SHARED_TYPES_H

#include "std_types.h"

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

#endif /* SHARED_TYPES_H */
