/**
 * shared.h
 * Application-level shared types for the Dual-Elevator system.
 * Only elevator logic lives here — NO dead code from other projects.
 */
#ifndef SHARED_H
#define SHARED_H

#include "std_types.h"

/* ─────────────────────────────────────────────────────────────────────────
 * FSM STATES
 * Values MUST match the ElevatorState_t used in the SPI IPC packet (Byte 2).
 * ───────────────────────────────────────────────────────────────────────── */
typedef enum {
    STATE_IDLE       = 0,
    STATE_UP         = 1,
    STATE_DOWN       = 2,
    STATE_DOOR_OPEN  = 3,
    STATE_EMERGENCY  = 4
} FSM_State_t;

/* ─────────────────────────────────────────────────────────────────────────
 * SPI IPC PACKET — 8-byte fixed-length frame
 *
 * Byte 0: Header    = 0xA5
 * Byte 1: floor     = current floor (0-3)
 * Byte 2: state     = FSM_State_t (0-4)
 * Byte 3: target    = target floor (0-3)
 * Byte 4: speed     = 0 / 20 / 100 (PWM duty %)
 * Byte 5: flags     = bit0=door_open, bit1=emergency
 * Byte 6: reserved  = 0x00
 * Byte 7: checksum  = XOR of bytes 0..6
 * ───────────────────────────────────────────────────────────────────────── */
#define IPC_HEADER          0xA5u
#define IPC_PACKET_SIZE     8u

typedef struct {
    u8 header;          /* Byte 0 */
    u8 current_floor;   /* Byte 1 */
    u8 fsm_state;       /* Byte 2 */
    u8 target_floor;    /* Byte 3 */
    u8 motor_speed;     /* Byte 4 */
    u8 flags;           /* Byte 5 */
    u8 reserved;        /* Byte 6 */
    u8 checksum;        /* Byte 7 */
} SPI_Packet_t;

/* flags byte bitmask */
#define FLAG_DOOR_OPEN      (1u << 0)
#define FLAG_EMERGENCY      (1u << 1)

/* ─────────────────────────────────────────────────────────────────────────
 * GLOBAL SHARED STATE
 * Single struct accessed by ALL modules (Elevator, Dispatcher, IPC, Logger).
 * All fields are volatile because they are written by ISRs and read by main.
 * Access with Enter_Critical() / Exit_Critical() for multi-field updates.
 * ───────────────────────────────────────────────────────────────────────── */
#define NUM_FLOORS          4u

/* Hallway call queue — indexed [floor][0=UP,1=DOWN] */
#define DIR_UP              0u
#define DIR_DOWN            1u

typedef struct {
    /* ── Master Elevator (Elevator A) ── */
    volatile u8  position;              /* Current floor 0-3            */
    volatile u8  target;                /* Target floor  0-3            */
    volatile u8  direction;             /* 0=none 1=up 2=down           */
    volatile u8  speed;                 /* PWM duty: 0, 20, or 100      */
    volatile u8  fsm_state;             /* FSM_State_t value            */
    volatile u8  emergency;             /* 1 = emergency stop active    */
    volatile u8  door_open;             /* 1 = doors currently open     */
    volatile u8  door_ticks;            /* Counts down from DOOR_TICKS  */

    /* ── Requests ── */
    volatile u8  cabin_request[NUM_FLOORS];    /* cabin buttons F1-F4   */
    volatile u8  hall_request[NUM_FLOORS][2];  /* hall[floor][UP/DOWN]  */

    /* ── IPC / Slave State ── */
    volatile u8  comm_fault;            /* 1 = IPC timeout              */
    volatile u8  slave_position;        /* Slave's current floor        */
    volatile u8  slave_fsm_state;       /* Slave's FSM state            */
    volatile u8  slave_target;          /* Slave's target floor         */
    volatile u8  slave_assigned_target; /* Target assigned to slave by Master */
    volatile u8  slave_speed;           /* Slave's PWM speed            */
    volatile u8  slave_flags;           /* Slave's status flags         */
    volatile u32 last_valid_rx_tick;    /* Tick of last good SPI RX     */

    /* ── Timing Flags (set by ISRs, cleared by main loop) ── */
    volatile u8  ipc_tick_flag;         /* Set by TIM2 IRQ  (50ms)     */
    volatile u8  telem_flag;            /* Set by TIM6 IRQ  (500ms)    */

    /* ── IPC Packets ── */
    SPI_Packet_t tx_packet;             /* Packet being sent to Slave   */
    SPI_Packet_t rx_packet;             /* Last received from Slave     */
} GlobalSharedState_t;

/* Defined once in Elevator.c, declared here for all modules */
extern volatile GlobalSharedState_t GSS;

/* Global tick counter incremented by TIM2 every 50ms */
extern volatile u32 g_ipc_tick;

#endif /* SHARED_H */
