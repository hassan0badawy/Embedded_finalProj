#ifndef DISPATCHER_H
#define DISPATCHER_H

#include "shared_types.h"

/* ─────────────────────────────────────────
 * HALLWAY CALL DEFINITIONS
 * ───────────────────────────────────────── */
typedef enum {
    DIR_UP   = 0,
    DIR_DOWN = 1
} CallDirection_t;

typedef struct {
    u8 floor;               /* 1 to 4 (Floor 1 to 4) */
    CallDirection_t dir;    /* DIR_UP or DIR_DOWN    */
    volatile u8 is_active;  /* 1 if call is pending  */
    volatile u8 assigned_to;/* 0=None, 1=Elv A, 2=Elv B */
} HallwayCall_t;

/* The 6 hallway buttons */
#define CALL_U1    0
#define CALL_D2    1
#define CALL_U2    2
#define CALL_D3    3
#define CALL_U3    4
#define CALL_D4    5
#define TOTAL_HALLWAY_CALLS 6

/* Elevator IDs */
#define ELV_NONE   0
#define ELV_A      1
#define ELV_B      2

/* ─────────────────────────────────────────
 * FUNCTION PROTOTYPES
 * ───────────────────────────────────────── */

void Dispatcher_Init(void);
void Dispatcher_RegisterCall(u8 floor, CallDirection_t direction);
void Dispatcher_Update(void);
u8 Dispatcher_AssignTask(u8 call_floor, CallDirection_t call_dir);

#endif /* DISPATCHER_H */
/* DISPATCHER_H */