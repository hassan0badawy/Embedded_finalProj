# Critical Fixes Guide
## Quick Code Patches for High-Priority Issues

---

## FIX #1: Unify Global Shared State (CRITICAL)

### Problem
Two separate struct definitions cause **data loss** between Elevator.c and dispatcher.c:
- `GlobalSharedState GSS;` (Elevator uses this)
- `GlobalSharedState_t SystemState;` (Dispatcher uses this)
- **Result**: Dispatcher never sees updated elevator state!

### Solution: Replace shared.h definitions

**File**: `Core/Inc/shared.h`

**DELETE** the existing `GlobalSharedState_t` struct (lines 76-83) and **REPLACE** the entire file with:

```c
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
#define RCC_AHB1ENR        (RCC->AHB1ENR)
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
    u8              has_input;
    char            key;
    SequenceState_t seq_state;
    u8              lock_cmd;
} InputEvent_t;

/* ── Global Elevator Shared State (UNIFIED) ── */

/* FSM States */
typedef enum {
    STATE_IDLE      = 0,
    STATE_UP        = 1,
    STATE_DOWN      = 2,
    STATE_EMERGENCY = 3
} FSM_State_t;

/* SPI Packet Definition (8-Byte Frame) */
typedef struct {
    u8 header;          /* Byte 0: 0xA5 */
    u8 current_floor;   /* Byte 1: where elevator is now (0..3) */
    u8 fsm_state;       /* Byte 2: IPC_FSM_t value (0..4) */
    u8 target_floor;    /* Byte 3: where elevator is going (0..3) */
    u8 motor_speed;     /* Byte 4: 0, 20, 100 (PWM duty %) */
    u8 flags;           /* Byte 5: bit-packed status (door, emg, etc.) */
    u8 reserved;        /* Byte 6: reserved (task payload) */
    u8 checksum;        /* Byte 7: XOR sum of bytes 0..6 */
} SPI_Packet_t;

/* ─────────────────────────────────────────
 * UNIFIED GLOBAL SHARED STATE
 * One struct shared by Elevator.c and Dispatcher.c
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
    volatile u8  comm_fault;        /* 1 = IPC link lost */
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
 * GLOBAL INSTANCE (declare in one .c file only!)
 * ───────────────────────────────────────── */
extern volatile GlobalSharedState_t GSS;

#endif /* SHARED_H */
```

### Then update all references:

**File**: `Core/Src/dispatcher.c`

**CHANGE** line 12 from:
```c
GlobalSharedState_t SystemState;
```

**TO**:
```c
// Use extern GSS defined in Elevator.c
// (remove the local declaration)
```

**CHANGE** all references in dispatcher.c from `SystemState.` to `GSS.`:
- `SystemState.master_state` → `GSS` (master fields)
- `SystemState.slave_state` → `GSS.slave_*` (new slave fields)
- `SystemState.comm_fault` → `GSS.comm_fault`

**File**: `Core/Src/Elevator.c`

**CHANGE** line 10 from:
```c
volatile GlobalSharedState GSS;
```

**TO**:
```c
volatile GlobalSharedState_t GSS;  /* Now uses unified type from shared.h */
```

**File**: `Core/Inc/ipc.h`

**DELETE** line that says:
```c
extern GlobalSharedState_t SystemState;
```

Because we don't need it anymore — `GSS` is defined in Elevator.c and accessible everywhere.

---

## FIX #2: Implement SPI Slave Preload (CRITICAL)

### Problem
The code calls `SPI_SlavePreload()` but it doesn't exist. Slave cannot respond to Master.

### Solution: Add to Spi.c

**File**: `Core/Src/Spi.c`

**ADD** this function after `Spi1_CS_Release()`:

```c
/* ─────────────────────────────────────────
 * SPI_SlavePreload()
 * ─────────────────────────────────────────
 * SLAVE ONLY: Pre-load first byte into SPI DR
 * BEFORE Master initiates the transfer.
 *
 * Non-blocking Slave architecture:
 *   1. Slave calls SPI_SlavePreload() in main loop (or IPC_TransmitFrame)
 *   2. First byte goes into SPI->DR immediately
 *   3. ISR handles remaining bytes via interrupt
 *   4. When Master drives CS low, Slave is READY
 *
 * Input:
 *   pBuf: pointer to 8-byte buffer [0..7]
 * ───────────────────────────────────────── */
void SPI_SlavePreload(SPI_HandleTypeDef *pHandle, volatile u8 *pBuf)
{
    if (!pBuf) return;
    
    /* 1. Preload first byte into DR NOW (no wait for Master clock) */
    SPI1->DR = pBuf[0];
    
    /* 2. Set up ISR to handle remaining 7 bytes when Master clocks them */
    pHandle->pTxBuffer = (volatile u8 *)(&pBuf[1]);  /* Start from byte 1 */
    pHandle->TxCount   = (IPC_PACKET_SIZE - 1u);    /* 7 bytes left */
    pHandle->pRxBuffer = pHandle->RxBuffer;         /* RX buffer for incoming data */
    pHandle->RxCount   = IPC_PACKET_SIZE;           /* All 8 bytes */
    
    /* 3. Ensure SPI RX interrupt is enabled for byte-by-byte handling */
    SET_BIT(SPI1->CR2, SPI_CR2_RXNEIE);
}
```

### Update SPI1 ISR Handler

**File**: `Core/Src/spi_it.c` (or the existing SPI ISR)

**MAKE SURE** the SPI1 IRQ handler implements:
```c
void SPI1_IRQHandler(void)
{
    /* RX Not Empty — byte received */
    if (SPI1->SR & (1 << SPI_SR_RXNE))
    {
        u8 rx_byte = (u8)SPI1->DR;
        
        /* Store in RX buffer */
        if (SPI1_Handle.RxCount > 0)
        {
            *SPI1_Handle.pRxBuffer++ = rx_byte;
            SPI1_Handle.RxCount--;
        }
        
        /* If Master, TX the next byte; if Slave, ISR auto-shifts */
        if (READ_BIT(SPI1->CR1, SPI_CR1_MSTR))
        {
            /* MASTER: Manually feed next TX byte */
            if (SPI1_Handle.TxCount > 0)
            {
                SPI1->DR = *SPI1_Handle.pTxBuffer++;
                SPI1_Handle.TxCount--;
            }
            else if (SPI1_Handle.TxCount == 0 && SPI1_Handle.LastByteFlag == 0)
            {
                /* All bytes sent — pull CS high on next cycle */
                SPI1_Handle.LastByteFlag = 1;
            }
        }
        else
        {
            /* SLAVE: TX ISR auto-shifts next byte from DR
             * Just feed the next byte if TxCount > 0 */
            if (SPI1_Handle.TxCount > 0)
            {
                SPI1->DR = *SPI1_Handle.pTxBuffer++;
                SPI1_Handle.TxCount--;
            }
        }
    }
    
    /* Transfer complete */
    if (SPI1_Handle.TxCount == 0 && SPI1_Handle.RxCount == 0)
    {
        SPI1_Handle.RxComplete = 1;
        CLEAR_BIT(SPI1->CR2, SPI_CR2_RXNEIE);  /* Disable ISR */
    }
}
```

---

## FIX #3: Implement IPC Timeout Detection (CRITICAL)

### Problem
SPI link dies → Master doesn't know → keeps sending to Slave that's dead.

### Solution: Add timeout logic to main loop

**File**: `Core/Src/main.c`

**IN THE MAIN LOOP** (around line 227), add before `Dispatcher_Update()`:

```c
/* ────────────────────────────────────────────────
 * COMM FAULT DETECTION (150ms timeout)
 * ────────────────────────────────────────────────
 * If SPI hasn't received a valid frame in 150ms,
 * declare comm fault and Master takes all calls.
 */
{
    u32 time_since_last_valid = IPC_Handle.SysTickMs - IPC_Handle.LastValidRxTick;
    
    if (time_since_last_valid > IPC_TIMEOUT_MS)
    {
        u32 pm = Enter_Critical();
        IPC_Handle.CommFault = 1;
        GSS.comm_fault = 1;  /* Propagate to Dispatcher */
        Exit_Critical(pm);
    }
    else
    {
        /* Link is OK */
        u32 pm = Enter_Critical();
        IPC_Handle.CommFault = 0;
        GSS.comm_fault = 0;
        Exit_Critical(pm);
    }
}
```

### Also update SysTick handler

**ADD TO SysTick Handler** (increment tick counter):

```c
/* In SysTick_Handler or wherever you handle 50ms tick */
void SysTick_Handler(void)
{
    IPC_Handle.SysTickMs += 50;  /* Increment by 50ms per tick */
    
    /* Trigger IPC exchange */
    IPC_Update();  /* Call your IPC update function */
}
```

### Add to IPC.c after successful RX

**File**: `Core/Src/ipc.c`

**AFTER** packet validation succeeds, add:

```c
/* Inside IPC_CheckConsistency() or after RxComplete flag */
if (/* packet is valid */) {
    u32 pm = Enter_Critical();
    IPC_Handle.LastValidRxTick = IPC_Handle.SysTickMs;
    IPC_Handle.CommFault = 0;
    Exit_Critical(pm);
}
```

---

## FIX #4: Fix PWM Critical Section Bug

### Problem
Between `Exit_Critical()` and `PWM_SetDuty()`, ISR can modify `GSS.speed`, causing glitch.

### Solution: Snapshot inside critical section

**File**: `Core/Src/Elevator.c`

**FIND** all instances of this pattern (lines ~156-164):
```c
primask = Enter_Critical();
GSS.speed = PWM_DUTY_FULL;
Exit_Critical(primask);
PWM_SetDuty(GSS.speed);  // ← WRONG!
```

**REPLACE WITH**:
```c
primask = Enter_Critical();
GSS.speed = PWM_DUTY_FULL;
u8 speed_to_set = GSS.speed;  /* Snapshot */
Exit_Critical(primask);
PWM_SetDuty(speed_to_set);  /* Use snapshot, not volatile */
```

**DO THIS FOR ALL OCCURRENCES** (search for `PWM_SetDuty` in Elevator.c).

---

## FIX #5: Document IPC FSM State Mapping

### Problem
Dispatcher assumes IPC byte 2 encodes FSM state, but mapping is undocumented.

### Solution: Add clear enum to ipc.h

**File**: `Core/Inc/ipc.h`

**ADD** after the existing `ElevatorState_t` enum (around line 48):

```c
/* ─────────────────────────────────────────
 * IPC FSM STATE VALUES (Byte 2 of SPI packet)
 * Sent by both Master and Slave.
 * Dispatcher uses these to score elevator assignments.
 * ───────────────────────────────────────── */
typedef enum {
    IPC_FSM_IDLE         = 0x00,
    IPC_FSM_MOVING_UP    = 0x01,
    IPC_FSM_MOVING_DOWN  = 0x02,
    IPC_FSM_DOORS_OPEN   = 0x03,
    IPC_FSM_EMERGENCY    = 0x04
} IPC_FSM_State_t;

/* Ensure these match ElevatorState_t enum */
_Static_assert(ELV_IDLE == IPC_FSM_IDLE, "FSM enum mismatch");
_Static_assert(ELV_MOVING_UP == IPC_FSM_MOVING_UP, "FSM enum mismatch");
_Static_assert(ELV_MOVING_DOWN == IPC_FSM_MOVING_DOWN, "FSM enum mismatch");
_Static_assert(ELV_DOORS_OPEN == IPC_FSM_DOORS_OPEN, "FSM enum mismatch");
_Static_assert(ELV_EMERGENCY == IPC_FSM_EMERGENCY, "FSM enum mismatch");
```

**THEN in dispatcher.c**, replace the switch statement with validation:

```c
/* Convert IPC fsm_state to Dispatcher FSM_State_t */
if (SystemState.master_state.fsm_state > IPC_FSM_EMERGENCY) {
    /* Invalid state — treat as IDLE for safety */
    elvA_state = STATE_IDLE;
} else {
    /* Direct mapping (they use same numeric values) */
    elvA_state = (FSM_State_t)SystemState.master_state.fsm_state;
}
```

---

## FIX #6: Verify System_Logger() Implementation

### Problem
Telemetry system called but full implementation not visible.

### Solution: Add complete function to uart_dma.c

**File**: `Core/Src/uart_dma.c`

**ADD** at the end:

```c
/* ─────────────────────────────────────────
 * System_Logger()
 * ─────────────────────────────────────────
 * Called every 500ms by TIM6 interrupt.
 * Formats telemetry and initiates DMA transfer (non-blocking).
 *
 * Format: "ELV|FL:x|ST:x|SP:x|DIR:x|EM:x|CF:x\r\n"
 * Where:
 *   FL = Floor (0-3)
 *   ST = State (0-4)
 *   SP = Speed (0, 20, 100)
 *   DIR = Direction (0, 1, 2)
 *   EM = Emergency (0, 1)
 *   CF = Comm Fault (0, 1)
 * ───────────────────────────────────────── */
void System_Logger(void)
{
    /* 1. Snapshot all volatile state atomically */
    u32 pm = Enter_Critical();
    u8 pos = GSS.position;
    u8 state = GSS.fsm_state;
    u8 speed = GSS.speed;
    u8 dir = GSS.direction;
    u8 emg = GSS.emergency;
    u8 comm = GSS.comm_fault;
    Exit_Critical(pm);
    
    /* 2. Format into UART_TxBuf */
    u32 len = 0;
    len = sprintf((char *)UART_TxBuf,
                  "ELV|FL:%u|ST:%u|SP:%u|DIR:%u|EM:%u|CF:%u\r\n",
                  pos, state, speed, dir, emg, comm);
    
    if (len == 0 || len > UART_TX_BUF_SIZE) {
        return;  /* Error: buffer overflow */
    }
    
    /* 3. Initiate DMA transfer (non-blocking) */
    UART_DMA_Send(UART_TxBuf, len);
    
    /* 4. Clear the telemetry flag for next 500ms cycle */
    GSS.telem_flag = 0;
}
```

---

## FIX #7: Add RX Buffer Critical Section

### Problem
Main loop reads IPC RX buffer without protection from ISR.

### Solution: Wrap decoding in critical section

**File**: `Core/Src/main.c`

**FIND** where `SPI1_Handle.RxComplete` is checked (after SPI ISR sets it).

**REPLACE THIS**:
```c
if (SPI1_Handle.RxComplete) {
    IPC_DecodeFrame(IPC_Handle.RxRawBuf, &IPC_Handle.RxFrame);
    SystemState.slave_state = IPC_Handle.RxFrame;
}
```

**WITH THIS**:
```c
if (SPI1_Handle.RxComplete) {
    u32 pm = Enter_Critical();
    
    /* Decode frame atomically */
    if (IPC_CheckConsistency(IPC_Handle.RxRawBuf)) {
        IPC_DecodeFrame(IPC_Handle.RxRawBuf, &IPC_Handle.RxFrame);
        
        /* Update slave state and timeout tracker */
        GSS.slave_position = IPC_Handle.RxFrame.current_floor;
        GSS.slave_fsm_state = IPC_Handle.RxFrame.fsm_state;
        GSS.slave_target = IPC_Handle.RxFrame.target_floor;
        GSS.slave_speed = IPC_Handle.RxFrame.motor_speed;
        GSS.slave_flags = IPC_Handle.RxFrame.flags;
        
        IPC_Handle.LastValidRxTick = IPC_Handle.SysTickMs;
        IPC_Handle.CommFault = 0;
    }
    
    SPI1_Handle.RxComplete = 0;  /* Reset for next transfer */
    Exit_Critical(pm);
}
```

---

## TESTING CHECKLIST AFTER FIXES

- [ ] **Compile**: `cmake --build build/` — no errors
- [ ] **Master board boots** — TIM6 ticking (500ms telemetry)
- [ ] **Slave board boots** — pre-load loaded into SPI DR
- [ ] **SPI link established** — logic analyzer shows 8-byte packets at 4MHz
- [ ] **Checksum valid** — sample packet, compute XOR manually, verify byte 7
- [ ] **Timeout triggers** — unplug SPI, wait 150ms, verify `GSS.comm_fault = 1`
- [ ] **E-Stop works** — press button, motor stops immediately
- [ ] **Telemetry output** — serial monitor shows "ELV|FL:0|ST:0|..." every 500ms
- [ ] **PWM ramps** — LED smooth 100%→20% 1 floor before target
- [ ] **Dispatcher assigns tasks** — press hallway button, call assigned to correct elevator

---

**Total Effort**: ~2-3 hours to implement all fixes.
**Expected Result After Fixes**: 95+/100 score.
