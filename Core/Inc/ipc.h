#ifndef IPC_H
#define IPC_H

#include "std_types.h"
#include "spi.h"

/* ─────────────────────────────────────────
 * PACKET CONSTANTS
 * ───────────────────────────────────────── */
#define IPC_HEADER          0xA5u   /* Fixed header byte — every valid frame starts with this */
#define IPC_PACKET_SIZE     8u      /* Total frame size in bytes                              */
#define IPC_TIMEOUT_MS      150u    /* If no valid frame in 150ms → comm fault                */

/* ─────────────────────────────────────────
 * 8-BYTE FRAME LAYOUT
 * ─────────────────────────────────────────
 *  Byte 0 → Header      (0xA5)
 *  Byte 1 → Current floor (0=F1, 1=F2, 2=F3, 3=F4)
 *  Byte 2 → FSM State
 *  Byte 3 → Target floor
 *  Byte 4 → Motor speed  (0, 20, 100 → maps to 0%, 20%, 100% PWM)
 *  Byte 5 → Flags        (bit-packed status)
 *  Byte 6 → Reserved     (always 0x00)
 *  Byte 7 → Checksum     (XOR of bytes 0 through 6)
 * ───────────────────────────────────────── */

/* ─────────────────────────────────────────
 * FSM STATES
 * Shared between Master and Slave so both
 * sides speak the same language over SPI
 * ───────────────────────────────────────── */
typedef enum {
    ELV_IDLE         = 0x00,
    ELV_MOVING_UP    = 0x01,
    ELV_MOVING_DOWN  = 0x02,
    ELV_DOORS_OPEN   = 0x03,
    ELV_EMERGENCY    = 0x04
} ElevatorState_t;

/* ─────────────────────────────────────────
 * FLAGS BYTE (Byte 5) BIT DEFINITIONS
 * ───────────────────────────────────────── */
#define IPC_FLAG_EMERGENCY  (1u << 0)   /* bit 0: emergency stop active      */
#define IPC_FLAG_DOOR_OPEN  (1u << 1)   /* bit 1: doors currently open       */
#define IPC_FLAG_MOVING_UP  (1u << 2)   /* bit 2: direction is up            */
#define IPC_FLAG_MOVING_DN  (1u << 3)   /* bit 3: direction is down          */
#define IPC_FLAG_TASK_ACK   (1u << 4)   /* bit 4: slave acknowledges task    */
#define IPC_FLAG_COMM_FAULT (1u << 5)   /* bit 5: communication fault active */

/* ─────────────────────────────────────────
 * IPC FRAME STRUCT
 * This is the structured view of one 8-byte
 * packet. Encode packs it into raw bytes.
 * Decode unpacks raw bytes back into this.
 * ───────────────────────────────────────── */
typedef struct {
    u8 header;          /* Byte 0: always 0xA5                */
    u8 current_floor;   /* Byte 1: where elevator is now      */
    u8 fsm_state;       /* Byte 2: ElevatorState_t value      */
    u8 target_floor;    /* Byte 3: where elevator is going    */
    u8 motor_speed;     /* Byte 4: 0, 20, or 100              */
    u8 flags;           /* Byte 5: bit-packed flags           */
    u8 reserved;        /* Byte 6: always 0x00                */
    u8 checksum;        /* Byte 7: XOR of bytes 0-6           */
} IPC_Frame_t;

/* ─────────────────────────────────────────
 * IPC HANDLE
 * Holds TX frame, RX frame, raw byte buffers
 * and comm-fault tracking for one MCU
 * ───────────────────────────────────────── */
typedef struct {
    IPC_Frame_t     TxFrame;                    /* Frame we are sending          */
    IPC_Frame_t     RxFrame;                    /* Last valid frame received      */

    volatile u8     TxRawBuf[IPC_PACKET_SIZE];  /* Raw bytes fed to SPI driver   */
    volatile u8     RxRawBuf[IPC_PACKET_SIZE];  /* Raw bytes received from SPI   */

    volatile u8     CommFault;                  /* 1 = SPI link lost             */
    volatile u32    LastValidRxTick;            /* Tick of last good RX          */
    volatile u32    SysTickMs;                  /* Running ms counter (SysTick)  */
} IPC_Handle_t;

/* ─────────────────────────────────────────
 * GLOBAL IPC HANDLE
 * ───────────────────────────────────────── */
extern IPC_Handle_t IPC_Handle;

/* ─────────────────────────────────────────
 * FUNCTION PROTOTYPES — YOUR 3 DELIVERABLES
 * ───────────────────────────────────────── */

/*
 * IPC_Init()
 * ──────────
 * Initializes the full IPC layer:
 *   - Calls SPI_MasterInit or SPI_SlaveInit
 *   - Clears TX/RX buffers
 *   - On Slave: preloads DR with a default status frame
 *   - Resets comm fault state
 *
 * Parameters:
 *   isMaster → 1 = this MCU is Master, 0 = Slave
 */
void IPC_Init(u8 isMaster);

/*
 * IPC_TransmitFrame()
 * ────────────────────
 * Builds a complete 8-byte packet from the
 * provided frame data, calculates checksum,
 * and starts a non-blocking SPI transfer.
 *
 * Master: pulls CS low then triggers transfer
 * Slave:  updates TX buffer + preloads DR
 *         for the next Master-initiated transfer
 *
 * Parameters:
 *   pFrame → pointer to filled IPC_Frame_t
 *            (header and checksum auto-filled)
 */
void IPC_TransmitFrame(IPC_Frame_t *pFrame);

/*
 * IPC_CheckConsistency()
 * ───────────────────────
 * Validates the last received raw packet:
 *   1. Checks header byte == 0xA5
 *   2. Recalculates XOR checksum of bytes 0-6
 *      and compares to byte 7
 *   3. If valid: decodes into IPC_Handle.RxFrame
 *                updates LastValidRxTick
 *                clears CommFault
 *   4. If invalid: increments fault counter
 *                  sets CommFault if timeout exceeded
 *
 * Returns:
 *   1 → frame is valid and decoded into RxFrame
 *   0 → frame is corrupt or comm fault detected
 */
u8 IPC_CheckConsistency(void);

/* ─────────────────────────────────────────
 * HELPER PROTOTYPES (used internally)
 * ───────────────────────────────────────── */

/* Pack IPC_Frame_t into 8 raw bytes + compute checksum */
void IPC_EncodeFrame(IPC_Frame_t *pFrame, volatile u8 *pBuf);

/* Unpack 8 raw bytes into IPC_Frame_t */
void IPC_DecodeFrame(volatile u8 *pBuf, IPC_Frame_t *pFrame);

/* Compute XOR checksum of bytes 0 through 6 */
u8 IPC_ComputeChecksum(volatile u8 *pBuf);

/* Call every 50ms from SysTick to drive the IPC exchange */
void IPC_Update(void);

#endif /* IPC_H */
