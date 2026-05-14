#include "../Inc/ipc.h"
#include "../Inc/Bit_Math.h"
#include "../Inc/shared.h"      /* GlobalSharedState_t GSS — single unified struct */
#include "../Inc/std_types.h"
#include "../Inc/spi_it.h"
#include "../Inc/Elevator.h"   /* Enter_Critical / Exit_Critical */

/* ─────────────────────────────────────────
 * GLOBAL IPC HANDLE INSTANCE
 * ───────────────────────────────────────── */
IPC_Handle_t IPC_Handle;

/* ─────────────────────────────────────────
 * IPC_ComputeChecksum()
 * XOR bytes 0 through 6 → result is byte 7.
 * Detects any single-bit and any odd-number
 * of bit errors. Fast, no division needed.
 * ───────────────────────────────────────── */
u8 IPC_ComputeChecksum(volatile u8 *pBuf)
{
    u8 checksum = 0u;
    u8 i;
    for (i = 0u; i < (IPC_PACKET_SIZE - 1u); i++)
    {
        checksum ^= pBuf[i];
    }
    return checksum;
}

/* ─────────────────────────────────────────
 * IPC_EncodeFrame()
 * Packs IPC_Frame_t → raw 8-byte buffer.
 * Forces header=0xA5, computes checksum.
 * Entire write done inside critical section
 * to prevent ISR/DMA partial-read corruption.
 * ───────────────────────────────────────── */
void IPC_EncodeFrame(IPC_Frame_t *pFrame, volatile u8 *pBuf)
{
    pFrame->header   = IPC_HEADER;
    pFrame->reserved = 0x00u;

    u32 _pm = Enter_Critical();

    pBuf[0] = pFrame->header;
    pBuf[1] = pFrame->current_floor;
    pBuf[2] = pFrame->fsm_state;
    pBuf[3] = pFrame->target_floor;
    pBuf[4] = pFrame->motor_speed;
    pBuf[5] = pFrame->flags;
    pBuf[6] = pFrame->reserved;
    pBuf[7] = IPC_ComputeChecksum(pBuf);

    pFrame->checksum = pBuf[7];

    Exit_Critical(_pm);
}

/* ─────────────────────────────────────────
 * IPC_DecodeFrame()
 * Unpacks validated raw bytes → IPC_Frame_t.
 * Only call AFTER IPC_CheckConsistency() == 1.
 * ───────────────────────────────────────── */
void IPC_DecodeFrame(volatile u8 *pBuf, IPC_Frame_t *pFrame)
{
    pFrame->header        = pBuf[0];
    pFrame->current_floor = pBuf[1];
    pFrame->fsm_state     = pBuf[2];
    pFrame->target_floor  = pBuf[3];
    pFrame->motor_speed   = pBuf[4];
    pFrame->flags         = pBuf[5];
    pFrame->reserved      = pBuf[6];
    pFrame->checksum      = pBuf[7];
}

/* ─────────────────────────────────────────
 * IPC_Init()
 * Call once in main() before the main loop.
 * isMaster: 1 = Master, 0 = Slave
 * ───────────────────────────────────────── */
void IPC_Init(u8 isMaster)
{
    u8 i;

    for (i = 0u; i < IPC_PACKET_SIZE; i++)
    {
        IPC_Handle.TxRawBuf[i] = 0x00u;
        IPC_Handle.RxRawBuf[i] = 0x00u;
    }

    IPC_Handle.CommFault       = 0u;
    IPC_Handle.LastValidRxTick = 0u;
    IPC_Handle.SysTickMs       = 0u;

    IPC_Handle.TxFrame.header        = IPC_HEADER;
    IPC_Handle.TxFrame.current_floor = 0u;
    IPC_Handle.TxFrame.fsm_state     = (u8)ELV_IDLE;
    IPC_Handle.TxFrame.target_floor  = 0u;
    IPC_Handle.TxFrame.motor_speed   = 0u;
    IPC_Handle.TxFrame.flags         = 0u;
    IPC_Handle.TxFrame.reserved      = 0x00u;
    IPC_Handle.TxFrame.checksum      = 0u;

    SPI1_Handle.Instance = SPI1;

    if (isMaster)
    {
        SPI_MasterInit(&SPI1_Handle);
    }
    else
    {
        SPI_SlaveInit(&SPI1_Handle);

        /* Slave MUST preload DR before Master drives CS low.
         * Build a default "IDLE at floor 0" frame and
         * load byte 0 into DR immediately.               */
        IPC_EncodeFrame(&IPC_Handle.TxFrame, IPC_Handle.TxRawBuf);
        SPI_SlavePreload(&SPI1_Handle, IPC_Handle.TxRawBuf);
    }
}

/* ─────────────────────────────────────────
 * IPC_TransmitFrame()
 * Master: encode → CS low → start IT transfer.
 * Slave:  encode → preload DR for next Master clk.
 * ───────────────────────────────────────── */
void IPC_TransmitFrame(IPC_Frame_t *pFrame)
{
    IPC_EncodeFrame(pFrame, IPC_Handle.TxRawBuf);

    if (READ_BIT(SPI1->CR1, SPI_CR1_MSTR))
    {
        /* ── MASTER PATH ── */
        if (SPI1_Handle.State != SPI_STATE_READY) return;

        SPI_CS_Enable();
        SPI_TransmitReceive_IT(&SPI1_Handle,
                                IPC_Handle.TxRawBuf,
                                IPC_Handle.RxRawBuf,
                                IPC_PACKET_SIZE);
        /* CS released by ISR after last byte */
    }
    else
    {
        /* ── SLAVE PATH ── */
        /* Re-preload DR with fresh byte 0; ISR handles remaining 7 */
        SPI_SlavePreload(&SPI1_Handle, IPC_Handle.TxRawBuf);

        SPI1_Handle.pRxBuffer = IPC_Handle.RxRawBuf;
        SPI1_Handle.RxCount   = IPC_PACKET_SIZE;
    }
}

/* ─────────────────────────────────────────
 * IPC_CheckConsistency()
 * Validates and decodes the last received
 * raw packet.
 *
 * Returns 1 → valid; RxFrame populated.
 * Returns 0 → corrupt or comm fault.
 *
 * FIX: Entire RxRawBuf read + decode wrapped
 * in a critical section so the SPI ISR cannot
 * overwrite the buffer while we are reading it.
 * ───────────────────────────────────────── */
u8 IPC_CheckConsistency(void)
{
    u8 snap[IPC_PACKET_SIZE];
    u8 i;
    u8 computed_checksum;

    /* ── FIX: snapshot RxRawBuf atomically ──
     * The SPI ISR can write to RxRawBuf at any time.
     * Copy all 8 bytes inside a critical section so we
     * never read a mix of bytes from two different frames. */
    {
        u32 pm = Enter_Critical();
        for (i = 0u; i < IPC_PACKET_SIZE; i++)
        {
            snap[i] = IPC_Handle.RxRawBuf[i];
        }
        Exit_Critical(pm);
    }

    /* Check 1: Header byte must be 0xA5 */
    if (snap[0] != IPC_HEADER)
    {
        IPC_Handle.CommFault = 1u;
        return 0u;
    }

    /* Check 2: Recompute XOR checksum */
    computed_checksum = 0u;
    for (i = 0u; i < (IPC_PACKET_SIZE - 1u); i++)
    {
        computed_checksum ^= snap[i];
    }
    if (computed_checksum != snap[IPC_PACKET_SIZE - 1u])
    {
        IPC_Handle.CommFault = 1u;
        return 0u;
    }

    /* Check 3: Field range validation */
    if (snap[1] > 3u)                        /* current_floor: 0–3 */
    {
        IPC_Handle.CommFault = 1u;
        return 0u;
    }
    if (snap[2] > (u8)ELV_EMERGENCY)         /* fsm_state: 0–4 */
    {
        IPC_Handle.CommFault = 1u;
        return 0u;
    }
    if (snap[4] != 0u && snap[4] != 20u && snap[4] != 99u)  /* motor_speed */
    {
        IPC_Handle.CommFault = 1u;
        return 0u;
    }

    /* ── All checks passed: decode into RxFrame ── */
    IPC_Handle.RxFrame.header        = snap[0];
    IPC_Handle.RxFrame.current_floor = snap[1];
    IPC_Handle.RxFrame.fsm_state     = snap[2];
    IPC_Handle.RxFrame.target_floor  = snap[3];
    IPC_Handle.RxFrame.motor_speed   = snap[4];
    IPC_Handle.RxFrame.flags         = snap[5];
    IPC_Handle.RxFrame.reserved      = snap[6];
    IPC_Handle.RxFrame.checksum      = snap[7];

    IPC_Handle.CommFault       = 0u;
    IPC_Handle.LastValidRxTick = IPC_Handle.SysTickMs;

    return 1u;
}

/* ─────────────────────────────────────────
 * IPC_Update()
 * Call every 50ms from SysTick_Handler.
 *
 * Drives the full IPC cycle:
 *   1. Increment SysTickMs counter
 *   2. Detect comm fault via timeout
 *   3. If transfer complete: validate + decode
 *   4. On Master: copy Slave state into GSS
 *   5. On Slave:  invoke IPC_OnMasterFrameReceived
 *   6. Trigger next transfer
 *
 * FIX: Slave state is now written to GSS.slave_*
 * fields (unified struct) instead of the old
 * SystemState.slave_state (which was a separate
 * disconnected struct causing data never to sync).
 * ───────────────────────────────────────── */
void IPC_Update(void)
{
    IPC_Handle.SysTickMs += 50u;

    /* ── Comm fault detection (150ms = 3 missed cycles) ── */
    if ((IPC_Handle.SysTickMs - IPC_Handle.LastValidRxTick) >= IPC_TIMEOUT_MS)
    {
        IPC_Handle.CommFault = 1u;
        IPC_Handle.TxFrame.flags |= IPC_FLAG_COMM_FAULT;

        /* Propagate to GSS so Dispatcher sees the fault */
        u32 pm = Enter_Critical();
        GSS.comm_fault = 1u;
        Exit_Critical(pm);
    }

    /* ── Process completed SPI transfer ── */
    if (SPI1_Handle.RxComplete)
    {
        SPI1_Handle.RxComplete = 0u;

        if (IPC_CheckConsistency())  /* Internally atomic — see above */
        {
            if (READ_BIT(SPI1->CR1, SPI_CR1_MSTR))
            {
                /* ── MASTER: copy Slave's state into GSS.slave_* fields ──
                 *
                 * FIX (Critical): The old code wrote to SystemState.slave_state.*
                 * which was a SEPARATE struct from GSS. Dispatcher reads GSS.
                 * This meant Dispatcher NEVER saw the Slave's actual state.
                 * Now we write directly into the unified GSS.slave_* fields
                 * so Dispatcher.CalculateScore() uses real Slave data.
                 */
                u32 pm = Enter_Critical();
                GSS.slave_position  = IPC_Handle.RxFrame.current_floor;
                GSS.slave_fsm_state = IPC_Handle.RxFrame.fsm_state;
                GSS.slave_target    = IPC_Handle.RxFrame.target_floor;
                GSS.slave_speed     = IPC_Handle.RxFrame.motor_speed;
                GSS.slave_flags     = IPC_Handle.RxFrame.flags;
                GSS.comm_fault      = 0u;  /* Valid frame received → link OK */
                /* Also store last packet for reference */
                GSS.last_rx_packet  = *(SPI_Packet_t*)(void*)&IPC_Handle.RxFrame;
                GSS.last_valid_rx_tick = IPC_Handle.SysTickMs;
                Exit_Critical(pm);
            }
            else
            {
                /* ── SLAVE: pass Master's frame to Elevator FSM ── */
                IPC_OnMasterFrameReceived(&IPC_Handle.RxFrame);
            }
        }
    }

    /* ── Trigger next 50ms transfer ── */
    IPC_TransmitFrame(&IPC_Handle.TxFrame);
}
