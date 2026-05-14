#include "ipc.h"
#include "Bit_Math.h"
#include "shared.h" /* for SystemState */
#include "stdint.h"
#include "spi_it.h"

/* ─────────────────────────────────────────
 * GLOBAL INSTANCES
 * ───────────────────────────────────────── */
IPC_Handle_t IPC_Handle;

/* ─────────────────────────────────────────
 * IPC_ComputeChecksum()
 * ─────────────────────────────────────────
 * XOR bytes 0 through 6 together.
 * Result goes into byte 7.
 *
 * XOR checksum properties:
 *   - Simple and fast (no division, no overflow)
 *   - Detects any single-bit error
 *   - Detects any odd number of bit errors
 *   - Perfect for 8-byte embedded packets
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
 * ─────────────────────────────────────────
 * Packs an IPC_Frame_t struct into the
 * raw 8-byte buffer that SPI will transmit.
 * Also forces header = 0xA5 and computes
 * the checksum automatically.
 * ───────────────────────────────────────── */
void IPC_EncodeFrame(IPC_Frame_t *pFrame, volatile u8 *pBuf)
{
    /* Always force the correct header */
    pFrame->header   = IPC_HEADER;
    pFrame->reserved = 0x00u;

    /* Pack struct fields into raw byte buffer inside a critical section
     * to avoid concurrent ISR/DMA reads of the raw buffer while we write it. */
    {
        u32 _pm = Enter_Critical();

        pBuf[0] = pFrame->header;
        pBuf[1] = pFrame->current_floor;
        pBuf[2] = pFrame->fsm_state;
        pBuf[3] = pFrame->target_floor;
        pBuf[4] = pFrame->motor_speed;
        pBuf[5] = pFrame->flags;
        pBuf[6] = pFrame->reserved;

        /* Compute and store checksum as byte 7 */
        pBuf[7] = IPC_ComputeChecksum(pBuf);

        /* Also store checksum back in struct for reference */
        pFrame->checksum = pBuf[7];

        Exit_Critical(_pm);
    }
}

/* ─────────────────────────────────────────
 * IPC_DecodeFrame()
 * ─────────────────────────────────────────
 * Unpacks a validated raw byte buffer back
 * into a human-readable IPC_Frame_t struct.
 * Only call this AFTER CheckConsistency passes.
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
 * ─────────────────────────────────────────
 * Initializes the full IPC layer.
 * Call this once in main() before the
 * main loop starts.
 * ───────────────────────────────────────── */
void IPC_Init(u8 isMaster)
{
    u8 i;

    /* 1. Zero out all buffers and state */
    for (i = 0u; i < IPC_PACKET_SIZE; i++)
    {
        IPC_Handle.TxRawBuf[i] = 0x00u;
        IPC_Handle.RxRawBuf[i] = 0x00u;
    }

    IPC_Handle.CommFault       = 0u;
    IPC_Handle.LastValidRxTick = 0u;
    IPC_Handle.SysTickMs       = 0u;

    /* 2. Clear frame structs */
    IPC_Handle.TxFrame.header        = IPC_HEADER;
    IPC_Handle.TxFrame.current_floor = 0u;
    IPC_Handle.TxFrame.fsm_state     = (u8)ELV_IDLE;
    IPC_Handle.TxFrame.target_floor  = 0u;
    IPC_Handle.TxFrame.motor_speed   = 0u;
    IPC_Handle.TxFrame.flags         = 0u;
    IPC_Handle.TxFrame.reserved      = 0x00u;
    IPC_Handle.TxFrame.checksum      = 0u;

    /* 3. Point SPI1 handle to SPI1 registers */
    SPI1_Handle.Instance = SPI1;

    /* 4. Init SPI in correct role */
    if (isMaster)
    {
        SPI_MasterInit(&SPI1_Handle);
    }
    else
    {
        SPI_SlaveInit(&SPI1_Handle);

        /* 5. Slave MUST preload DR before Master can start
         *    Build a default "I am idle at floor 1" frame
         *    and load its first byte into the SPI DR now    */
        IPC_EncodeFrame(&IPC_Handle.TxFrame, IPC_Handle.TxRawBuf);
        SPI_SlavePreload(&SPI1_Handle, IPC_Handle.TxRawBuf);
    }
}

/* ─────────────────────────────────────────
 * IPC_TransmitFrame()
 * ─────────────────────────────────────────
 * Encodes the frame into raw bytes and
 * starts the SPI transfer.
 *
 * Master flow:
 *   encode → pull CS low → start IT transfer
 *   ISR handles byte-by-byte TX and pulls CS high when done
 *
 * Slave flow:
 *   encode → update TX buffer → preload DR
 *   Master will clock the data out on its next transfer
 * ───────────────────────────────────────── */
void IPC_TransmitFrame(IPC_Frame_t *pFrame)
{
    /* Step 1: Encode struct into raw bytes + compute checksum */
    IPC_EncodeFrame(pFrame, IPC_Handle.TxRawBuf);

    /* Step 2: Check which role this MCU is */
    if (READ_BIT(SPI1->CR1, SPI_CR1_MSTR))
    {
        /* ── MASTER PATH ── */
        /* Guard: only transmit if SPI is ready */
        if (SPI1_Handle.State != SPI_STATE_READY) return;

        /* Pull CS low — this tells the Slave a transfer is starting */
        SPI_CS_Enable();

        /* Start non-blocking full-duplex transfer
         * TX: our status frame
         * RX: Slave's status frame (received simultaneously) */
        SPI_TransmitReceive_IT(&SPI1_Handle,
                                IPC_Handle.TxRawBuf,
                                IPC_Handle.RxRawBuf,
                                IPC_PACKET_SIZE);
        /* CS is pulled high by the ISR after last byte */
    }
    else
    {
        /* ── SLAVE PATH ── */
        /* Update the TX buffer with fresh data
         * Then preload the first byte into DR so it's
         * ready BEFORE the Master drives CS low         */
        SPI_SlavePreload(&SPI1_Handle, IPC_Handle.TxRawBuf);

        /* Also set up the RX buffer for the incoming Master frame */
        SPI1_Handle.pRxBuffer = IPC_Handle.RxRawBuf;
        SPI1_Handle.RxCount   = IPC_PACKET_SIZE;
    }
}

/* ─────────────────────────────────────────
 * IPC_CheckConsistency()
 * ─────────────────────────────────────────
 * Call this after SPI1_Handle.RxComplete == 1
 * to validate and decode the received packet.
 *
 * Returns 1 → valid frame decoded into RxFrame
 * Returns 0 → invalid frame or comm fault
 * ───────────────────────────────────────── */
u8 IPC_CheckConsistency(void)
{
    u8 computed_checksum;
    u8 received_checksum;

    /* ── Check 1: Header byte must be 0xA5 ── */
    if (IPC_Handle.RxRawBuf[0] != IPC_HEADER)
    {
        /* Header mismatch — packet is garbage or out of sync */
        IPC_Handle.CommFault = 1u;
        return 0u;
    }

    /* ── Check 2: Recompute XOR checksum and compare ── */
    computed_checksum = IPC_ComputeChecksum(IPC_Handle.RxRawBuf);
    received_checksum = IPC_Handle.RxRawBuf[IPC_PACKET_SIZE - 1u];

    if (computed_checksum != received_checksum)
    {
        /* Checksum mismatch — data was corrupted in transit */
        IPC_Handle.CommFault = 1u;
        return 0u;
    }

    /* ── Check 3: Sanity check field ranges ── */
    /* Floor must be 0–3 (4 floors) */
    if (IPC_Handle.RxRawBuf[1] > 3u)
    {
        IPC_Handle.CommFault = 1u;
        return 0u;
    }
    /* FSM state must be a known value */
    if (IPC_Handle.RxRawBuf[2] > (u8)ELV_EMERGENCY)
    {
        IPC_Handle.CommFault = 1u;
        return 0u;
    }
    /* Motor speed must be 0, 20, or 100 */
    if (IPC_Handle.RxRawBuf[4] != 0u   &&
        IPC_Handle.RxRawBuf[4] != 20u  &&
        IPC_Handle.RxRawBuf[4] != 100u)
    {
        IPC_Handle.CommFault = 1u;
        return 0u;
    }

    /* ── All checks passed ── */
    /* Decode raw bytes into the RxFrame struct */
    IPC_DecodeFrame(IPC_Handle.RxRawBuf, &IPC_Handle.RxFrame);

    /* Update comm fault tracking */
    IPC_Handle.CommFault       = 0u;
    IPC_Handle.LastValidRxTick = IPC_Handle.SysTickMs;

    return 1u;
}

/* ─────────────────────────────────────────
 * IPC_Update()
 * ─────────────────────────────────────────
 * Call this every 50ms from SysTick handler
 * or a timer ISR.
 *
 * Drives the full IPC cycle:
 *   1. Check if last RX was valid
 *   2. Detect comm fault (timeout)
 *   3. On Master: trigger next TX
 *   4. On Slave:  refresh preloaded TX data
 * ───────────────────────────────────────── */
void IPC_Update(void)
{
    /* Increment our ms tick counter */
    IPC_Handle.SysTickMs += 50u;

    /* ── Comm fault detection ──
     * If we haven't received a valid frame within
     * IPC_TIMEOUT_MS (150ms = 3 missed cycles),
     * declare a communication fault              */
    if ((IPC_Handle.SysTickMs - IPC_Handle.LastValidRxTick) >= IPC_TIMEOUT_MS)
    {
        IPC_Handle.CommFault = 1u;
        /* Set the fault flag in our TX frame so the other side knows */
        IPC_Handle.TxFrame.flags |= IPC_FLAG_COMM_FAULT;
    }

    /* ── Check if last transfer completed ── */
    if (SPI1_Handle.RxComplete)
    {
        /* Reset the flag */
        SPI1_Handle.RxComplete = 0u;

        /* Validate received packet */
        if (IPC_CheckConsistency())
        {
            /* On master: copy received slave status into SystemState.slave_state */
            if (READ_BIT(SPI1->CR1, SPI_CR1_MSTR))
            {
                SystemState.slave_state.header       = IPC_Handle.RxFrame.header;
                SystemState.slave_state.current_floor= IPC_Handle.RxFrame.current_floor;
                SystemState.slave_state.fsm_state    = IPC_Handle.RxFrame.fsm_state;
                SystemState.slave_state.target_floor = IPC_Handle.RxFrame.target_floor;
                SystemState.slave_state.motor_speed  = IPC_Handle.RxFrame.motor_speed;
                SystemState.slave_state.flags        = IPC_Handle.RxFrame.flags;
                SystemState.slave_state.reserved     = IPC_Handle.RxFrame.reserved;
                SystemState.slave_state.checksum     = IPC_Handle.RxFrame.checksum;
            }
            else
            {
                /* On slave: hand received master frame to a callback
                 * implemented in Elevator.c (IPC_OnMasterFrameReceived)
                 * to apply the master's command into local GSS. */
                IPC_OnMasterFrameReceived(&IPC_Handle.RxFrame);
            }
        }
    }

    /* ── On Master: start next 50ms transfer ── */
    if (READ_BIT(SPI1->CR1, SPI_CR1_MSTR))
    {
        /* Update TX frame with current elevator state
         * (FSM layer will have updated IPC_Handle.TxFrame
         *  before this point in the scheduling cycle)      */
        IPC_TransmitFrame(&IPC_Handle.TxFrame);
    }
    else
    {
        /* ── On Slave: refresh the preloaded TX data ──
         * FSM layer updates IPC_Handle.TxFrame.
         * We re-encode and preload so Master gets
         * fresh data on its next transfer              */
        IPC_TransmitFrame(&IPC_Handle.TxFrame);
    }
}
