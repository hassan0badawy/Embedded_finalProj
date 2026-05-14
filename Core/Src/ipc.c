/**
 * ipc.c
 * IPC core logic: encoding/decoding packets and running the 50ms sync.
 */
#include "ipc.h"
#include "stm32f401ve.h"
#include "Timer.h"

static volatile u8 tx_raw[IPC_PACKET_SIZE];
static volatile u8 rx_raw[IPC_PACKET_SIZE];
static u8 is_master_node = 0;

static void IPC_EncodeFrame(SPI_Packet_t *pFrame, volatile u8 *pBuf)
{
    u8 csum = 0;
    
    pFrame->header = IPC_HEADER;
    
    pBuf[0] = pFrame->header;
    pBuf[1] = pFrame->current_floor;
    pBuf[2] = pFrame->fsm_state;
    pBuf[3] = pFrame->target_floor;
    pBuf[4] = pFrame->motor_speed;
    pBuf[5] = pFrame->flags;
    pBuf[6] = pFrame->reserved;
    
    for (u8 i = 0; i < 7u; i++) {
        csum ^= pBuf[i];
    }
    
    pFrame->checksum = csum;
    pBuf[7] = csum;
}

static void IPC_DecodeFrame(volatile u8 *pBuf, SPI_Packet_t *pFrame)
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

void IPC_Init(u8 isMaster)
{
    is_master_node = isMaster;
    
    if (isMaster) {
        SPI1_Init_Master();
    } else {
        SPI1_Init_Slave();
        /* Preload Slave with empty packet initially */
        SPI_Packet_t empty = {0};
        empty.header = IPC_HEADER;
        IPC_EncodeFrame(&empty, tx_raw);
        SPI_SlavePreload(&SPI1_Handle, tx_raw);
    }
}

void IPC_TransmitFrame(SPI_Packet_t *pFrame)
{
    IPC_EncodeFrame(pFrame, tx_raw);
    
    if (is_master_node) {
        SPI_MasterTransfer_IT(&SPI1_Handle, tx_raw, rx_raw, IPC_PACKET_SIZE);
    } else {
        /* Slave updates its preload buffer for the NEXT transfer */
        SPI_SlavePreload(&SPI1_Handle, tx_raw);
    }
}

u8 IPC_CheckConsistency(void)
{
    u8 csum = 0;
    
    if (rx_raw[0] != IPC_HEADER) {
        return 0;
    }
    
    for (u8 i = 0; i < 7u; i++) {
        csum ^= rx_raw[i];
    }
    
    if (csum != rx_raw[7]) {
        return 0;
    }
    
    IPC_DecodeFrame(rx_raw, (SPI_Packet_t *)&GSS.rx_packet);
    GSS.last_valid_rx_tick = Timer_GetMs();
    GSS.comm_fault = 0;
    
    return 1;
}

void IPC_Update(void)
{
    /* If Master, check previous transfer result */
    if (is_master_node && SPI1_Handle.RxComplete) {
        SPI1_Handle.RxComplete = 0;
        IPC_CheckConsistency();
    }
    
    /* If Slave, Check consistency of received data */
    if (!is_master_node && SPI1_Handle.RxComplete) {
        SPI1_Handle.RxComplete = 0;
        IPC_CheckConsistency();
    }
    
    /* Check for comm fault */
    if ((Timer_GetMs() - GSS.last_valid_rx_tick) > IPC_TIMEOUT_MS) {
        GSS.comm_fault = 1;
    }
    
    /* Prepare next frame to send */
    u32 pm = Enter_Critical();
    GSS.tx_packet.current_floor = GSS.position;
    GSS.tx_packet.fsm_state     = GSS.fsm_state;
    GSS.tx_packet.target_floor  = GSS.target;
    GSS.tx_packet.motor_speed   = GSS.speed;
    
    /* Master sends assigned target via reserved byte */
    GSS.tx_packet.reserved      = is_master_node ? GSS.slave_assigned_target : 0x00u;
    
    GSS.tx_packet.flags = 0;
    if (GSS.door_open) GSS.tx_packet.flags |= FLAG_DOOR_OPEN;
    if (GSS.emergency) GSS.tx_packet.flags |= FLAG_EMERGENCY;
    if (GSS.comm_fault) GSS.tx_packet.flags |= IPC_FLAG_COMM_FAULT;
    Exit_Critical(pm);
    
    IPC_TransmitFrame((SPI_Packet_t *)&GSS.tx_packet);
    
    /* Update slave state in GSS if valid data */
    if (!GSS.comm_fault) {
        pm = Enter_Critical();
        GSS.slave_position  = GSS.rx_packet.current_floor;
        GSS.slave_fsm_state = GSS.rx_packet.fsm_state;
        GSS.slave_target    = GSS.rx_packet.target_floor;
        GSS.slave_speed     = GSS.rx_packet.motor_speed;
        GSS.slave_flags     = GSS.rx_packet.flags;
        
        if (!is_master_node) {
            /* Slave reads the target assigned by Master */
            u8 assigned_floor = GSS.rx_packet.reserved;
            if (assigned_floor < NUM_FLOORS) {
                GSS.floor_request[assigned_floor] = 1u;
            }
        }
        
        Exit_Critical(pm);
    }
}
