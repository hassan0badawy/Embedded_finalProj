/**
 * ipc.h
 * Inter-Processor Communication (SPI) logic.
 * Uses SPI_Packet_t defined in shared.h.
 */
#ifndef IPC_H
#define IPC_H

#include "std_types.h"
#include "spi.h"
#include "shared.h"
#include "spi_it.h"

#define IPC_TIMEOUT_MS      150u    /* Comm fault timeout */

/* IPC flags not already in shared.h */
#define IPC_FLAG_MOVING_UP  (1u << 2)
#define IPC_FLAG_MOVING_DN  (1u << 3)
#define IPC_FLAG_COMM_FAULT (1u << 5)

void IPC_Init(u8 isMaster);
void IPC_TransmitFrame(SPI_Packet_t *pFrame);
u8 IPC_CheckConsistency(void);
void IPC_Update(void);

#endif /* IPC_H */
