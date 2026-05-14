#include "../Inc/shared.h"
#include "../Inc/ipc.h"

/*
 * Provide a weak SystemState instance so both Master and Slave
 * builds link cleanly. The Dispatcher (Master firmware) provides
 * a strong definition of `SystemState` in dispatcher.c, so this
 * weak definition will be overridden there. Slave builds will
 * use this weak instance so IPC can update received master frames
 * into `SystemState` if desired.
 */
__attribute__((weak)) GlobalSharedState_t SystemState;

/*
 * Callback hook invoked on Slave when a Master frame is received
 * and decoded. Elevator.c (Slave firmware) can implement this
 * (without being required) to apply the master's command to GSS.
 */
__attribute__((weak)) void IPC_OnMasterFrameReceived(const IPC_Frame_t *pFrame)
{
    /* Default: no-op */
    (void)pFrame;
}
