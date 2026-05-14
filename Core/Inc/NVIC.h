#ifndef NVIC_H
#define NVIC_H

#include "std_types.h"

/* NVIC ISER registers — 8 registers cover all 240 possible IRQs */
#define NVIC_ISER_BASE    ((volatile u32*)0xE000E100UL)

/* Enable an IRQ in the NVIC (works for IRQ 0..239) */
static inline void Nvic_EnableIrq(uint8 IrqNum)
{
    NVIC_ISER_BASE[IrqNum >> 5u] = (1UL << (IrqNum & 0x1Fu));
}

/* Set the priority of an IRQ (works for IRQ 0..239) */
static inline void Nvic_SetPriority(uint8 IrqNum, uint8 Priority)
{
    volatile uint8 *IP = (volatile uint8*)0xE000E400UL;
    IP[IrqNum] = (uint8)(Priority << 4u);
}

/* Convenience macro — same as Nvic_EnableIrq */
#define NVIC_ENABLE_IRQ(irq)         Nvic_EnableIrq((uint8)(irq))
#define NVIC_SET_PRIORITY(irq, pri)  Nvic_SetPriority((uint8)(irq), (uint8)(pri))

#endif /* NVIC_H */
