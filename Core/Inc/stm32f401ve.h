#ifndef STM32F401VE_H
#define STM32F401VE_H

#include "std_types.h"

/* CMSIS intrinsics — inline ARM assembly replacements */
static inline u32 __get_PRIMASK(void) {
    u32 result;
    __asm volatile ("MRS %0, primask" : "=r"(result));
    return result;
}
static inline void __disable_irq(void) {
    __asm volatile ("CPSID I" ::: "memory");
}
static inline void __set_PRIMASK(u32 priMask) {
    __asm volatile ("MSR primask, %0" :: "r"(priMask) : "memory");
}


/* ─────────────────────────────────────────
 * BASE ADDRESSES
 * ───────────────────────────────────────── */
#define PERIPH_BASE         0x40000000UL
#define APB1_BASE           (PERIPH_BASE + 0x00000000UL)
#define APB2_BASE           (PERIPH_BASE + 0x00010000UL)
#define AHB1_BASE           (PERIPH_BASE + 0x00020000UL)

#define SPI1_BASE           (APB2_BASE + 0x3000UL)   /* SPI1 on APB2 */
#define SPI2_BASE           (APB1_BASE + 0x3800UL)   /* SPI2 on APB1 */
#define SPI3_BASE           (APB1_BASE + 0x3C00UL)   /* SPI3 on APB1 */

#define SYSCFG_BASE         (APB2_BASE + 0x3800UL)
#define EXTI_BASE           (APB2_BASE + 0x3C00UL)

#define NVIC_BASE           0xE000E100UL
#define SCB_BASE            0xE000ED00UL

/* ─────────────────────────────────────────
 * SPI REGISTER STRUCT
 * ───────────────────────────────────────── */
typedef struct {
    volatile u32 CR1;       /* Control Register 1        offset 0x00 */
    volatile u32 CR2;       /* Control Register 2        offset 0x04 */
    volatile u32 SR;        /* Status Register           offset 0x08 */
    volatile u32 DR;        /* Data Register             offset 0x0C */
    volatile u32 CRCPR;     /* CRC Polynomial Register   offset 0x10 */
    volatile u32 RXCRCR;    /* RX CRC Register           offset 0x14 */
    volatile u32 TXCRCR;    /* TX CRC Register           offset 0x18 */
    volatile u32 I2SCFGR;   /* I2S Config Register       offset 0x1C */
    volatile u32 I2SPR;     /* I2S Prescaler Register    offset 0x20 */
} SPI_RegDef_t;

/* ─────────────────────────────────────────
 * SPI PERIPHERAL POINTERS
 * ───────────────────────────────────────── */
#define SPI1                ((SPI_RegDef_t *) SPI1_BASE)
#define SPI2                ((SPI_RegDef_t *) SPI2_BASE)
#define SPI3                ((SPI_RegDef_t *) SPI3_BASE)

/* ─────────────────────────────────────────
 * SPI CR1 BIT POSITIONS
 * ───────────────────────────────────────── */
#define SPI_CR1_CPHA        0   /* Clock phase              */
#define SPI_CR1_CPOL        1   /* Clock polarity           */
#define SPI_CR1_MSTR        2   /* Master selection         */
#define SPI_CR1_BR0         3   /* Baud rate bit 0          */
#define SPI_CR1_BR1         4   /* Baud rate bit 1          */
#define SPI_CR1_BR2         5   /* Baud rate bit 2          */
#define SPI_CR1_SPE         6   /* SPI enable               */
#define SPI_CR1_LSBFIRST    7   /* LSB first                */
#define SPI_CR1_SSI         8   /* Internal slave select    */
#define SPI_CR1_SSM         9   /* Software slave mgmt      */
#define SPI_CR1_RXONLY      10  /* Receive only             */
#define SPI_CR1_DFF         11  /* Data frame format (8/16) */
#define SPI_CR1_BIDIOE      14  /* Bidirectional output en  */
#define SPI_CR1_BIDIMODE    15  /* Bidirectional mode       */

/* ─────────────────────────────────────────
 * SPI CR2 BIT POSITIONS
 * ───────────────────────────────────────── */
#define SPI_CR2_RXDMAEN     0   /* RX DMA enable            */
#define SPI_CR2_TXDMAEN     1   /* TX DMA enable            */
#define SPI_CR2_SSOE        2   /* SS output enable         */
#define SPI_CR2_ERRIE       5   /* Error interrupt enable   */
#define SPI_CR2_RXNEIE      6   /* RX not empty IRQ enable  */
#define SPI_CR2_TXEIE       7   /* TX empty IRQ enable      */

/* ─────────────────────────────────────────
 * SPI SR BIT POSITIONS
 * ───────────────────────────────────────── */
#define SPI_SR_RXNE         0   /* Receive buffer not empty */
#define SPI_SR_TXE          1   /* Transmit buffer empty    */
#define SPI_SR_CHSIDE       2   /* Channel side             */
#define SPI_SR_UDR          3   /* Underrun flag            */
#define SPI_SR_CRCERR       4   /* CRC error flag           */
#define SPI_SR_MODF         5   /* Mode fault               */
#define SPI_SR_OVR          6   /* Overrun flag             */
#define SPI_SR_BSY          7   /* Busy flag                */


/* ── TIMER REGISTER STRUCT ── */
typedef struct {
    volatile u32 CR1;
    volatile u32 CR2;
    volatile u32 SMCR;
    volatile u32 DIER;
    volatile u32 SR;
    volatile u32 EGR;
    volatile u32 CCMR1;
    volatile u32 CCMR2;
    volatile u32 CCER;
    volatile u32 CNT;
    volatile u32 PSC;
    volatile u32 ARR;
    volatile u32 RCR;
    volatile u32 CCR1;
    volatile u32 CCR2;
    volatile u32 CCR3;
    volatile u32 CCR4;
    volatile u32 BDTR;
    volatile u32 DCR;
    volatile u32 DMAR;
} TIM_RegDef_t;

#define TIM1_BASE           (APB2_BASE + 0x0000UL)
#define TIM2_BASE           (APB1_BASE + 0x0000UL)


#define TIM1                ((TIM_RegDef_t *) TIM1_BASE)
#define TIM2                ((TIM_RegDef_t *) TIM2_BASE)


/* TIM CR1 bits */
#define TIM_CR1_CEN         0u
#define TIM_CR1_ARPE        7u

/* TIM CCMR1 bits */
#define TIM_OC_PWM_MODE1    (0x6UL << 4)   /* OC1M = 110 in bits [6:4] */
#define TIM_CCMR1_OC1PE     3u

/* TIM CCER bits */
#define TIM_CCER_CC1E       0u

/* TIM BDTR bits */
#define TIM_BDTR_MOE        15u

/* RCC APB1ENR bits for timers */
#define RCC_APB1ENR_TIM2EN  (1u << 0)
#define RCC_APB1ENR_TIM6EN  (1u << 4)
/* RCC APB2ENR bit for TIM1 already defined above with SPI1 */
#define RCC_APB2ENR_TIM1EN  (1u << 0)


/* ─────────────────────────────────────────
 * RCC APB2ENR BIT for SPI1
 * ───────────────────────────────────────── */
#define RCC_APB2ENR_SPI1EN  (1u << 12)

/* ─────────────────────────────────────────
 * EXTI REGISTER STRUCT
 * ───────────────────────────────────────── */
typedef struct {
    volatile u32 IMR;       /* Interrupt mask register   */
    volatile u32 EMR;       /* Event mask register       */
    volatile u32 RTSR;      /* Rising trigger selection  */
    volatile u32 FTSR;      /* Falling trigger selection */
    volatile u32 SWIER;     /* Software interrupt event  */
    volatile u32 PR;        /* Pending register          */
} EXTI_RegDef_t;

#define EXTI                ((EXTI_RegDef_t *) EXTI_BASE)

/* ─────────────────────────────────────────
 * NVIC REGISTER STRUCT
 * ───────────────────────────────────────── */
typedef struct {
    volatile u32 ISER[8];   /* Interrupt set-enable      */
    u32 RESERVED0[24];
    volatile u32 ICER[8];   /* Interrupt clear-enable    */
    u32 RESERVED1[24];
    volatile u32 ISPR[8];   /* Interrupt set-pending     */
    u32 RESERVED2[24];
    volatile u32 ICPR[8];   /* Interrupt clear-pending   */
    u32 RESERVED3[24];
    volatile u32 IABR[8];   /* Interrupt active bit      */
    u32 RESERVED4[56];
    volatile u8  IP[240];   /* Interrupt priority        */
    u32 RESERVED5[644];
    volatile u32 STIR;      /* Software trigger IRQ      */
} NVIC_RegDef_t;

#define NVIC                ((NVIC_RegDef_t *) NVIC_BASE)

/* ─────────────────────────────────────────
 * IRQ NUMBERS (STM32F401VE)
 * ───────────────────────────────────────── */
#define IRQ_SPI1            35  /* SPI1 global interrupt    */
#define IRQ_EXTI0           6   /* EXTI line 0              */
#define IRQ_EXTI1           7   /* EXTI line 1              */
#define IRQ_EXTI2           8   /* EXTI line 2              */
#define IRQ_EXTI3           9   /* EXTI line 3              */
#define IRQ_EXTI4           10  /* EXTI line 4              */
#define IRQ_EXTI9_5         23  /* EXTI lines 5-9           */

/* ─────────────────────────────────────────
 * NVIC HELPER MACROS
 * ───────────────────────────────────────── */
#define NVIC_ENABLE_IRQ(irq)        (NVIC->ISER[(irq) >> 5] = (1UL << ((irq) & 0x1F)))
#define NVIC_DISABLE_IRQ(irq)       (NVIC->ICER[(irq) >> 5] = (1UL << ((irq) & 0x1F)))
#define NVIC_SET_PRIORITY(irq, pri) (NVIC->IP[irq] = (u8)((pri) << 4))

/* Aliases for case-insensitive/mixed-case usage */
#define Nvic_EnableIrq(irq)         NVIC_ENABLE_IRQ(irq)
#define Nvic_DisableIrq(irq)        NVIC_DISABLE_IRQ(irq)
#define Nvic_SetPriority(irq, pri)  NVIC_SET_PRIORITY(irq, pri)

/* ─────────────────────────────────────────
 * CRITICAL SECTION MACROS
 * ───────────────────────────────────────── */
#define ENTER_CRITICAL()    __asm volatile ("CPSID I" ::: "memory")
#define EXIT_CRITICAL()     __asm volatile ("CPSIE I" ::: "memory")

/* Aliases for case-insensitive/mixed-case usage */
#define Enter_Critical()    ({ u32 _pm = __get_PRIMASK(); __disable_irq(); _pm; })
#define Exit_Critical(pm)   __set_PRIMASK(pm)

#endif /* STM32F401VE_H */
