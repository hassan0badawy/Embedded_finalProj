/**
 * stm32f401ve.h
 * Master Hardware Header — STM32F401VE (Cortex-M4)
 *
 * All peripheral register structs, base addresses, peripheral pointers,
 * IRQ numbers, and critical section helpers live here.
 * Every module must include this file (directly or via Elevator.h).
 */
#ifndef STM32F401VE_H
#define STM32F401VE_H

#include "std_types.h"

/* ─────────────────────────────────────────────────────────────────────────
 * CMSIS Inline Intrinsics
 * ───────────────────────────────────────────────────────────────────────── */
static inline u32 __get_PRIMASK(void)
{
    u32 result;
    __asm volatile ("MRS %0, primask" : "=r"(result));
    return result;
}
static inline void __disable_irq(void)
{
    __asm volatile ("CPSID I" ::: "memory");
}
static inline void __set_PRIMASK(u32 priMask)
{
    __asm volatile ("MSR primask, %0" :: "r"(priMask) : "memory");
}

/* ─────────────────────────────────────────────────────────────────────────
 * Critical Section Helpers (inline functions — no -Wpedantic warnings)
 * Usage:
 *   u32 pm = Enter_Critical();
 *   ... atomic operation ...
 *   Exit_Critical(pm);
 * ───────────────────────────────────────────────────────────────────────── */
static inline u32 Enter_Critical(void)
{
    u32 pm = __get_PRIMASK();
    __disable_irq();
    return pm;
}
static inline void Exit_Critical(u32 pm)
{
    __set_PRIMASK(pm);
}

/* ─────────────────────────────────────────────────────────────────────────
 * PERIPHERAL BASE ADDRESSES
 * ───────────────────────────────────────────────────────────────────────── */
#define PERIPH_BASE         0x40000000UL
#define APB1_BASE           (PERIPH_BASE + 0x00000000UL)
#define APB2_BASE           (PERIPH_BASE + 0x00010000UL)
#define AHB1_BASE           (PERIPH_BASE + 0x00020000UL)

/* APB1 */
#define TIM2_BASE           (APB1_BASE + 0x0000UL)
#define TIM3_BASE           (APB1_BASE + 0x0400UL)
#define TIM4_BASE           (APB1_BASE + 0x0800UL)
#define TIM5_BASE           (APB1_BASE + 0x0C00UL)
#define TIM6_BASE           (APB1_BASE + 0x1000UL)
#define SPI2_BASE           (APB1_BASE + 0x3800UL)

/* APB2 */
#define TIM1_BASE           (APB2_BASE + 0x0000UL)
#define USART1_BASE         (APB2_BASE + 0x1000UL)
#define SPI1_BASE           (APB2_BASE + 0x3000UL)
#define SYSCFG_BASE         (APB2_BASE + 0x3800UL)
#define EXTI_BASE           (APB2_BASE + 0x3C00UL)

/* AHB1 */
#define GPIOA_BASE          (AHB1_BASE + 0x0000UL)
#define GPIOB_BASE          (AHB1_BASE + 0x0400UL)
#define GPIOC_BASE          (AHB1_BASE + 0x0800UL)
#define GPIOD_BASE          (AHB1_BASE + 0x0C00UL)
#define RCC_BASE_ADDR       (AHB1_BASE + 0x3800UL)
#define FLASH_BASE          (AHB1_BASE + 0x3C00UL)
#define DMA1_BASE           (AHB1_BASE + 0x6000UL)
#define DMA2_BASE           (AHB1_BASE + 0x6400UL)

/* Cortex-M4 System */
#define NVIC_BASE           0xE000E100UL
#define SCB_BASE            0xE000ED00UL
#define SYSTICK_BASE        0xE000E010UL

/* ─────────────────────────────────────────────────────────────────────────
 * RCC REGISTER STRUCT
 * ───────────────────────────────────────────────────────────────────────── */
typedef struct {
    volatile u32 CR;
    volatile u32 PLLCFGR;
    volatile u32 CFGR;
    volatile u32 CIR;
    volatile u32 AHB1RSTR;
    volatile u32 AHB2RSTR;
    volatile u32 AHB3RSTR;
    u32          Reserved0;
    volatile u32 APB1RSTR;
    volatile u32 APB2RSTR;
    u32          Reserved1[2];
    volatile u32 AHB1ENR;
    volatile u32 AHB2ENR;
    volatile u32 AHB3ENR;
    u32          Reserved2;
    volatile u32 APB1ENR;
    volatile u32 APB2ENR;
} RCC_RegDef_t;

#define RCC                 ((RCC_RegDef_t *) RCC_BASE_ADDR)

/* RCC AHB1ENR bits */
#define RCC_AHB1ENR_GPIOAEN (1u << 0)
#define RCC_AHB1ENR_GPIOBEN (1u << 1)
#define RCC_AHB1ENR_GPIOCEN (1u << 2)
#define RCC_AHB1ENR_GPIODEN (1u << 3)
#define RCC_AHB1ENR_DMA1EN  (1u << 21)
#define RCC_AHB1ENR_DMA2EN  (1u << 22)

/* RCC APB1ENR bits */
#define RCC_APB1ENR_TIM2EN  (1u << 0)
#define RCC_APB1ENR_TIM3EN  (1u << 1)
#define RCC_APB1ENR_TIM4EN  (1u << 2)
#define RCC_APB1ENR_TIM5EN  (1u << 3)
#define RCC_APB1ENR_TIM6EN  (1u << 4)

/* RCC APB2ENR bits */
#define RCC_APB2ENR_TIM1EN  (1u << 0)
#define RCC_APB2ENR_USART1EN (1u << 4)
#define RCC_APB2ENR_SPI1EN  (1u << 12)
#define RCC_APB2ENR_SYSCFGEN (1u << 14)

/* ─────────────────────────────────────────────────────────────────────────
 * GPIO REGISTER STRUCT (for EXTI config in Elevator.c)
 * Note: Gpio.c/Gpio_Private.h use 'GpioType' with GPIO_ prefix field names.
 *       This struct uses standard names for EXTI init code.
 * ───────────────────────────────────────────────────────────────────────── */
typedef struct {
    volatile u32 MODER;
    volatile u32 OTYPER;
    volatile u32 OSPEEDR;
    volatile u32 PUPDR;
    volatile u32 IDR;
    volatile u32 ODR;
    volatile u32 BSRR;
    volatile u32 LCKR;
    volatile u32 AFRL;
    volatile u32 AFRH;
} GPIO_RegDef_t;

#define GPIOA               ((GPIO_RegDef_t *) GPIOA_BASE)
#define GPIOB               ((GPIO_RegDef_t *) GPIOB_BASE)
#define GPIOC               ((GPIO_RegDef_t *) GPIOC_BASE)
#define GPIOD               ((GPIO_RegDef_t *) GPIOD_BASE)

/* ─────────────────────────────────────────────────────────────────────────
 * SPI REGISTER STRUCT
 * ───────────────────────────────────────────────────────────────────────── */
typedef struct {
    volatile u32 CR1;
    volatile u32 CR2;
    volatile u32 SR;
    volatile u32 DR;
    volatile u32 CRCPR;
    volatile u32 RXCRCR;
    volatile u32 TXCRCR;
    volatile u32 I2SCFGR;
    volatile u32 I2SPR;
} SPI_RegDef_t;

#define SPI1                ((SPI_RegDef_t *) SPI1_BASE)
#define SPI2                ((SPI_RegDef_t *) SPI2_BASE)

/* SPI CR1 bit positions */
#define SPI_CR1_CPHA        0u
#define SPI_CR1_CPOL        1u
#define SPI_CR1_MSTR        2u
#define SPI_CR1_BR0         3u
#define SPI_CR1_SPE         6u
#define SPI_CR1_SSI         8u
#define SPI_CR1_SSM         9u

/* SPI CR2 bit positions */
#define SPI_CR2_RXNEIE      6u
#define SPI_CR2_TXEIE       7u

/* SPI SR bit positions */
#define SPI_SR_RXNE         0u
#define SPI_SR_TXE          1u
#define SPI_SR_BSY          7u

/* ─────────────────────────────────────────────────────────────────────────
 * USART REGISTER STRUCT
 * ───────────────────────────────────────────────────────────────────────── */
typedef struct {
    volatile u32 SR;
    volatile u32 DR;
    volatile u32 BRR;
    volatile u32 CR1;
    volatile u32 CR2;
    volatile u32 CR3;
    volatile u32 GTPR;
} USART_RegDef_t;

#define USART1              ((USART_RegDef_t *) USART1_BASE)

/* USART SR bits */
#define USART_SR_TC         6u
#define USART_SR_TXE        7u

/* USART CR1 bits */
#define USART_CR1_UE        13u
#define USART_CR1_TE        3u
#define USART_CR1_RE        2u

/* USART CR3 bits */
#define USART_CR3_DMAT      7u

/* ─────────────────────────────────────────────────────────────────────────
 * DMA REGISTER STRUCTS
 * ───────────────────────────────────────────────────────────────────────── */
typedef struct {
    volatile u32 CR;
    volatile u32 NDTR;
    volatile u32 PAR;
    volatile u32 M0AR;
    volatile u32 M1AR;
    volatile u32 FCR;
} DMA_Stream_RegDef_t;

typedef struct {
    volatile u32 LISR;
    volatile u32 HISR;
    volatile u32 LIFCR;
    volatile u32 HIFCR;
} DMA_RegDef_t;

#define DMA2                ((DMA_RegDef_t *)    DMA2_BASE)
#define DMA2_Stream7        ((DMA_Stream_RegDef_t *)(DMA2_BASE + 0xB8UL))

/* DMA Stream CR bits */
#define DMA_CR_EN           0u
#define DMA_CR_TCIE         4u
#define DMA_CR_DIR_MEM2PER  (1u << 6)
#define DMA_CR_MINC         10u
#define DMA_CR_CHSEL_SHIFT  25u

/* DMA2 HISR/HIFCR bits for Stream7 (bits 22-27) */
#define DMA2_HISR_TCIF7     (1u << 27)
#define DMA2_HIFCR_CTCIF7   (1u << 27)
#define DMA2_HIFCR_CTEIF7   (1u << 25)
#define DMA2_HIFCR_CDMEIF7  (1u << 24)
#define DMA2_HIFCR_CFEIF7   (1u << 22)

/* ─────────────────────────────────────────────────────────────────────────
 * TIMER REGISTER STRUCTS
 * ───────────────────────────────────────────────────────────────────────── */

/* Advanced-Control Timer (TIM1) */
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
} TIM_Adv_RegDef_t;

#define TIM1                ((TIM_Adv_RegDef_t *) TIM1_BASE)

/* General-Purpose Timer (TIM2-5) */
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
} TIM_GP_RegDef_t;

#define TIM2                ((TIM_GP_RegDef_t *) TIM2_BASE)

/* Basic Timer (TIM6) */
typedef struct {
    volatile u32 CR1;
    volatile u32 CR2;
    u32          RESERVED0;
    volatile u32 DIER;
    volatile u32 SR;
    volatile u32 EGR;
    u32          RESERVED1[3];
    volatile u32 CNT;
    volatile u32 PSC;
    volatile u32 ARR;
} TIM_Basic_RegDef_t;

#define TIM6                ((TIM_Basic_RegDef_t *) TIM6_BASE)

/* Common TIM CR1 bits */
#define TIM_CR1_CEN         0u
#define TIM_CR1_ARPE        7u

/* Common TIM DIER bits */
#define TIM_DIER_UIE        0u

/* Common TIM SR bits */
#define TIM_SR_UIF          0u

/* Common TIM EGR bits */
#define TIM_EGR_UG          0u

/* TIM1 BDTR bits */
#define TIM_BDTR_MOE        15u

/* PWM mode bits (CCMR1/CCMR2) */
#define TIM_CCMR_OC1M_PWM1 (0x6u << 4)
#define TIM_CCMR_OC1PE      (1u << 3)
#define TIM_CCMR_OC2M_PWM1 (0x6u << 12)
#define TIM_CCMR_OC2PE      (1u << 11)

/* TIM CCER bits */
#define TIM_CCER_CC1E       0u

/* ─────────────────────────────────────────────────────────────────────────
 * EXTI REGISTER STRUCT
 * ───────────────────────────────────────────────────────────────────────── */
typedef struct {
    volatile u32 IMR;
    volatile u32 EMR;
    volatile u32 RTSR;
    volatile u32 FTSR;
    volatile u32 SWIER;
    volatile u32 PR;
} EXTI_RegDef_t;

#define EXTI                ((EXTI_RegDef_t *) EXTI_BASE)

/* ─────────────────────────────────────────────────────────────────────────
 * SYSCFG REGISTER STRUCT
 * ───────────────────────────────────────────────────────────────────────── */
typedef struct {
    volatile u32 MEMRMP;
    volatile u32 PMC;
    volatile u32 EXTICR[4];
    u32          RESERVED[2];
    volatile u32 CMPCR;
} SYSCFG_RegDef_t;

#define SYSCFG              ((SYSCFG_RegDef_t *) SYSCFG_BASE)

/* SYSCFG port codes for EXTICR */
#define SYSCFG_PORT_A       0x0u
#define SYSCFG_PORT_B       0x1u
#define SYSCFG_PORT_C       0x2u
#define SYSCFG_PORT_D       0x3u

/* ─────────────────────────────────────────────────────────────────────────
 * FLASH REGISTER (for latency)
 * ───────────────────────────────────────────────────────────────────────── */
#define FLASH_ACR           (*(volatile u32 *)(FLASH_BASE + 0x00UL))

/* ─────────────────────────────────────────────────────────────────────────
 * IRQ NUMBERS (STM32F401VE)
 * ───────────────────────────────────────────────────────────────────────── */
#define IRQ_EXTI0           6
#define IRQ_EXTI1           7
#define IRQ_EXTI2           8
#define IRQ_EXTI3           9
#define IRQ_EXTI4           10
#define IRQ_EXTI9_5         23
#define IRQ_EXTI15_10       40
#define IRQ_TIM2            28
#define IRQ_TIM6_DAC        54
#define IRQ_SPI1            35
#define IRQ_USART1          37
#define IRQ_DMA2_STREAM7    70

/* ─────────────────────────────────────────────────────────────────────────
 * NVIC HELPERS — forward to nvic.h
 * ───────────────────────────────────────────────────────────────────────── */
#include "nvic.h"

#endif /* STM32F401VE_H */
