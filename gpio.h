#ifndef GPIO_H
#define GPIO_H

#include "std_types.h"

#define GPIOA_BASE          0x40020000UL
#define GPIOB_BASE          0x40020400UL
#define GPIOC_BASE          0x40020800UL

typedef struct {
    volatile u32 MODER;    /* GPIO port mode register               */
    volatile u32 OTYPER;   /* GPIO port output type register        */
    volatile u32 OSPEEDR;  /* GPIO port output speed register       */
    volatile u32 PUPDR;    /* GPIO port pull-up/pull-down register  */
    volatile u32 IDR;      /* GPIO port input data register         */
    volatile u32 ODR;      /* GPIO port output data register        */
    volatile u32 BSRR;     /* GPIO port bit set/reset register      */
    volatile u32 LCKR;     /* GPIO port configuration lock register */
    volatile u32 AFRL;     /* GPIO alternate function low register  */
    volatile u32 AFRH;     /* GPIO alternate function high register */
} GPIO_RegDef_t;

#define GPIOA               ((GPIO_RegDef_t *) GPIOA_BASE)
#define GPIOB               ((GPIO_RegDef_t *) GPIOB_BASE)
#define GPIOC               ((GPIO_RegDef_t *) GPIOC_BASE)

#endif /* GPIO_H */
