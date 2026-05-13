#ifndef BIT_MATH_H
#define BIT_MATH_H

#define SET_BIT(REG, BIT)     ((REG) |= (1UL << (BIT)))
#define CLEAR_BIT(REG, BIT)   ((REG) &= ~(1UL << (BIT)))
#define TOGGLE_BIT(REG, BIT)  ((REG) ^= (1UL << (BIT)))
#define READ_BIT(REG, BIT)    (((REG) >> (BIT)) & 1UL)

#endif
