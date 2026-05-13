#ifndef STD_TYPES_H
#define STD_TYPES_H

#include <stdint.h>

/* Standard Short Types */
typedef unsigned char       u8;
typedef signed char         s8;
typedef unsigned short      u16;
typedef signed short        s16;
typedef unsigned int        u32;
typedef signed int          s32;
typedef unsigned long long  u64;
typedef signed long long    s64;
typedef float               f32;
typedef double              f64;

/* Compatibility Types for provided logic */
typedef u8  uint8;
typedef u16 uint16;
typedef u32 uint32;
typedef u64 uint64;

/* Standard C99 types */
typedef uint8_t  uint8_t;
typedef uint16_t uint16_t;
typedef uint32_t uint32_t;

#ifndef NULL
#define NULL ((void*)0)
#endif

#endif /* STD_TYPES_H */
