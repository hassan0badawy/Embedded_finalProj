/**
 * Spi.h
 * SPI1 low-level initialization header.
 */
#ifndef SPI_H_
#define SPI_H_

#include "std_types.h"

/* Chip Select pin: PA4 (manual software NSS) */
#define SPI_CS_PORT     'A'
#define SPI_CS_PIN      4u

void SPI1_Init_Master(void);
void SPI1_Init_Slave(void);
void SPI1_CS_Assert(void);
void SPI1_CS_Release(void);

#endif /* SPI_H_ */
