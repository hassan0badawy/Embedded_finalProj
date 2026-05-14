#ifndef SPI_H_
#define SPI_H_

#include "std_types.h"

#define SPI_OK      0u
#define SPI_NOK     1u

#define SPI_SLAVE   0u
#define SPI_MASTER  1u

/* PC13 is now the Chip Select to avoid PB12 & PA4 conflicts */
#define IPC_CS_PORT  GPIO_A
#define IPC_CS_PIN   4

void Spi1_Init(uint8 MasterSlave, uint8 ClkPol, uint8 ClkPhase);
uint8 Spi1_TransmitReceiveByte(uint8 TxData, uint8* RxData);
void Spi1_CS_Select(void);
void Spi1_CS_Release(void);


#endif /* SPI_H_ */
