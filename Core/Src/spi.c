/**
 * Spi.c
 * SPI1 hardware initialization for the IPC link.
 *
 * Pins (AF5):
 *   PB3 = SCK, PB4 = MISO, PB5 = MOSI, PA4 = CS (manual software NSS)
 *
 * SPI Clock math (PCLK2 = 16MHz):
 *   BR[2:0] = 010 → Baud = PCLK2/8 = 2MHz (safe for Proteus)
 *
 * Mode: Full-Duplex, 8-bit, CPOL=0, CPHA=0 (Mode 0)
 * SSM=1, SSI=1 (software slave management — no hardware NSS)
 */
#include "Spi.h"
#include "stm32f401ve.h"
#include "RCC.h"
#include "Gpio.h"
#include "spi_it.h"
#include "Bit_Math.h"

SPI_HandleTypeDef SPI1_Handle;

static void spi_gpio_init(void)
{
    /* PB3=SCK, PB4=MISO, PB5=MOSI — all AF5 */
    Gpio_Init(GPIO_B, 3, GPIO_AF, GPIO_PUSH_PULL);
    Gpio_Init(GPIO_B, 4, GPIO_AF, GPIO_PUSH_PULL);
    Gpio_Init(GPIO_B, 5, GPIO_AF, GPIO_PUSH_PULL);
    Gpio_SetAF(GPIO_B, 3, GPIO_AF5);
    Gpio_SetAF(GPIO_B, 4, GPIO_AF5);
    Gpio_SetAF(GPIO_B, 5, GPIO_AF5);

    /* PA4 = CS (output, default high = deselected) */
    Gpio_Init(GPIO_A, 4, GPIO_OUTPUT, GPIO_PUSH_PULL);
    SPI1_CS_Release();
}

static void spi_hw_config(u8 is_master)
{
    /* Enable SPI1 clock */
    RCC_EnableClock(RCC_SPI1);

    /* Reset CR1 */
    SPI1->CR1 = 0;

    /* Software NSS management (SSM=1, SSI=1) */
    SPI1->CR1 |= (1u << SPI_CR1_SSM) | (1u << SPI_CR1_SSI);

    /* Mode 0: CPOL=0, CPHA=0 */

    /* Baud rate: PCLK2/8 = 2MHz (BR[2:0]=010) */
    SPI1->CR1 |= (0x2u << SPI_CR1_BR0);

    /* Master/Slave selection */
    if (is_master) {
        SPI1->CR1 |= (1u << SPI_CR1_MSTR);
    }

    /* Enable RXNE interrupt */
    SPI1->CR2 |= (1u << SPI_CR2_RXNEIE);

    /* Enable SPI1 in NVIC, priority 1 */
    NVIC_SET_PRIORITY(IRQ_SPI1, 1u);
    NVIC_ENABLE_IRQ(IRQ_SPI1);

    /* Enable SPI peripheral */
    SPI1->CR1 |= (1u << SPI_CR1_SPE);
}

void SPI1_Init_Master(void)
{
    spi_gpio_init();
    spi_hw_config(1u);

    SPI1_Handle.State      = SPI_STATE_READY;
    SPI1_Handle.RxComplete = 0u;
}

void SPI1_Init_Slave(void)
{
    spi_gpio_init();
    spi_hw_config(0u);

    SPI1_Handle.State      = SPI_STATE_READY;
    SPI1_Handle.RxComplete = 0u;
}

void SPI1_CS_Assert(void)
{
    Gpio_WritePin(GPIO_A, SPI_CS_PIN, 0u);
}

void SPI1_CS_Release(void)
{
    Gpio_WritePin(GPIO_A, SPI_CS_PIN, 1u);
}
