#include "board.h"

// SPI2 Driver
// PB15     28      SPI2_MOSI
// PB14     27      SPI2_MISO
// PB13     26      SPI2_SCK
// PB12     25      SPI2_NSS

//static bool spiTest(SPI_TypeDef *SPIx);
#define SPI_BUSE            SPI2

bool spiInit(void)
{
    gpio_config_t gpio;
    SPI_InitTypeDef spi;

    // Enable SPI2 clock
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI2, ENABLE);

    // MOSI + SCK as output
    gpio.mode = Mode_AF_PP;
    gpio.pin = Pin_13 | Pin_15;
    gpio.speed = Speed_50MHz;
    gpioInit(GPIOB, &gpio);
    // MISO as input
    gpio.pin = Pin_14;
    gpio.mode = Mode_IN_FLOATING;
    gpioInit(GPIOB, &gpio);
    // NSS as gpio slave select
    gpio.pin = Pin_12;
    gpio.mode = Mode_Out_PP;
    gpioInit(GPIOB, &gpio);

    // Init SPI2 hardware
    SPI_I2S_DeInit(SPI_BUSE);
    spi.SPI_Mode = SPI_Mode_Master;
    spi.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    spi.SPI_DataSize = SPI_DataSize_8b;
    spi.SPI_NSS = SPI_NSS_Soft;
    spi.SPI_FirstBit = SPI_FirstBit_MSB;
    spi.SPI_CRCPolynomial = 7;
    spi.SPI_CPOL = SPI_CPOL_High;
    spi.SPI_CPHA = SPI_CPHA_2Edge;
    spi.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
    SPI_Init(SPI_BUSE, &spi);
    SPI_Cmd(SPI_BUSE, ENABLE);

    return true;
}

void spiSelect(bool select)
{
    if (select) {
        digitalLo(GPIOB, Pin_12);
    } else {
        digitalHi(GPIOB, Pin_12);
    }
}

uint8_t spiTransferByte(uint8_t in)
{
    uint8_t rx;
    SPI_BUSE->DR;
    SPI_BUSE->DR = in;
    while (!(SPI_BUSE->SR & SPI_I2S_FLAG_RXNE))
        ;
    rx = SPI_BUSE->DR;
    while (!(SPI_BUSE->SR & SPI_I2S_FLAG_TXE))
        ;
    while (SPI_BUSE->SR & SPI_I2S_FLAG_BSY)
        ;
    return rx;
}
// duplicate common code
static volatile uint16_t spiErrorCount = 0;
// return -1, then check SPIErrorCount
uint32_t spiTimeoutUserCallback()
{
    spiErrorCount++;
    return -1;
}

uint8_t spiTransfer(uint8_t *out, uint8_t *in, int len)
{

    uint16_t spiTimeout;
    uint8_t b;

    SPI_BUSE->DR;

    while (len--) {
        b = in ? *(in++) : 0xFF;
        spiTimeout = 1000;
        while (SPI_I2S_GetFlagStatus(SPI_BUSE, SPI_I2S_FLAG_TXE) == RESET) {
            if ((spiTimeout--) == 0)
                return spiTimeoutUserCallback();
        }
        SPI_I2S_SendData(SPI_BUSE, b);
        spiTimeout = 1000;
        while (SPI_I2S_GetFlagStatus(SPI_BUSE, SPI_I2S_FLAG_RXNE) == RESET) {
            if ((spiTimeout--) == 0)
                return spiTimeoutUserCallback();
        }

        b = SPI_I2S_ReceiveData(SPI_BUSE);
        if (out)
            *(out++) = b;
    }

    return true;
}

