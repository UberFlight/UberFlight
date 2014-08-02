/*
 August 2013

 Focused Flight32 Rev -

 Copyright (c) 2013 John Ihlein.  All rights reserved.

 Open Source STM32 Based Multicopter Controller Software

 Designed to run on the Naze32Pro Flight Control Board

 Includes code and/or ideas from:

 1)AeroQuad
 2)BaseFlight
 3)CH Robotics
 4)MultiWii
 5)Paparazzi UAV
 5)S.O.H. Madgwick
 6)UAVX

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

///////////////////////////////////////////////////////////////////////////////
#include "board.h"

///////////////////////////////////////////////////////////////////////////////

// SPI2
// SCK  PB13
// MISO PB14
// MOSI PB15

///////////////////////////////////////////////////////////////////////////////
// SPI Defines and Variables
///////////////////////////////////////////////////////////////////////////////
#if defined(QUANTOM)
#define SPI_BUSE            SPI1
#define SPI_GPIO            GPIOA
#define SPI_SCK_PIN         GPIO_Pin_5
#define SPI_SCK_PIN_SOURCE  GPIO_PinSource5
#define SPI_SCK_CLK         RCC_AHB1Periph_GPIOA
#define SPI_MISO_PIN        GPIO_Pin_6
#define SPI_MISO_PIN_SOURCE GPIO_PinSource6
#define SPI_MISO_CLK        RCC_AHB1Periph_GPIOA
#define SPI_MOSI_PIN        GPIO_Pin_7
#define SPI_MOSI_PIN_SOURCE GPIO_PinSource7
#define SPI_MOSI_CLK        RCC_AHB1Periph_GPIOA
#endif

static volatile uint16_t spiErrorCount = 0;

///////////////////////////////////////////////////////////////////////////////
// SPI Initialize
///////////////////////////////////////////////////////////////////////////////

bool spiInit(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    SPI_InitTypeDef SPI_InitStructure;

    ///////////////////////////////////

//    RCC_AHB2PeriphClockCmd(SPI_SCK_CLK | SPI_MISO_CLK | SPI_MOSI_CLK, ENABLE);


    GPIO_PinAFConfig(SPI_GPIO, SPI_SCK_PIN_SOURCE, GPIO_AF_SPI1);
    GPIO_PinAFConfig(SPI_GPIO, SPI_MISO_PIN_SOURCE, GPIO_AF_SPI1);
    GPIO_PinAFConfig(SPI_GPIO, SPI_MOSI_PIN_SOURCE, GPIO_AF_SPI1);

    // Init pins
    GPIO_InitStructure.GPIO_Pin = SPI_SCK_PIN | SPI_MISO_PIN | SPI_MOSI_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

    GPIO_Init(SPI_GPIO, &GPIO_InitStructure);

    ///////////////////////////////

    GPIO_InitStructure.GPIO_Pin = EEPROM_CS_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

    GPIO_Init(EEPROM_CS_GPIO, &GPIO_InitStructure);

    DISABLE_EEPROM;

    ///////////////////////////////

//    GPIO_InitStructure.GPIO_Pin = HMC5983_CS_PIN;
//    GPIO_Init(HMC5983_CS_GPIO, &GPIO_InitStructure);
//    DISABLE_HMC5983;

    ///////////////////////////////

    GPIO_InitStructure.GPIO_Pin = MPU6000_CS_PIN;
    GPIO_Init(MPU6000_CS_GPIO, &GPIO_InitStructure);
    DISABLE_MPU6000;

    ///////////////////////////////
//
//    GPIO_InitStructure.GPIO_Pin = MS5611_CS_PIN;
//    GPIO_Init(MS5611_CS_GPIO, &GPIO_InitStructure);
//    DISABLE_MS5611;

    ///////////////////////////////

    SPI_I2S_DeInit(SPI_BUSE);

    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;  // 36/64 = 0.5625 MHz SPI Clock
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;

    SPI_Init(SPI_BUSE, &SPI_InitStructure);

//    SPI_RxFIFOThresholdConfig(SPI_BUSE, SPI_RxFIFOThreshold_QF);

    SPI_CalculateCRC(SPI_BUSE, DISABLE);

    SPI_Cmd(SPI_BUSE, ENABLE);


    return true;
}

///////////////////////////////////////////////////////////////////////////////
// SPI Timeout Callback
///////////////////////////////////////////////////////////////////////////////

// return -1, then check SPIErrorCount
uint32_t spiTimeoutUserCallback()
{
    spiErrorCount++;
    return -1;
}

///////////////////////////////////////////////////////////////////////////////
// SPI Transfer
///////////////////////////////////////////////////////////////////////////////

// return uint8_t value or -1 when failure
uint8_t spiTransferByte(uint8_t data)
{
    uint16_t spiTimeout = 1000;

    while (SPI_I2S_GetFlagStatus(SPI_BUSE, SPI_I2S_FLAG_TXE) == RESET)
        if ((spiTimeout--) == 0)
            return spiTimeoutUserCallback();

    SPI_SendData(SPI_BUSE, data);

    spiTimeout = 1000;
    while (SPI_I2S_GetFlagStatus(SPI_BUSE, SPI_I2S_FLAG_RXNE) == RESET)
        if ((spiTimeout--) == 0)
            return spiTimeoutUserCallback();

    return ((uint8_t)SPI_ReceiveData(SPI_BUSE));
}

// return true or -1 when failure
uint8_t spiTransfer(uint8_t *out, uint8_t *in, int len)
{
    uint16_t spiTimeout;
    uint8_t b;

    while (len--) {
        b = in ? *(in++) : 0xFF;
        spiTimeout = 1000;
        while (SPI_I2S_GetFlagStatus(SPI_BUSE, SPI_I2S_FLAG_TXE) == RESET) {
            if ((spiTimeout--) == 0)
                return spiTimeoutUserCallback();
        }
        SPI_SendData(SPI_BUSE, b);
        spiTimeout = 1000;
        while (SPI_I2S_GetFlagStatus(SPI_BUSE, SPI_I2S_FLAG_RXNE) == RESET) {
            if ((spiTimeout--) == 0)
                return spiTimeoutUserCallback();
        }

        b = SPI_ReceiveData(SPI_BUSE);
        if (out)
            *(out++) = b;
    }

    return true;
}

///////////////////////////////////////////////////////////////////////////////
// Set SPI Divisor
///////////////////////////////////////////////////////////////////////////////

void setSPIdivisor(uint16_t divisor)
{
#define BR_CLEAR_MASK 0xFFC7

    uint16_t tempRegister;

    SPI_Cmd(SPI_BUSE, DISABLE);

    tempRegister = SPI_BUSE->CR1;

    switch (divisor) {
        case 2:
            tempRegister &= BR_CLEAR_MASK;
            tempRegister |= SPI_BaudRatePrescaler_2;
            break;

        case 4:
            tempRegister &= BR_CLEAR_MASK;
            tempRegister |= SPI_BaudRatePrescaler_4;
            break;

        case 8:
            tempRegister &= BR_CLEAR_MASK;
            tempRegister |= SPI_BaudRatePrescaler_8;
            break;

        case 16:
            tempRegister &= BR_CLEAR_MASK;
            tempRegister |= SPI_BaudRatePrescaler_16;
            break;

        case 32:
            tempRegister &= BR_CLEAR_MASK;
            tempRegister |= SPI_BaudRatePrescaler_32;
            break;

        case 64:
            tempRegister &= BR_CLEAR_MASK;
            tempRegister |= SPI_BaudRatePrescaler_64;
            break;

        case 128:
            tempRegister &= BR_CLEAR_MASK;
            tempRegister |= SPI_BaudRatePrescaler_128;
            break;

        case 256:
            tempRegister &= BR_CLEAR_MASK;
            tempRegister |= SPI_BaudRatePrescaler_256;
            break;
    }

    SPI_BUSE->CR1 = tempRegister;

    SPI_Cmd(SPI_BUSE, ENABLE);
}

///////////////////////////////////////////////////////////////////////////////
// Get SPI Error Count
///////////////////////////////////////////////////////////////////////////////

uint16_t spiGetErrorCounter(void)
{
    return spiErrorCount;
}

void spiResetErrorCounter(void)
{
    spiErrorCount = 0;
}
///////////////////////////////////////////////////////////////////////////////
