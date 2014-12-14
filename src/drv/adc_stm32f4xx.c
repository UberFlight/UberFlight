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
//  ADC Defines and Variables
///////////////////////////////////////////////////////////////////////////////

#define VBATT_PIN              GPIO_Pin_4
#define VBATT_GPIO             GPIOA
#define VBATT_CHANNEL          ADC_Channel_1

#define ADC_PIN                GPIO_Pin_5
#define ADC_GPIO               GPIOA
#define ADC_CHANNEL            ADC_Channel_2

///////////////////////////////////////

uint16_t adc2ConvertedValues[8] = { 0, 0, 0, 0, 0, 0, 0, 0, };

///////////////////////////////////////////////////////////////////////////////
//  ADC Initialization
///////////////////////////////////////////////////////////////////////////////

void adcInit(drv_adc_config_t *init)
{

}

///////////////////////////////////////////////////////////////////////////////
//  Voltage Monitor
///////////////////////////////////////////////////////////////////////////////

float voltageMonitor(void)
{
    uint8_t i;
    uint16_t convertedSum = 0;

    for (i = 0; i < 4; i++)
        convertedSum += adc2ConvertedValues[i];

    return (float)convertedSum / 4.0f;
}

///////////////////////////////////////////////////////////////////////////////
//  ADC Channel
///////////////////////////////////////////////////////////////////////////////

float adcChannel(void)
{
    uint8_t i;
    uint16_t convertedSum = 0;

    for (i = 4; i < 8; i++)
        convertedSum += adc2ConvertedValues[i];

    return (float)convertedSum / 4.0f;
}

///////////////////////////////////////////////////////////////////////////////

uint16_t adcGetChannel(uint8_t channel)
{
    return 0;
}
