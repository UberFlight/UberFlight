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
#pragma once

typedef enum {
    ADC_BATTERY = 0,
    ADC_EXTERNAL1 = 1,
    ADC_EXTERNAL2 = 2,
    ADC_CHANNEL_MAX = 3
} AdcChannel;
///////////////////////////////////////////////////////////////////////////////

#define VOLTS_PER_BIT   (3.3f / 4096.0f)


///////////////////////////////////////////////////////////////////////////////
//  Voltage Monitor
///////////////////////////////////////////////////////////////////////////////

float voltageMonitor(void);



typedef struct drv_adc_config_t {
    uint8_t powerAdcChannel;     // which channel used for current monitor, allowed PA1, PB1 (ADC_Channel_1, ADC_Channel_9)
} drv_adc_config_t;

void adcInit(drv_adc_config_t *init);
uint16_t adcGetChannel(uint8_t channel);
