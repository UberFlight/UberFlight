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


#include "board.h"

///////////////////////////////////////////////////////////////////////////////
// GPIO Defines
////////////////////////////////////////////////////////////////////////////////
#if defined(QUANTOM)
#define digitalHi(p, i)     { p->BSRRH = i; }
#define digitalLo(p, i)     { p->BSRRL = i; }
#else
#define digitalHi(p, i)     { p->BSRR = i; }
#define digitalLo(p, i)     { p->BRR = i; }
#endif
#define digitalToggle(p, i) { p->ODR ^= i; }
#define digitalIn(p, i)     (p->IDR & i)
///////////////////////////////////////

#define BEEP_OFF      digitalHi(BEEP_GPIO, BEEP_PIN)
#define BEEP_ON       digitalLo(BEEP_GPIO, BEEP_PIN)
#define BEEP_TOGGLE   digitalToggle(BEEP_GPIO, BEEP_PIN)

#define LED0_OFF      digitalHi(LED0_GPIO, LED0_PIN)
#define LED0_ON       digitalLo(LED0_GPIO, LED0_PIN)
#define LED0_TOGGLE   digitalToggle(LED0_GPIO, LED0_PIN)

#define LED1_OFF      digitalHi(LED1_GPIO, LED1_PIN)
#define LED1_ON       digitalLo(LED1_GPIO, LED1_PIN)
#define LED1_TOGGLE   digitalToggle(LED1_GPIO, LED1_PIN)



#if defined(NAZE)

typedef enum {
    Mode_AIN = GPIO_Mode_AIN,
    Mode_IN_FLOATING = GPIO_Mode_IN_FLOATING,
    Mode_IPD = GPIO_Mode_IPD,
    Mode_IPU = GPIO_Mode_IPU,
    Mode_Out_OD = GPIO_Mode_Out_OD,
    Mode_Out_PP = GPIO_Mode_Out_PP,
    Mode_AF_OD = GPIO_Mode_AF_OD,
    Mode_AF_PP = GPIO_Mode_AF_PP
} GPIO_Mode;

#endif

#if defined(NAZEPRO) || defined(QUANTOM)

typedef enum
{
    Mode_AIN = (GPIO_PuPd_NOPULL << 2) | GPIO_Mode_AN,
    Mode_IN_FLOATING = (GPIO_PuPd_NOPULL << 2) | GPIO_Mode_IN,
    Mode_IPD = (GPIO_PuPd_DOWN << 2) | GPIO_Mode_IN,
    Mode_IPU = (GPIO_PuPd_UP << 2) | GPIO_Mode_IN,
    Mode_Out_OD = (GPIO_OType_OD << 4) | GPIO_Mode_OUT,
    Mode_Out_PP = (GPIO_OType_PP << 4) | GPIO_Mode_OUT,
    Mode_AF_OD = (GPIO_OType_OD << 4) | GPIO_Mode_AF,
    Mode_AF_PP = (GPIO_OType_PP << 4) | GPIO_Mode_AF ,
    Mode_AF_PP_PD = (GPIO_OType_PP << 4) | (GPIO_PuPd_DOWN << 2) | GPIO_Mode_AF,
    Mode_AF_PP_PU = (GPIO_OType_PP << 4) | (GPIO_PuPd_UP << 2) | GPIO_Mode_AF
} GPIO_Mode;

#endif
typedef enum {
    Speed_10MHz = 1,
    Speed_2MHz,
    Speed_50MHz,
    Speed_25MHz
} GPIO_Speed;

typedef struct {
    uint16_t pin;
    GPIO_Mode mode;
    GPIO_Speed speed;
} gpio_config_t;


///////////////////////////////////////////////////////////////////////////////
// GPIO Initialization
///////////////////////////////////////////////////////////////////////////////

void gpioInit(GPIO_TypeDef *gpio, gpio_config_t *config);
void gpioStart(void);
///////////////////////////////////////////////////////////////////////////////
