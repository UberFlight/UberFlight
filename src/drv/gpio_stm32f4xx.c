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
#include "mw.h"
#include "gpio.h"


#define MODE_OFFSET 0
#define PUPD_OFFSET 2
#define OUTPUT_OFFSET 4

#define MODE_MASK ((1|2) << MODE_OFFSET)
#define PUPD_MASK ((1|2) << PUPD_OFFSET)
#define OUTPUT_MASK ((1|2) << OUTPUT_OFFSET)

void gpioStart(void)
{

    gpio_config_t gpio;

    //set swd
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource13, GPIO_AF_SWJ);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource14, GPIO_AF_SWJ);

    // Make all GPIO in by default to save power and reduce noise
    gpio.pin = Pin_All & ~(Pin_13 | Pin_14 | Pin_15);
    gpio.mode = Mode_AIN;
    gpioInit(GPIOA, &gpio);
    gpio.pin = Pin_All;
    gpioInit(GPIOB, &gpio);
    gpioInit(GPIOC, &gpio);
    gpioInit(GPIOD, &gpio);

    struct {
        GPIO_TypeDef *gpio;
        gpio_config_t cfg;
    } gpio_setup[] = {
#ifdef LED0
            { .gpio = LED0_GPIO, .cfg = { LED0_PIN, Mode_Out_OD, Speed_50MHz } },
#endif
#ifdef LED1

            {
                .gpio = LED1_GPIO,
                .cfg = {LED1_PIN, Mode_Out_OD, Speed_50MHz}
            },
#endif
#ifdef BUZZER
            { .gpio = BEEP_GPIO, .cfg = { BEEP_PIN, Mode_Out_OD, Speed_50MHz } },
#endif
            };

    uint32_t i;
    uint8_t gpio_count = sizeof(gpio_setup) / sizeof(gpio_setup[0]);
    for (i = 0; i < gpio_count; i++) {
        gpioInit(gpio_setup[i].gpio, &gpio_setup[i].cfg);
    }

    {

        //TODO F4
//        // 6x3 header
//        GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_1);
//        GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_1);
//        GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_9);
//        GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_9);
//        GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_1);
//        GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_1);
//
//        // 2x5 header rc pin
//        GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_6);
////        if (feature(FEATURE_I2C)) {
////            GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_4);
////            GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_4);
////        } else
//        {
//            GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_6);
//            GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_6);
//        }
//
//        GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_2);
//        GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_2);
//        GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_2);
//        GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_2);
//        GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_2);

    }
}

void gpioInit(GPIO_TypeDef *gpio, gpio_config_t *config)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    uint32_t pinIndex;
    for (pinIndex = 0; pinIndex < 16; pinIndex++) {
// are we doing this pin?
        uint32_t pinMask = (0x1 << pinIndex);
        if (config->pin & pinMask) {

            GPIO_InitStructure.GPIO_Pin = config->pin;
            GPIO_InitStructure.GPIO_Mode = config->mode;

            GPIOSpeed_TypeDef speed = GPIO_Speed_25MHz;
            switch (config->speed) {
                case Speed_10MHz:
                case Speed_25MHz:
                    speed = GPIO_Speed_25MHz;
                    break;
                case Speed_2MHz:
                    speed = GPIO_Speed_2MHz;
                    break;
                case Speed_50MHz:
                    speed = GPIO_Speed_50MHz;
                    break;
            }

            GPIO_InitStructure.GPIO_Speed = speed;
            GPIO_InitStructure.GPIO_OType = (config->mode >> OUTPUT_OFFSET) & OUTPUT_MASK;
            GPIO_InitStructure.GPIO_PuPd = (config->mode >> PUPD_OFFSET) & PUPD_MASK;
            GPIO_Init(gpio, &GPIO_InitStructure);
        }
    }

}

void gpioExtiLineConfig(uint8_t portsrc, uint8_t pinsrc)
{
// FIXME needed yet? implement?
}
