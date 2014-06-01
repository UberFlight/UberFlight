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

// Cycles per microsecond
static volatile uint32_t usTicks = 0;

///////////////////////////////////////////////////////////////////////////////

// Current uptime for 1kHz systick timer. will rollover after 49 days.
// Hopefully we won't care.
static volatile uint32_t sysTickUptime = 0;


///////////////////////////////////////////////////////////////////////////////
// Cycle Counter
///////////////////////////////////////////////////////////////////////////////

static void cycleCounterInit(void)
{
    RCC_ClocksTypeDef clocks;
    RCC_GetClocksFreq(&clocks);
    usTicks = clocks.SYSCLK_Frequency / 1000000;

    // enable DWT access
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

}

///////////////////////////////////////////////////////////////////////////////
// SysTick
///////////////////////////////////////////////////////////////////////////////

void SysTick_Handler(void)
{
    sysTickUptime++;
}

///////////////////////////////////////////////////////////////////////////////
// System Time in Microseconds
//
// Note: This can be called from within IRQ Handlers, so uses LDREX/STREX.
// If a higher priority IRQ or DMA or anything happens the STREX will fail
// and restart the loop. Otherwise the same number that was read is harmlessly
// written back.
///////////////////////////////////////////////////////////////////////////////
uint32_t micros(void)
{
    register uint32_t ms, cycle_cnt;
    do {
        ms = sysTickUptime;
        cycle_cnt = SysTick->VAL;
    } while (ms != sysTickUptime);
    return (ms * 1000) + (usTicks * 1000 - cycle_cnt) / usTicks;
}

///////////////////////////////////////////////////////////////////////////////
// System Time in Milliseconds
///////////////////////////////////////////////////////////////////////////////

uint32_t millis(void)
{
    return sysTickUptime;
}

///////////////////////////////////////////////////////////////////////////////
// System Initialization
///////////////////////////////////////////////////////////////////////////////

void systemInit(bool overclock)
{
    //RCC_ClocksTypeDef rccClocks;
    int i;

    // start fpu
    SCB->CPACR = (0x3 << (10*2)) | (0x3 << (11*2));

    // Init cycle counter
    cycleCounterInit();

    // SysTick
    SysTick_Config(SystemCoreClock / 1000);

    RCC_ADCCLKConfig(RCC_ADC12PLLCLK_Div256);  // 72 MHz divided by 256 = 281.25 kHz

    // Turn on peripherial clocks
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_ADC12, ENABLE);

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);  // USART1, USART2
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);  // ADC2

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);


    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);  // PWM Out  + PWM RX
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);  // PWM Out
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);  // PWM Out  + PWM RX
//    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);  // i2c
//    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);  //
//
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);  // PPM + PWM RX
//    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);  //
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM15, ENABLE);  // PWM Out
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM16, ENABLE);  // PWM Out
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM17, ENABLE);  // PWM Out

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);  // Telemetry
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);  // GPS
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);  // Spektrum RX

    RCC_ClearFlag();

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  // 2 bits for pre-emption priority, 2 bits for subpriority

    gpioStart();

    spiInit(SPI2);
//    if feature(FEATURE_I2C)
//        i2cInit(I2C2);

    for (i = 0; i < 10; i++) {
        LED0_TOGGLE
        delay(25);
        BEEP_ON
        delay(25);
        BEEP_OFF
    }
    LED0_OFF

}

void failureMode(uint8_t mode)
{
    LED0_OFF
    while (1) {
        LED0_TOGGLE
        delay(475 * mode - 2);
        BEEP_ON
        delay(25);
        BEEP_OFF
    }
}
///////////////////////////////////////////////////////////////////////////////
// Delay Microseconds
///////////////////////////////////////////////////////////////////////////////

void delayMicroseconds(uint32_t us)
{
    uint32_t now = micros();
    while (micros() - now < us)
        ;
}

///////////////////////////////////////////////////////////////////////////////
// Delay Milliseconds
///////////////////////////////////////////////////////////////////////////////

void delay(uint32_t ms)
{
    while (ms--)
        delayMicroseconds(1000);
}

///////////////////////////////////////////////////////////////////////////////
// System Reset
///////////////////////////////////////////////////////////////////////////////

#define AIRCR_VECTKEY_MASK    ((uint32_t)0x05FA0000)

void systemReset(bool toBootloader)
{
    if (toBootloader) {
        // 1FFFF000 -> 20000200 -> SP
        // 1FFFF004 -> 1FFFF021 -> PC
        *((uint32_t *)0x20009FFC) = 0xDEADBEEF; // 40KB SRAM STM32F30X
    }

    // Generate system reset
    SCB->AIRCR = AIRCR_VECTKEY_MASK | (uint32_t)0x04;
    while (1);
}

void systemUnPause(void)
{
    uartUnPause(3);

}
void systemPause(void)
{
    uartPause(3);
}
