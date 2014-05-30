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

///////////////////////////////////////////////////////////////////////////////
// EEPROM Defines
////////////////////////////////////////////////////////////////////////////////

#define WRITE_ENABLE                    0x06
#define WRITE_DISABLE                   0x04
#define READ_STATUS_REGISTER            0x05
#define WRITE_STATUS_REGISTER           0x01
#define READ_DATA                       0x03
#define FAST_READ                       0x0B
#define FAST_READ_DUAL_OUTPUT           0x3B
#define FAST_READ_DUAL_IO               0xBB
#define PAGE_PROGRAM_256_BYTES          0x02
#define SECTOR_ERASE_4K                 0x20
#define BLOCK_ERASE_32KB                0x52
#define BLOCK_ERASE_64KB                0xD8
#define CHIP_ERASE                      0xC7
#define POWER_DOWN                      0xB9
#define RELEASE_POWER_DOWN              0xAB
#define MANUFACTURER_DEVICE_ID          0x90
#define MANUFACTURER_DEVICE_ID_DUAL_IO  0x92
#define JEDEC_ID                        0x9F
#define READ_UNIQUE_ID                  0x4B

#define EEPROM_SPI          SPI2

#define EEPROM_CS_GPIO      GPIOB
#define EEPROM_CS_PIN       GPIO_Pin_2
#define EEPROM_CS_GPIO_CLK  RCC_AHBPeriph_GPIOB

#define ENABLE_EEPROM       GPIO_ResetBits(EEPROM_CS_GPIO, EEPROM_CS_PIN)

#define DISABLE_EEPROM      GPIO_SetBits(EEPROM_CS_GPIO,   EEPROM_CS_PIN)


