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


#if defined(QUANTON)
#define MPU6000_CS_GPIO       GPIOC
#define MPU6000_CS_PIN        GPIO_Pin_4
#define MPU6000_INT_GPIO      GPIOC
#define MPU6000_INT_PIN       GPIO_Pin_0

#else
#define MPU6000_CS_GPIO       GPIOC
#define MPU6000_CS_PIN        GPIO_Pin_15
#define MPU6000_INT_GPIO      GPIOB
#define MPU6000_INT_PIN       GPIO_Pin_1
#endif

#define DISABLE_MPU6000       GPIO_SetBits(MPU6000_CS_GPIO,   MPU6000_CS_PIN)
#define ENABLE_MPU6000        GPIO_ResetBits(MPU6000_CS_GPIO, MPU6000_CS_PIN)


///////////////////////////////////////

#define MPU6000_CONFIG		    	0x1A

#define BITS_DLPF_CFG_256HZ         0x00
#define BITS_DLPF_CFG_188HZ         0x01
#define BITS_DLPF_CFG_98HZ          0x02
#define BITS_DLPF_CFG_42HZ          0x03

#define GYRO_SCALE_FACTOR  0.00053292f  // (4/131) * pi/180   (32.75 LSB = 1 DPS)


bool mpu6000DetectSpi(sensor_t *acc, sensor_t *gyro, uint16_t lpf );

bool mpu6000GyroRead(int16_t *gyroData);
bool mpu6000AccRead(int16_t *gyroData);
