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

//#define SOFT_I2C

#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <ctype.h>
#include <string.h>
#include <stdio.h>

#include "printf.h"

#define SENSORS_SET (SENSOR_GYRO | SENSOR_ACC | SENSOR_BARO | SENSOR_MAG)
#define GYRO
#define ACC
#define MAG
#define BARO
//#define LEDRING
//#define SONAR
#define BUZZER
#define LED0





// 96-bit Chip Unique ID on st F103/F303
#define U_ID_0 (*(uint32_t*)0x1FFFF7E8)
#define U_ID_1 (*(uint32_t*)0x1FFFF7EC)
#define U_ID_2 (*(uint32_t*)0x1FFFF7F0)

typedef enum {
    SENSOR_GYRO = 1 << 0, // always present
    SENSOR_ACC = 1 << 1,
    SENSOR_BARO = 1 << 2,
    SENSOR_MAG = 1 << 3,
    SENSOR_SONAR = 1 << 4,
    SENSOR_GPS = 1 << 5,
    SENSOR_GPSMAG = 1 << 6,
} AvailableSensors;

// Type of accelerometer used/detected
typedef enum AccelSensors {
    ACC_DEFAULT = 0,
    ACC_ADXL345 = 1,
    ACC_MPU6050 = 2,
    ACC_MMA8452 = 3,
    ACC_BMA280 = 4,
    ACC_NONE = 5
} AccelSensors;

typedef enum {
    FEATURE_PPM = 1 << 0,
    FEATURE_VBAT = 1 << 1,
    FEATURE_INFLIGHT_ACC_CAL = 1 << 2,
    FEATURE_SERIALRX = 1 << 3,
    FEATURE_MOTOR_STOP = 1 << 4,
    FEATURE_SERVO_TILT = 1 << 5,
    FEATURE_SOFTSERIAL = 1 << 6,
    FEATURE_LED_RING = 1 << 7,
    FEATURE_GPS = 1 << 8,
    FEATURE_FAILSAFE = 1 << 9,
    FEATURE_SONAR = 1 << 10,
    FEATURE_TELEMETRY = 1 << 11,
    FEATURE_POWERMETER = 1 << 12,
    FEATURE_VARIO = 1 << 13,
    FEATURE_3D = 1 << 14,
    FEATURE_I2C = 1 << 15,
    FEATURE_AF = 1 << 16,
} AvailableFeatures;

typedef enum {
    SERIALRX_SPEKTRUM1024 = 0,
    SERIALRX_SPEKTRUM2048 = 1,
    SERIALRX_SBUS = 2,
    SERIALRX_SUMD = 3,
    SERIALRX_MSP = 4,
    SERIALRX_PROVIDER_MAX = SERIALRX_MSP,
} SerialRXType;

typedef enum {
    GPS_NMEA = 0,
    GPS_UBLOX,
    GPS_MTK_NMEA,
    GPS_MTK_BINARY,
    GPS_MAG_BINARY,
    GPS_HARDWARE_MAX = GPS_MAG_BINARY,
} GPSHardware;

typedef enum {
    GPS_BAUD_115200 = 0,
    GPS_BAUD_57600,
    GPS_BAUD_38400,
    GPS_BAUD_19200,
    GPS_BAUD_9600,
    GPS_BAUD_MAX = GPS_BAUD_9600
} GPSBaudRates;

typedef enum {
    TELEMETRY_PROVIDER_MSP = 0,
    TELEMETRY_PROVIDER_FRSKY,
    TELEMETRY_PROVIDER_HOTT,
    TELEMETRY_PROVIDER_MAX = TELEMETRY_PROVIDER_HOTT
} TelemetryProvider;

typedef enum {
    TELEMETRY_PORT_UART = 0,
    TELEMETRY_PORT_SOFTSERIAL_1, // Requires FEATURE_SOFTSERIAL
    TELEMETRY_PORT_SOFTSERIAL_2, // Requires FEATURE_SOFTSERIAL
    TELEMETRY_PORT_MAX = TELEMETRY_PORT_SOFTSERIAL_2
} TelemetryPort;

typedef enum {
    X = 0,
    Y,
    Z
} sensor_axis_e;

typedef enum {
    ALIGN_DEFAULT = 0,                                      // driver-provided alignment
    CW0_DEG = 1,
    CW90_DEG = 2,
    CW180_DEG = 3,
    CW270_DEG = 4,
    CW0_DEG_FLIP = 5,
    CW90_DEG_FLIP = 6,
    CW180_DEG_FLIP = 7,
    CW270_DEG_FLIP = 8
} sensor_align_e;

enum {
    GYRO_UPDATED = 1 << 0,
    ACC_UPDATED = 1 << 1,
    MAG_UPDATED = 1 << 2,
    TEMP_UPDATED = 1 << 3
};

typedef struct sensor_data_t {
    int16_t gyro[3];
    int16_t acc[3];
    int16_t mag[3];
    float temperature;
    int updated;
} sensor_data_t;

typedef void (*sensorInitFuncPtr)(sensor_align_e align);   // sensor init prototype
typedef bool (*sensorReadFuncPtr)(int16_t *data);          // sensor read and align prototype
typedef void (*baroOpFuncPtr)(void);                       // baro start operation
typedef void (*baroCalculateFuncPtr)(int32_t *pressure, int32_t *temperature);             // baro calculation (filled params are pressure and temperature)

typedef uint16_t (*rcReadRawDataPtr)(uint8_t chan);        // used by receiver driver to return channel data
typedef void (*pidControllerFuncPtr)(void);                // pid controller function prototype

typedef struct sensor_t {
    sensorInitFuncPtr init;                                 // initialize function
    sensorReadFuncPtr read;                                 // read 3 axis data function
    sensorReadFuncPtr temperature;                          // read temperature if available
    float scale;                                            // scalefactor (currently used for gyro only, todo for accel)
} sensor_t;

typedef struct baro_t {
    uint16_t ut_delay;
    uint16_t up_delay;
    baroOpFuncPtr start_ut;
    baroOpFuncPtr get_ut;
    baroOpFuncPtr start_up;
    baroOpFuncPtr get_up;
    baroCalculateFuncPtr calculate;
} baro_t;

///////////////////////////////////////////////////////////////////////////////

#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <ctype.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <errno.h>

///////////////////////////////////////////////////////////////////////////////

#if defined(NAZEPRO)

#include "stm32f30x.h"
#include "stm32f30x_conf.h"

#include "usb_lib.h"
#include "vcp/usb_desc.h"
#include "vcp/usb_pwr.h"

#define BEEP_PIN    GPIO_Pin_10
#define BEEP_GPIO   GPIOB

#define LED0_PIN    GPIO_Pin_12
#define LED0_GPIO   GPIOB

#define LED1_PIN    GPIO_Pin_12
#define LED1_GPIO   GPIOB

#define MASTER_SPEKTRUM_UART_PIN       GPIO_Pin_11
#define MASTER_SPEKTRUM_UART_GPIO      GPIOB
#define MASTER_SPEKTRUM_UART_PINSOURCE GPIO_PinSource11

#define UART1_TX_PIN GPIO_Pin_9
#define UART1_RX_PIN GPIO_Pin_10
#define UART1_GPIO GPIOA
#define UART1_TX_PINSOURCE GPIO_PinSource9
#define UART1_RX_PINSOURCE GPIO_PinSource10

#define UART2_TX_PIN        GPIO_Pin_3
#define UART2_RX_PIN        GPIO_Pin_15
#define UART2_TX_GPIO       GPIOB
#define UART2_RX_GPIO       GPIOA
#define UART2_TX_PINSOURCE  GPIO_PinSource3
#define UART2_RX_PINSOURCE  GPIO_PinSource15

#define UART_HEADER_RXTX USART2
#define UART_HEADER_RC USART1
#define UART_HEADER_FLEX USART3

#endif


#if defined(NAZE)
#include "stm32f10x.h"
#include "stm32f10x_conf.h"
#define LED0_GPIO   GPIOB
#define LED0_PIN    Pin_3 // PB3 (LED)
#define LED1_GPIO   GPIOB
#define LED1_PIN    Pin_4 // PB4 (LED)
#define BEEP_GPIO   GPIOA
#define BEEP_PIN    Pin_12 // PA12 (Buzzer)
#define BARO_GPIO   GPIOC
#define BARO_PIN    Pin_13

// Fixme
#define MASTER_SPEKTRUM_UART_PIN       GPIO_Pin_11
#define MASTER_SPEKTRUM_UART_GPIO      GPIOB
#define MASTER_SPEKTRUM_UART_PINSOURCE GPIO_PinSource11
#define UART1_TX_PIN GPIO_Pin_9
#define UART1_RX_PIN GPIO_Pin_10
#define UART1_GPIO GPIOA
#define UART1_TX_PINSOURCE GPIO_PinSource9
#define UART1_RX_PINSOURCE GPIO_PinSource10

#define UART2_TX_PIN        GPIO_Pin_3
#define UART2_RX_PIN        GPIO_Pin_15
#define UART2_TX_GPIO       GPIOB
#define UART2_RX_GPIO       GPIOA
#define UART2_TX_PINSOURCE  GPIO_PinSource3
#define UART2_RX_PINSOURCE  GPIO_PinSource15

#define UART_HEADER_RXTX    USART1 //
#define UART_HEADER_RC      USART2 // rc pin 4
#define UART_HEADER_FLEX    USART2 // rc pin 4

#endif

///////////////////////////////////////////////////////////////////////////////




#include "drv/adc.h"
#include "drv/crc.h"
#include "drv/gpio.h"
#include "drv/i2c.h"
#include "drv/pwm.h"
#include "drv/serial.h"
#include "drv/softserial.h"
#include "drv/spi.h"
#include "drv/system.h"
#include "drv/timer.h"
#include "drv/uart.h"
#include "drv/usb.h"


#include "sensors.h"

#include "sensors/i2c_adxl345.h"
#include "sensors/i2c_bma280.h"
#include "sensors/i2c_bmp085.h"
#include "sensors/i2c_hmc5883l.h"
#include "sensors/i2c_l3g4200d.h"
#include "sensors/i2c_ledring.h"
#include "sensors/i2c_mma845x.h"
#include "sensors/i2c_mpu3050.h"
#include "sensors/i2c_mpu6050.h"
#include "sensors/i2c_ms5611.h"
#include "sensors/spi_hmc5983.h"
#include "sensors/spi_mpu6000.h"
#include "sensors/spi_ms5611.h"

#include "sensors/spi_hmc5983.h"
#include "sensors/spi_mpu6000.h"
#include "sensors/spi_ms5611.h"
#include "sensors/spi_eeprom.h"


#define BOARD_I2C_PORT I2C2
