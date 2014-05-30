/*
 Copyright (c) 2013 John Ihlein.  All rights reserved.

 Open Source STM32 Based Multicopter Controller Software

 Designed to run on the Naze32Pro Flight Control Board


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

///////////////////////////////////////////////////////////////////////////////
// TODO fixme HMC5983_SELF_TEST_GAUSS
#define HMC5983_X_SELF_TEST_GAUSS (+1.16f) // X axis level when bias current is applied.
#define HMC5983_Y_SELF_TEST_GAUSS (+1.16f) // Y axis level when bias current is applied.
#define HMC5983_Z_SELF_TEST_GAUSS (+1.08f) // Z axis level when bias current is applied.

#define HMC5983_ADDRESS 0x1E

#define HMC5983_CONFIG_REG_A     0x00
#define HMC5983_CONFIG_REG_B     0x01
#define HMC5983_MODE_REG         0x02
#define HMC5983_DATA_X_MSB_REG   0x03
#define HMC5983_STATUS_REG       0x09
#define HMC5983_TEMP_OUT_MSB_REG 0x31

///////////////////////////////////////////////////////////////////////////////
#define TS                               0x80  // Temperature compensation enable
#define SENSOR_CONFIG                    0x70  // 8 Sample average, 15 Hz
#define NORMAL_MEASUREMENT_CONFIGURATION 0x00
#define POSITIVE_BIAS_CONFIGURATION      0x01
#define NEGATIVE_BIAS_CONFIGURATION      0x02
///////////////////////////////////////////////////////////////////////////////

//#define SENSOR_GAIN 0x00  // +/- 0.88 Ga
#define SENSOR_GAIN 0x20        // +/- 1.3  Ga (default)
//#define SENSOR_GAIN 0x40  // +/- 1.9  Ga
#define SENSOR_INIT_GAIN 0x60  // +/- 2.5  Ga
//#define SENSOR_GAIN 0x80  // +/- 4.0  Ga
//#define SENSOR_GAIN 0xA0  // +/- 4.7  Ga
//#define SENSOR_GAIN 0xC0  // +/- 5.6  Ga
//#define SENSOR_GAIN 0xE0  // +/- 8.1  Ga

#define OP_MODE_CONTINUOUS 0x00 // Continuous conversion
#define OP_MODE_SINGLE     0x01 // Single conversion

#define STATUS_RDY         0x01 // Data Ready

static float magGain[3] = { 1.0f, 1.0f, 1.0f };

static sensor_align_e magAlign = CW180_DEG;

bool readHMC5983(int16_t *magADC)
{
    uint8_t buf[6];
    int16_t mag[3];
    setSPIdivisor(HMC5983_SPI, 8);  // 4.5 MHz SPI clock

    ENABLE_HMC5983;
    spiTransfer(HMC5983_SPI, HMC5983_DATA_X_MSB_REG + 0x80 + 0x40);
    buf[0] = spiTransfer(HMC5983_SPI, 0x00);
    buf[1] = spiTransfer(HMC5983_SPI, 0x00);
    buf[2] = spiTransfer(HMC5983_SPI, 0x00);
    buf[3] = spiTransfer(HMC5983_SPI, 0x00);
    buf[4] = spiTransfer(HMC5983_SPI, 0x00);
    buf[5] = spiTransfer(HMC5983_SPI, 0x00);
    DISABLE_HMC5983;

    mag[X] = (int16_t)(buf[0] << 8 | buf[1]) * magGain[X];
    mag[Z] = (int16_t)(buf[2] << 8 | buf[3]) * magGain[Z];
    mag[Y] = (int16_t)(buf[4] << 8 | buf[5]) * magGain[Y];

    alignSensors(mag, magADC, magAlign);
    // check for valid data
    if (mag[X] == -4096 || mag[Y] == -4096 || mag[Z] == -4096) {
        return false;
    } else {
        return true;
    }
}

void hmc5983Init(sensor_align_e align)
{
    if (align > 0)
        magAlign = align;
}
///////////////////////////////////////////////////////////////////////////////
// Initialize Magnetometer
///////////////////////////////////////////////////////////////////////////////

bool hmc5983DetectSpi(sensor_t *mag, sensor_align_e align)
{
    int16_t magADC[3];
    int32_t xyz_total[3] = { 0, 0, 0 }; // 32 bit totals so they won't overflow.

    spiResetErrorCounter(HMC5983_SPI);

    uint8_t hmc5983Status = 0;
    uint8_t i, calibrationSteps = 2;
    setSPIdivisor(HMC5983_SPI, 8);  // 4.5 MHz SPI clock

    ENABLE_HMC5983;
    spiTransfer(HMC5983_SPI, HMC5983_CONFIG_REG_A);
    spiTransfer(HMC5983_SPI, TS | SENSOR_CONFIG | POSITIVE_BIAS_CONFIGURATION);
    DISABLE_HMC5983;

    delay(50);

    ENABLE_HMC5983;
    spiTransfer(HMC5983_SPI, HMC5983_CONFIG_REG_B);
    spiTransfer(HMC5983_SPI, SENSOR_INIT_GAIN);
    DISABLE_HMC5983;

    delay(20);

    readHMC5983(magADC);

    for (i = 0; i < 10; i++) {
        ENABLE_HMC5983;
        spiTransfer(HMC5983_SPI, HMC5983_MODE_REG);
        spiTransfer(HMC5983_SPI, OP_MODE_SINGLE);
        DISABLE_HMC5983;

        delay(20);

        while ((hmc5983Status && STATUS_RDY) == 0x00) {
            ENABLE_HMC5983;
            spiTransfer(HMC5983_SPI, HMC5983_STATUS_REG + 0x80);
            hmc5983Status = spiTransfer(HMC5983_SPI, 0x00);
            DISABLE_HMC5983;
        }
        if (!readHMC5983(magADC)) // check for valid data
            break;          // breaks out of the for loop if sensor saturated.
        xyz_total[X] += magADC[X];
        xyz_total[Y] += magADC[Y];
        xyz_total[Z] += magADC[Z];
    }
    if (i == 10) // loop completed
        calibrationSteps--;

    ENABLE_HMC5983;
    spiTransfer(HMC5983_SPI, HMC5983_CONFIG_REG_A);
    spiTransfer(HMC5983_SPI, SENSOR_CONFIG | NEGATIVE_BIAS_CONFIGURATION);
    DISABLE_HMC5983;

    delay(20);
    readHMC5983(magADC);

    for (i = 0; i < 10; i++) {
        ENABLE_HMC5983;
        spiTransfer(HMC5983_SPI, HMC5983_MODE_REG);
        spiTransfer(HMC5983_SPI, OP_MODE_SINGLE);
        DISABLE_HMC5983;

        delay(20);

        while ((hmc5983Status && STATUS_RDY) == 0x00) {
            ENABLE_HMC5983;
            spiTransfer(HMC5983_SPI, HMC5983_STATUS_REG + 0x80);
            hmc5983Status = spiTransfer(HMC5983_SPI, 0x00);
            DISABLE_HMC5983;
        }
        if (!readHMC5983(magADC))
            break;
        xyz_total[X] += magADC[X];
        xyz_total[Y] += magADC[Y];
        xyz_total[Z] += magADC[Z];
    }
    if (i == 10) // loop completed
        calibrationSteps--;

    ENABLE_HMC5983;
    spiTransfer(HMC5983_SPI, HMC5983_CONFIG_REG_A);
    spiTransfer(HMC5983_SPI, TS | SENSOR_CONFIG | NORMAL_MEASUREMENT_CONFIGURATION);
    DISABLE_HMC5983;

    ENABLE_HMC5983;
    spiTransfer(HMC5983_SPI, HMC5983_CONFIG_REG_B);
    spiTransfer(HMC5983_SPI, SENSOR_GAIN);
    DISABLE_HMC5983;

    delay(50);

    ENABLE_HMC5983;
    spiTransfer(HMC5983_SPI, HMC5983_MODE_REG);
    spiTransfer(HMC5983_SPI, OP_MODE_CONTINUOUS);
    DISABLE_HMC5983;

    delay(20);

    readHMC5983(magADC);

    if ((((int8_t)magADC[1]) == -1 && ((int8_t)magADC[0]) == -1) || spiGetErrorCounter(HMC5983_SPI) != 0) {
        spiResetErrorCounter(HMC5983_SPI);
        return false;
    }
//    magGain[X] = fabsf(660.0f * HMC5983_X_SELF_TEST_GAUSS * 2.0f * 10.0f / xyz_total[X]);
//    magGain[Y] = fabsf(660.0f * HMC5983_Y_SELF_TEST_GAUSS * 2.0f * 10.0f / xyz_total[Y]);
//    magGain[Z] = fabsf(660.0f * HMC5983_Z_SELF_TEST_GAUSS * 2.0f * 10.0f / xyz_total[Z]);

    mag->init = hmc5983Init;
    mag->read = readHMC5983;

    return (calibrationSteps == 0);
}

