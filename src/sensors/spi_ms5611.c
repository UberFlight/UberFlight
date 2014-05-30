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

///////////////////////////////////////

//#define OSR  256  // 0.60 mSec conversion time (1666.67 Hz)
//#define OSR  512  // 1.17 mSec conversion time ( 854.70 Hz)
//#define OSR 1024  // 2.28 mSec conversion time ( 357.14 Hz)
//#define OSR 2048  // 4.54 mSec conversion time ( 220.26 Hz)
#define OSR 4096  // 9.04 mSec conversion time ( 110.62 Hz)

///////////////////////////////////////

//uint16andUint8_t
uint8_t c1[2], c2[2], c3[2], c4[2], c5[2], c6[2];

uint32_t d1, d2;

uint8_t rawADC[3];

int32_t dT;

int32_t ms5611Temperature;




///////////////////////////////////////////////////////////////////////////////
// Read Temperature Request Pressure
///////////////////////////////////////////////////////////////////////////////

void readTemperature(void)
{
    setSPIdivisor(MS5611_SPI, 2);  // 18 MHz SPI clock

    ENABLE_MS5611;
    spiTransfer(MS5611_SPI, 0x00);
    rawADC[2] = spiTransfer(MS5611_SPI, 0x00);
    rawADC[1] = spiTransfer(MS5611_SPI, 0x00);
    rawADC[0] = spiTransfer(MS5611_SPI, 0x00);
    DISABLE_MS5611;

    d2 = (rawADC[0] << 16) | (rawADC[1] << 8) | rawADC[2];
//    delayMicroseconds(1);
}

void readPressure(void)
{

    setSPIdivisor(MS5611_SPI, 2);  // 18 MHz SPI clock

    ENABLE_MS5611;
    spiTransfer(MS5611_SPI, 0x00);
    rawADC[2] = spiTransfer(MS5611_SPI, 0x00);
    rawADC[1] = spiTransfer(MS5611_SPI, 0x00);
    rawADC[0] = spiTransfer(MS5611_SPI, 0x00);
    DISABLE_MS5611;

    d1 = (rawADC[0] << 16) | (rawADC[1] << 8) | rawADC[2];

//    delayMicroseconds(1);

}

void requestTemperature(void)
{
    setSPIdivisor(MS5611_SPI, 2);  // 18 MHz SPI clock

    ENABLE_MS5611;                      // Request temperature conversion
#if   (OSR ==  256)
    spiTransfer(MS5611_SPI, 0x50);
#elif (OSR ==  512)
    spiTransfer(MS5611_SPI, 0x52);
#elif (OSR == 1024)
    spiTransfer(MS5611_SPI, 0x54);
#elif (OSR == 2048)
    spiTransfer(MS5611_SPI, 0x56);
#elif (OSR == 4096)
    spiTransfer(MS5611_SPI, 0x58);
#endif
    DISABLE_MS5611;

    delayMicroseconds(1);

}
void requestPressure(void)
{
    setSPIdivisor(MS5611_SPI, 2);  // 18 MHz SPI clock

    ENABLE_MS5611;                      // Request pressure conversion
#if   (OSR ==  256)
    spiTransfer(MS5611_SPI, 0x40);
#elif (OSR ==  512)
    spiTransfer(MS5611_SPI, 0x42);
#elif (OSR == 1024)
    spiTransfer(MS5611_SPI, 0x44);
#elif (OSR == 2048)
    spiTransfer(MS5611_SPI, 0x46);
#elif (OSR == 4096)
    spiTransfer(MS5611_SPI, 0x48);
#endif
    DISABLE_MS5611;

    delayMicroseconds(1);
}


///////////////////////////////////////////////////////////////////////////////
// Calculate Temperature
///////////////////////////////////////////////////////////////////////////////

int32_t calculateTemperature(void)
{
    dT = (int32_t)d2 - ((int32_t)c5 << 8);
    ms5611Temperature = 2000 + (int32_t)(((int64_t)dT * ( (c6[0] << 8) | c6[1] )) >> 23);
    return ms5611Temperature;
}

///////////////////////////////////////////////////////////////////////////////
// Calculate Pressure Altitude
///////////////////////////////////////////////////////////////////////////////

void calculatePressureAltitude(int32_t *pressure, int32_t *temperature)
{
    int64_t offset;
    int64_t offset2 = 0;
    int64_t sensitivity;
    int64_t sensitivity2 = 0;
    int64_t f;
    int32_t p;

    int32_t ms5611Temp2 = 0;

    offset = ((int64_t)( (c2[0] << 8) | c2[1] ) << 16) + (((int64_t)( (c4[0] << 8) | c4[1] ) * dT) >> 7);
    sensitivity = ((int64_t)( (c1[0] << 8) | c1[1] ) << 15) + (((int64_t)( (c3[0] << 8) | c3[1] ) * dT) >> 8);

    if (ms5611Temperature < 2000) {
        ms5611Temp2 = (dT * dT) >> 31;

        f = ms5611Temperature - 2000;
        f = f * f;
        offset2 = 5 * f >> 1;
        sensitivity2 = 5 * f >> 2;

        if (ms5611Temperature < -1500) {
            f = (ms5611Temperature + 1500);
            f = f * f;
            offset2 += 7 * f;
            sensitivity2 += 11 * f >> 1;
        }

        ms5611Temperature -= ms5611Temp2;

        offset -= offset2;
        sensitivity -= sensitivity2;
    }

    p = ((((int64_t)d1 * sensitivity) >> 21) - offset) >> 15;
    if (pressure)
           *pressure = p;
       if (temperature)
           *temperature = calculateTemperature();
//    return  (44330.0f * (1.0f - pow((float)p / 101325.0f, 1.0f / 5.255f)));
    //cliPrintF("%9.4f\n\r", sensors.pressureAlt50Hz);
}

///////////////////////////////////////////////////////////////////////////////
// Pressure Initialization
///////////////////////////////////////////////////////////////////////////////

bool ms5611DetectSpi(baro_t *baro)
{
    spiResetErrorCounter(MS5611_SPI);
    setSPIdivisor(MS5611_SPI, 2);  // 18 MHz SPI clock

    ENABLE_MS5611;   // Reset Device
    spiTransfer(MS5611_SPI, 0x1E);
    delay(3);
    DISABLE_MS5611;

    delayMicroseconds(1);

    ENABLE_MS5611;   // Read Calibration Data C1
    spiTransfer(MS5611_SPI, 0xA2);
    c1[1] = spiTransfer(MS5611_SPI, 0x00);
    c1[0] = spiTransfer(MS5611_SPI, 0x00);
    DISABLE_MS5611;

    delayMicroseconds(1);

    ENABLE_MS5611;   // Read Calibration Data C2
    spiTransfer(MS5611_SPI, 0xA4);
    c2[1] = spiTransfer(MS5611_SPI, 0x00);
    c2[0] = spiTransfer(MS5611_SPI, 0x00);
    DISABLE_MS5611;

    delayMicroseconds(1);

    ENABLE_MS5611;   // Read Calibration Data C3
    spiTransfer(MS5611_SPI, 0xA6);
    c3[1] = spiTransfer(MS5611_SPI, 0x00);
    c3[0] = spiTransfer(MS5611_SPI, 0x00);
    DISABLE_MS5611;

    delayMicroseconds(1);

    ENABLE_MS5611;   // Read Calibration Data C4
    spiTransfer(MS5611_SPI, 0xA8);
    c4[1] = spiTransfer(MS5611_SPI, 0x00);
    c4[0] = spiTransfer(MS5611_SPI, 0x00);
    DISABLE_MS5611;

    delayMicroseconds(1);

    ENABLE_MS5611;   // Read Calibration Data C5
    spiTransfer(MS5611_SPI, 0xAA);
    c5[1] = spiTransfer(MS5611_SPI, 0x00);
    c5[0] = spiTransfer(MS5611_SPI, 0x00);
    DISABLE_MS5611;

    delayMicroseconds(1);

    ENABLE_MS5611;   // Read Calibration Data C6
    spiTransfer(MS5611_SPI, 0xAC);
    c6[1] = spiTransfer(MS5611_SPI, 0x00);
    c6[0] = spiTransfer(MS5611_SPI, 0x00);
    DISABLE_MS5611;


    if (((int8_t )c6[1]) == -1 || spiGetErrorCounter(MS5611_SPI)!=0) {
        spiResetErrorCounter(MS5611_SPI);
        return false;
    }

    delay(10);

    requestTemperature();

    delay(10);

//    baro->start_ut = ms5611_start_ut;
//    baro->get_ut = ms5611_get_ut;
//    baro->start_up = ms5611_start_up;
//    baro->get_up = ms5611_get_up;
//    baro->calculate = ms5611_calculate;

    baro->ut_delay = 10000;
        baro->up_delay = 10000;
        baro->start_ut = requestTemperature;
        baro->get_ut = readTemperature;
        baro->start_up = requestPressure;
        baro->get_up = readPressure;
        baro->calculate = calculatePressureAltitude;


    return true;
}
