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
#define SPEK_FRAME_SIZE 16

#define MAX_SPEKTRUM_CHANNELS       7

uint32_t spekChannelData[MAX_SPEKTRUM_CHANNELS];

static uint8_t spek_chan_shift;
static uint8_t spek_chan_mask;
static bool rcFrameComplete = false;
static bool spekHiRes = false;
static bool spekDataIncoming = false;
volatile uint8_t spekFrame[SPEK_FRAME_SIZE];

uint16_t spektrumRead(uint8_t channel);

static inline void spektrumParser(uint8_t c)
{

    uint32_t spekTime;
    static uint32_t spekTimeLast, spekTimeInterval;
    static uint8_t spekFramePosition;

    spekDataIncoming = true;
    spekTime = micros();
    spekTimeInterval = spekTime - spekTimeLast;
    spekTimeLast = spekTime;
    if (spekTimeInterval > 5000)
        spekFramePosition = 0;
    spekFrame[spekFramePosition] = (uint8_t)c;
    if (spekFramePosition == SPEK_FRAME_SIZE - 1) {
        rcFrameComplete = true;
        failsafeCnt = 0;   // clear FailSafe counter
    } else {
        spekFramePosition++;
    }

}

bool spektrumFrameComplete(void)
{
    return rcFrameComplete;
}


void spektrumInit(rcReadRawDataPtr *callback)
{
    switch (mcfg.serialrx_type) {
        case SERIALRX_SPEKTRUM2048:
            // 11 bit frames
            spek_chan_shift = 3;
            spek_chan_mask = 0x07;
            spekHiRes = true;
            break;
        case SERIALRX_SPEKTRUM1024:
            // 10 bit frames
            spek_chan_shift = 2;
            spek_chan_mask = 0x03;
            spekHiRes = false;
            break;
    }

    core.rcvrport = uartOpen(UART_HEADER_FLEX, spektrumParser, 115200, MODE_RX, SERIAL_NOT_INVERTED);
       if (callback)
           *callback = spektrumRead;

}

uint16_t spektrumRead(uint8_t chan)
{
    uint16_t data;
    static uint32_t spekChannelData[MAX_SPEKTRUM_CHANNELS];
    uint8_t b;

    if (rcFrameComplete) {
        for (b = 3; b < SPEK_FRAME_SIZE; b += 2) {
            uint8_t spekChannel = 0x0F & (spekFrame[b - 1] >> spek_chan_shift);
            if (spekChannel < MAX_SPEKTRUM_CHANNELS)
                spekChannelData[spekChannel] = ((uint32_t)(spekFrame[b - 1] & spek_chan_mask) << 8) + spekFrame[b];
        }
        rcFrameComplete = false;
    }

    if (chan >= MAX_SPEKTRUM_CHANNELS || !spekDataIncoming) {
        data = mcfg.midrc;
    } else {
        if (spekHiRes)
            data = 988 + (spekChannelData[mcfg.rcmap[chan]] >> 1);   // 2048 mode
        else
            data = 988 + spekChannelData[mcfg.rcmap[chan]];          // 1024 mode
    }

    return data;
}

