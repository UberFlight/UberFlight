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

#define USB_TIMEOUT  50

static uartPort_t uartPortUSB;

void usbSetBaudRate(serialPort_t *instance, uint32_t baudRate)
{
// TODO restart usb with baudrate
    ;
}

void usbSetMode(serialPort_t *instance, portMode_t mode)
{
    // TODO check if really nothing to do
}

bool isUsbTransmitBufferEmpty(serialPort_t *instance)
{
    return true;
}
static void usbPrintf(void *p, char c)
{
    usbPrint( NULL, c);
}

///////////////////////////////////////////////////////////////////////////////
// CLI Available
///////////////////////////////////////////////////////////////////////////////

uint32_t usbAvailable(serialPort_t *instance)
{
    return receiveLength;
}

///////////////////////////////////////////////////////////////////////////////
// CLI Read
///////////////////////////////////////////////////////////////////////////////

uint8_t usbRead(serialPort_t *instance)
{
    uint8_t buf[1];

    uint32_t rxed = 0;

    while (rxed < 1) {
        rxed += CDC_Receive_DATA((uint8_t*)buf + rxed, 1 - rxed);
    }

    return buf[0];
}

///////////////////////////////////////////////////////////////////////////////
// CLI Print
///////////////////////////////////////////////////////////////////////////////

void usbPrintStr(const char *str)
{
    uint32_t len;
    uint32_t oldTxed;
    uint32_t start;
    uint32_t txed;

    if (!(usbIsConnected() && usbIsConfigured()) || !str) {
        return;
    }

    len = strlen(str);

    txed = 0;
    oldTxed = 0;

    start = millis();

    while (txed < len && (millis() - start < USB_TIMEOUT)) {
        txed += CDC_Send_DATA((uint8_t*)str + txed, len - txed);

        if (oldTxed != txed) {
            start = millis();
        }

        oldTxed = txed;
    }
}

void usbPrint(serialPort_t *instance, uint8_t c)
{
    uint32_t txed;
    uint32_t start = millis();

    if (!(usbIsConnected() && usbIsConfigured())) {
        return;
    }

    do {
        txed = CDC_Send_DATA((uint8_t*)&c, 1);
    } while (txed < 1 && (millis() - start < USB_TIMEOUT));

}

const struct serialPortVTable usbVTable[] = { { usbPrint, usbAvailable, usbRead, usbSetBaudRate, isUsbTransmitBufferEmpty, usbSetMode, } };

serialPort_t *usbInit(void)
{
    uartPort_t *s;

    Set_System();
    Set_USBClock();
    USB_Interrupts_Config();
    USB_Init();
    init_printf(NULL, usbPrintf);

    s = &uartPortUSB;
    s->port.vTable = usbVTable;

    return (serialPort_t *)s;

}
