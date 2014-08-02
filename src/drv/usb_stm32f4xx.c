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
//__ALIGN_BEGIN
USB_OTG_CORE_HANDLE    USB_OTG_dev;
//__ALIGN_END;
uint8_t usbDeviceConfigured = false;
#define USB_TIMEOUT  50

static uartPort_t uartPortUSB;

void usbSetBaudRate(serialPort_t *instance, uint32_t baudRate)
{
// TODO restart usb with baudrate
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
    return (VCP_DataRX_IsCharReady() != 0);
}

///////////////////////////////////////////////////////////////////////////////
// CLI Read
///////////////////////////////////////////////////////////////////////////////

uint8_t usbRead(serialPort_t *instance)
{
    uint8_t buf;

    if (VCP_get_char(&buf))
            return buf;
        else
            return(0);
}

///////////////////////////////////////////////////////////////////////////////
// CLI Print
///////////////////////////////////////////////////////////////////////////////

void usbPrintStr(const char *str)
{
    if (usbDeviceConfigured == true)
      {
        VCP_send_buffer((uint8_t*)str, strlen(str));
      }
}

void usbPrint(serialPort_t *instance, uint8_t c)
{
    if (usbDeviceConfigured == true)
        {
        VCP_send_buffer(&c, 1);
        }

}

const struct serialPortVTable usbVTable[] = { { usbPrint, usbAvailable, usbRead, usbSetBaudRate, isUsbTransmitBufferEmpty, usbSetMode, } };

serialPort_t *usbInit(void)
{
    uartPort_t *s;


#define USB_DISCONNECT_PIN GPIO_Pin_12
#define USB_DISCONNECT_GPIO GPIOA

    GPIO_InitTypeDef  GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Pin   = USB_DISCONNECT_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;

    GPIO_Init(USB_DISCONNECT_GPIO, &GPIO_InitStructure);

    GPIO_ResetBits(USB_DISCONNECT_GPIO, USB_DISCONNECT_PIN);

    delay(200);

    GPIO_SetBits(USB_DISCONNECT_GPIO, USB_DISCONNECT_PIN);

    USBD_Init(&USB_OTG_dev, USB_OTG_FS_CORE_ID, &USR_desc, &USBD_CDC_cb, &USR_cb);



    init_printf(NULL, usbPrintf);

    s = &uartPortUSB;
    s->port.vTable = usbVTable;
    usbDeviceConfigured = true;
    return (serialPort_t *)s;

}
