//*****************************************************************************
//
// usbstdio.h - Prototypes for the USB console functions.
//
// Copyright (c) 2007-2012 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 9453 of the Stellaris Firmware Development Package.
//
//*****************************************************************************
#include <stdbool.h>

#ifndef __USBSTDIO_H__
#define __USBSTDIO_H__

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
// If built for buffered operation, the following labels define the sizes of
// the USB Stdio transmit and receive buffers respectively.
//
//*****************************************************************************
#ifdef USB_BUFFERED
#ifndef USB_RX_BUFFER_SIZE
#define USB_RX_BUFFER_SIZE     128
#endif
#ifndef USB_TX_BUFFER_SIZE
#define USB_TX_BUFFER_SIZE     1024
#endif
#endif

//*****************************************************************************
//
// Flags used to pass commands from interrupt context to the main loop.
//
//*****************************************************************************
#define COMMAND_PACKET_RECEIVED 0x00000001
#define COMMAND_STATUS_UPDATE   0x00000002

extern volatile unsigned long g_ulFlags;
extern char * g_pcStatus;
extern volatile bool g_bUSBConfigured;

//*****************************************************************************
//
// Prototypes for the APIs.
//
//*****************************************************************************
extern void USBStdioInit(void);
extern int USBgets(char *pcBuf, unsigned long ulLen);
extern unsigned char USBgetc(void);
extern void USBprintf(const char *pcString, ...);
extern int USBwrite(const char *pcBuf, unsigned long ulLen);
#ifdef USB_BUFFERED
extern int USBPeek(unsigned char ucChar);
extern void USBFlushTx(bool bDiscard);
extern void USBFlushRx(void);
extern int USBRxBytesAvail(void);
extern int USBTxBytesFree(void);
extern void USBEchoSet(bool bEnable);
#endif

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif // __USBSTDIO_H__
