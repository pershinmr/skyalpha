//*****************************************************************************
//
// usbstdio.c - Utility driver to provide simple USB console functions.
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

#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_uart.h"
#include "driverlib/debug.h"
#include "driverlib/interrupt.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/usb.h"
#include "driverlib/rom.h"
#include "utils/uartstdio.h"
#include "usblib/usblib.h"
#include "usblib/usbcdc.h"
#include "usblib/usb-ids.h"
#include "usblib/device/usbdevice.h"
#include "usblib/device/usbdcdc.h"
#include "usbstdio.h"
#include "usb_serial_structs.h"
#include "usb_dev_serial.h"

//*****************************************************************************
//
//! \addtogroup usbstdio_api
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
// Flags used to pass commands from USB interrupt context to the main loop.
//
//*****************************************************************************
volatile unsigned long g_ulFlags = 0;
char *g_pcStatus;

//*****************************************************************************
//
// Global flag indicating that a USB configuration has been set.
//
//*****************************************************************************
volatile bool g_bUSBConfigured = false;

//*****************************************************************************
//
// If buffered mode is defined, set aside USB Stdio RX and TX buffers and
// read/write pointers to control them.
//
//*****************************************************************************
#ifdef USB_BUFFERED

//*****************************************************************************
//
// This global controls whether or not we are echoing characters back to the
// transmitter.  By default, echo is enabled but if using this module as a
// convenient method of implementing a buffered serial interface over which
// you will be running an application protocol, you are likely to want to
// disable echo by calling USBEchoSet(false).
//
//*****************************************************************************
static bool g_bDisableEcho;

//*****************************************************************************
//
// USB Stdio output ring buffer.  Buffer is full if g_ulUSBTxReadIndex is one ahead of
// g_ulUSBTxWriteIndex.  Buffer is empty if the two indices are the same.
//
//*****************************************************************************
static unsigned char g_pcUSBStdioTxBuffer[USB_TX_BUFFER_SIZE];
static volatile unsigned long g_ulUSBTxWriteIndex = 0;
static volatile unsigned long g_ulUSBTxReadIndex = 0;

//*****************************************************************************
//
// USB Stdio input ring buffer.  Buffer is full if g_ulUSBTxReadIndex is one ahead of
// g_ulUSBTxWriteIndex.  Buffer is empty if the two indices are the same.
//
//*****************************************************************************
static unsigned char g_pcUSBStdioRxBuffer[USB_RX_BUFFER_SIZE];
static volatile unsigned long g_ulUSBRxWriteIndex = 0;
static volatile unsigned long g_ulUSBRxReadIndex = 0;

//*****************************************************************************
//
// Macros to determine number of free and used bytes in the USB Stdio transmit buffer.
//
//*****************************************************************************
#define TX_BUFFER_USED          (GetBufferCount(&g_ulUSBTxReadIndex,  \
                                                &g_ulUSBTxWriteIndex, \
                                                USB_TX_BUFFER_SIZE))
#define TX_BUFFER_FREE          (USB_TX_BUFFER_SIZE - TX_BUFFER_USED)
#define TX_BUFFER_EMPTY         (IsBufferEmpty(&g_ulUSBTxReadIndex,   \
                                               &g_ulUSBTxWriteIndex))
#define TX_BUFFER_FULL          (IsBufferFull(&g_ulUSBTxReadIndex,  \
                                              &g_ulUSBTxWriteIndex, \
                                              USB_TX_BUFFER_SIZE))
#define ADVANCE_TX_BUFFER_INDEX(Index) \
                                (Index) = ((Index) + 1) % USB_TX_BUFFER_SIZE

//*****************************************************************************
//
// Macros to determine number of free and used bytes in the USB Stdio receive buffer.
//
//*****************************************************************************
#define RX_BUFFER_USED          (GetBufferCount(&g_ulUSBRxReadIndex,  \
                                                &g_ulUSBRxWriteIndex, \
                                                USB_RX_BUFFER_SIZE))
#define RX_BUFFER_FREE          (USB_RX_BUFFER_SIZE - RX_BUFFER_USED)
#define RX_BUFFER_EMPTY         (IsBufferEmpty(&g_ulUSBRxReadIndex,   \
                                               &g_ulUSBRxWriteIndex))
#define RX_BUFFER_FULL          (IsBufferFull(&g_ulUSBRxReadIndex,  \
                                              &g_ulUSBRxWriteIndex, \
                                              USB_RX_BUFFER_SIZE))
#define ADVANCE_RX_BUFFER_INDEX(Index) \
                                (Index) = ((Index) + 1) % USB_RX_BUFFER_SIZE
#endif

//*****************************************************************************
//
// This global controls whether or not we are in the process of sending a USB
// packet.
//
//*****************************************************************************
static bool g_bUSBTxSending;

//*****************************************************************************
//
// The base address of the chosen USB.
//
//*****************************************************************************
static unsigned long g_ulBase = 0;

//*****************************************************************************
//
// A mapping from an integer between 0 and 15 to its ASCII character
// equivalent.
//
//*****************************************************************************
static const char * const g_pcHex = "0123456789abcdef";

//*****************************************************************************
//
// Internal function prototypes.
//
//*****************************************************************************
static void GetLineCoding(tLineCoding *psLineCoding);
#ifdef USB_BUFFERED
static void USBRxBufferPut(void);
static bool USBRxCharPutNonBlocking(char ucData);
#else
static void USBCharPut(unsigned char ucData);
static char USBCharGet(void);
#endif

//*****************************************************************************
//
//! Determines whether the ring buffer whose pointers and size are provided
//! is full or not.
//!
//! \param pulRead points to the read index for the buffer.
//! \param pulWrite points to the write index for the buffer.
//! \param ulSize is the size of the buffer in bytes.
//!
//! This function is used to determine whether or not a given ring buffer is
//! full.  The structure of the code is specifically to ensure that we do not
//! see warnings from the compiler related to the order of volatile accesses
//! being undefined.
//!
//! \return Returns \b true if the buffer is full or \b false otherwise.
//
//*****************************************************************************
#ifdef USB_BUFFERED
static bool
IsBufferFull(volatile unsigned long *pulRead,
             volatile unsigned long *pulWrite, unsigned long ulSize)
{
    unsigned long ulWrite;
    unsigned long ulRead;

    ulWrite = *pulWrite;
    ulRead = *pulRead;

    return((((ulWrite + 1) % ulSize) == ulRead) ? true : false);
}
#endif

//*****************************************************************************
//
//! Determines whether the ring buffer whose pointers and size are provided
//! is empty or not.
//!
//! \param pulRead points to the read index for the buffer.
//! \param pulWrite points to the write index for the buffer.
//!
//! This function is used to determine whether or not a given ring buffer is
//! empty.  The structure of the code is specifically to ensure that we do not
//! see warnings from the compiler related to the order of volatile accesses
//! being undefined.
//!
//! \return Returns \b true if the buffer is empty or \b false otherwise.
//
//*****************************************************************************
#ifdef USB_BUFFERED
static bool
IsBufferEmpty(volatile unsigned long *pulRead,
              volatile unsigned long *pulWrite)
{
    unsigned long ulWrite;
    unsigned long ulRead;

    ulWrite = *pulWrite;
    ulRead = *pulRead;

    return((ulWrite  == ulRead) ? true : false);
}
#endif

//*****************************************************************************
//
//! Determines the number of bytes of data contained in a USB Stdio ring buffer.
//!
//! \param pulRead points to the read index for the buffer.
//! \param pulWrite points to the write index for the buffer.
//! \param ulSize is the size of the buffer in bytes.
//!
//! This function is used to determine how many bytes of data a given ring
//! buffer currently contains.  The structure of the code is specifically to
//! ensure that we do not see warnings from the compiler related to the order
//! of volatile accesses being undefined.
//!
//! \return Returns the number of bytes of data currently in the USB Stdio buffer.
//
//*****************************************************************************
#ifdef USB_BUFFERED
static unsigned long
GetBufferCount(volatile unsigned long *pulRead,
               volatile unsigned long *pulWrite, unsigned long ulSize)
{
    unsigned long ulWrite;
    unsigned long ulRead;

    ulWrite = *pulWrite;
    ulRead = *pulRead;

    return((ulWrite >= ulRead) ? (ulWrite - ulRead) :
                                 (ulSize - (ulRead - ulWrite)));
}
#endif

//*****************************************************************************
//
// Take as many bytes from the USB Stdio transmit buffer as we have space for
// and move them into the USB transmit buffer.
//
//*****************************************************************************
#ifdef USB_BUFFERED
static void USBPrimeTransmit(void)
{
    //
    // Do we have any data to transmit?
    //
    if (!TX_BUFFER_EMPTY && !g_bUSBTxSending)   // don't move data if in process of sending USB packet
    {
        //
        // Disable the USB interrupt. If we don't do this there is a race
        // condition which can cause the read index to be corrupted.
        //
        MAP_IntDisable(INT_USB0);

        g_bUSBTxSending = true;     // indicate in process of sending USB packet

        //
        // Yes - take some characters out of the USB Stdio transmit buffer and feed
        // them to the USB transmit buffer.
        //
        while (USBBufferSpaceAvailable((tUSBBuffer *)&g_sTxBuffer) && !TX_BUFFER_EMPTY)
        {
            USBBufferWrite((tUSBBuffer *)&g_sTxBuffer, &g_pcUSBStdioTxBuffer[g_ulUSBTxReadIndex], 1);
            ADVANCE_TX_BUFFER_INDEX(g_ulUSBTxReadIndex);
        }

        //
        // Reenable the USB interrupt.
        //
        MAP_IntEnable(INT_USB0);
    }
}
#endif

//*****************************************************************************
//
//! Configures the USB console.
//!
//! This function must be called prior to using any of the other USB console
//! functions: USBprintf() or USBgets().  This function assumes that the
//! caller has previously configured the relevant USB pins for operation as a
//! USB rather than as GPIOs.
//!
//! \return None.
//
//*****************************************************************************
void USBStdioInit(void)
{
#ifdef USB_BUFFERED
    //
    // In buffered mode, we only allow a single instance to be opened.
    //
    ASSERT(g_ulBase == 0);
#endif

    //
    // Initialize the USB transmit and receive buffers.
    //
    USBBufferInit((tUSBBuffer *)&g_sTxBuffer);
    USBBufferInit((tUSBBuffer *)&g_sRxBuffer);

    //
    // Set the USB stack mode to Device mode with VBUS monitoring.
    //
    USBStackModeSet(0, USB_MODE_DEVICE, 0);

    //
    // Pass our device information to the USB library and place the device
    // on the bus.
    //
    USBDCDCInit(0, (tUSBDCDCDevice *)&g_sCDCDevice);

    //
    // Select the base address of the USB port.
    //
    g_ulBase = USB0_BASE;

#ifdef USB_BUFFERED
    //
    // Flush both the USB Stdio buffers.
    //
    USBFlushRx();
    USBFlushTx(true);
    
    g_bUSBTxSending = false;        // not sending a USB packet
#endif
}

//*****************************************************************************
//
//! Writes a string of characters to the USB output.
//!
//! \param pcBuf points to a buffer containing the string to transmit.
//! \param ulLen is the length of the string to transmit.
//!
//! This function will transmit the string to the USB output.  The number of
//! characters transmitted is determined by the \e ulLen parameter.  This
//! function does no interpretation or translation of any characters.  Since
//! the output is sent to the USB CDC, any LF (/n) characters encountered will be
//! replaced with a CRLF pair.
//!
//! Besides using the \e ulLen parameter to stop transmitting the string, if a
//! null character (0) is encountered, then no more characters will be
//! transmitted and the function will return.
//!
//! In non-buffered mode, this function is blocking and will not return until
//! all the characters have been written to the output.  In buffered mode,
//! the characters are written to the USB Stdio transmit buffer and the call returns
//! immediately.  If insufficient space remains in the USB Stdio transmit buffer,
//! additional characters are discarded.
//!
//! \return Returns the count of characters written.
//
//*****************************************************************************
int
USBwrite(const char *pcBuf, unsigned long ulLen)
{
#ifdef USB_BUFFERED
    unsigned int uIdx;

    //
    // Check for valid arguments.
    //
    ASSERT(pcBuf != 0);
    ASSERT(g_ulBase != 0);

    //
    // Send the characters
    //
    for(uIdx = 0; uIdx < ulLen; uIdx++)
    {
        //
        // If the character to the USB port is \n, then add a \r before it so that
        // \n is translated to \r\n in the output.
        //
        if(pcBuf[uIdx] == '\n')
        {
            if(!TX_BUFFER_FULL)
            {
                g_pcUSBStdioTxBuffer[g_ulUSBTxWriteIndex] = '\r';
                ADVANCE_TX_BUFFER_INDEX(g_ulUSBTxWriteIndex);
            }
            else
            {
                //
                // Buffer is full - discard remaining characters and return.
                //
                break;
            }
        }

        //
        // Send the character to the USB Stdio output buffer.
        //
        if(!TX_BUFFER_FULL)
        {
            g_pcUSBStdioTxBuffer[g_ulUSBTxWriteIndex] = pcBuf[uIdx];
            ADVANCE_TX_BUFFER_INDEX(g_ulUSBTxWriteIndex);
        }
        else
        {
            //
            // Buffer is full - discard remaining characters and return.
            //
            break;
        }
    }

    //
    // If we have anything in the buffer, make sure that the USB is set
    // up to transmit it.
    //
    if(!TX_BUFFER_EMPTY)
    {
        USBPrimeTransmit();
    }

    //
    // Return the number of characters written.
    //
    return(uIdx);
#else
    unsigned int uIdx;

    //
    // Check for valid USB base address, and valid arguments.
    //
    ASSERT(g_ulBase != 0);
    ASSERT(pcBuf != 0);

    //
    // Send the characters
    //
    for(uIdx = 0; uIdx < ulLen; uIdx++)
    {
        //
        // If the character to the USB port is \n, then add a \r before it so that
        // \n is translated to \r\n in the output.
        //
        if(pcBuf[uIdx] == '\n')
        {
            USBCharPut('\r');
        }

        //
        // Send the character to the USB output.
        //
        USBCharPut(pcBuf[uIdx]);
    }

    //
    // Return the number of characters written.
    //
    return(uIdx);
#endif
}

//*****************************************************************************
//
//! A simple USB based get string function, with some line processing.
//!
//! \param pcBuf points to a buffer for the incoming string from the USB port.
//! \param ulLen is the length of the buffer for storage of the string,
//! including the trailing 0.
//!
//! This function will receive a string from the USB input and store the
//! characters in the buffer pointed to by \e pcBuf.  The characters will
//! continue to be stored until a termination character is received.  The
//! termination characters are CR, LF, or ESC.  A CRLF pair is treated as a
//! single termination character.  The termination characters are not stored in
//! the string.  The string will be terminated with a 0 and the function will
//! return.
//!
//! In both buffered and unbuffered modes, this function will block until
//! a termination character is received.  If non-blocking operation is required
//! in buffered mode, a call to USBPeek() may be made to determine whether
//! a termination character already exists in the receive buffer prior to
//! calling USBgets().
//!
//! Since the string will be null terminated, the user must ensure that the
//! buffer is sized to allow for the additional null character.
//!
//! \return Returns the count of characters that were stored, not including
//! the trailing 0.
//
//*****************************************************************************
int
USBgets(char *pcBuf, unsigned long ulLen)
{
#ifdef USB_BUFFERED
    unsigned long ulCount = 0;
    char cChar;

    //
    // Check the arguments.
    //
    ASSERT(pcBuf != 0);
    ASSERT(ulLen != 0);
    ASSERT(g_ulBase != 0);

    //
    // Adjust the length back by 1 to leave space for the trailing
    // null terminator.
    //
    ulLen--;

    //
    // Process characters until a newline is received.
    //
    while(1)
    {
        //
        // Read the next character from the USB Stdio receive buffer.
        //
        if(!RX_BUFFER_EMPTY)
        {
            cChar = g_pcUSBStdioRxBuffer[g_ulUSBRxReadIndex];
            ADVANCE_RX_BUFFER_INDEX(g_ulUSBRxReadIndex);

            //
            // See if a newline or escape character was received.
            //
            if((cChar == '\r') || (cChar == '\n') || (cChar == 0x1b))
            {
                //
                // Stop processing the input and end the line.
                //
                break;
            }

            //
            // Process the received character as long as we are not at the end
            // of the buffer.  If the end of the buffer has been reached then
            // all additional characters are ignored until a newline is
            // received.
            //
            if(ulCount < ulLen)
            {
                //
                // Store the character in the caller supplied buffer.
                //
                pcBuf[ulCount] = cChar;

                //
                // Increment the count of characters received.
                //
                ulCount++;
            }
        }
    }

    //
    // Add a null termination to the string.
    //
    pcBuf[ulCount] = 0;

    //
    // Return the count of chars in the buffer, not counting the trailing 0.
    //
    return(ulCount);
#else
    unsigned long ulCount = 0;
    char cChar;
    static char bLastWasCR = 0;

    //
    // Check the arguments.
    //
    ASSERT(pcBuf != 0);
    ASSERT(ulLen != 0);
    ASSERT(g_ulBase != 0);

    //
    // Adjust the length back by 1 to leave space for the trailing
    // null terminator.
    //
    ulLen--;

    //
    // Process characters until a newline is received.
    //
    while(1)
    {
        //
        // Read the next character from the console.
        //
        cChar = USBCharGet();

        //
        // See if the backspace key was pressed.
        //
        if(cChar == '\b')
        {
            //
            // If there are any characters already in the buffer, then delete
            // the last.
            //
            if(ulCount)
            {
                //
                // Rub out the previous character.
                //
                USBwrite("\b \b", 3);

                //
                // Decrement the number of characters in the buffer.
                //
                ulCount--;
            }

            //
            // Skip ahead to read the next character.
            //
            continue;
        }

        //
        // If this character is LF and last was CR, then just gobble up the
        // character because the EOL processing was taken care of with the CR.
        //
        if((cChar == '\n') && bLastWasCR)
        {
            bLastWasCR = 0;
            continue;
        }

        //
        // See if a newline or escape character was received.
        //
        if((cChar == '\r') || (cChar == '\n') || (cChar == 0x1b))
        {
            //
            // If the character is a CR, then it may be followed by a LF which
            // should be paired with the CR.  So remember that a CR was
            // received.
            //
            if(cChar == '\r')
            {
                bLastWasCR = 1;
            }

            //
            // Stop processing the input and end the line.
            //
            break;
        }

        //
        // Process the received character as long as we are not at the end of
        // the buffer.  If the end of the buffer has been reached then all
        // additional characters are ignored until a newline is received.
        //
        if(ulCount < ulLen)
        {
            //
            // Store the character in the caller supplied buffer.
            //
            pcBuf[ulCount] = cChar;

            //
            // Increment the count of characters received.
            //
            ulCount++;

            //
            // Reflect the character back to the user.
            //
            USBCharPut(cChar);
        }
    }

    //
    // Add a null termination to the string.
    //
    pcBuf[ulCount] = 0;

    //
    // Send a CRLF pair to the terminal to end the line.
    //
    USBwrite("\r\n", 2);

    //
    // Return the count of chars in the buffer, not counting the trailing 0.
    //
    return(ulCount);
#endif
}

//*****************************************************************************
//
//! Read a single character from the USB port, blocking if necessary.
//!
//! This function will receive a single character from the USB port and store it at
//! the supplied address.
//!
//! In both buffered and unbuffered modes, this function will block until a
//! character is received.  If non-blocking operation is required in buffered
//! mode, a call to USBRxAvail() may be made to determine whether any
//! characters are currently available for reading.
//!
//! \return Returns the character read.
//
//*****************************************************************************
unsigned char
USBgetc(void)
{
#ifdef USB_BUFFERED
    unsigned char cChar;

    //
    // Wait for a character to be received.
    //
    while(RX_BUFFER_EMPTY)
    {
        //
        // Block waiting for a character to be received (if the buffer is
        // currently empty).
        //
    }

    //
    // Read a character from the USB Stdio receive buffer.
    //
    cChar = g_pcUSBStdioRxBuffer[g_ulUSBRxReadIndex];
    ADVANCE_RX_BUFFER_INDEX(g_ulUSBRxReadIndex);

    //
    // Return the character to the caller.
    //
    return(cChar);
#else
    //
    // Block until a character is received by the USB port then return it to
    // the caller.
    //
    return USBCharGet();
#endif
}

//*****************************************************************************
//
//! A simple USB based printf function supporting \%c, \%d, \%p, \%s, \%u,
//! \%x, and \%X.
//!
//! \param pcString is the format string.
//! \param ... are the optional arguments, which depend on the contents of the
//! format string.
//!
//! This function is very similar to the C library <tt>fprintf()</tt> function.
//! All of its output will be sent to the USB port. Only the following formatting
//! characters are supported:
//!
//! - \%c to print a character
//! - \%d or \%i to print a decimal value
//! - \%s to print a string
//! - \%u to print an unsigned decimal value
//! - \%x to print a hexadecimal value using lower case letters
//! - \%X to print a hexadecimal value using lower case letters (not upper case
//! letters as would typically be used)
//! - \%p to print a pointer as a hexadecimal value
//! - \%\% to print out a \% character
//!
//! For \%s, \%d, \%i, \%u, \%p, \%x, and \%X, an optional number may reside
//! between the \% and the format character, which specifies the minimum number
//! of characters to use for that value; if preceded by a 0 then the extra
//! characters will be filled with zeros instead of spaces.  For example,
//! ''\%8d'' will use eight characters to print the decimal value with spaces
//! added to reach eight; ''\%08d'' will use eight characters as well but will
//! add zeroes instead of spaces.
//!
//! The type of the arguments after \e pcString must match the requirements of
//! the format string.  For example, if an integer was passed where a string
//! was expected, an error of some kind will most likely occur.
//!
//! \return None.
//
//*****************************************************************************
void
USBprintf(const char *pcString, ...)
{
    unsigned long ulIdx, ulValue, ulPos, ulCount, ulBase, ulNeg;
    char *pcStr, pcBuf[16], cFill;
    va_list vaArgP;

    //
    // Check the arguments.
    //
    ASSERT(pcString != 0);

    //
    // Start the varargs processing.
    //
    va_start(vaArgP, pcString);

    //
    // Loop while there are more characters in the string.
    //
    while(*pcString)
    {
        //
        // Find the first non-% character, or the end of the string.
        //
        for(ulIdx = 0; (pcString[ulIdx] != '%') && (pcString[ulIdx] != '\0');
            ulIdx++)
        {
        }

        //
        // Write this portion of the string.
        //
        USBwrite(pcString, ulIdx);

        //
        // Skip the portion of the string that was written.
        //
        pcString += ulIdx;

        //
        // See if the next character is a %.
        //
        if(*pcString == '%')
        {
            //
            // Skip the %.
            //
            pcString++;

            //
            // Set the digit count to zero, and the fill character to space
            // (i.e. to the defaults).
            //
            ulCount = 0;
            cFill = ' ';

            //
            // It may be necessary to get back here to process more characters.
            // Goto's aren't pretty, but effective.  I feel extremely dirty for
            // using not one but two of the beasts.
            //
again:

            //
            // Determine how to handle the next character.
            //
            switch(*pcString++)
            {
                //
                // Handle the digit characters.
                //
                case '0':
                case '1':
                case '2':
                case '3':
                case '4':
                case '5':
                case '6':
                case '7':
                case '8':
                case '9':
                {
                    //
                    // If this is a zero, and it is the first digit, then the
                    // fill character is a zero instead of a space.
                    //
                    if((pcString[-1] == '0') && (ulCount == 0))
                    {
                        cFill = '0';
                    }

                    //
                    // Update the digit count.
                    //
                    ulCount *= 10;
                    ulCount += pcString[-1] - '0';

                    //
                    // Get the next character.
                    //
                    goto again;
                }

                //
                // Handle the %c command.
                //
                case 'c':
                {
                    //
                    // Get the value from the varargs.
                    //
                    ulValue = va_arg(vaArgP, unsigned long);

                    //
                    // Print out the character.
                    //
                    USBwrite((char *)&ulValue, 1);

                    //
                    // This command has been handled.
                    //
                    break;
                }

                //
                // Handle the %d and %i commands.
                //
                case 'd':
                case 'i':
                {
                    //
                    // Get the value from the varargs.
                    //
                    ulValue = va_arg(vaArgP, unsigned long);

                    //
                    // Reset the buffer position.
                    //
                    ulPos = 0;

                    //
                    // If the value is negative, make it positive and indicate
                    // that a minus sign is needed.
                    //
                    if((long)ulValue < 0)
                    {
                        //
                        // Make the value positive.
                        //
                        ulValue = -(long)ulValue;

                        //
                        // Indicate that the value is negative.
                        //
                        ulNeg = 1;
                    }
                    else
                    {
                        //
                        // Indicate that the value is positive so that a minus
                        // sign isn't inserted.
                        //
                        ulNeg = 0;
                    }

                    //
                    // Set the base to 10.
                    //
                    ulBase = 10;

                    //
                    // Convert the value to ASCII.
                    //
                    goto convert;
                }

                //
                // Handle the %s command.
                //
                case 's':
                {
                    //
                    // Get the string pointer from the varargs.
                    //
                    pcStr = va_arg(vaArgP, char *);

                    //
                    // Determine the length of the string.
                    //
                    for(ulIdx = 0; pcStr[ulIdx] != '\0'; ulIdx++)
                    {
                    }

                    //
                    // Write the string.
                    //
                    USBwrite(pcStr, ulIdx);

                    //
                    // Write any required padding spaces
                    //
                    if(ulCount > ulIdx)
                    {
                        ulCount -= ulIdx;
                        while(ulCount--)
                        {
                            USBwrite(" ", 1);
                        }
                    }
                    //
                    // This command has been handled.
                    //
                    break;
                }

                //
                // Handle the %u command.
                //
                case 'u':
                {
                    //
                    // Get the value from the varargs.
                    //
                    ulValue = va_arg(vaArgP, unsigned long);

                    //
                    // Reset the buffer position.
                    //
                    ulPos = 0;

                    //
                    // Set the base to 10.
                    //
                    ulBase = 10;

                    //
                    // Indicate that the value is positive so that a minus sign
                    // isn't inserted.
                    //
                    ulNeg = 0;

                    //
                    // Convert the value to ASCII.
                    //
                    goto convert;
                }

                //
                // Handle the %x and %X commands.  Note that they are treated
                // identically; i.e. %X will use lower case letters for a-f
                // instead of the upper case letters is should use.  We also
                // alias %p to %x.
                //
                case 'x':
                case 'X':
                case 'p':
                {
                    //
                    // Get the value from the varargs.
                    //
                    ulValue = va_arg(vaArgP, unsigned long);

                    //
                    // Reset the buffer position.
                    //
                    ulPos = 0;

                    //
                    // Set the base to 16.
                    //
                    ulBase = 16;

                    //
                    // Indicate that the value is positive so that a minus sign
                    // isn't inserted.
                    //
                    ulNeg = 0;

                    //
                    // Determine the number of digits in the string version of
                    // the value.
                    //
convert:
                    for(ulIdx = 1;
                        (((ulIdx * ulBase) <= ulValue) &&
                         (((ulIdx * ulBase) / ulBase) == ulIdx));
                        ulIdx *= ulBase, ulCount--)
                    {
                    }

                    //
                    // If the value is negative, reduce the count of padding
                    // characters needed.
                    //
                    if(ulNeg)
                    {
                        ulCount--;
                    }

                    //
                    // If the value is negative and the value is padded with
                    // zeros, then place the minus sign before the padding.
                    //
                    if(ulNeg && (cFill == '0'))
                    {
                        //
                        // Place the minus sign in the output buffer.
                        //
                        pcBuf[ulPos++] = '-';

                        //
                        // The minus sign has been placed, so turn off the
                        // negative flag.
                        //
                        ulNeg = 0;
                    }

                    //
                    // Provide additional padding at the beginning of the
                    // string conversion if needed.
                    //
                    if((ulCount > 1) && (ulCount < 16))
                    {
                        for(ulCount--; ulCount; ulCount--)
                        {
                            pcBuf[ulPos++] = cFill;
                        }
                    }

                    //
                    // If the value is negative, then place the minus sign
                    // before the number.
                    //
                    if(ulNeg)
                    {
                        //
                        // Place the minus sign in the output buffer.
                        //
                        pcBuf[ulPos++] = '-';
                    }

                    //
                    // Convert the value into a string.
                    //
                    for(; ulIdx; ulIdx /= ulBase)
                    {
                        pcBuf[ulPos++] = g_pcHex[(ulValue / ulIdx) % ulBase];
                    }

                    //
                    // Write the string.
                    //
                    USBwrite(pcBuf, ulPos);

                    //
                    // This command has been handled.
                    //
                    break;
                }

                //
                // Handle the %% command.
                //
                case '%':
                {
                    //
                    // Simply write a single %.
                    //
                    USBwrite(pcString - 1, 1);

                    //
                    // This command has been handled.
                    //
                    break;
                }

                //
                // Handle all other commands.
                //
                default:
                {
                    //
                    // Indicate an error.
                    //
                    USBwrite("ERROR", 5);

                    //
                    // This command has been handled.
                    //
                    break;
                }
            }
        }
    }

    //
    // End the varargs processing.
    //
    va_end(vaArgP);
}

//*****************************************************************************
//
//! Returns the number of bytes available in the USB Stdio receive buffer.
//!
//! This function, available only when the module is built to operate in
//! buffered mode using \b USB_BUFFERED, may be used to determine the number
//! of bytes of data currently available in the USB Stdio receive buffer.
//!
//! \return Returns the number of available bytes.
//
//*****************************************************************************
#if defined(USB_BUFFERED) || defined(DOXYGEN)
int
USBRxBytesAvail(void)
{
    return(RX_BUFFER_USED);
}
#endif

#if defined(USB_BUFFERED) || defined(DOXYGEN)
//*****************************************************************************
//
//! Returns the number of bytes free in the USB Stdio transmit buffer.
//!
//! This function, available only when the module is built to operate in
//! buffered mode using \b USB_BUFFERED, may be used to determine the amount
//! of space currently available in the USB Stdio transmit buffer.
//!
//! \return Returns the number of free bytes.
//
//*****************************************************************************
int
USBTxBytesFree(void)
{
    return(TX_BUFFER_FREE);
}
#endif

//*****************************************************************************
//
//! Looks ahead in the USB Stdio receive buffer for a particular character.
//!
//! \param ucChar is the character that is to be searched for.
//!
//! This function, available only when the module is built to operate in
//! buffered mode using \b USB_BUFFERED, may be used to look ahead in the
//! USB Stdio receive buffer for a particular character and report its position if found.
//! It is typically used to determine whether a complete line of user input is
//! available, in which case ucChar should be set to CR ('\\r') which is used
//! as the line end marker in the receive buffer.
//!
//! \return Returns -1 to indicate that the requested character does not exist
//! in the receive buffer.  Returns a non-negative number if the character was
//! found in which case the value represents the position of the first instance
//! of \e ucChar relative to the receive buffer read pointer.
//
//*****************************************************************************
#if defined(USB_BUFFERED) || defined(DOXYGEN)
int
USBPeek(unsigned char ucChar)
{
    int iCount;
    int iAvail;
    unsigned long ulReadIndex;

    //
    // How many characters are there in the receive buffer?
    //
    iAvail = (int)RX_BUFFER_USED;
    ulReadIndex = g_ulUSBRxReadIndex;

    //
    // Check all the unread characters looking for the one passed.
    //
    for(iCount = 0; iCount < iAvail; iCount++)
    {
        if(g_pcUSBStdioRxBuffer[ulReadIndex] == ucChar)
        {
            //
            // We found it so return the index
            //
            return(iCount);
        }
        else
        {
            //
            // This one didn't match so move on to the next character.
            //
            ADVANCE_RX_BUFFER_INDEX(ulReadIndex);
        }
    }

    //
    // If we drop out of the loop, we didn't find the character in the receive
    // buffer.
    //
    return(-1);
}
#endif

//*****************************************************************************
//
//! Flushes the USB Stdio receive buffer.
//!
//! This function, available only when the module is built to operate in
//! buffered mode using \b USB_BUFFERED, may be used to discard any data
//! received from the USB port but not yet read using USBgets().
//!
//! \return None.
//
//*****************************************************************************
#if defined(USB_BUFFERED) || defined(DOXYGEN)
void
USBFlushRx(void)
{
    unsigned long ulInt;

    //
    // Temporarily turn off interrupts.
    //
    ulInt = IntMasterDisable();

    //
    // Flush the receive buffer.
    //
    g_ulUSBRxReadIndex = 0;
    g_ulUSBRxWriteIndex = 0;

    //
    // If interrupts were enabled when we turned them off, turn them
    // back on again.
    //
    if(!ulInt)
    {
        IntMasterEnable();
    }
}
#endif

//*****************************************************************************
//
//! Flushes the USB Stdio transmit buffer.
//!
//! \param bDiscard indicates whether any remaining data in the buffer should
//! be discarded (\b true) or transmitted (\b false).
//!
//! This function, available only when the module is built to operate in
//! buffered mode using \b USB_BUFFERED, may be used to flush the transmit
//! buffer, either discarding or transmitting any data received via calls to
//! USBprintf() that is waiting to be transmitted.  On return, the transmit
//! buffer will be empty.
//!
//! \return None.
//
//*****************************************************************************
#if defined(USB_BUFFERED) || defined(DOXYGEN)
void
USBFlushTx(bool bDiscard)
{
    unsigned long ulInt;

    //
    // Should the remaining data be discarded or transmitted?
    //
    if(bDiscard)
    {
        //
        // The remaining data should be discarded, so temporarily turn off
        // interrupts.
        //
        ulInt = IntMasterDisable();

        //
        // Flush the transmit buffer.
        //
        g_ulUSBTxReadIndex = 0;
        g_ulUSBTxWriteIndex = 0;

        //
        // If interrupts were enabled when we turned them off, turn them
        // back on again.
        //
        if(!ulInt)
        {
            IntMasterEnable();
        }
    }
    else
    {
        //
        // Wait for all remaining data to be transmitted before returning.
        //
        while(!TX_BUFFER_EMPTY)
        {
        }
    }
}
#endif

//*****************************************************************************
//
//! Enables or disables echoing of received characters to the transmitter.
//!
//! \param bEnable must be set to \b true to enable echo or \b false to
//! disable it.
//!
//! This function, available only when the module is built to operate in
//! buffered mode using \b USB_BUFFERED, may be used to control whether or not
//! received characters are automatically echoed back to the transmitter.  By
//! default, echo is enabled and this is typically the desired behavior if
//! the module is being used to support a serial command line.  In applications
//! where this module is being used to provide a convenient, buffered serial
//! interface over which application-specific binary protocols are being run,
//! however, echo may be undesirable and this function can be used to disable
//! it.
//!
//! \return None.
//
//*****************************************************************************
#if defined(USB_BUFFERED) || defined(DOXYGEN)
void
USBEchoSet(bool bEnable)
{
    g_bDisableEcho = !bEnable;
}
#endif

#ifdef USB_BUFFERED
//*****************************************************************************
//
//! Places a character into the USB Stdio receive buffer.
//!
//! \param ucData is the character to be transmitted.
//!
//! This function, available only when the module is built to operate in
//! buffered mode using \b USB_BUFFERED, writes the character \e ucData to
//! the USB Stdio receive buffer.
//! This function does not block, so if there is no space available, then a
//! \b false is returned and the application must retry the function later.
//!
//! \return Returns \b true if the character was successfully placed in the
//! USB Stdio receive buffer or \b false if there was no space available in
//! the USB Stdio receive buffer.
//
//*****************************************************************************
static bool USBRxCharPutNonBlocking(char cData)
{
    //
    // If there is space in the receive buffer, put the character
    // there, otherwise throw it away.
    //
    if (!RX_BUFFER_FULL)
    {
        //
        // Store the new character in the receive buffer
        //
        g_pcUSBStdioRxBuffer[g_ulUSBRxWriteIndex] = cData;
        ADVANCE_RX_BUFFER_INDEX(g_ulUSBRxWriteIndex);

        //
        // If echo is enabled, write the character to the transmit
        // buffer so that the user gets some immediate feedback.
        //
        if (!g_bDisableEcho)
        {
            if (cData == '\r')
            {
                cData = '\n';       // tranlsate it (will be echoed as "\r\n")
            }
            USBwrite(&cData, 1);
        }

        return true;        // char stored
    }
    else
    {
        return false;       // no room for char
    }
}

#else

//*****************************************************************************
//
//! Waits to send a character via the USB port.
//!
//! \param ucData is the character to be transmitted.
//!
//! This function sends the character \e ucData to the USB transmit buffer.
//! If there is no space available in the USB transmit buffer, this function waits
//! until there is space available before returning.
//!
//! \return None.
//
//*****************************************************************************
static void USBCharPut(unsigned char ucData)
{
    //
    // Wait until space is available.
    //
    while (USBBufferSpaceAvailable((tUSBBuffer *)&g_sTxBuffer) == 0)
    { ; }

    //
    // Send the char.
    //
    USBBufferWrite((tUSBBuffer *)&g_sTxBuffer, &ucData, 1);
}

//*****************************************************************************
//
//! Waits for a character from the USB port.
//!
//! This function gets a character from the USB receive buffer.
//! If there are no characters available, this function waits until a
//! character is received before returning.
//!
//! \return Returns the character read from the USB port.
//
//*****************************************************************************
static char USBCharGet(void)
{
    unsigned char ucChar;

    //
    // Wait until a char is available.
    //
    while (USBBufferDataAvailable((tUSBBuffer *)&g_sRxBuffer) == 0)
    { ; }

    //
    // Get a character from the USB receive buffer.
    //
    USBBufferRead((tUSBBuffer *)&g_sRxBuffer, &ucChar, 1);

    //
    // Now return the char.
    //
    return (ucChar);
}
#endif

#ifdef USB_BUFFERED
//*****************************************************************************
//
// Take as many bytes from the USB receive buffer as we have space for and move
// them into the USB Stdio receive buffer.
//
//*****************************************************************************
static void USBRxBufferPut(void)
{
    unsigned long ulRead;
    unsigned char ucChar;

    //
    // If there is space in the USB Stdio receive buffer, try to read some characters
    // from the USB receive buffer to fill it again.
    //
    while (!RX_BUFFER_FULL)
    {
        //
        // Get a character from the USB receive buffer.
        //
        ulRead = USBBufferRead((tUSBBuffer *)&g_sRxBuffer, &ucChar, 1);

        //
        // Did we get a character?
        //
        if(ulRead)
        {
            //
            // Place the character in the USB Stdio receive buffer.
            //
            USBRxCharPutNonBlocking(ucChar);
        }
        else
        {
            //
            // We ran out of characters so exit the function.
            //
            return;
        }
    }
}
#endif

//*****************************************************************************
//
// Get the communication parameters in use.
//
//*****************************************************************************
static void GetLineCoding(tLineCoding * psLineCoding)
{
    // return defaults
    psLineCoding->ucDatabits = 8;
    psLineCoding->ucParity = USB_CDC_PARITY_NONE;
    psLineCoding->ucStop = USB_CDC_STOP_BITS_1;
}

//*****************************************************************************
//
// Handles CDC driver notifications related to control and setup of the device.
//
// \param pvCBData is the client-supplied callback pointer for this channel.
// \param ulEvent identifies the event we are being notified about.
// \param ulMsgValue is an event-specific value.
// \param pvMsgData is an event-specific pointer.
//
// This function is called by the CDC driver to perform control-related
// operations on behalf of the USB host.  These functions include setting
// and querying the serial communication parameters, setting handshake line
// states and sending break conditions.
//
// \return The return value is event-specific.
//
//*****************************************************************************
unsigned long ControlHandler(void * pvCBData, unsigned long ulEvent,
                             unsigned long ulMsgValue, void * pvMsgData)
{
    unsigned long ulIntsOff;

    //
    // Which event are we being asked to process?
    //
    switch(ulEvent)
    {
        //
        // We are connected to a host and communication is now possible.
        //
        case USB_EVENT_CONNECTED:
            g_bUSBConfigured = true;

            //
            // Flush our buffers.
            //
            USBBufferFlush(&g_sTxBuffer);
            USBBufferFlush(&g_sRxBuffer);

            //
            // Tell the main loop to update the display.
            //
            ulIntsOff = MAP_IntMasterDisable();
            g_pcStatus = "Connected";
            g_ulFlags |= COMMAND_STATUS_UPDATE;
            if(!ulIntsOff)
            {
                MAP_IntMasterEnable();
            }
            break;

        //
        // The host has disconnected.
        //
        case USB_EVENT_DISCONNECTED:
            g_bUSBConfigured = false;
            ulIntsOff = MAP_IntMasterDisable();
            g_pcStatus = "Disconnected";
            g_ulFlags |= COMMAND_STATUS_UPDATE;
            if(!ulIntsOff)
            {
                MAP_IntMasterEnable();
            }
            break;

        //
        // Return the current serial communication parameters.
        //
        case USBD_CDC_EVENT_GET_LINE_CODING:
            GetLineCoding(pvMsgData);
            break;

        //
        // Set the current serial communication parameters.
        //
        case USBD_CDC_EVENT_SET_LINE_CODING:
            break;

        //
        // Set the current serial communication parameters.
        //
        case USBD_CDC_EVENT_SET_CONTROL_LINE_STATE:
            break;

        //
        // Send a break condition on the serial line.
        //
        case USBD_CDC_EVENT_SEND_BREAK:
            break;

        //
        // Clear the break condition on the serial line.
        //
        case USBD_CDC_EVENT_CLEAR_BREAK:
            break;

        //
        // Ignore SUSPEND and RESUME for now.
        //
        case USB_EVENT_SUSPEND:
        case USB_EVENT_RESUME:
            break;

        //
        // We don't expect to receive any other events. Ignore any that show up.
        //
        default:
            break;
    }

    return(0);
}

//*****************************************************************************
//
// Handles CDC driver notifications related to the transmit channel (data to
// the USB host).
//
// \param ulCBData is the client-supplied callback pointer for this channel.
// \param ulEvent identifies the event we are being notified about.
// \param ulMsgValue is an event-specific value.
// \param pvMsgData is an event-specific pointer.
//
// This function is called by the CDC driver to notify us of any events
// related to operation of the transmit data channel (the IN channel carrying
// data to the USB host).
//
// \return The return value is event-specific.
//
//*****************************************************************************
unsigned long TxHandler(void * pvCBData, unsigned long ulEvent, unsigned long ulMsgValue, void * pvMsgData)
{
    //
    // Which event have we been sent?
    //
    switch(ulEvent)
    {
        case USB_EVENT_TX_COMPLETE:
#ifdef USB_BUFFERED
            g_bUSBTxSending = false;
//UARTprintf("USB_EVENT_TX_COMPLETE\n");

            //
            // If we have anything in the USB Stdio transmit buffer, move it to the USB
            // transmit buffer to transmit it.
            //
            if (!TX_BUFFER_EMPTY)
            {
                USBPrimeTransmit();
            }
#endif
            break;

        //
        // We don't expect to receive any other events. Ignore any that show up.
        //
        default:
            break;

    }
    return(0);
}

//*****************************************************************************
//
// Handles CDC driver notifications related to the receive channel (data from
// the USB host).
//
// \param ulCBData is the client-supplied callback data value for this channel.
// \param ulEvent identifies the event we are being notified about.
// \param ulMsgValue is an event-specific value.
// \param pvMsgData is an event-specific pointer.
//
// This function is called by the CDC driver to notify us of any events
// related to operation of the receive data channel (the OUT channel carrying
// data from the USB host).
//
// \return The return value is event-specific.
//
//*****************************************************************************
unsigned long RxHandler(void * pvCBData, unsigned long ulEvent, unsigned long ulMsgValue, void * pvMsgData)
{
    unsigned long ulCount = 0;

    //
    // Which event are we being sent?
    //
    switch(ulEvent)
    {
        //
        // A new packet has been received.
        //
        case USB_EVENT_RX_AVAILABLE:
        {
#ifdef USB_BUFFERED
            //
            // Feed some received characters into the USB Stdio receive buffer.
            //
            USBRxBufferPut();
#endif
            break;
        }

        //
        // We are being asked how much unprocessed data we have still to
        // process. We return 0 if the USB Stdio receive buffer is currently empty
        // or 1 if it is not empty. The actual number of bytes in the USB
        // Stdio receive buffer is not important here, merely whether or
        // not everything previously sent to us has been processed.
        //
        case USB_EVENT_DATA_REMAINING:
        {
#ifdef USB_BUFFERED
            ulCount = RX_BUFFER_EMPTY ? 0 : 1;
#endif
            return(ulCount);
        }

        //
        // We are being asked to provide a buffer into which the next packet
        // can be read. We do not support this mode of receiving data so let
        // the driver know by returning 0. The CDC driver should not be sending
        // this message but this is included just for illustration and
        // completeness.
        //
        case USB_EVENT_REQUEST_BUFFER:
        {
            return(0);
        }

        //
        // We don't expect to receive any other events.  Ignore any that show up.
        //
        default:
            break;
    }

    return(0);
}

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
