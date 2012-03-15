/*
    FreeRTOS V7.1.0 - Copyright (C) 2011 Real Time Engineers Ltd.
	

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS tutorial books are available in pdf and paperback.        *
     *    Complete, revised, and edited pdf reference manuals are also       *
     *    available.                                                         *
     *                                                                       *
     *    Purchasing FreeRTOS documentation will not only help you, by       *
     *    ensuring you get running as quickly as possible and with an        *
     *    in-depth knowledge of how to use FreeRTOS, it will also help       *
     *    the FreeRTOS project to continue with its mission of providing     *
     *    professional grade, cross platform, de facto standard solutions    *
     *    for microcontrollers - completely free of charge!                  *
     *                                                                       *
     *    >>> See http://www.FreeRTOS.org/Documentation for details. <<<     *
     *                                                                       *
     *    Thank you for using FreeRTOS, and thank you for your support!      *
     *                                                                       *
    ***************************************************************************


    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation AND MODIFIED BY the FreeRTOS exception.
    >>>NOTE<<< The modification to the GPL is included to allow you to
    distribute a combined work that includes FreeRTOS without being obliged to
    provide the source code for proprietary components outside of the FreeRTOS
    kernel.  FreeRTOS is distributed in the hope that it will be useful, but
    WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
    or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
    more details. You should have received a copy of the GNU General Public
    License and the FreeRTOS license exception along with FreeRTOS; if not it
    can be viewed here: http://www.freertos.org/a00114.html and also obtained
    by writing to Richard Barry, contact details for whom are available on the
    FreeRTOS WEB site.

    1 tab == 4 spaces!

    http://www.FreeRTOS.org - Documentation, latest information, license and
    contact details.

    http://www.SafeRTOS.com - A version that is certified for use in safety
    critical systems.

    http://www.OpenRTOS.com - Commercial support, development, porting,
    licensing and training services.
*/

/*
Changes from V1.2.5

	+  Clear overrun errors in the Rx ISR.  Overrun errors prevent any further
	   characters being received.

Changes from V2.0.0

	+ Use portTickType in place of unsigned pdLONG for delay periods.
	+ cQueueReieveFromISR() used in place of xQueueReceive() in ISR.
*/

/* BASIC INTERRUPT DRIVEN SERIAL PORT DRIVER. */

#include "usart.h"

#include "portmacro.h"

/* Scheduler header files. */
#include "FreeRTOS.h"
#include "task.h"
#include "serial.h"
#include "queue.h"

/*
 * Prototypes for ISR's.  The PIC architecture means that these functions
 * have to be called from port.c.  The prototypes are not however included
 * in the header as the header is common to all ports.
 */
void vSerialTxISR( void );
void vSerialRxISR( void );

/* Hardware pin definitions. */
#define serTX_PIN	TRISCbits.TRISC6
#define serRX_PIN	TRISCbits.TRISC7

/* Bit/register definitions. */
#define serINPUT				( 1 )
#define serOUTPUT				( 0 )

#define serINTERRUPT_ENABLED 	( ( unsigned short ) 1 )
#define serINTERRUPT_DISABLED 	( ( unsigned short ) 0 )

#define serNO_BLOCK				( ( portTickType ) 0 )

#define serITR_PRIORITY itrLOW_PRIORITY

/*-----------------------------------------------------------*/

/* Queues to interface between comms API and interrupt routines. */
static xQueueHandle xRxedChars;
static xQueueHandle xCharsForTx;

/*-----------------------------------------------------------*/

xComPortHandle xSerialPortInitMinimal( unsigned long ulWantedBaud, unsigned portBASE_TYPE uxQueueLength )
{
    unsigned long ulBaud;

	/* Calculate the baud rate generator constant.
	SPBRG = ( (FOSC / Desired Baud Rate) / 16 ) - 1 */
	ulBaud = configCPU_CLOCK_HZ / ulWantedBaud;
	ulBaud /= ( unsigned long ) 16;
	ulBaud -= ( unsigned long ) 1;

	/* Create the queues used by the ISR's to interface to tasks. */
	xRxedChars = xQueueCreate( uxQueueLength, ( unsigned portBASE_TYPE ) sizeof( char ) );
	xCharsForTx = xQueueCreate( uxQueueLength, ( unsigned portBASE_TYPE ) sizeof( char ) );

	portENTER_CRITICAL();

    /* Setup the IO pins to enable the USART IO. */
    serTX_PIN = serOUTPUT;
    serRX_PIN = serINPUT;

    //-------------------------configure USART ---------------------------------------------------------
    // API configures USART for desired parameters:
    // - RX interrupt turned on, TX interrupt turned off
    // - Asynchronous mode
    // - 8 bits
    // - Continuous Receive Enabled
    // - High speed baud rate generator mode
#if defined(USE_OR_MASKS)
    OpenUSART(USART_TX_INT_ON
            | USART_RX_INT_ON
            | USART_ASYNCH_MODE
            | USART_EIGHT_BIT
            | USART_CONT_RX
            | USART_BRGH_HIGH, ulBaud);
#else
    OpenUSART(USART_TX_INT_ON
            & USART_RX_INT_ON
            & USART_ASYNCH_MODE
            & USART_EIGHT_BIT
            & USART_CONT_RX
            & USART_BRGH_HIGH, ulBaud);
#endif

    /* Set receive and transmit interrupts priority */
    IPR1bits.TXIP = serITR_PRIORITY;
    IPR1bits.RCIP = serITR_PRIORITY;

    /* Enable the Rx interrupt now, the Tx interrupt will get enabled when
       we have data to send. */
    PIE1bits.RCIE = 1;

	portEXIT_CRITICAL();

	/* Unlike other ports, this serial code does not allow for more than one
	com port.  We therefore don't return a pointer to a port structure and 
	can	instead just return NULL. */
	return NULL;
}
/*-----------------------------------------------------------*/

xComPortHandle xSerialPortInit( eCOMPort ePort, eBaud eWantedBaud, eParity eWantedParity, eDataBits eWantedDataBits, eStopBits eWantedStopBits, unsigned portBASE_TYPE uxBufferLength )
{
	/* This is not implemented in this port.
	Use xSerialPortInitMinimal() instead. */
}
/*-----------------------------------------------------------*/

portBASE_TYPE xSerialGetChar( xComPortHandle pxPort, signed char *pcRxedChar, portTickType xBlockTime )
{
	/* Get the next character from the buffer.  Return false if no characters
	are available, or arrive before xBlockTime expires. */
	if ( xQueueReceive( xRxedChars, pcRxedChar, xBlockTime ) ) {
		return pdTRUE;
	} else {
		return pdFALSE;
	}
}
/*-----------------------------------------------------------*/

portBASE_TYPE xSerialPutChar( xComPortHandle pxPort, signed char cOutChar, portTickType xBlockTime )
{
	/* Return false if after the block time there is no room on the Tx queue. */
	if ( xQueueSendToBack( xCharsForTx, ( const void * ) &cOutChar, xBlockTime ) != pdPASS ) {
		return pdFAIL;
	}

	/* Turn interrupt on - ensure the compiler only generates a single
	instruction for this. */
	PIE1bits.TXIE = serINTERRUPT_ENABLED;

	return pdPASS;
}

void vSerialPutRomString( xComPortHandle pxPort, rom const char * const pcString, unsigned short usStringLength )
{
    signed char cOutChar;
    rom const signed char *s = pcString;
	( void ) usStringLength;

    if (!s)
        return;

	while ( *s ) {
        cOutChar = *s;
      retry:
        /* If there is no room on the Tx queue then enable interrupt and try again. */
        if ( xQueueSendToBack( xCharsForTx, ( const void * ) &cOutChar, serNO_BLOCK ) != pdPASS ) {
            PIE1bits.TXIE = serINTERRUPT_ENABLED;
            goto retry;
        }

		++s;
	}

    if ( !xQueueIsQueueEmptyFromISR( xCharsForTx ) )
        PIE1bits.TXIE = serINTERRUPT_ENABLED;
}

void vSerialPutString( xComPortHandle pxPort, const signed char * const pcString, unsigned short usStringLength )
{
    signed char cOutChar;
    const signed char *s = pcString;
	( void ) usStringLength;

    if (!s)
        return;

	while ( *s ) {
        cOutChar = *s;
      retry:
        /* If there is no room on the Tx queue then enable interrupt and try again. */
        if ( xQueueSendToBack( xCharsForTx, ( const void * ) &cOutChar, serNO_BLOCK ) != pdPASS ) {
            PIE1bits.TXIE = serINTERRUPT_ENABLED;
            goto retry;
        }

		++s;
	}

    if ( !xQueueIsQueueEmptyFromISR( xCharsForTx ) )
        PIE1bits.TXIE = serINTERRUPT_ENABLED;
}

/*-----------------------------------------------------------*/

void vSerialClose( xComPortHandle xPort )
{
	/* Not implemented for this port.
       To implement, turn off the interrupts and delete the memory
       allocated to the queues. */
}
/*-----------------------------------------------------------*/

void vSerialRxISR( void )
{
    char cChar;
    portBASE_TYPE cTaskWoken = pdFALSE;

	/* Get the character and post it on the queue of Rxed characters.
       If the post causes a task to wake force a context switch as the woken task
       may have a higher priority than the task we have interrupted. */
    cChar = RCREG;

    /* Clear the interrupt flag */
    PIR1bits.RCIF = 0;

	xQueueSendToBackFromISR( xRxedChars, ( const void * ) &cChar, &cTaskWoken );

	/* if ( cTaskWoken ) { */
	/* 	taskYIELD(); */
	/* } */
}
/*-----------------------------------------------------------*/

void vSerialTxISR( void )
{
    char cChar;
    portBASE_TYPE cTaskWoken = pdFALSE;

	while ( TXSTAbits.TRMT && xQueueReceiveFromISR( xCharsForTx, &cChar, &cTaskWoken ) == pdTRUE )
        TXREG = cChar; // Send the next character queued for Tx

    /* Queue empty, nothing to send. */
    if ( xQueueIsQueueEmptyFromISR( xCharsForTx ) )
        PIE1bits.TXIE = serINTERRUPT_DISABLED;

	/* if ( cTaskWoken ) { */
	/* 	taskYIELD(); */
	/* } */
}
