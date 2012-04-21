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
 * Instead of the normal single demo application, the PIC18F demo is split 
 * into several smaller programs of which this is the first.  This enables the 
 * demo's to be executed on the RAM limited 40 pin devices.  The 64 and 80 pin 
 * devices require a more costly development platform and are not so readily 
 * available.
 *
 * The RTOSDemo1 project is configured for a PIC18F452 device.  Main1.c starts 5 
 * tasks (including the idle task).
 *
 * The first task runs at the idle priority.  It repeatedly performs a 32bit 
 * calculation and checks it's result against the expected value.  This checks 
 * that the temporary storage utilised by the compiler to hold intermediate 
 * results does not get corrupted when the task gets switched in and out.  See 
 * demo/common/minimal/integer.c for more information.
 *
 * The second and third tasks pass an incrementing value between each other on 
 * a message queue.  See demo/common/minimal/PollQ.c for more information.
 *
 * Main1.c also creates a check task.  This periodically checks that all the 
 * other tasks are still running and have not experienced any unexpected 
 * results.  If all the other tasks are executing correctly an LED is flashed 
 * once every mainCHECK_PERIOD milliseconds.  If any of the tasks have not 
 * executed, or report and error, the frequency of the LED flash will increase 
 * to mainERROR_FLASH_RATE.
 *
 * On entry to main an 'X' is transmitted.  Monitoring the serial port using a
 * dumb terminal allows for verification that the device is not continuously 
 * being reset (no more than one 'X' should be transmitted).
 *
 * http://www.FreeRTOS.org contains important information on the use of the 
 * PIC18F port.
 */

/*
Changes from V2.0.0

	+ Delay periods are now specified using variables and constants of
	  portTickType rather than unsigned long.
*/

#if defined(__18F2550)
#include <p18f2550.h>

/*
 * since 20 MHz crystal is used then pre-PLL is needed to divide by 5 to become 4MHz
 * then the PLL will get it to 96 MHz, for USB it will be divided by 2 to get 48 MHz needed for USB
 */

#pragma config PLLDIV   = 5         // Divide by 5 (20 MHz oscillator input)
#pragma config CPUDIV   = OSC1_PLL2 // [OSC1/OSC2 Src: /1][96 MHz PLL Src: /2]: 48MHz
#pragma config USBDIV   = 2         // USB clock source comes from the 96 MHz PLL divided by 2
#pragma config FOSC     = HSPLL_HS  // HS oscillator, PLL enabled (HSPLL)
#pragma config FCMEN    = OFF       // Fail-Safe Clock Monitor disabled
#pragma config IESO     = OFF       // Oscillator Switchover mode disabled
#pragma config PWRT     = OFF       // PWRT(Power-up Timer) disabled
#pragma config BOR      = ON        // Brown-out Reset enabled in hardware only (SBOREN is disabled)
#pragma config BORV     = 3         // Brown-out Reset Voltage
#pragma config VREGEN   = ON        // USB Voltage Regulator enabled
#pragma config WDT      = OFF       // WDT disabled (control is placed on the SWDTEN bit)
#pragma config WDTPS    = 32768     // Watchdog Timer Postscale: 1:32768
#pragma config MCLRE    = OFF       // MCLR pin disabled; RE3 input pin enabled
#pragma config LPT1OSC  = OFF       // Timer1 configured for higher power operation
#pragma config PBADEN   = OFF       // PORTB<4:0> pins are configured as digital I/O on Reset
#pragma config CCP2MX   = OFF       // CCP2 input/output is multiplexed with RB3
#pragma config STVREN   = ON        // Stack full/underflow will cause Reset
#pragma config XINST    = OFF       // Instruction set extension and Indexed Addressing mode enabled
#pragma config CP0      = OFF       // Block 0 (000800-001FFFh) is not code-protected
#pragma config CP1      = OFF       // Block 1 (002000-003FFFh) is not code-protected
#pragma config CP2      = OFF       // Block 2 (004000-005FFFh) is not code-protected
#pragma config CPB      = OFF       // Boot block (000000-0007FFh) is not code-protected
#pragma config CPD      = OFF       // Data EEPROM is not code-protected
#pragma config WRT0     = OFF       // Block 0 (000800-001FFFh) is not write-protected
#pragma config WRT1     = OFF       // Block 1 (002000-003FFFh) is not write-protected
#pragma config WRT2     = OFF       // Block 2 (004000-005FFFh) is not write-protected
#pragma config WRTB     = OFF       // Boot block (000000-0007FFh) is not write-protected
#pragma config WRTC     = OFF       // Configuration registers (300000-3000FFh) are not write-protected
#pragma config WRTD     = OFF       // Data EEPROM is not write-protected
#pragma config EBTR0    = OFF       // Block 0 (000800-001FFFh) is not protected from table reads executed in other blocks
#pragma config EBTR1    = OFF       // Block 1 (002000-003FFFh) is not protected from table reads executed in other blocks
#pragma config EBTR2    = OFF       // Block 2 (004000-005FFFh) is not protected from table reads executed in other blocks
#pragma config EBTRB    = OFF       // Boot block (000000-0007FFh) is not protected from table reads executed in other block
#pragma config DEBUG    = OFF       // Background debugger disabled, RB6 and RB7 configured as general purpose I/O pins
#else
#error Building for not-supported target
#endif

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"

#include "CommandInterpreter.h"

#include "serial.h"
#include "string.h"

/* Priority definitions for some of the tasks.  Other tasks just use the idle
priority. */
#define mainCMD_TASK_PRIORITY			( tskIDLE_PRIORITY + 1 )

#define mainCOMMS_QUEUE_LENGTH			( 20 )
#define mainNO_BLOCK					( ( portTickType ) 0 )
#define mainBAUD_RATE					( ( unsigned long ) 38400 )

#define mainCMD_INPUT_LINE_LENGTH       ( 8 )
#define mainCMD_OUTPUT_LINE_LENGTH      ( 64 )

void vSerialPutRomString( xComPortHandle pxPort, rom const char * const pcString, unsigned short usStringLength );

static void vCmdTask( void *pvParameters );

/** I/O pin definitions **************************************/
#define INPUT_PIN 1
#define OUTPUT_PIN 0

/*-----------------------------------------------------------*/
#define DEBUG 0
/*-----------------------------------------------------------*/

static void vPlaInit(void)
{
    ADCON0 = 0x00; // A/D off
    ADCON1 = 0x0f; // A/D pins are digital
    CMCON  = 0x07; // comparator pins are digital

    /* Enable interrupt priority */
    RCONbits.IPEN = 1;

#if DEBUG
    TRISAbits.TRISA0 = OUTPUT_PIN;
    TRISAbits.TRISA1 = OUTPUT_PIN;
    TRISAbits.TRISA2 = OUTPUT_PIN;
    TRISAbits.TRISA3 = OUTPUT_PIN;

    PORTAbits.RA0 = 0;
    PORTAbits.RA1 = 1;
    PORTAbits.RA2 = 0;
    PORTAbits.RA3 = 0;
#endif
}

static portBASE_TYPE prvInfoCommand( signed char *pcWriteBuffer, size_t xWriteBufferLen, const signed char *pcCommandString )
{
#if defined(__18F2550)
    strncpypgm2ram( pcWriteBuffer, "PIC18f2550\r\n", xWriteBufferLen );
#endif

    return pdFALSE;
}

static const xCommandLineInput xInfoCommand = {
	"info",
	"info: Board info\r\n",
	prvInfoCommand,
	0
};

/* Creates the tasks, then starts the scheduler. */
void main( void )
{
	/* Initialise the required hardware. */
    vPlaInit();
	vPortInitialiseBlocks();

	/* Send a character so we have some visible feedback of a reset. */
	xSerialPortInitMinimal( mainBAUD_RATE, mainCOMMS_QUEUE_LENGTH );

    xCmdIntRegisterCommand(&xInfoCommand);

	/* Start the check task defined in this file. */
	xTaskCreate( vCmdTask, ( const char * const ) "cmd", 256, NULL, mainCMD_TASK_PRIORITY, NULL );

	/* Start the scheduler.  Will never return here. */
	vTaskStartScheduler();
}
/*-----------------------------------------------------------*/

static void vCmdTask( void *pvParameters )
{
    char idx = 0, c;
    portBASE_TYPE xReturned;
    signed char in[mainCMD_INPUT_LINE_LENGTH] = {0x0, };
    signed char out[mainCMD_OUTPUT_LINE_LENGTH] = {0x0, };

    vSerialPutRomString(NULL, "Board initialized\r\n", 0);

  outer:
    vSerialPutRomString(vSerialPutRomString, "\r\n->", 0);
    for( ;; ) {
        while ( xSerialGetChar( NULL, &c, mainNO_BLOCK ) != pdTRUE );

        if (c == '\r' /* || c == '\n' */) {
            if (idx > 0) {
                vSerialPutRomString(vSerialPutRomString, "\r\n", 0);
                in[idx] = 0;
                do {
                    xReturned = xCmdIntProcessCommand( in, out, mainCMD_OUTPUT_LINE_LENGTH );
                    vSerialPutString(NULL, out, 0);
                } while( xReturned == pdTRUE );
                idx = 0;
            }
            goto outer;
        } else if (idx < mainCMD_INPUT_LINE_LENGTH - 2) {
            xSerialPutChar(NULL, c, mainNO_BLOCK);
            in[idx++] = c;
        }
    }
}
/*-----------------------------------------------------------*/

