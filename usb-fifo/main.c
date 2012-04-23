#if defined(__18F2550)
#include <p18f2550.h>

#define SIMULATION

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

#include <usart.h>

// USB drivers
#include <USB/usb.h>
#include <USB/usb_device.h>
#include <USB/usb_function_generic.h>
/* #include <USB/usb_function_hid.h> */

#include "HardwareProfile.h"

#pragma udata
#pragma udata USB_VARIABLES=0x500
unsigned char OutPacket[USBGEN_EP_SIZE];	// Buffer for receiving and holding OUT packets sent from the host
unsigned char InPacket[USBGEN_EP_SIZE];		// Buffer for sending IN packets to the host
#pragma udata

USB_HANDLE UsbOutHandle;
USB_HANDLE UsbInHandle;

static void uartRxHandler (void);

//On PIC18 devices, addresses 0x00, 0x08, and 0x18 are used for
//the reset, high priority interrupt, and low priority interrupt
//vectors.

// Remap vectors for compatibilty with Microchip USB boot loaders
#if defined(PROGRAMMABLE_WITH_USB_HID_BOOTLOADER)
	#define REMAPPED_RESET_VECTOR_ADDRESS			0x1000
	#define REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS	0x1008
	#define REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS	0x1018
#elif defined(PROGRAMMABLE_WITH_USB_MCHPUSB_BOOTLOADER)
	#define REMAPPED_RESET_VECTOR_ADDRESS			0x800
	#define REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS	0x808
	#define REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS	0x818
#else
	#define REMAPPED_RESET_VECTOR_ADDRESS			0x00
	#define REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS	0x08
	#define REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS	0x18
#endif

void highPrioISR(void);
void lowPrioISR(void);

#if defined(PROGRAMMABLE_WITH_USB_HID_BOOTLOADER) || defined(PROGRAMMABLE_WITH_USB_MCHPUSB_BOOTLOADER)
extern void _startup (void);
#pragma code REMAPPED_RESET_VECTOR = REMAPPED_RESET_VECTOR_ADDRESS
void _reset (void)
{
    _asm goto _startup _endasm
}
#pragma code
#endif

#pragma code REMAPPED_HIGH_INTERRUPT_VECTOR = REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS
void remappedHighISR (void)
{
    _asm goto highPrioISR _endasm
}
#pragma code

#pragma code REMAPPED_LOW_INTERRUPT_VECTOR = REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS
void remappedLowISR (void)
{
    _asm goto lowPrioISR _endasm
}
#pragma code

#if defined(PROGRAMMABLE_WITH_USB_HID_BOOTLOADER) || defined(PROGRAMMABLE_WITH_USB_MCHPUSB_BOOTLOADER)
#pragma code HIGH_INTERRUPT_VECTOR = 0x08
void HighISR (void)
{
     _asm goto REMAPPED_HIGH_INTERRUPT_VECTOR_ADDRESS _endasm
}
#pragma code

#pragma code LOW_INTERRUPT_VECTOR = 0x18
void LowISR (void)
{
     _asm goto REMAPPED_LOW_INTERRUPT_VECTOR_ADDRESS _endasm
}
#pragma code
#endif

#pragma interrupt highPrioISR
void highPrioISR(void)
{
#if defined(USB_INTERRUPT)
    if (PIR2bits.USBIF)
        USBDeviceTasks();
#endif
}	//This return will be a "retfie fast", since this is in a #pragma interrupt section

#pragma interruptlow lowPrioISR
void lowPrioISR(void)
{
    if (PIR1bits.RCIF)
        uartRxHandler();
}	//This return will be a "retfie", since this is in a #pragma interruptlow section


/*-----------------------------------------------------------*/
#define DEBUG 0
/*-----------------------------------------------------------*/


static void plaInit(void)
{
    ADCON0 = 0x00; // A/D off
    ADCON1 = 0x0f; // A/D pins are digital
    CMCON  = 0x07; // comparator pins are digital

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

static void uartRxHandler (void)
{
    unsigned char c;

    /* Get the character received from the USART */
    c = ReadUSART();
    /* Put the character received from the USART */
    WriteUSART(c);

    /* Clear the interrupt flag */
    PIR1bits.RCIF = 0;
}

// BAUD_RATE_GEN is calculated as = [Fosc / (16 * Desired Baudrate)] - 1
#define BAUD_RATE 38400
#define BAUD_RATE_GEN ((CLOCK_FREQ / (16 * BAUD_RATE)) - 1)

static void uartInit(void)
{
    //-------------------------configure USART ---------------------------------------------------------
    // API configures USART for desired parameters:
    // - RX interrupt turned on, TX interrupt turned off
    // - Asynchronous mode
    // - 8 bits
    // - Continuous Receive Enabled
    // - High speed baud rate generator mode
#if defined(USE_OR_MASKS)
    OpenUSART(USART_TX_INT_OFF
            | USART_RX_INT_ON
            | USART_ASYNCH_MODE
            | USART_EIGHT_BIT
            | USART_CONT_RX
            | USART_BRGH_HIGH, BAUD_RATE_GEN);
#else
    OpenUSART(USART_TX_INT_OFF
            & USART_RX_INT_ON
            & USART_ASYNCH_MODE
            & USART_EIGHT_BIT
            & USART_CONT_RX
            & USART_BRGH_HIGH, BAUD_RATE_GEN);
#endif

    /* Enable interrupt priority */
    RCONbits.IPEN = 1;
    /* Make receive interrupt LOW priority */
    IPR1bits.RCIP = 0;
    /* Enable receive interrupt */
    PIE1bits.RCIE = 1;
    /* Enable all LOW priority interrupts */
    INTCONbits.GIEL = 1;
}

static void usbInit(void)
{
    //	The USB specifications require that USB peripheral devices must never source
    //	current onto the Vbus pin.  Additionally, USB peripherals should not source
    //	current on D+ or D- when the host/hub is not actively powering the Vbus line.
    //	When designing a self powered (as opposed to bus powered) USB peripheral
    //	device, the firmware should make sure not to turn on the USB module and D+
    //	or D- pull up resistor unless Vbus is actively powered.  Therefore, the
    //	firmware needs some means to detect when Vbus is being powered by the host.
    //	A 5V tolerant I/O pin can be connected to Vbus (through a resistor), and
    // 	can be used to detect when Vbus is high (host actively powering), or low
    //	(host is shut down or otherwise not supplying power).  The USB firmware
    // 	can then periodically poll this I/O pin to know when it is okay to turn on
    //	the USB module/D+/D- pull up resistor.  When designing a purely bus powered
    //	peripheral device, it is not possible to source current on D+ or D- when the
    //	host is not actively providing power on Vbus. Therefore, implementing this
    //	bus sense feature is optional.  This firmware can be made to use this bus
    //	sense feature by making sure "USE_USB_BUS_SENSE_IO" has been defined in the
    //	HardwareProfile.h file.
#if defined(USE_USB_BUS_SENSE_IO)
    tris_usb_bus_sense = INPUT_PIN; // See HardwareProfile.h
#endif

    //	If the host PC sends a GetStatus (device) request, the firmware must respond
    //	and let the host know if the USB peripheral device is currently bus powered
    //	or self powered.  See chapter 9 in the official USB specifications for details
    //	regarding this request.  If the peripheral device is capable of being both
    //	self and bus powered, it should not return a hard coded value for this request.
    //	Instead, firmware should check if it is currently self or bus powered, and
    //	respond accordingly.  If the hardware has been configured like demonstrated
    //	on the PICDEM FS USB Demo Board, an I/O pin can be polled to determine the
    //	currently selected power source.  On the PICDEM FS USB Demo Board, "RA2" 
    //	is used for	this purpose.  If using this feature, make sure "USE_SELF_POWER_SENSE_IO"
    //	has been defined in HardwareProfile.h, and that an appropriate I/O pin has been mapped
    //	to it in HardwareProfile.h.
#if defined(USE_SELF_POWER_SENSE_IO)
    tris_self_power = INPUT_PIN;	// See HardwareProfile.h
#endif

	UsbOutHandle = 0;
	UsbInHandle = 0;

    USBDeviceInit();	//usb_device.c.  Initializes USB module SFRs and firmware
    					//variables to known states.

#if defined(USB_INTERRUPT)
    USBDeviceAttach();
#endif
}

static void processUsbCommands(void)
{
#if defined(USB_POLLING)
    // Check bus status and service USB interrupts.
    USBDeviceTasks(); // Interrupt or polling method.  If using polling, must call
    // this function periodically.  This function will take care
    // of processing and responding to SETUP transactions
    // (such as during the enumeration process when you first
    // plug in).  USB hosts require that USB devices should accept
    // and process SETUP packets in a timely fashion.  Therefore,
    // when using polling, this function should be called
    // regularly (such as once every 1.8ms or faster** [see
    // inline code comments in usb_device.c for explanation when
    // "or faster" applies])  In most cases, the USBDeviceTasks()
    // function does not take very long to execute (ex: <100
    // instruction cycles) before it returns.
#endif

    // Note: The user application should not begin attempting to read/write over the USB
    // until after the device has been fully enumerated.  After the device is fully
    // enumerated, the USBDeviceState will be set to "CONFIGURED_STATE".
    if ((USBDeviceState < CONFIGURED_STATE)||(USBSuspendControl==1))
        return;

    // As the device completes the enumeration process, the UsbCbInitEP() function will
    // get called.  In this function, we initialize the user application endpoints (in this
    // example code, the user application makes use of endpoint 1 IN and endpoint 1 OUT).
    // The USBGenRead() function call in the UsbCbInitEP() function initializes endpoint 1 OUT
    // and "arms" it so that it can receive a packet of data from the host.  Once the endpoint
    // has been armed, the host can then send data to it (assuming some kind of application software
    // is running on the host, and the application software tries to send data to the USB device).

    // If the host sends a packet of data to the endpoint 1 OUT buffer, the hardware of the SIE will
    // automatically receive it and store the data at the memory location pointed to when we called
    // USBGenRead().  Additionally, the endpoint handle (in this case UsbOutHandle) will indicate
    // that the endpoint is no longer busy.  At this point, it is safe for this firmware to begin reading
    // from the endpoint buffer, and processing the data.  In this example, we have implemented a few very
    // simple commands.  For example, if the host sends a packet of data to the endpoint 1 OUT buffer, with the
    // first byte = 0x80, this is being used as a command to indicate that the firmware should "Toggle LED(s)".
    if (!USBHandleBusy(UsbOutHandle)) {		// Check if the endpoint has received any data from the host.
        switch (OutPacket[0]) {                     // Data arrived, check what kind of command might be in the packet of data.
            default:
                InPacket[0] = OutPacket[0];	// Echo back to the host the command we are fulfilling in the first byte.

                // Now check to make sure no previous attempts to send data to the host are still pending.  If any attemps are still
                // pending, we do not want to write to the endpoint 1 IN buffer again, until the previous transaction is complete.
                // Otherwise the unsent data waiting in the buffer will get overwritten and will result in unexpected behavior.
                if (!USBHandleBusy(UsbInHandle)) {
                    // The endpoint was not "busy", therefore it is safe to write to the buffer and arm the endpoint.
                    // The USBGenWrite() function call "arms" the endpoint (and makes the handle indicate the endpoint is busy).
                    // Once armed, the data will be automatically sent to the host (in hardware by the SIE) the next time the
                    // host polls the endpoint.  Once the data is successfully sent, the handle (in this case UsbInHandle)
                    // will indicate the the endpoint is no longer busy.
                    UsbInHandle = USBGenWrite(USBGEN_EP_NUM,(BYTE*)&InPacket,USBGEN_EP_SIZE);
                }
                break;
        }
    }

    // Re-arm the OUT endpoint for the next packet:
    // The USBGenRead() function call "arms" the endpoint (and makes it "busy").  If the endpoint is armed, the SIE will
    // automatically accept data from the host, if the host tries to send a packet of data to the endpoint.  Once a data
    // packet addressed to this endpoint is received from the host, the endpoint will no longer be busy, and the application
    // can read the data which will be sitting in the buffer.
    UsbOutHandle = USBGenRead(USBGEN_EP_NUM,(BYTE*)&OutPacket, USBGEN_EP_SIZE);
}

void main(void)
{
    plaInit();
    uartInit();
    usbInit();

    putrsUSART("Peripherals initialized\r\n");

    for (;;) {
        processUsbCommands();
    }
}

// ******************************************************************************************************
// ************** USB Callback Functions ****************************************************************
// ******************************************************************************************************
// The USB firmware stack will call the callback functions UsbCbxxx() in response to certain USB related
// events.  For example, if the host PC is powering down, it will stop sending out Start of Frame (SOF)
// packets to your device.  In response to this, all USB devices are supposed to decrease their power
// consumption from the USB Vbus to <2.5mA each.  The USB module detects this condition (which according
// to the USB specifications is 3+ms of no bus activity/SOF packets) and then calls the UsbCbSuspend()
// function.  You should modify these callback functions to take appropriate actions for each of these
// conditions.  For example, in the UsbCbSuspend(), you may wish to add code that will decrease power
// consumption from Vbus to <2.5mA (such as by clock switching, turning off LEDs, putting the
// microcontroller to sleep, etc.).  Then, in the UsbCbWakeFromSuspend() function, you may then wish to
// add code that undoes the power saving things done in the UsbCbSuspend() function.

// The UsbCbSendResume() function is special, in that the USB stack will not automatically call this
// function.  This function is meant to be called from the application firmware instead.  See the
// additional comments near the function.

/******************************************************************************
 * Function:        static void UsbCbSuspend(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Call back that is invoked when a USB suspend is detected
 *
 * Note:            None
 *****************************************************************************/
static void UsbCbSuspend(void)
{
	//Example power saving code.  Insert appropriate code here for the desired
	//application behavior.  If the microcontroller will be put to sleep, a
	//process similar to that shown below may be used:

	//ConfigureIOPinsForLowPower();
	//SaveStateOfAllInterruptEnableBits();
	//DisableAllInterruptEnableBits();
	//EnableOnlyTheInterruptsWhichWillBeUsedToWakeTheMicro();	//should enable at least USBActivityIF as a wake source
	//Sleep();
	//RestoreStateOfAllPreviouslySavedInterruptEnableBits();	//Preferrably, this should be done in the UsbCbWakeFromSuspend() function instead.
	//RestoreIOPinsToNormal();									//Preferrably, this should be done in the UsbCbWakeFromSuspend() function instead.

	//IMPORTANT NOTE: Do not clear the USBActivityIF (ACTVIF) bit here.  This bit is
	//cleared inside the usb_device.c file.  Clearing USBActivityIF here will cause
	//things to not work as intended.
}


/******************************************************************************
 * Function:        static void _USB1Interrupt(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is called when the USB interrupt bit is set
 *					In this example the interrupt is only used when the device
 *					goes to sleep when it receives a USB suspend command
 *
 * Note:            None
 *****************************************************************************/
#if 0
static void __attribute__ ((interrupt)) _USB1Interrupt(void)
{
    #if !defined(self_powered)
        if(U1OTGIRbits.ACTVIF)
        {
            IEC5bits.USB1IE = 0;
            U1OTGIEbits.ACTVIE = 0;
            IFS5bits.USB1IF = 0;

            //USBClearInterruptFlag(USBActivityIFReg,USBActivityIFBitNum);
            USBClearInterruptFlag(USBIdleIFReg,USBIdleIFBitNum);
            //USBSuspendControl = 0;
        }
    #endif
}
#endif

/******************************************************************************
 * Function:        static void UsbCbWakeFromSuspend(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The host may put USB peripheral devices in low power
 *					suspend mode (by "sending" 3+ms of idle).  Once in suspend
 *					mode, the host may wake the device back up by sending non-
 *					idle state signalling.
 *
 *					This call back is invoked when a wakeup from USB suspend
 *					is detected.
 *
 * Note:            None
 *****************************************************************************/
static void UsbCbWakeFromSuspend(void)
{
	// If clock switching or other power savings measures were taken when
	// executing the UsbCbSuspend() function, now would be a good time to
	// switch back to normal full power run mode conditions.  The host allows
	// a few milliseconds of wakeup time, after which the device must be
	// fully back to normal, and capable of receiving and processing USB
	// packets.  In order to do this, the USB module must receive proper
	// clocking (IE: 48MHz clock must be available to SIE for full speed USB
	// operation).
}

/********************************************************************
 * Function:        static void UsbCbSofHandler(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The USB host sends out a SOF packet to full-speed
 *                  devices every 1 ms. This interrupt may be useful
 *                  for isochronous pipes. End designers should
 *                  implement callback routine as necessary.
 *
 * Note:            None
 *******************************************************************/
static void UsbCbSofHandler(void)
{
    // No need to clear UIRbits.SOFIF to 0 here.
    // Callback caller is already doing that.
}

/*******************************************************************
 * Function:        static void UsbCbErrorHandler(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The purpose of this callback is mainly for
 *                  debugging during development. Check UEIR to see
 *                  which error causes the interrupt.
 *
 * Note:            None
 *******************************************************************/
static void UsbCbErrorHandler(void)
{
    // No need to clear UEIR to 0 here.
    // Callback caller is already doing that.

	// Typically, user firmware does not need to do anything special
	// if a USB error occurs.  For example, if the host sends an OUT
	// packet to your device, but the packet gets corrupted (ex:
	// because of a bad connection, or the user unplugs the
	// USB cable during the transmission) this will typically set
	// one or more USB error interrupt flags.  Nothing specific
	// needs to be done however, since the SIE will automatically
	// send a "NAK" packet to the host.  In response to this, the
	// host will normally retry to send the packet again, and no
	// data loss occurs.  The system will typically recover
	// automatically, without the need for application firmware
	// intervention.

	// Nevertheless, this callback function is provided, such as
	// for debugging purposes.
}


/*******************************************************************
 * Function:        static void UsbCbCheckOtherReq(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        When SETUP packets arrive from the host, some
 * 					firmware must process the request and respond
 *					appropriately to fulfill the request.  Some of
 *					the SETUP packets will be for standard
 *					USB "chapter 9" (as in, fulfilling chapter 9 of
 *					the official USB specifications) requests, while
 *					others may be specific to the USB device class
 *					that is being implemented.  For example, a HID
 *					class device needs to be able to respond to
 *					"GET REPORT" type of requests.  This
 *					is not a standard USB chapter 9 request, and
 *					therefore not handled by usb_device.c.  Instead
 *					this request should be handled by class specific
 *					firmware, such as that contained in usb_function_hid.c.
 *
 * Note:            None
 *****************************************************************************/
static void UsbCbCheckOtherReq(void)
{
}//end


/*******************************************************************
 * Function:        static void UsbCbStdSetDscHandler(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The UsbCbStdSetDscHandler() callback function is
 *					called when a SETUP, bRequest: SET_DESCRIPTOR request
 *					arrives.  Typically SET_DESCRIPTOR requests are
 *					not used in most applications, and it is
 *					optional to support this type of request.
 *
 * Note:            None
 *****************************************************************************/
static void UsbCbStdSetDscHandler(void)
{
    // Must claim session ownership if supporting this request
}//end


/******************************************************************************
 * Function:        static void UsbCbInitEP(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is called when the device becomes
 *                  initialized, which occurs after the host sends a
 * 					SET_CONFIGURATION (wValue not = 0) request.  This
 *					callback function should initialize the endpoints
 *					for the device's usage according to the current
 *					configuration.
 *
 * Note:            None
 *****************************************************************************/
static void UsbCbInitEP(void)
{
    USBEnableEndpoint(USBGEN_EP_NUM, USB_OUT_ENABLED|USB_IN_ENABLED|USB_HANDSHAKE_ENABLED|USB_DISALLOW_SETUP);
    UsbOutHandle = USBGenRead(USBGEN_EP_NUM, (BYTE*)&OutPacket, USBGEN_EP_SIZE);
}

/********************************************************************
 * Function:        static void UsbCbSendResume(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        The USB specifications allow some types of USB
 * 					peripheral devices to wake up a host PC (such
 *					as if it is in a low power suspend to RAM state).
 *					This can be a very useful feature in some
 *					USB applications, such as an Infrared remote
 *					control	receiver.  If a user presses the "power"
 *					button on a remote control, it is nice that the
 *					IR receiver can detect this signalling, and then
 *					send a USB "command" to the PC to wake up.
 *
 *					The UsbCbSendResume() "callback" function is used
 *					to send this special USB signalling which wakes
 *					up the PC.  This function may be called by
 *					application firmware to wake up the PC.  This
 *					function will only be able to wake up the host if
 *                  all of the below are true:
 *
 *					1.  The USB driver used on the host PC supports
 *						the remote wakeup capability.
 *					2.  The USB configuration descriptor indicates
 *						the device is remote wakeup capable in the
 *						bmAttributes field.
 *					3.  The USB host PC is currently sleeping,
 *						and has previously sent your device a SET
 *						FEATURE setup packet which "armed" the
 *						remote wakeup capability.
 *
 *                  If the host has not armed the device to perform remote wakeup,
 *                  then this function will return without actually performing a
 *                  remote wakeup sequence.  This is the required behavior,
 *                  as a USB device that has not been armed to perform remote
 *                  wakeup must not drive remote wakeup signalling onto the bus;
 *                  doing so will cause USB compliance testing failure.
 *
 *					This callback should send a RESUME signal that
 *                  has the period of 1-15ms.
 *
 * Note:            This function does nothing and returns quickly, if the USB
 *                  bus and host are not in a suspended condition, or are
 *                  otherwise not in a remote wakeup ready state.  Therefore, it
 *                  is safe to optionally call this function regularly, ex:
 *                  anytime application stimulus occurs, as the function will
 *                  have no effect, until the bus really is in a state ready
 *                  to accept remote wakeup.
 *
 *                  When this function executes, it may perform clock switching,
 *                  depending upon the application specific code in
 *                  UsbCbWakeFromSuspend().  This is needed, since the USB
 *                  bus will no longer be suspended by the time this function
 *                  returns.  Therefore, the USB module will need to be ready
 *                  to receive traffic from the host.
 *
 *                  The modifiable section in this routine may be changed
 *                  to meet the application needs. Current implementation
 *                  temporary blocks other functions from executing for a
 *                  period of ~3-15 ms depending on the core frequency.
 *
 *                  According to USB 2.0 specification section 7.1.7.7,
 *                  "The remote wakeup device must hold the resume signaling
 *                  for at least 1 ms but for no more than 15 ms."
 *                  The idea here is to use a delay counter loop, using a
 *                  common value that would work over a wide range of core
 *                  frequencies.
 *                  That value selected is 1800. See table below:
 *                  ==========================================================
 *                  Core Freq(MHz)      MIP         RESUME Signal Period (ms)
 *                  ==========================================================
 *                      48              12          1.05
 *                       4              1           12.6
 *                  ==========================================================
 *                  * These timing could be incorrect when using code
 *                    optimization or extended instruction mode,
 *                    or when having other interrupts enabled.
 *                    Make sure to verify using the MPLAB SIM's Stopwatch
 *                    and verify the actual signal on an oscilloscope.
 *******************************************************************/
#if 0
static void UsbCbSendResume(void)
{
    static WORD delay_count;

    //First verify that the host has armed us to perform remote wakeup.
    //It does this by sending a SET_FEATURE request to enable remote wakeup,
    //usually just before the host goes to standby mode (note: it will only
    //send this SET_FEATURE request if the configuration descriptor declares
    //the device as remote wakeup capable, AND, if the feature is enabled
    //on the host (ex: on Windows based hosts, in the device manager
    //properties page for the USB device, power management tab, the
    //"Allow this device to bring the computer out of standby." checkbox
    //should be checked).
    if(USBGetRemoteWakeupStatus() == TRUE) {
        //Verify that the USB bus is in fact suspended, before we send
        //remote wakeup signalling.
        if(USBIsBusSuspended() == TRUE) {
            USBMaskInterrupts();

            //Clock switch to settings consistent with normal USB operation.
            UsbCbWakeFromSuspend();
            USBSuspendControl = 0;
            USBBusIsSuspended = FALSE;  //So we don't execute this code again,
                                        //until a new suspend condition is detected.

            //Section 7.1.7.7 of the USB 2.0 specifications indicates a USB
            //device must continuously see 5ms+ of idle on the bus, before it sends
            //remote wakeup signalling.  One way to be certain that this parameter
            //gets met, is to add a 2ms+ blocking delay here (2ms plus at
            //least 3ms from bus idle to USBIsBusSuspended() == TRUE, yeilds
            //5ms+ total delay since start of idle).
            delay_count = 3600U;
            do {
                delay_count--;
            } while(delay_count);

            //Now drive the resume K-state signalling onto the USB bus.
            USBResumeControl = 1;       // Start RESUME signaling
            delay_count = 1800U;        // Set RESUME line for 1-13 ms
            do {
                delay_count--;
            } while(delay_count);
            USBResumeControl = 0;       //Finished driving resume signalling

            USBUnmaskInterrupts();
        }
    }
}
#endif

/*******************************************************************
 * Function:        BOOL USER_USB_CALLBACK_EVENT_HANDLER(
 *                        USB_EVENT event, static void *pdata, WORD size)
 *
 * PreCondition:    None
 *
 * Input:           USB_EVENT event - the type of event
 *                  static void *pdata - pointer to the event data
 *                  WORD size - size of the event data
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is called from the USB stack to
 *                  notify a user application that a USB event
 *                  occured.  This callback is in interrupt context
 *                  when the USB_INTERRUPT option is selected.
 *
 * Note:            None
 *******************************************************************/
BOOL USER_USB_CALLBACK_EVENT_HANDLER(USB_EVENT event, static void *pdata, WORD size)
{
    switch(event) {
        case EVENT_TRANSFER:
            //Add application specific callback task or callback function here if desired.
            break;
        case EVENT_SOF:
            UsbCbSofHandler();
            break;
        case EVENT_SUSPEND:
            UsbCbSuspend();
            break;
        case EVENT_RESUME:
            UsbCbWakeFromSuspend();
            break;
        case EVENT_CONFIGURED:
            UsbCbInitEP();
            break;
        case EVENT_SET_DESCRIPTOR:
            UsbCbStdSetDscHandler();
            break;
        case EVENT_EP0_REQUEST:
            UsbCbCheckOtherReq();
            break;
        case EVENT_BUS_ERROR:
            UsbCbErrorHandler();
            break;
        case EVENT_TRANSFER_TERMINATED:
            //Add application specific callback task or callback function here if desired.
            //The EVENT_TRANSFER_TERMINATED event occurs when the host performs a CLEAR
            //FEATURE (endpoint halt) request on an application endpoint which was
            //previously armed (UOWN was = 1).  Here would be a good place to:
            //1.  Determine which endpoint the transaction that just got terminated was
            //      on, by checking the handle value in the *pdata.
            //2.  Re-arm the endpoint if desired (typically would be the case for OUT
            //      endpoints).
            break;
        default:
            break;
    }
    return TRUE;
}
