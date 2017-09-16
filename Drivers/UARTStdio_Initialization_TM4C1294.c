/*--UARTStdio_Initialization.c
 *
 * 		Author: 		Gary J. Minden
 *		Organization:	KU/EECS/EECS 388
 *		Date:			2014-10-18
 *		Version:		1.0
 *
 *		Description:	This file defines the subroutin to
 *						initialize the UARTStdio sub-system
 *
 */
 
//*****************************************************************************
//
//! \addtogroup KU_EECS_690_SensorHub
//! @{
//
//*****************************************************************************

#include	"inc/hw_ints.h"
#include	"inc/hw_memmap.h"
#include	"inc/hw_types.h"
#include	"inc/hw_uart.h"

#include	<stddef.h>
#include	<stdbool.h>
#include	<stdint.h>
#include	<stdarg.h>

#include	"driverlib/pin_map.h"
#include	"driverlib/gpio.h"
#include	"driverlib/sysctl.h"
#include	"driverlib/uart.h"

#include	"Drivers/Processor_Initialization.h"
#include	"Drivers/uartstdio.h"
#include	"Drivers/UARTStdio_Initialization.h"

uint32_t		UARTStdioInitFlag = 0;

//*****************************************************************************
//
//	Define the UARTStdio initization subroutine. Initialize UART_0 on
//		PortA<1..0>.
//
//*****************************************************************************

uint32_t		theSystemClockFrequency = 0;

//*****************************************************************************
//
//!	The UARTStdio initialization subroutine configures PortA<1..0> for UART0
//!	and initializes UARTStdio for 115,200 bits per second.
//
//*****************************************************************************

extern uint32_t UARTStdio_Initialization() {

	if ( UARTStdioInitFlag == 0 ) {

		Processor_Initialization();

	    //
	    // Enable UART0, to be used as a serial console.
	    //
	    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

	    GPIOPinConfigure( GPIO_PA0_U0RX );
	    GPIOPinConfigure( GPIO_PA1_U0TX );
	    GPIOPinTypeUART( GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1 );

	    UARTClockSourceSet( UART0_BASE, UART_CLOCK_SYSTEM );

	    //
	    //	Initialize UARTStdio
	    //
	    UARTStdioConfig( 0, 115200, g_ulSystemClock );

		UARTStdioInitFlag = 1;			// Set flag indicating initialization complete.
	}

    return( 1 );

 }

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************

