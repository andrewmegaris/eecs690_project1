/*--Task_I2C7_Handler.c
 *
 *  Author:			Gary J. Minden
 *	Organization:	KU/EECS/EECS 690
 *  Date:			March 13, 2016 (B60313)
 *
 *  Description:	Management task for I2C7 for SensorHub.
 *  				This task is for the TM4C1294 Tiva
 *  				micro-controller.
 *
 *  Modification:	Gary J. Minden
 *  				2016-09-24 (B60924)
 *  				Moved initialization code to an extern
 *  				subroutine.
 *
 */

#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_uart.h"

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdarg.h>

#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"

#include "sensorlib/i2cm_drv.h"

#include "Drivers/Processor_Initialization.h"
#include "Drivers/UARTStdio_Initialization.h"
#include "drivers/uartstdio.h"

#include "Tasks/Task_I2C7_Handler.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "stdio.h"

//
//	SysTickClock Frequency
//
#define SysTickFrequency configTICK_RATE_HZ

//
//	The number of I2C7 Interrupts taken and
//	the number of callbacks taken.
//
uint32_t	I2C7_Interrupts_Nbr = 0;
uint32_t	I2C7_Callbacks_Nbr = 0;

//
// The I2C7 master driver instance data and pointer to instance.
//
tI2CMInstance I2C7_Instance;
extern	tI2CMInstance* I2C7_Instance_Ref = &I2C7_Instance;

bool I2C7_Initialized = false;

uint8_t I2C7WriteData[8] = { 0x10, 0x11, 0x12, 0x13,
								0x14, 0x15, 0x16, 0x17 };

uint32_t I2C7_Status = 0;
extern uint32_t I2C7_HWStatus = 0;

//
// A boolean that is set when an I2C transaction is completed.
//
volatile bool I2C7_SimpleDone = false;

//
// The interrupt handler for the I2C7 handler.
//
extern void I2C7_IntServiceRoutine( ) {

	I2C7_Interrupts_Nbr++;

	//
	// Call the I2C master driver interrupt handler.
	//
	I2CMIntHandler( &I2C7_Instance );
	}

//
// The function that is provided as a callback when I2C
// transactions have completed.
//
void I2CMSimpleCallback(void *pvData, uint_fast8_t ui8Status) {

	I2C7_Callbacks_Nbr++;

	//
	// See if an error occurred.
	//
	if(ui8Status != I2CM_STATUS_SUCCESS) {
		//
		// An error occurred, so handle it here if required.
		//
		UARTprintf( ">>>>I2C7 Error: %02X\n", ui8Status );
	}
	//
	// Indicate that the I2C transaction has completed.
	//
	I2C7_SimpleDone = true;
}

//
//	The initialization subroutine for I2C7
//
extern uint32_t I2C7_Initialization() {

	if ( !I2C7_Initialized ) {

		//
		//	Set up GPIO_D_0 and GPIO_D_1 for I2C7 use
		//
	    SysCtlPeripheralEnable( SYSCTL_PERIPH_GPIOD );
	    SysCtlPeripheralEnable( SYSCTL_PERIPH_I2C7 );

	    GPIOPinConfigure( GPIO_PD0_I2C7SCL );
	    GPIOPinConfigure( GPIO_PD1_I2C7SDA );
	    GPIOPinTypeI2CSCL( GPIO_PORTD_BASE, GPIO_PIN_0 );
	    GPIOPinTypeI2C( GPIO_PORTD_BASE, GPIO_PIN_1 );

	    //
	    //	Make sure UARTStdio is initialized
	    //
	    UARTStdio_Initialization();

	    //
	    //  Initialize the I2C master driver.
	    //	Wait for completion
	    //
	    I2CMInit( &I2C7_Instance, I2C7_BASE, INT_I2C7, 0xff, 0xff, g_ulSystemClock );

	    I2C7_SimpleDone = false;
	 	I2C7_Status = I2CMWrite( &I2C7_Instance, 0x45,
							I2C7WriteData, 1,
							I2CMSimpleCallback, NULL );

	 	while ( !I2C7_SimpleDone ) {
	 	}

	    I2C7_Initialized = true;

		UARTprintf( ">>>>I2C7_Handler; Status: %02X\n", I2C7_Status );

	}

	return( 1 );
}

//================================================================
//
// The simple I2C master driver example.
//
extern void Task_I2C7_Handler( void ) {

	I2C7_Initialization();

    while ( 1 ) {
    	vTaskDelay( (SysTickFrequency * 100) / 1000 );
    }

}

