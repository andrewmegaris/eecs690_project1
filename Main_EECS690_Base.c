/*
 * main.c
 */

#include	"inc/hw_ints.h"
#include	"inc/hw_memmap.h"
#include	"inc/hw_types.h"
#include	"inc/hw_uart.h"

#include	<stddef.h>
#include	<stdbool.h>
#include	<stdint.h>
#include	<stdarg.h>

#include	"driverlib/sysctl.h"
#include	"driverlib/pin_map.h"
#include	"driverlib/gpio.h"

#include	"Drivers/Processor_Initialization.h"
#include	"Drivers/UARTStdio_Initialization.h"
#include	"Drivers/uartstdio.h"

#include	"FreeRTOS.h"
#include	"task.h"

#include	<stdio.h>

extern void Task_Blink_LED_PortN_1( void *pvParameters );
extern void Task_ReportTime( void *pvParameters );
extern void Task_ReportData( void *pvParameters );
extern void Task_I2C7_Handler( void *pvParameters );
extern void Task_SharpDisplay( void *pvParameters );

int main( void ) {

	Processor_Initialization();
	UARTStdio_Initialization();

	//
	//	Create a task to blink LED
	//
	xTaskCreate( Task_Blink_LED_PortN_1, "Blinky", 128, NULL, 1, NULL );
	
	//
	//	Create a task to report data.
	//
	xTaskCreate( Task_ReportData, "ReportData", 512, NULL, 1, NULL );

	//
	//	Create a task to report SysTickCount
	//
	xTaskCreate( Task_ReportTime, "ReportTime", 512, NULL, 1, NULL );

	//
	//	Create a task to initialize I2C7_Handler.
	//
	xTaskCreate( Task_I2C7_Handler, "I2C7_Handler", 1024, NULL, 2, NULL );

	UARTprintf( "FreeRTOS Starting!\n" );

	//
	//	Start FreeRTOS Task Scheduler
	//
	vTaskStartScheduler();

	while ( 1 ) {

	}

}
