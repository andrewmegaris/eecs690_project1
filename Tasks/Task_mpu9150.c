#include	"inc/hw_ints.h"
#include	"inc/hw_memmap.h"
#include	"inc/hw_types.h"
#include	"inc/hw_uart.h"

#include	<stddef.h>
#include	<stdbool.h>
#include	<stdint.h>
#include	<stdarg.h>
#include 	<stdio.h>

#include	"driverlib/sysctl.h"
#include	"driverlib/pin_map.h"
#include	"driverlib/gpio.h"

#include	"FreeRTOS.h"
#include	"task.h"
#include 	"driverlib/timer.h"
#include	"Drivers/uartstdio.h"

#include 	"semphr.h"
#include 	"Task_I2C7_Handler.h"

#include	"sensorlib/ak8975.h"
#include 	"sensorlib/mpu9150.h"

#include	"Tasks/Task_ReportData.h"

#include	"Magnetometer.h"

//global variables
extern bool isMPU9150Initialized;
float fMagneto[3];
tMPU9150 sMPU9150;
volatile bool g_bMPU9150Done;
extern tI2CMInstance I2C7_Instance;
ReportData_Item	magnetometerReport;
extern volatile uint32_t xPortSysTickCount;
extern void I2C7_IntServiceRoutine(void);

extern void MagnetometerInit(){

	//check if the MPU9150 is already initialized
	if (!isMPU9150Initialized){
		//add the interrupt handler to the vector table
		IntRegister(INT_I2C7, I2C7_IntServiceRoutine);

		//initialize I2C master
		I2C7_Initialization();

		//Initialize the mpu9150
		uint_fast8_t stat9150;
		stat9150 = MPU9150Init(&sMPU9150, I2C7_Instance_Ref, 0x68, MPU9150Callback1, 0);
		while(!g_bMPU9150Done){}

		//set the MPU9150 initialization flag to true
		isMPU9150Initialized = true;

		UARTprintf( ">>>>MPU9150Init; Status: %02X\n", stat9150 );
	}
}

/**
*	Set a status flag when the data is ready to be read.
*	@param pvCallbackData FreeRTOS required pointer
*	@param ui8Status is a flag that is raised when the MPU9150 is ready to be read 
*/
void MPU9150Callback1(void *pvCallbackData, uint_fast8_t ui8Status) {

	// See if an error occurred.s
	if (ui8Status != I2CM_STATUS_SUCCESS) {

		// An error occurred, so handle it here if required.
		UARTprintf( ">>>>I2C7 in 9150 callback Error: %02X\n", ui8Status );
	}

	// Indicate that the MPU9150 transaction has completed.
	g_bMPU9150Done = true;
}

extern void Task_Magnetometer(void *pvParameters) {

	//initialize the magnetometer
	MagnetometerInit();

	//FreeRTOS task loop
	while (1) {
		//Read the data from the magnetometer
		g_bMPU9150Done = false;
		MPU9150DataRead(&sMPU9150, MPU9150Callback1, 0);
		while(!g_bMPU9150Done){}
		MPU9150DataMagnetoGetFloat(&sMPU9150, &fMagneto[0], &fMagneto[1], &fMagneto[2]);

		//load the data into the magnetometer report
		magnetometerReport.TimeStamp = xPortSysTickCount;
		magnetometerReport.ReportName = 130;
		magnetometerReport.ReportValue_0 = 	(int)(fMagneto[0] * 1000000);
		magnetometerReport.ReportValue_1 = 	(int)(fMagneto[1] * 1000000);
		magnetometerReport.ReportValue_2 =	(int)(fMagneto[2] * 1000000);
		magnetometerReport.ReportValue_3 = 	0;

		//send the report data to the data report task
		xQueueSend(  ReportData_Queue, &magnetometerReport, 0 );

		//FreeRTOS task delay
		vTaskDelay((10000 * configTICK_RATE_HZ) / 10000);
	}
}
