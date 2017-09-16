#ifndef MAGNETOMETER_H
#define MAGNETOMETER_H

//!initialization flag 
//!"extern bool isMPU9150Initialized = false;"
//!must be declared in the main file for the MPU9150initialization functions.
extern bool isMPU9150Initialized;

//function prototypes
/**
*	This function initializes the MPU9150, 12C, and UART if they
*	have not already been initialized
*/
extern void MagnetometerInit();

/**
*	Set a status flag when the data is ready to be read.
*	@param pvCallbackData FreeRTOS required pointer
*	@param ui8Status is a flag that is raised when the MPU9150 is ready to be read 
*/
void MPU9150Callback1(void *pvCallbackData, uint_fast8_t ui8Status);

/**
*	FreeRTOS task to prepare and read data from the MPU9150 magnetometer
*	and send the data to ReportData
*	@param pvParameters FreeRTOS required pointer
*/
extern void Task_Magnetometer(void *pvParameters);

#endif
