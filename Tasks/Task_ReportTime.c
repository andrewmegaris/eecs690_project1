/*--Task_ReportTime.c
 *
 *  Author:			Gary J. Minden
 *	Organization:	KU/EECS/EECS 388
 *  Date:			February 23, 2016
 *
 *  Description:	Reports SysTickCount via printf.
 *
 */


#include	<stddef.h>
#include	<stdbool.h>
#include	<stdint.h>
#include	<stdarg.h>

#include	"Tasks/Task_ReportData.h"

#include	"FreeRTOS.h"
#include	"task.h"

//
//	Reference SysTickCount
//
extern volatile uint32_t xPortSysTickCount;

extern void Task_ReportTime( void *pvParameters ) {

	ReportData_Item		theTimeReport;

	//
	//	No set-up necessary
	//

	//
	//	Delay a while to let ReportData_Queue to initialize.
	//	This is a hack.
	//
	vTaskDelay( 2 * configTICK_RATE_HZ );

	//
	//	Enter task loop
	//
	while ( 1 ) {

		theTimeReport.TimeStamp = xPortSysTickCount;
		theTimeReport.ReportName = 1;
		theTimeReport.ReportValue_0 = 0;
		theTimeReport.ReportValue_1 = 0;

		xQueueSend(  ReportData_Queue, &theTimeReport, 0 );

		vTaskDelay( 2 * configTICK_RATE_HZ );
	}
}
