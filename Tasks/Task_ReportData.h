/*--Task_ReportData.h
 *
 *  Author:			Gary J. Minden
 *	Organization:	KU/EECS/EECS 690
 *  Date:			March 13, 2016 (B60313)
 *
 *  Description:	Transmits data via UART to PC.
 *
 *  Modification:	2016-09-22 (B60922)
 *  				Changed number of value to 4 from 2.
 *
 */

#ifndef TASKS_TASK_REPORTDATA_H_
#define TASKS_TASK_REPORTDATA_H_

#include	<stddef.h>
#include	<stdbool.h>
#include	<stdint.h>
#include	<stdarg.h>

#include	"utils/uartstdio.h"		//	Access original *.h file as the uartstdio.c file does

#include	"FreeRTOS.h"
#include	"task.h"
#include	"queue.h"

//
//	Reference SysTickCount
//
extern volatile uint32_t xPortSysTickCount;

//
//	Define the ReportData Task
//
extern void Task_ReportData( void *pvParameters );

//
//	Define the ReportData Queue
//
extern QueueHandle_t ReportData_Queue;
//
//	Define a structure to hold a data report
//
typedef struct ReportData_Item {	uint32_t	TimeStamp;
									uint32_t	ReportName;
									uint32_t	ReportValue_0;
									uint32_t	ReportValue_1;
									uint32_t	ReportValue_2;
									uint32_t	ReportValue_3; } ReportData_Item;

typedef enum ReportData_OutputFormat { Excel_CSV, Mathematica_List } ReportData_OutputFormat;

#endif /* TASKS_TASK_REPORTDATA_H_ */
