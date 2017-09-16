/*--Task_I2C7_Handler.h
 *
 *  Author:			Gary J. Minden
 *	Organization:	KU/EECS/EECS 690
 *  Date:			2016-09-24 (B60924)
 *
 *  Description:	API for Task_I2C7_Handler
 *
 */

#ifndef TASK_I2C7_HANDLER_H_
#define TASK_I2C7_HANDLER_H_

#include	<stddef.h>
#include	<stdbool.h>
#include	<stdint.h>
#include	<stdarg.h>

//
//	Define the I2C7_Initialization subroutine
//
extern uint32_t I2C7_Initialization();
//
//	Define the Task_I2C7_Handler instance
//
extern	tI2CMInstance* I2C7_Instance_Ref;

#endif /* TASK_I2C7_HANDLER_H_ */
