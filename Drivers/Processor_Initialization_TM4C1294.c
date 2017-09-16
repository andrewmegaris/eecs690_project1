/*--Processor_Initialization_TM4C1294.c
 *
 * 		Author: 		Gary J. Minden
 *		Organization:	KU/EECS/EECS 388
 *		Date:			2014-10-18
 *		Version:		1.0
 *
 *		Description:	This file defines the explicit initialization for
 *							the TM4C1294 Tiva micro-controller.
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
#include	"driverlib/sysctl.h"
#include	"driverlib/FPU.h"

#include	"Drivers/Processor_Initialization.h"
 
//*****************************************************************************
//
// The speed of the processor clock in Hertz, which is therefore the speed of the
// clock that is fed to the peripherals.
//
//*****************************************************************************
extern		uint32_t		g_ulSystemClock = 0;

			uint32_t		ProcessorInitFlag = 0;
			uint32_t		FPUInitFlag = 0;
//*****************************************************************************
//
//!	The processor initialization subroutine initializes the micro-controller
//!	system clock and sets a global variable for other tasks.
//
//*****************************************************************************

extern uint32_t Processor_Initialization() {
 
	if ( ProcessorInitFlag == 0 ) {
		//
		//		Initialize the system clock on the TM4C1294 to 120 MHz
		//
		g_ulSystemClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
												SYSCTL_OSC_MAIN |
												SYSCTL_USE_PLL |
												SYSCTL_CFG_VCO_480), 120000000);
		//
		//		Enable the floating point processor on the TM4C1294 processor
		//
		/*On the TM4C1294 the FPU should be enabled because some compilers will use floating-
		 * point registers, even for non-floating-point code.  If the FPU is not
		 * enabled this will cause a fault.  This also ensures that floating-
		 * point operations could be added to this application and would work
		 * correctly and use the hardware floating-point unit. Finally, lazy
		 * stacking is enabled for interrupt handlers.  This allows floating-
		 * point instructions to be used within interrupt handlers, but at the
		 * expense of extra stack usage.
		 */

		//--GJM	B60312		Disabled until FPU can be checked
		//
//		FPUEnable();
//		FPUStackingEnable();
		//--GJM

		ProcessorInitFlag = 1;			// Set flag indicating processor initialization complete.
		FPUInitFlag = 0;				// Set flag indicating FPU initialization complete.
	}

    return( 1 );
    
 }

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************


