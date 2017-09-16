//--Sharp96x96.c - Tiva interface to the Sharp Memory LCD BoosterPack.
//
// Copyright (c) 2015 Donald Rich.
// Portions derived from TI Sharp96x96.c in EXP430FR5969 Sharp LCD sample code.
//
// This software is supplied solely as a programming example.
//
// This file is part of SharpLcdTivaInterface.
//
// SharpLcdTivaInterface is free software: you can redistribute it and/or
// modify it under the terms of the GNU General Public License as published
// by the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// SharpLcdTivaInterface is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with SharpLcdTivaInterface. If not, see
// <http://www.gnu.org/licenses/>.
//
//*****************************************************************************
/*
 *
 *  Author:			Gary J. Minden
 *  Organization:	KU/EECS/EECS 690
 *  Date:			August 7, 2017
 *  Description:	Task interface to the Tiva Sharp96x96
 *					display
 *
 *  Modification:	Adapted from TI's sharp.h example program.
 *					Further adapted from Brandon Givens'
 *					EECS 690 class project.
 *
 *	Modification:
 *	Author:			Gary J. Minden
 *  Organization:	KU/EECS/EECS 690
 *	Date:			2017-09-07 (B70907)
 *	Description:	Changed code from a FreeRTOS Task to a
 *					standalone initialization subroutine.
 *					Added DOxygen comments.
 *
 */

#include	"inc/hw_ints.h"
#include	"inc/hw_memmap.h"
#include	"inc/hw_types.h"

#include	<stddef.h>
#include	<stdbool.h>
#include	<stdint.h>
#include	<stdarg.h>

#include	"driverlib/gpio.h"
#include	"driverlib/pin_map.h"
#include	"driverlib/ssi.h"
#include	"driverlib/sysctl.h"

#include	"grlib/grlib.h"

#include	"Drivers/Processor_Initialization.h"
#include	"Drivers/UARTStdio_Initialization.h"
#include	"drivers/uartstdio.h"

#include	"Drivers/Sharp96x96_Display.h"

#include	"FreeRTOS.h"
#include	"task.h"
#include	"timers.h"

// LCD Screen Size
#define VERTICAL_MAX 96
#define HORIZONTAL_MAX 96

//*****************************************************************************
//
// Macros for the Display Driver
//
//*****************************************************************************
#define 	BLACK 						0x00
#define 	WHITE						0xFF
#define 	SEND_TOGGLE_VCOM_COMMAND	0x01
#define 	SKIP_TOGGLE_VCOM_COMMAND	0x00
#define 	TRAILER_BYTE				0x00
#define		VCOM_TOGGLE_BIT				0x40
#define		CMD_CHANGE_VCOM				0x00
#define 	CMD_CLEAR_SCREEN			0x20
#define 	CMD_WRITE_LINE				0x80

//
// Ports, port base addresses, pins, and SSI base address
// were modified by Brandon Givens to make this driver
// compatible with the TM4C1294XL
//
// Ports for TM4C1294XL connections to LCD
#define		SPI_SIMO_PORT				SYSCTL_PERIPH_GPIOQ
#define		SPI_CLK_PORT				SYSCTL_PERIPH_GPIOQ
#define		DISP_PORT					SYSCTL_PERIPH_GPIOD
#define		POWER_PORT					SYSCTL_PERIPH_GPIOD
#define		SPI_CS_PORT					SYSCTL_PERIPH_GPIOD

// Port base address for TM4C1294XL connections to LCD
#define		SPI_SIMO_PORT_BASE			GPIO_PORTQ_BASE
#define		SPI_CLK_PORT_BASE			GPIO_PORTQ_BASE
#define		DISP_PORT_BASE				GPIO_PORTD_BASE
#define		POWER_PORT_BASE				GPIO_PORTD_BASE
#define		SPI_CS_PORT_BASE			GPIO_PORTD_BASE

// Pins for TM4C1294XL connections to LCD
#define		SPI_SIMO_PIN				GPIO_PIN_2
#define		SPI_CLK_PIN					GPIO_PIN_0
#define		DISP_PIN					GPIO_PIN_4
#define		POWER_PIN					GPIO_PIN_2
#define		SPI_CS_PIN					GPIO_PIN_5

// Definition of SSI base address to be used for SPI communication
#define		LCD_SSI_PORT				SYSCTL_PERIPH_SSI3
#define		LCD_SSI_BASE				SSI3_BASE

//
//	Define VCOM bit and VCOM state.
static uint8_t	VCOMbit = 0x40;
static uint8_t	currentVCOMbit = 0x40;

// Caution: The following declaration only works if HORIZONTAL_MAX is a
// multiple of 8!
static uint8_t DisplayBuffer[VERTICAL_MAX][HORIZONTAL_MAX/8];

//*****************************************************************************
//
// Clears CS line
//
// This function clears the Chip Select (CS) line
//
// \return None
//
//*****************************************************************************
static void ClearCS(void){
	GPIOPinWrite(SPI_CS_PORT_BASE, SPI_CS_PIN, 0);
}

//*****************************************************************************
//
// Set CS line
//
// This function sets the Chip Select (CS) line
//
// \return None
//
//*****************************************************************************
static void SetCS(void) {
	GPIOPinWrite(SPI_CS_PORT_BASE, SPI_CS_PIN, SPI_CS_PIN);
}

//*****************************************************************************
//
// Get CS line
//
// This function gets the Chip Select (CS) line
//
// \return long
//
//*****************************************************************************
static long GetCS(void) {
	return GPIOPinRead(SPI_CS_PORT_BASE, SPI_CS_PIN);
}

// Delay counts used to enforce SPI timing constraints.
static uint32_t twoUsDelayCount;
static uint32_t sixUsDelayCount;

//*****************************************************************************
//
// Waits until the SPI communication with the LCD is finished a command to
// the LCD Driver
//
// \param None
//
// \return None
//*****************************************************************************
static void WaitUntilLcdWriteDone(void) {
	while (SSIBusy(LCD_SSI_BASE)) {}
}

//*****************************************************************************
//
// Writes command or data to the LCD Driver
//
// \param ucCmdData is the 8 or 16 bit command to send to the LCD driver
// Uses the SET_DATA macro
//
// \return None
//
//*****************************************************************************
static void WriteByte(uint8_t byte) {
	WaitUntilLcdWriteDone();
	SSIDataPut(LCD_SSI_BASE, byte);
}

static void WriteCommand( uint8_t *command, uint8_t length) {
	uint32_t i;
	uint8_t *bytePointer = command;

	SetCS();

	//
	// Ensure a 6us min delay to meet the LCD's tsSCS
	SysCtlDelay(sixUsDelayCount);

//	UARTprintf( ">>>>Sharp: Ready to write command!\n" );

	for(i = 0; i < length; i++) {
		WriteByte(*bytePointer++);
	}

	// Wait for last byte to be sent
	WaitUntilLcdWriteDone();

	// Ensure a 2us min delay to meet the LCD's thSCS
	SysCtlDelay(twoUsDelayCount);
	ClearCS();

	// Ensure a 2us delay to meet the LCD's twSCSL
	SysCtlDelay(twoUsDelayCount);
}
//*****************************************************************************
//
//! Send toggle VCOM command.
//!
//! This function toggles the state of VCOM which prevents a DC bias from being
//! built up within the panel.
//!
//! \return None.
//
//*****************************************************************************
static uint8_t ToggleVComCommand[] = {
						CMD_CHANGE_VCOM,
						TRAILER_BYTE
					};

void SendToggleVCOMCommand(void) {

	// If CS is low, not currently sending command
	if( !GetCS() )  {

		//
		// Toggle the VCOM bit.
		currentVCOMbit ^= VCOMbit;
		ToggleVComCommand[0] = currentVCOMbit;
		WriteCommand( ToggleVComCommand, 2 );

//		UARTprintf( ">>>>VCOM: %02X\n", ToggleVComCommand[0] );
	}
}

//	Original:
//	Enable the timer for periodic interrupts to drive the VCOM inversion
// 	and other general timed functions.
//	The clock is configured to timeout every half second to give a 1 hz
//	alternation of VCOM.
//
//	Modified -- GJM:
//	Use a FreeRTOS software timer to handle VCOM toggles.
//	Set a flag, VCOM_InversionPending, if the SSI is busy
//	when the timer expires.

TimerHandle_t	VCOM_Timer;

uint32_t		VCOM_Timer_Count = 0;
bool			VCOM_InversionPending = false;

static void VCOM_TimerHandler( TimerHandle_t xTimer ) {

	VCOM_Timer_Count++;			//	Increment timer count

	if ( !GetCS() ) {
		SendToggleVCOMCommand();
	} else {
		VCOM_InversionPending = true;
	}
}

//*****************************************************************************
//
//! Initializes the display driver.
//!
//! This function initializes the Sharp96x96 display. This function
//! configures the GPIO pins used to control the LCD display when the basic
//! GPIO interface is in use. On exit, the LCD has been reset and is ready to
//! receive command and data writes.
//!
//! \return None.
//
//*****************************************************************************
extern int32_t Sharp96x96_InitizeDisplay( void ) {

	//
	//	Make sure Processor and UARTStdio are initialized
	//
	Processor_Initialization();
	UARTStdio_Initialization();

	// Clock frequency is in g_ulSystemClock for TM4C1294.
	// Clock frequency / 3 (cycles per Delay count).
	twoUsDelayCount = ((g_ulSystemClock / (1000000*3)) * 2) + 1;
	sixUsDelayCount  = ((g_ulSystemClock / (1000000*3)) * 6) + 1;

	// Configure power port for output to power the LCD Display.
	SysCtlPeripheralEnable(POWER_PORT);
	GPIOPinTypeGPIOOutput(POWER_PORT_BASE, POWER_PIN);
	GPIOPinWrite(POWER_PORT_BASE, POWER_PIN, POWER_PIN);

	// Configure the LCD control port for output to enable the LCD Display.
	SysCtlPeripheralEnable(DISP_PORT);
	GPIOPinTypeGPIOOutput(DISP_PORT_BASE, DISP_PIN);
	GPIOPinWrite(DISP_PORT_BASE, DISP_PIN, DISP_PIN);

	UARTprintf( ">>>>InitDisplay: Power and Display configured!\n" );

	// Configure the SPI port and pin for SPI Chip Select for the LCD Display.
	SysCtlPeripheralEnable(LCD_SSI_PORT);
	SysCtlPeripheralEnable(SPI_SIMO_PORT);
	SysCtlPeripheralEnable(SPI_CLK_PORT);
	SysCtlPeripheralEnable(SPI_CS_PORT);

	// Setup SPI_CS pin.
	GPIOPinTypeGPIOOutput( SPI_CS_PORT_BASE, SPI_CS_PIN );
	ClearCS();

	UARTprintf( ">>>>InitDisplay: CS configured!\n" );

	// Set the GPIO pin directions
	// Split if SPI_SIMO and SPI_CLK are on different GPIO ports.
	GPIOPinTypeGPIOOutput( SPI_SIMO_PORT_BASE,
							SPI_SIMO_PIN | SPI_CLK_PIN );

	// CS is controlled by a simple GPIO output signal.
	GPIOPinConfigure(GPIO_PQ0_SSI3CLK);
	GPIOPinConfigure(GPIO_PQ2_SSI3XDAT0);

	UARTprintf( ">>>>InitDisplay: GPIO/SSI configured!\n" );

	GPIOPinTypeSSI(SPI_SIMO_PORT_BASE, SPI_SIMO_PIN | SPI_CLK_PIN);

	SSIClockSourceSet(LCD_SSI_BASE, SSI_CLOCK_SYSTEM);
	SSIConfigSetExpClk(LCD_SSI_BASE, g_ulSystemClock, SSI_FRF_MOTO_MODE_0,
						SSI_MODE_MASTER, 1000000, 8);

	SSIEnable(LCD_SSI_BASE);

	//
	//	Initialize the VCOM_Timer.
	VCOM_Timer = xTimerCreate ( "VCOM",
								pdMS_TO_TICKS( 50 ),
								pdTRUE,
								( void * ) 1,
								VCOM_TimerHandler );
	xTimerStart( VCOM_Timer, 0 );

	return( 1 );
}


static uint8_t ClearCommand[] = {	CMD_CLEAR_SCREEN,
									TRAILER_BYTE
								};
static void ClearScreen(void) {

	WriteCommand ( ClearCommand, 2 );

}

extern void Sharp96x96_disable(void) {
	// Faulting here, figure out why
	GPIOPinWrite(POWER_PORT_BASE, POWER_PIN, 0);
	GPIOPinWrite(DISP_PORT_BASE, DISP_PIN, 0);
}

extern void Sharp96x96_enable(void) {
	GPIOPinWrite(POWER_PORT_BASE, POWER_PIN, POWER_PIN);
	GPIOPinWrite(DISP_PORT_BASE, DISP_PIN, DISP_PIN);
}

//*****************************************************************************
//
//! Draws a pixel on the screen.
//!
//! \param pvDisplayData is a pointer to the driver-specific data for this
//! display driver.
//! \param lX is the X coordinate of the pixel.
//! \param lY is the Y coordinate of the pixel.
//! \param ulValue is the color of the pixel.
//!
//! This function sets the given pixel to a particular color. The coordinates
//! of the pixel are assumed to be within the extents of the display.
//!
//! \return None.
//
//*****************************************************************************
static void PixelDraw(void *pvDisplayData, int32_t lX, int32_t lY,
						uint32_t ulValue) {
	if( ClrBlack == ulValue ) {
		DisplayBuffer[lY][lX>>3] &= ~(0x80 >> (lX & 0x7));
	} else {
		DisplayBuffer[lY][lX>>3] |= (0x80 >> (lX & 0x7));
	}
}

//*****************************************************************************
//
//! Draws a horizontal sequence of pixels on the screen.
//!
//! \param pvDisplayData is a pointer to the driver-specific data for this
//! display driver.
//! \param lX is the X coordinate of the first pixel.
//! \param lY is the Y coordinate of the first pixel.
//! \param lX0 is sub-pixel offset within the pixel data, which is valid for 1
//! or 4 bit per pixel formats.
//! \param lCount is the number of pixels to draw.
//! \param lBPP is the number of bits per pixel; must be 1, 4, or 8.
//! \param pucData is a pointer to the pixel data. For 1 and 4 bit per pixel
//! formats, the most significant bit(s) represent the left-most pixel.
//! \param pucPalette is a pointer to the palette used to draw the pixels.
//!
//! This function draws a horizontal sequence of pixels on the screen, using
//! the supplied palette. For 1 bit per pixel format, the palette contains
//! pre-translated colors; for 4 and 8 bit per pixel formats, the palette
//! contains 24-bit RGB values that must be translated before being written to
//! the display.
//!
//! \return None.
//
//*****************************************************************************
static void DrawMultiple(void *pvDisplayData, int32_t lX,
						int32_t lY, int32_t lX0, int32_t lCount, int32_t lBPP,
						const uint8_t *pucData, const uint8_t *pucPalette) {
	uint8_t *pData = &DisplayBuffer[lY][lX>>3];
	uint32_t xj = 0;

	//Write bytes of data to the display buffer
	for(xj=0;xj<(lCount >> 3);xj++) {
		*pData++ = *pucData++;
	}

	//Write last data byte to the display buffer
	*pData = (*pData & (0xFF >> (lCount & 0x7))) | *pucData;
}

//*****************************************************************************
//
//! Draws a horizontal line.
//!
//! \param pvDisplayData is a pointer to the driver-specific data for this
//! display driver.
//! \param lX1 is the X coordinate of the start of the line.
//! \param lX2 is the X coordinate of the end of the line.
//! \param lY is the Y coordinate of the line.
//! \param ulValue is the color of the line.
//!
//! This function draws a horizontal line on the display. The coordinates of
//! the line are assumed to be within the extents of the display.
//!
//! \return None.
//
//*****************************************************************************
static void LineDrawH(void *pvDisplayData, int32_t lX1, int32_t lX2,
						int32_t lY, uint32_t ulValue) {
	uint16_t xi = 0;
	uint16_t x_index_min = lX1>>3;
	uint16_t x_index_max = lX2>>3;
	uint8_t *pucData, ucfirst_x_byte, uclast_x_byte;

	//calculate first byte
	//mod by 8 and shift this # bits
	ucfirst_x_byte = (0xFF >> (lX1 & 0x7));

	//calculate last byte
	//mod by 8 and shift this # bits
	uclast_x_byte = (0xFF << (7-(lX2 & 0x7)));

	//check if more than one data byte
	if(x_index_min != x_index_max) {
		//set buffer to correct location
		pucData = &DisplayBuffer[lY][x_index_min];
		//black pixels (clear bits)
		if(ClrBlack == ulValue) {
			//write first byte
			*pucData++ &= ~ucfirst_x_byte;
			//write middle bytes
			for(xi = x_index_min; xi < x_index_max-1; xi++) {
				*pucData++ = 0x00;
			}
			//write last byte
			*pucData &= ~uclast_x_byte;
		} else {

		//white pixels (set bits)

			//write first byte
			*pucData++ |= ucfirst_x_byte;
			//write middle bytes
			for(xi = x_index_min; xi < x_index_max-1; xi++)	{
				*pucData++ = 0xFF;
			}
			//write last byte
			*pucData |= uclast_x_byte;
		}
	} else {

		//only one data byte
		//calculate value of single byte
		ucfirst_x_byte &= uclast_x_byte;
		//set buffer to correct location
		pucData = &DisplayBuffer[lY][x_index_min];
		//draw black pixels (clear bits)
		if(ClrBlack == ulValue) {
			*pucData++ &= ~ucfirst_x_byte;
		} else {
			//white pixels (set bits)
			*pucData++ |= ucfirst_x_byte;
		}
	}
}

//*****************************************************************************
//
//! Draws a vertical line.
//!
//! \param pvDisplayData is a pointer to the driver-specific data for this
//! display driver.
//! \param lX is the X coordinate of the line.
//! \param lY1 is the Y coordinate of the start of the line.
//! \param lY2 is the Y coordinate of the end of the line.
//! \param ulValue is the color of the line.
//!
//! This function draws a vertical line on the display. The coordinates of the
//! line are assumed to be within the extents of the display.
//!
//! \return None.
//
//*****************************************************************************
static void LineDrawV(void *pvDisplayData, int32_t lX, int32_t lY1,
						int32_t lY2, uint32_t ulValue) {
	uint16_t yi = 0;
	uint16_t x_index = lX>>3;
	uint8_t data_byte;

	//calculate data byte
	//mod by 8 and shift this # bits
	data_byte = (0x80 >> (lX & 0x7));

	//write data to the display buffer
	for(yi = lY1; yi <= lY2; yi++) {
		//black pixels (clear bits)
		if(ClrBlack == ulValue) {
			DisplayBuffer[yi][x_index] &= ~data_byte;
		} else {

			//white pixels (set bits)
			DisplayBuffer[yi][x_index] |= data_byte;
		}
	}
}

//*****************************************************************************
//
//! Fills a rectangle.
//!
//! \param pvDisplayData is a pointer to the driver-specific data for this
//! display driver.
//! \param pRect is a pointer to the structure describing the rectangle.
//! \param ulValue is the color of the rectangle.
//!
//! This function fills a rectangle on the display. The coordinates of the
//! rectangle are assumed to be within the extents of the display, and the
//! rectangle specification is fully inclusive (in other words, both sXMin and
//! sXMax are drawn, along with sYMin and sYMax).
//!
//! \return None.
//
//*****************************************************************************
static void RectFill(void *pvDisplayData, const tRectangle *pRect,
						uint32_t color) {
	uint32_t y;

	for(y = pRect->i16YMin; y <= pRect->i16YMax; y++) {
		LineDrawH(0, pRect->i16XMin, pRect->i16XMax, y, color);
	}
}

//*****************************************************************************
//
//! Translates a 24-bit RGB color to a display driver-specific color.
//!
//! \param pvDisplayData is a pointer to the driver-specific data for this
//! display driver
//! \param ulValue is the 24-bit RGB color. The least-significant byte is the
//! blue channel, the next byte is the green channel, and the third byte is
//! the red channel.
//!
//! This fucntion translates a 24-bit RGB color into a value that can be written
//! into the display's frame buffer in order to reproduce that color, or the
//! closest possible approximation of that color. This particular driver
//! requires the 8-8-8 24 bit RGB color to convert into mono color
//! 1 = White, 0 = Black
//!
//! \return Returns the display-driver specific color
//
//*****************************************************************************
static uint32_t ColorTranslate(void *pvDisplayData,
								uint32_t ulValue) {
	//
	// Translate from a 24-bit RGB color to mono color.
	//
	return(((ulValue != 0) ? ulValue = 1 : ulValue ));
}

//*******************************************************************************
//
//! Reverses the bit order.- Since the bit reversal function is called
//! frequently by the several driver function this function is implemented
//! to maximize code execution
// Taken from TI Sharp96x96.c in EXP430FR5969 Sharp LCD sample code.
//
//*******************************************************************************
static const uint8_t reverse_data[] = {	0x0, 0x8, 0x4, 0xC,
										0x2, 0xA, 0x6, 0xE,
										0x1, 0x9, 0x5, 0xD,
										0x3, 0xB, 0x7, 0xF };
static uint8_t Reverse( uint8_t x ) {
	uint8_t b = 0;
	b = reverse_data[x & 0xF]<<4;
	b |= reverse_data[(x & 0xF0)>>4];
	return b;
}

//*****************************************************************************
//
//! Flushes any cached drawing operations.
//!
//! \param pvDisplayData is a pointer to the driver-specific data for this
//! display driver.
//!
//!
//! This functions flushes any cached drawing operations to the display. This
//! is useful when a local frame buffer is used for drawing operations, and the
//! flush would copy the local frame buffer to the display.
//!
//! \return None.
//
//*****************************************************************************
static void Flush (void *pvDisplayData) {
	uint8_t *pucData = &(DisplayBuffer[0][0]);
	int32_t xi =0;
	int32_t xj = 0;
	uint8_t command = CMD_WRITE_LINE;

	SetCS();
	// Ensure a 6us min delay to meet the LCD's tsSCS
	SysCtlDelay(sixUsDelayCount);
	WriteByte(command);
	for(xj=0; xj<VERTICAL_MAX; xj++) {
		WriteByte(Reverse(xj + 1));
		for(xi=0; xi<(HORIZONTAL_MAX>>3); xi++) {
			WriteByte(*(pucData++));
		}

		WriteByte(TRAILER_BYTE);
	}

	WriteByte(TRAILER_BYTE);

	// Wait for last byte to be sent, then drop SCS
	WaitUntilLcdWriteDone();
	// Ensure a 2us min delay to meet the LCD's thSCS
	SysCtlDelay(twoUsDelayCount);
	ClearCS();

	// Ensure a 2us min delay to meet the LCD's twSCSL
	SysCtlDelay(twoUsDelayCount);

	//
	//	If a VCOM_Timer event occured while flushing DisplayBuffer,
	//	toggle the VCOM bit.
	if ( VCOM_InversionPending ) {
		SendToggleVCOMCommand();
		VCOM_InversionPending = false;
	}
}

//*****************************************************************************
//
//! The display structure that describes the driver for the
//! sharpLCD panel
//
//*****************************************************************************
extern const tDisplay g_sharp96x96LCD = {
											sizeof(tDisplay),
											DisplayBuffer,
											HORIZONTAL_MAX,
											VERTICAL_MAX,
											PixelDraw,
											DrawMultiple,
											LineDrawH,
											LineDrawV,
											RectFill,
											ColorTranslate,
											Flush
									};

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************

//*****************************************************************************
// Task definition
//*****************************************************************************
//
//	Some utility/debug functions
//
//	Print one row of Display buffer to UART.
//	This is specific to the Sharp96x96 display driver.
//
void PrintOneRow( uint32_t Row_Idx ) {

	uint32_t	Col_Idx;

	for ( Col_Idx = 0; Col_Idx < (HORIZONTAL_MAX>>3); Col_Idx++ ) {
		UARTprintf( "%02X", DisplayBuffer[Row_Idx][Col_Idx] );
	}
}

//
//	Print a few of the first lines of the DisplayBuffer
//
void PrintDisplayBuffer( void ) {

#define		NumLines	5

	uint32_t	Row_Idx;

	UARTprintf( ">>>DisplayBuffer: \n" );
	for ( Row_Idx = 0; Row_Idx < NumLines; Row_Idx++ ) {
		UARTprintf( "Row %02d: ", Row_Idx );
		PrintOneRow( Row_Idx );
		UARTprintf( "\n" );
	}
}

//
//	Define a clear DisplayBuffer subroutine.
//	Cannot figure out why this is not standard.
//
extern void Sharp96x96_ClearDisplayBuffer( void ) {
	uint32_t	Row_Idx, Col_Idx;

	for ( Row_Idx = 0; Row_Idx < VERTICAL_MAX; Row_Idx++ ) {
		for (Col_Idx = 0; Col_Idx < (HORIZONTAL_MAX>>3); Col_Idx++ ) {
			DisplayBuffer[Row_Idx][Col_Idx] = 0x00;
		}
	}
}

