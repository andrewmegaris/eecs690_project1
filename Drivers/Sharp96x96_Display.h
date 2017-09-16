//-- Sharp96x96.h - Tiva interface to the Sharp Memory LCD BoosterPack.
//
// Copyright (c) 2015 Donald Rich.
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
 *					display.
 *
 *  Modification:	Adapted from TI's sharp.h example program.
 *					Further adapted from Brandon Givens'
 *					EECS 690 class project.
 *
 *	Modification:
 *	Author:			Gary J. Minden
 *  Organization:	KU/EECS/EECS 690
 *  Date:			2017-09-07 (B70907)
 *	Description:	Changed code from a FreeRTOS Task to a
 *					standalone initialization subroutine.
 *					Added DOxygen comments.

 *
 */

#ifndef __SHARPLCD_H__
#define __SHARPLCD_H__

// The Graphics_Display object contains pointers to the public
// functions of the interface used in the graphics library.

extern const tDisplay g_sharp96x96LCD;

extern int32_t Sharp96x96_InitizeDisplay( void );
extern void Sharp96x96_disable(void);
extern void Sharp96x96_enable(void);
extern void Sharp96x96_ClearDisplayBuffer( void );

#endif // __SHARPLCD_H__i

