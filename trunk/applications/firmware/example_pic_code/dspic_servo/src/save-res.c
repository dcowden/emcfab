/* interface.c  
 * dsPic30f4011 driver program for the N2PK Vector Network Analyzer
 * This file contains code for the operators interface.
 *
 *  Copyright (C) 2006 By Lawrence Glaister VE7IT
 *             (ve7it@shaw.ca)
 *
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 *  Compile using: mplab ide
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>		// for PI etc
#include "dspicservo.h"
#include "DataEEPROM.h"		// for eeprom read and write routines

extern struct PID pid;

// we need to pull some tricks to get it all done
// structure with all setup constants that is stored in eeprom
// aligned on 32 byte boundary
// seed eeprom with a few default values
struct PID _EEDATA(32) pidEE={0.0};

//=============================================================================
// Routine to calculate a checksum on a section of memory
// call with array size in 16 bit words and ptr to start.
//=============================================================================
int calc_cksum(int sizew, int *adr)
{
	int i;
	int cksum = 0;

	for (i=0; i < sizew; i++)
		cksum += *adr++;

//	printf("cksum of %d 16bit words is %d\r\n",sizew,cksum);
	return cksum; 

}

//=============================================================================
// Routine to save setup structure into eeprom
// 
//=============================================================================
int save_setup( void )
{
	int size = sizeof(pid);
	int *sptr = (int *)&pid;
	int res;
	int offset = 0;

	// compute correct checksum for upper part of array
	// and place it in the checsum variable
	pid.cksum = -calc_cksum((sizeof(pid)-sizeof(int))/sizeof(int),
							 (int*)&pid);

	// this routine attempts to write the entire ram setup structure
	// into the eeprom on board.
	// write 16 words of structure at a time
	
	while (size > 0)
	{
		// Erase 16 words (1 row in dsPIC30F DataEEPROM) in Data EEPROM 
		// from calEE structure
		res = EraseEE(__builtin_tblpage(&pidEE), 
                      __builtin_tbloffset(&pidEE)+offset, ROW);
		if (res)
			printf("clr of eeprom failed at %d\r\n",offset);

		res = WriteEE(sptr, __builtin_tblpage(&pidEE),
							__builtin_tbloffset(&pidEE)+offset, ROW);
		if (res)
			printf("write to eeprom failed at offset %d\r\n",offset);

		offset += ROW*2;			// bump offset to destination 32 bytes up 
		sptr   += ROW;			// bump source ptr up 16 words
		size   -= ROW*2;	    // 16 words or 32 bytes/write
	}
	return res;
}

//=============================================================================
//  routine to restore setup data 
// 
//=============================================================================
int restore_setup( void )
{
	int size = sizeof(pid);
	int *dptr = (int *)&pid;
	int res = 0;
	int offset = 0;

	// this routine attempts to read the entire calibration structure
	// into the ram on board.
	// read 16 words of structure at a time
	while (size > 0)
	{
		res = ReadEE(__builtin_tblpage(&pidEE),
					 __builtin_tbloffset(&pidEE)+offset,
					dptr, ROW);
		if (res)
			printf("%d read from eeprom failed at offset %d\r\n",
					res,offset);

		offset += ROW*2;		// bump offset to destination 32 bytes up 
		dptr   += ROW;			// bump source ptr up 16 words
		size   -= ROW*2;	    // 16 words or 32 bytes/write
	}
	return res;
}
