//---------------------------------------------------------------------
//	File:		capture.c
//
//	Written By:	Lawrence Glaister VE7IT
//
// Purpose: This set of routines deals with using 2 pins
// connected to the PC as a quadrature signal for incrementing or 
// decementing commanded position. The IC1 and IC2 pins are used
// and are setup to generate an interrupt on each edge change.
// The ISR's then looks at the state of IC1 and IC2 and uses
// a state machine to incr or decr the commanded position.
// The IC1 and IC2 pins are smidt trigger inputs.
// The CN inputs could also have been used to detect change of state on
// the 2 pc command pins as we dont use most of the input capture 
// functionality.
//      
// 
//---------------------------------------------------------------------
//
// Revision History
//
// Aug 7 2006 --    first version Lawrence Glaister
// Aug 15 2006		added pc command pulse multiplier option
//---------------------------------------------------------------------- 
#include "dspicservo.h"

extern struct PID pid;

volatile unsigned short int cmd_posn;			// current posn cmd from PC
volatile unsigned short int cmd_err;			// number of bogus encoder positions detected
volatile unsigned short int cmd_bits;	// a 4 bit number with old and new port values

// 4 functions that perform adjustments to the commanded position
void change_NO(void)
{
	cmd_err++;		// got an intr with no change in port (spike?)
}

void change_UP(void)
{
	cmd_posn += pid.multiplier;
}
void change_DN(void)
{
	cmd_posn -= pid.multiplier;
}
void change_ER(void)
{
	cmd_err++;		// both bits changed.... overspeed???
}

// define the array of functions needed to handle changes
               // Encoder lines
               // Before Now 
// change_NO   // 0 0   0 0
// change_UP   // 0 0   0 1
// change_DN   // 0 0   1 0
// change_ER   // 0 0   1 1

// change_DN   // 0 1   0 0
// change_NO   // 0 1   0 1
// change_ER   // 0 1   1 0
// change_UP   // 0 1   1 1

// change_UP   // 1 0   0 0
// change_ER   // 1 0   0 1
// change_NO   // 1 0   1 0
// change_DN   // 1 0   1 1

// change_ER   // 1 1   0 0
// change_DN   // 1 1   0 1
// change_UP   // 1 1   1 0
// change_NO   // 1 1   1 1

void (*funcArr[16])(void)={	change_NO,change_UP,change_DN,change_ER,
                            change_DN,change_NO,change_ER,change_UP,
 							change_UP,change_ER,change_NO,change_DN,
							change_ER,change_DN,change_UP,change_NO};

/*********************************************************************
  Function:        void __attribute__((__interrupt__)) _IC1Interrupt(void)

  PreCondition:    None.
 
  Input:           None

  Output:          None.

  Side Effects:    None.

  Overview:        handles changes on IC1 pin 

  Note:            None.
********************************************************************/
void __attribute__((__interrupt__)) _IC1Interrupt(void)
{
    IFS0bits.IC1IF = 0;                    	// Clear IF bit
	cmd_bits = ((cmd_bits << 2) & 0x000c) + 	// old bits move left
			  (PORTD & 0x03);				// bits 0 and 1 are valid new bits
	(*funcArr[cmd_bits])();					// process cmd from pc
}

/*********************************************************************
  Function:        void __attribute__((__interrupt__)) _IC2Interrupt(void)

  PreCondition:    None.
 
  Input:           None

  Output:          None.

  Side Effects:    None.

  Overview:        handles changes on IC2 pin 

  Note:            None.
********************************************************************/
void __attribute__((__interrupt__)) _IC2Interrupt(void)
{
    IFS0bits.IC2IF = 0;                    /* Clear IF bit */
	cmd_bits = ((cmd_bits << 2) & 0x000c) + 	// old bits move left
			  (PORTD & 0x03);				// bits 0 and 1 are valid new bits
	(*funcArr[cmd_bits])();					// process cmd from pc
}


/*********************************************************************
  Function:        void setup_capture(void)

  PreCondition:    None.
 
  Input:           None

  Output:          None.

  Side Effects:    None.

  Overview:        

  Note:            None.
********************************************************************/
void setup_capture(void)
{
	/* start with a clean slate in the control words */
	/* also disables IC module 1 and 2*/
	IC1CON = 0;						
	IC2CON = 0;

	/*	disable interrupts */
    IEC0bits.IC1IE = 0;
    IEC0bits.IC2IE = 0;

	/* Clean up any pending IF bits */
    IFS0bits.IC1IF = 0;             
	IFS0bits.IC2IF = 0;

	/* assign Interrupt Priority to IPC Register  (4 is default)*/
    IPC0bits.IC1IP = 0x0004;     
    IPC1bits.IC2IP = 0x0004;

    /* Config contains Clock source (0=timer 3, we dont care), 
       number of Captures per interuppt (0 = every event (not used))
       and Capture Mode (001=every edge)*/
	
    IC1CON = 0x0001;
    IC2CON = 0x0001;

	cmd_posn = 0;
	cmd_err = 0;
	cmd_bits = PORTD & 0x03;	// current port state

	/*	go live... enable interrupts */
    IEC0bits.IC1IE = 1;
    IEC0bits.IC2IE = 1;
}
