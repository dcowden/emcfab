//---------------------------------------------------------------------
//	File:		timer1.c
//
//	Written By:	Lawrence Glaister VE7IT
//
// Purpose: routines to setup and use timer 1
//      
// 
//---------------------------------------------------------------------
//
// Revision History
//
// Nov 5 2005 -- first version 
//---------------------------------------------------------------------- 
#include "dspicservo.h"

volatile unsigned short int timer_test;
volatile unsigned short int do_servo;
extern struct PID pid;

/*********************************************************************
  Function:        void __attribute__((__interrupt__)) _T1Interrupt (void)

  Input:           None.

  Output:          None.

  Side Effects:    None.

  Overview:        This isr is used to periodically do certain tasks
          		   Basic rate is set to 10000intr/sec in the init function

********************************************************************/

void __attribute__((__interrupt__)) _T1Interrupt (void)
{
	static short gear = 0;

	IFS0bits.T1IF = 0;

	if ( pid.ticksperservo < 2) pid.ticksperservo = 2;

	if (++gear >= pid.ticksperservo)
	{
		gear = 0;
		do_servo = 1;		// trigger servo calcs in main()
	}
	// this block of timers is used in the software for delays
    if ( timer_test > 0 ) --timer_test;

	return;
}


/*********************************************************************
  Function:        void setupTMR1(void)

  PreCondition:    None.
 
  Input:           None.

  Output:          None.

  Side Effects:    None.

  Overview:        Initialization of timer 1 as a periodic interrupt 
                   each 0.1 ms (10khz)

  Note:            None.
********************************************************************/

void setup_TMR1(void)
{
	T1CON = 0x0020;			// internal Tcy/64 clock
	TMR1 = 0;
#define PR1PRESET  (FCY/64/10000)
	PR1 = PR1PRESET;		// 0.1 ms interrupts (100us)
	T1CONbits.TON = 1;		// turn on timer 1 
	return;
}




