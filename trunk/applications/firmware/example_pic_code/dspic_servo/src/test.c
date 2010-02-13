//---------------------------------------------------------------------
//	File:		test.c
//
//	Written By:	Lawrence Glaister VE7IT
//
// Purpose: This set of routines deals with various test
//          routines that were used to verify parts of the
//          hardware and software design. They are not
//			needed in the running system.
// 
//---------------------------------------------------------------------
//
// Revision History
//
// Aug 8 2006 --    first version Lawrence Glaister
// 
//---------------------------------------------------------------------- 
#include "dspicservo.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

extern volatile unsigned short int timer_test;
extern volatile unsigned short int do_servo;
extern unsigned short int cmd_posn;			// current posn cmd from PC
extern unsigned short int cmd_err;			// number of bogus encoder positions detected
extern unsigned short int cmd_bits;			// a 4 bit number with old and new port values
extern volatile unsigned short int do_servo;
extern struct PID pid;

extern void set_pwm(float amplitude);
extern void calc_pid( void );

void test_pwm_interface( void )
{
	float max_amps = 6.0;
	float amps;
    float amp_incr = 0.05;
	unsigned short holdtime = 10;  // 1ms/current level

	printf("Testing current ramps via pwm and OPA549\r\n");
	timer_test = holdtime;
	while (1)
	{
		for ( amps = -max_amps; amps <= max_amps; amps += amp_incr )
		{
			set_pwm(amps);
			// 5ms at each torque value (current)
			while( timer_test );
			timer_test = holdtime;
		}
		for ( amps = max_amps; amps >= -max_amps; amps -= amp_incr )
		{
			set_pwm(amps);
			// 5ms at each torque value (current)
			while( timer_test );
			timer_test = holdtime;
		}
	}
}

// every second, echo cmded position to serial port
// (this tests the quadrature interface via the IC1 and IC2 pins)
// use emc2 to jog axis in each direction to monitor operation
void test_pc_interface( void )
{
	printf("Testing pc command interface on IC1 and IC2\r\n");
	timer_test = 10000;
	while (1)
	{
		// 1 second betwen prints
		while( timer_test );
		timer_test = 10000;
		printf("cmd = 0x%04X, err = 0x%04X, bits= 0x%02X\r\n",cmd_posn, cmd_err, cmd_bits);

		if (STATUS_LED)			// blink test led for operator comfort
			STATUS_LED = 0;
		else
			STATUS_LED = 1;

	}
}

// every second, echo some pid status to serial port
void test_pid_interface( void )
{
	printf("Testing pid loop operation\r\n");
	timer_test = 10000;
	while (1)
	{
		if ( do_servo )
		{
			do_servo = 0;
			pid.command = (float)((short)cmd_posn);
			pid.feedback = (float)((short)POSCNT);
			calc_pid();
			if ( pid.output > 0)
				SVO_DIR = 1;
			else
				SVO_DIR = 0;
			set_pwm(fabs(pid.output));
		}

		if ( timer_test == 0)
		{
			timer_test = 10000;
			printf("cmd = %6.0f, feedback = %6.0f err = %6.0f, output = %6.0f\r\n",
				(double)pid.command, (double)pid.feedback, (double)pid.error, (double)pid.output);

			if (STATUS_LED)			// blink test led for operator comfort
				STATUS_LED = 0;
			else
				STATUS_LED = 1;
		}
	}
}
