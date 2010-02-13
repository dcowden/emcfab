//---------------------------------------------------------------------
//	File:		dspicservo.h
//
//	Written By:	Lawrence Glaister VE7IT
//
// Purpose: Various header info for the project
//      
// 
//---------------------------------------------------------------------
//
// Revision History
// March 11 2006 --   formatted into multi file project
// Sept 25 2006 -  6x pwm rate
//---------------------------------------------------------------------- 
// define which chip we are using (peripherals change)
#include <p30f4012.h>

// the following version string is printed on powerup (also in every object module)
#define CPWRT  "\r\ndspic-servo by L.Glaister\r\n"
#define VERSION "(c) 25-Sept-2006 by VE7IT"

// this number was tweaked by hand until serial port baud was correct
#define FCY  (6000000 * 16 / 4)       // 24 MIPS ==> 6mhz osc * 16pll  / 4

/* define required pwm rate... dont make it too high as we loose resolution */
#define FPWM 24000		// 48000 gives approx +- 10 bit current control

// define some i/o bits for the various modules
#define STATUS_LED 	_LATE1		
#define SVO_DIR     _LATE2
#define PID_ACTIVE  _LATE3
#define SVO_ENABLE   _RE8

// for some reason PI may not be defined in math.h on some systems
#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif

#define	TRUE	(1)
#define	FALSE	(0)	

struct PID{
    long int command;	/* commanded value */
    long int feedback;	/* feedback value */
    float error;		/* command - feedback */
    float deadband;		/* param: deadband */
    float maxerror;		/* param: limit for error */
    float maxerror_i;	/* param: limit for integrated error */
    float maxerror_d;	/* param: limit for differentiated error */
    float maxcmd_d;		/* param: limit for differentiated cmd */
    float error_i;		/* opt. param: integrated error */
    float prev_error;	/* previous error for differentiator */
    float error_d;		/* opt. param: differentiated error */
    long int prev_cmd;	/* previous command for differentiator */
    float cmd_d;		/* opt. param: differentiated command */
    float bias;			/* param: steady state offset */
    float pgain;		/* param: proportional gain */
    float igain;		/* param: integral gain */
    float dgain;		/* param: derivative gain */
    float ff0gain;		/* param: feedforward proportional */
    float ff1gain;		/* param: feedforward derivative */
    float maxoutput;	/* param: limit for PID output */
    float output;		/* the output value */
    short enable;		/* enable input */
    short limit_state;	/* 1 if in limit, else 0 */
	short multiplier;	/* pc command multiplier */
	short ticksperservo; /* number of 100us ticks/servo cycle */
    short cksum;		/* data block cksum used to verify eeprom */
};
