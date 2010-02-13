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


// the following version string is printed on powerup (also in every object module)



/***********************************************************************
*                  LOCAL FUNCTION DECLARATIONS                         *
************************************************************************/

extern struct PIDStruct {
    signed long command;	/* commanded value  4 bytes*/
    signed long feedback;	/* feedback value   4 bytes*/
    long error;		/* command - feedback 4 bytes */
    int deadband;		/* param: deadband  2 bytes */
    signed long maxerror;		/* param: limit for error 4 bytes */
    int maxerror_i;	/* param: limit for integrated error */
    long maxerror_d;	/* param: limit for differentiated error */
    long maxcmd_d;		/* param: limit for differentiated cmd */
    int error_i;		/* opt. param: integrated error */
    long prev_error;	/* previous error for differentiator */
    long error_d;		/* opt. param: differentiated error */
    long prev_cmd;	/* previous command for differentiator */
    long cmd_d;		/* opt. param: differentiated command */
    int bias;			/* param: steady state offset */
    float pgain;		/* param: proportional gain */
    float igain;		/* param: integral gain */
    float dgain;		/* param: derivative gain */
    float ff0gain;		/* param: feedforward proportional */
    float ff1gain;		/* param: feedforward derivative */
    int maxoutput;	/* param: limit for PID output */
    int output;		/* the output value -- outputs are duty cycles, no need for float*/
    unsigned short enable;		/* enable input */
    unsigned short limit_state;	/* 1 if in limit, else 0 */
};

void calc_pid(struct PIDStruct *ps);