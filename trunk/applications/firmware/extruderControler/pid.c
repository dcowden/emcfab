/********************************************************************
* Description:  pid.c
*               This file, 'pid.c', is a single
*               Proportional/Integeral/Derivative control loop used
*				for a single axis of position control. It was derived from
*				GPL2'd code by John Kasunich in the EMC2 project.
*				Copyright (C) 2003 John Kasunich
*/

/*
    The three most important components are 'command', 'feedback', and
    'output'.  For a position loop, 'command' and 'feedback' are
    in position units.  For a linear axis, this could be inches,
    mm, metres, or whatever is relavent.  Likewise, for a angular
    axis, it could be degrees, radians, etc.  The units of the
    'output' pin represent the change needed to make the feedback
    match the command.  As such, for a position loop 'Output' is
    a velocity, in inches/sec, mm/sec, degrees/sec, etc.

    Each loop has several other pins as well.  'error' is equal to
    'command' minus 'feedback'.  'enable' is a bit that enables
    the loop.  If 'enable' is false, all integrators are reset,
    and the output is forced to zero.  If 'enable' is true, the
    loop operates normally.

    The PID gains, limits, and other 'tunable' features of the
    loop are implemented as parameters.  These are as follows:

    Pgain	Proportional gain
    Igain	Integral gain
    Dgain	Derivative gain
    bias	Constant offset on output
    FF0		Zeroth order Feedforward gain
    FF1		First order Feedforward gain
    deadband	Amount of error that will be ignored
    maxerror	Limit on error
    maxerrorI	Limit on error integrator
    maxerrorD	Limit on error differentiator
    maxcmdD	Limit on command differentiator
    maxoutput	Limit on output value

    All of the limits (max____) are implemented such that if the
    parameter value is zero, there is no limit.

    errorI	Integral of error
    errorD	Derivative of error
    commandD	Derivative of the command

    The PID loop calculations are as follows (see the code for
    all the nitty gritty details):

    error = command - feedback
    if ( abs(error) < deadband ) then error = 0
    limit error to +/- maxerror
    errorI += error * period
    limit errorI to +/- maxerrorI
    errorD = (error - previouserror) / period
    limit errorD to +/- paxerrorD
    commandD = (command - previouscommand) / period
    limit commandD to +/- maxcmdD
    output = bias + error * Pgain + errorI * Igain +
             errorD * Dgain + command * FF0 + commandD * FF1
    limit output to +/- maxoutput

*/

#include "pid.h"



/***********************************************************************
*                   REALTIME PID LOOP CALCULATIONS                     *
* 	this code is embedded inside a periodic ISR that defines the servo 
*	loop timing.
*
************************************************************************/
void calc_pid( struct PIDStruct *ps )
{

    long tmp1;
    float tmp2;

    //fetch some variables to save memory
    //access of structur member repeatedly is more efficient rhrough
    //stack variable
    //do this for variables that are accessed read-only
    //variables we mutate have to be accessed directly

    long maxerror;
    long maxerror_i;
    long maxerror_d;
    long error_i;
    long error_d;
    long maxcmd_d;
    long cmd_d;
    unsigned short enable;
    int deadband;
    int maxoutput;
    int output;
    short limitstate;
    
    /* get the enable bit */
    enable = ps->enable;
    maxerror = ps->maxerror;
    deadband = ps->deadband;
    maxerror_i = ps->maxerror_i;
    maxerror_d = ps->maxerror_d;
    maxoutput = ps->maxoutput;
    maxcmd_d = ps->maxcmd_d;
    
    /* calculate the error */
    tmp1 = (ps->command - ps->feedback);
    ps->error = tmp1;
    
    /* apply error limits */
    if (maxerror != 0){
  		if (tmp1 > maxerror){
  	    	tmp1 = maxerror;
  	    	//disable the drive too
  	    	ps->enable = 0u;
  		}
  		else if (tmp1 < -maxerror){
  	    	tmp1 = -maxerror;
  	    	ps->enable = 0u;
  		}
    }
    
    /* apply the deadband */
    if (tmp1 > deadband) {
		    tmp1 -= deadband;
    } 
	  else if (tmp1 < -deadband){
		    tmp1 += deadband;
    }
	  else{
		    tmp1 = 0;
		    //once within the deadband, reset integrator
		    ps->error_i = 0;
    }

    /* do integrator calcs only if enabled */
    error_i = ps->error_i;
    if (enable != 0u) {
  		/* if output is in limit, don't let integrator wind up */
  		if ( ps->limit_state == 0u ){
  	    	/* compute integral term */
  	    	error_i += tmp1;
  		}
  		/* apply integrator limits */

  		if (maxerror_i != 0u)	{
  	    	if (error_i > maxerror_i){
  				   error_i = maxerror_i;
  	    	}
  			  else if (error_i < (-maxerror_i)){
  				     error_i = -maxerror_i;
  	      }
  		}
    } 
	  else{
		   /* not enabled, reset integrator */
		   error_i = 0;
    }
    ps->error_i = error_i;

    /* calculate derivative term */
    error_d =  (tmp1 - ps->prev_error);
    ps->prev_error = tmp1;
    /* apply derivative limits */
    if (maxerror_d != 0u) {
  		if (error_d > maxerror_d){
  	    	error_d = maxerror_d;
  		}
  		else if (error_d < -maxerror_d){
  	    	error_d = -maxerror_d;
  		}
    }
    ps->error_d = error_d;
    
    /* calculate derivative of command */

      cmd_d =  (long)(ps->command - ps->prev_cmd);
      ps->prev_cmd = ps->command;

      /* apply derivative limits */
      if (maxcmd_d != 0u) {
    		if (cmd_d > maxcmd_d){
    	    	cmd_d = maxcmd_d;
    		}
    		else if (cmd_d < -maxcmd_d){
    	    	cmd_d = -maxcmd_d;
    		}
      }
      ps->cmd_d = cmd_d;

    /* do output calcs only if enabled */
    output=0;
    if (enable != 0u) {
  		/* calculate the output value  */
  		//tmp2 =
  	  //  	(float)ps->bias + ( (float)ps->pgain * tmp1 ) +
  		//	         ((float)ps->error_i * (float)ps->igain) +
  	  //	            ((float)ps->dgain * (float)ps->error_d);
      tmp2 =
            (float)ps->bias + ( (float)ps->pgain * tmp1 ) +
                    ((float)ps->error_i * (float)ps->igain) +
                      ((float)ps->dgain * (float)ps->error_d);
                      

  		   tmp2 += ((float)ps->command * (float)ps->ff0gain );

         tmp2 += ((float)ps->cmd_d * (float)ps->ff1gain);

  		
  		//watch for limts of output
  		//cannot assign floating point to an int
  		//via cast in case the value is bigger than int can hold
  		limitstate=0u;

  		if ( tmp2 > maxoutput){
         output = maxoutput;
         limitstate=1u;
      }
      else if ( tmp2 < -maxoutput ){
         output = -maxoutput;
         limitstate=1u;
      }
      else{
         output = (int)tmp2;
      }

    } 
	  else{
    		/* not enabled, force output to zero */
    		output = 0;
    		limitstate = 0;
    }
    
    ps->output = output;
    ps->limit_state = limitstate;
    /* done */
}

