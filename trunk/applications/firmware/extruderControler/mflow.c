#include "mflow.h"
/*
  Accept struct containing position information
  and melt flow compensation constants,
  and compute output signal for a motor


*/
void calc_vel(struct MeltFlowStruct *ps){

    signed long vel;
    signed long accel;
    float Qtarget;
    float Qout;
    float Qmf;
    int maxoutput;
    int output;
    short limitstate;
    maxoutput = ps->maxoutput;
    Qmf = ps->Qmf;
    
    //compute velocity
    vel = ps->position - ps->prev_position;
    
    //compute acceleration
    accel  =  vel - ps->prev_vel;
    
    //target flow rate
    Qtarget = ps->mfgain * ( (float)vel + ( (float)accel * ps->tex ));
    
    //melt flow compensated flow rate
    Qout  = ps->qgain * ( Qtarget - Qmf );
    
    //new melt flow rate
    Qmf += ps->tmf * ( (0.09 * Qout ) - Qmf );
    
    //update previous and next values
    ps->prev_position = ps->position;
    ps->prev_vel = vel;
    ps->Qmf = Qmf;
    
		//watch for limts of output
		//cannot assign floating point to an int
		//via cast in case the value is bigger than int can hold
		limitstate=0u;

		if ( Qout > maxoutput){
       output = maxoutput;
       limitstate=1u;
    }
    else if ( Qout < -maxoutput ){
       output = -maxoutput;
       limitstate=1u;
    }
    else{
       output = (int)Qout;
    }

    if ( ps->enable == 0 ){
       output = 0;
       limitstate = 0;
    }
    ps->limit_state = limitstate;
    ps->output = output;
}