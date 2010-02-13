#line 1 "D:/projects/rapid_proto/firmware/mflow.c"
#line 1 "d:/projects/rapid_proto/firmware/mflow.h"



extern struct MeltFlowStruct {


 signed long position;
 signed long feedback;
 signed long prev_position;
 signed long prev_vel;
 float Qmf;
 float tmf;
 float tex;
 float pct_mf;
 float mfgain;
 float qgain;
 int maxoutput;
 int output;
 unsigned short enable;
 unsigned short limit_state;

};

void calc_vel(struct MeltFlowStruct *ps);
#line 9 "D:/projects/rapid_proto/firmware/mflow.c"
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


 vel = ps->position - ps->prev_position;


 accel = vel - ps->prev_vel;


 Qtarget = ps->mfgain * ( (float)vel + ( (float)accel * ps->tex ));


 Qout = ps->qgain * ( Qtarget - Qmf );


 Qmf += ps->tmf * ( (0.09 * Qout ) - Qmf );


 ps->prev_position = ps->position;
 ps->prev_vel = vel;
 ps->Qmf = Qmf;




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
