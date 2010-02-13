#line 1 "C:/data/emcfdm/src/pic/extruderControler/pid.c"
#line 1 "c:/data/emcfdm/src/pic/extrudercontroler/pid.h"
#line 26 "c:/data/emcfdm/src/pic/extrudercontroler/pid.h"
extern struct PIDStruct {
 signed long command;
 signed long feedback;
 long error;
 int deadband;
 signed long maxerror;
 int maxerror_i;
 long maxerror_d;
 long maxcmd_d;
 int error_i;
 long prev_error;
 long error_d;
 long prev_cmd;
 long cmd_d;
 int bias;
 float pgain;
 float igain;
 float dgain;
 float ff0gain;
 float ff1gain;
 int maxoutput;
 int output;
 unsigned short enable;
 unsigned short limit_state;
};

void calc_pid(struct PIDStruct *ps);
#line 77 "C:/data/emcfdm/src/pic/extruderControler/pid.c"
void calc_pid( struct PIDStruct *ps )
{

 long tmp1;
 float tmp2;







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


 enable = ps->enable;
 maxerror = ps->maxerror;
 deadband = ps->deadband;
 maxerror_i = ps->maxerror_i;
 maxerror_d = ps->maxerror_d;
 maxoutput = ps->maxoutput;
 maxcmd_d = ps->maxcmd_d;


 tmp1 = (ps->command - ps->feedback);
 ps->error = tmp1;


 if (maxerror != 0){
 if (tmp1 > maxerror){
 tmp1 = maxerror;

 ps->enable = 0u;
 }
 else if (tmp1 < -maxerror){
 tmp1 = -maxerror;
 ps->enable = 0u;
 }
 }


 if (tmp1 > deadband) {
 tmp1 -= deadband;
 }
 else if (tmp1 < -deadband){
 tmp1 += deadband;
 }
 else{
 tmp1 = 0;

 ps->error_i = 0;
 }


 error_i = ps->error_i;
 if (enable != 0u) {

 if ( ps->limit_state == 0u ){

 error_i += tmp1;
 }


 if (maxerror_i != 0u) {
 if (error_i > maxerror_i){
 error_i = maxerror_i;
 }
 else if (error_i < (-maxerror_i)){
 error_i = -maxerror_i;
 }
 }
 }
 else{

 error_i = 0;
 }
 ps->error_i = error_i;


 error_d = (tmp1 - ps->prev_error);
 ps->prev_error = tmp1;

 if (maxerror_d != 0u) {
 if (error_d > maxerror_d){
 error_d = maxerror_d;
 }
 else if (error_d < -maxerror_d){
 error_d = -maxerror_d;
 }
 }
 ps->error_d = error_d;



 cmd_d = (long)(ps->command - ps->prev_cmd);
 ps->prev_cmd = ps->command;


 if (maxcmd_d != 0u) {
 if (cmd_d > maxcmd_d){
 cmd_d = maxcmd_d;
 }
 else if (cmd_d < -maxcmd_d){
 cmd_d = -maxcmd_d;
 }
 }
 ps->cmd_d = cmd_d;


 output=0;
 if (enable != 0u) {





 tmp2 =
 (float)ps->bias + ( (float)ps->pgain * tmp1 ) +
 ((float)ps->error_i * (float)ps->igain) +
 ((float)ps->dgain * (float)ps->error_d);


 tmp2 += ((float)ps->command * (float)ps->ff0gain );

 tmp2 += ((float)ps->cmd_d * (float)ps->ff1gain);





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

 output = 0;
 limitstate = 0;
 }

 ps->output = output;
 ps->limit_state = limitstate;

}
