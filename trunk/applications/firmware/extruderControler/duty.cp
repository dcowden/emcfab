#line 1 "C:/data/emcfab/applications/firmware/extruderControler/duty.c"
#line 1 "c:/data/emcfab/applications/firmware/extrudercontroler/duty.h"




void initDuty( unsigned short dtyperiod );
void dutyInterruptProc(void);
void setDuty( unsigned short setpnt );
#line 22 "C:/data/emcfab/applications/firmware/extruderControler/duty.c"
unsigned short setpoint;
unsigned short duty_count;
unsigned short duty_period;

void initDuty( unsigned short dtyperiod ){
 setpoint = 0;
 duty_period = dtyperiod;
 duty_count = duty_period;
}

void setDuty( unsigned short setpnt ){








 setpoint = 255 - setpnt;
 setpoint = setpoint >> 1;
}



void dutyInterruptProc(void){

 duty_count--;
 if ( duty_count < setpoint ){
  PORTD.F5  = 1;
 }
 else{
  PORTD.F5  = 0;
 }
 if ( duty_count == 0 ){
 duty_count = duty_period;
 }
}
