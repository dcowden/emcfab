#include "duty.h"

#define DUTY_PIN PORTD.F5
#define MAX_DUTY 255
/*
  Library to create slow duty cycles
  for triac control.
  
  library is for cycle-based control
  duty cycle is unsigned short ( 0 = 0ff, 255 = on )
  
  MAX_DUTY is the maximum duty cycle
  
  PERIOD WIDTH is the number of interrupts over which the period
     of the duty cycle should occur.
     
     for example, if interrupts are @ 60 hz, and PERIOD_WIDTH = 128,
     then the period of the duty cycle will be about 2 seconds
     
*/

unsigned short setpoint;
unsigned short duty_count;
unsigned short duty_period;

void initDuty( unsigned short dtyperiod ){
     setpoint = 0;
     duty_period = dtyperiod;
     duty_count = duty_period;
}

void setDuty( unsigned short setpnt ){

     //setpoint = (int)( (float)(setpnt / MAX_DUTY) * (float)duty_period);
     //the triac has inverted duty cycle, so subtract from 255
     //this version does not scale the duty cycle period
     //to match the frquency.
     //the period should be 120 hz, but this calculation is based
     //on 128. close enough.

     setpoint = 255 - setpnt;
     setpoint = setpoint >> 1;
}



void dutyInterruptProc(void){

       duty_count--;
       if ( duty_count < setpoint ){
          DUTY_PIN = 1;
       }
       else{
          DUTY_PIN = 0;
       }
       if ( duty_count == 0 ){
          duty_count = duty_period;
       }
}