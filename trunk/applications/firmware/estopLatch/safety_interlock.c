#include "built_in.h"


/*
  Estop latch code.
  Watches up to 4 limit switches,
  up to 4 servo amps,
  1 pc enable,
  and 1 estop switch, and toggles an enable and error
  output.
  
  The basic logic is: if limits, estop, servo faults, and pc is enabled,
  the output is enabled, otherwise it is disabled.
  
  see below to configure what value of signals indicates an error on
  limits, servos, pc, and estop
  
  also set REQUIRE_RESET_ON_FAULT if you want to disable the circuit
  until a reset occurs after an error ( charge pump behavior )

  This circuit manages the outputs from SV-500 servo controllers.
  These controllers have an enable pin that is both an input and an output.
  each pin is tied to +5v with a resistor on the board. the output is
  therefore +5v when the board is ok.
  
  when a fault occurs, the board will pull the pin low. This results in
  a low output on error. It also means that any other connected pins
  are pulled low.  this means that connecting the enable pins of all three
  boards will mean a failure on any board will also cause faults on the
  other boards.
  
  
  
  It also means that it would have saved 3 pins to simply assume that all
  three pins would be linked, and have only one input. but this design
  provides more flexibility
  
*/



#define LIMIT_Z PORTA.F3
#define LIMIT_A PORTA.F5
#define LIMIT_Y PORTC.F5
#define LIMIT_X PORTC.F4

#define FAULT_A PORTA.F4
#define FAULT_Z PORTA.F2
#define FAULT_Y PORTA.F1
#define FAULT_X PORTA.F0

#define PC_ENABLE PORTC.F3
#define ESTOP PORTC.F2
#define ENABLE_OUT PORTC.F1
#define ERR_OUT PORTC.F0

//controls blink rate of the error LED.  # of seconds between blinks
#define BLINK_INTERVAL_PC 1
#define BLINK_INTERVAL_ESTOP 2
#define BLINK_INTERVAL_LIMITS 3
#define BLINK_INTERVAL_FAULT 4

// limit switches are normally closed, with pull-up resistor to +5v
//so error is +5v  (Fail-safe)
#define LIMIT_ERR 1

//sv-500 enable pin outputs low on error, +5v otherwise. see above
#define FAULT_ERR 0

//pc is in error state on low, +5v required from pc to enable.
#define PC_ERR 0

//estop switch is a normally closed switch with pull-up resistor to +5v
#define ESTOP_ERR 1

#define REQUIRE_RESET_ON_FAULT 0
#define BLINK_INTERVAL 16

//the current state of the latch
unsigned int ok = 0;
unsigned int cause = 0;
unsigned int blinkCounter =  BLINK_INTERVAL;

void enable(){
    if ( REQUIRE_RESET_ON_FAULT == 1  ) return;

    ERR_OUT = 0;
    ok = 1;
    ENABLE_OUT = 0;

    INTCON.PEIE = 0;

}

void disable(){

       ERR_OUT = 1;
       ok = 0;
       blinkCounter =  0;
       //enable interrupts to signal what the problem is
       INTCON.PEIE = 1;
}


void interrupt(){

    //this routine will blink the light at different rates/patterns
    //depending on the cause of the error
    
    //even with timer1 prescale @ 16bits, each interrupt is 60ms.
    if ( PIR1.TMR1IF == 1 ){
    
      //if we are in an error state, toggle the error output
      //at a particular rate
      if ( ok == 0 ){
          if ( blinkCounter >  0 ){
               blinkCounter --;
          }
          else{
              blinkCounter = BLINK_INTERVAL;
              blinkCounter = (int)(cause * BLINK_INTERVAL);

          }
      }
      

      PIR1.TMR1IF = 0;
      
    }
}


void checkLatch(){
    //the latch interlock is fairly simple.
    
    //the only complex thing is a feedback loop between pc erorr
    //and machine error.
    //if the pc is in an error state, and attempst to signal it is
    //ok, it may read an error and turn the output back off before
    //the board has read the input again.
    
    //the fix is to temporarily enable the output to give
    //the pc time to read an ok input value
    unsigned int newLatch = 1;

    //check for errors on limits
    if ( LIMIT_Z == LIMIT_ERR || LIMIT_Y == LIMIT_ERR ||
         LIMIT_X == LIMIT_ERR || LIMIT_A == LIMIT_ERR ){
        newLatch = 0;
        cause = BLINK_INTERVAL_LIMITS;
    }

    //check fault inputs
    if ( FAULT_A == FAULT_ERR || FAULT_Z == FAULT_ERR ||
         FAULT_Y == FAULT_ERR || FAULT_X == FAULT_ERR ){
        newLatch = 0;
        cause =  BLINK_INTERVAL_FAULT;
    }

    //check estop
    if ( ESTOP == ESTOP_ERR ){
        newLatch = 0;
        cause =  BLINK_INTERVAL_ESTOP;
    }

    if ( PC_ENABLE == PC_ERR ){
         newLatch = 0;
         cause = BLINK_INTERVAL_PC;
    }
    
    if ( newLatch == 1){
       enable();
    }
    else{
       disable();
    }

}



void initRegisters(){
  //configure port A
  ANSEL = 0x00; //turn off port a analog
  CMCON0 =0b00000111; //connect digital io to port a

  TRISA = 0b11111111;
  WPUA = 0b00111111;
  OPTION_REG = 0x00;  ///enable porta pullups
  
  //configure port C
  TRISC = 0b11111100;

  //set up timer 1 for light blinking
  
  INTCON.GIE = 1;
  T1CON= 0b00110101;


  //initial state is enabled
  ERR_OUT = 0;
  ENABLE_OUT = 1;
}

void main() {

  initRegisters();
  while(1) {
      checkLatch();
  }
}