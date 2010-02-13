#line 1 "C:/data/emcfdm/src/pic/estopLatch/safety_interlock.c"
#line 1 "c:/apps/mikroc/include/built_in.h"
#line 80 "C:/data/emcfdm/src/pic/estopLatch/safety_interlock.c"
unsigned int ok = 0;
unsigned int cause = 0;
unsigned int blinkCounter =  16 ;

void enable(){
 if (  0  == 1 ) return;

  PORTC.F0  = 0;
 ok = 1;
  PORTC.F1  = 0;

 INTCON.PEIE = 0;

}

void disable(){

  PORTC.F0  = 1;
 ok = 0;
 blinkCounter = 0;

 INTCON.PEIE = 1;
}


void interrupt(){





 if ( PIR1.TMR1IF == 1 ){



 if ( ok == 0 ){
 if ( blinkCounter > 0 ){
 blinkCounter --;
 }
 else{
 blinkCounter =  16 ;
 blinkCounter = (int)(cause *  16 );

 }
 }


 PIR1.TMR1IF = 0;

 }
}


void checkLatch(){










 unsigned int newLatch = 1;


 if (  PORTA.F3  ==  1  ||  PORTC.F5  ==  1  ||
  PORTC.F4  ==  1  ||  PORTA.F5  ==  1  ){
 newLatch = 0;
 cause =  3 ;
 }


 if (  PORTA.F4  ==  0  ||  PORTA.F2  ==  0  ||
  PORTA.F1  ==  0  ||  PORTA.F0  ==  0  ){
 newLatch = 0;
 cause =  4 ;
 }


 if (  PORTC.F2  ==  1  ){
 newLatch = 0;
 cause =  2 ;
 }

 if (  PORTC.F3  ==  0  ){
 newLatch = 0;
 cause =  1 ;
 }

 if ( newLatch == 1){
 enable();
 }
 else{
 disable();
 }

}



void initRegisters(){

 ANSEL = 0x00;
 CMCON0 =0b00000111;

 TRISA = 0b11111111;
 WPUA = 0b00111111;
 OPTION_REG = 0x00;


 TRISC = 0b11111100;



 INTCON.GIE = 1;
 T1CON= 0b00110101;



  PORTC.F0  = 0;
  PORTC.F1  = 1;
}

void main() {

 initRegisters();
 while(1) {
 checkLatch();
 }
}
