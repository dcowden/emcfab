#line 1 "C:/data/emcfdm/src/pic/cncController/cnc_control.c"
#line 1 "c:/data/emcfdm/src/pic/cnccontroller/eeprom.h"
typedef unsigned char byte;
typedef unsigned int word;


void Eeprom_Write_Obj(word addr,void *obj,byte size);
void Eeprom_Read_Obj(word addr,void *obj,byte size);
#line 1 "c:/apps/mikroc/include/built_in.h"
#line 57 "C:/data/emcfdm/src/pic/cncController/cnc_control.c"
extern struct AXISShort {
 unsigned short x;
 unsigned short y;
 unsigned short z;
 unsigned short a;
 unsigned short enable;
 unsigned short delayCycles;
 unsigned short stepLengthCycles;
};

unsigned short buffer = 0xFF;
char txtBuffer[40];
char cmdBuffer[40];
char *currentStatus;
unsigned short txtPos = 0;


const char *splash = "CNC Controller v0.1\0";
const char *cmdPrompt = "\nCmd:>";
const char *noEeprom = "\nNo EEPROM Data.";
const char *unknownCommand = "\n?:";

const char *msg_atLimits = "At Limits.";
const char *msg_eStop = "E-STOP";
const char *msg_servoFault = "Servo Fault";
const char *msg_ok = "Ok";
const char *msg_disabled = "Disabled ";




unsigned short EEPROM_VERSION_ID = 112u;





const char *cmd_status = "s";
const char *cmd_saveEEprom = "sv";
const char *cmd_readEEprom = "rd";
const char *cmd_defaults = "def";
const char *cmd_step_multiplier_x = "smx";
const char *cmd_step_multiplier_y = "smy";
const char *cmd_step_multiplier_z = "smz";
const char *cmd_step_multiplier_a = "sma";
const char *cmd_enable_multiplers = "m";
const char *cmd_delay_cycles = "dcyc";





struct AXISShort multipliers = {
 1u,
 1u,
 1u,
 1u,
 0u,
 3u,
 2u,
#line 139 "C:/data/emcfdm/src/pic/cncController/cnc_control.c"
};


unsigned short statusError = 0;
unsigned short currentSteps = 0x00;








void strConstCpy(char *dest, const char *source) {
 while(*source){
 *dest++ = *source++ ;
 *dest = 0 ;
 }
}

void USART_Send_String( char *data){
 char c;
 while( *data != 0u ){
 c = *data;
 USART_Write(c);
 data++;
 }
 USART_Write(13);
}

int pushChar ( char ch ){
 if ( ch == 10u || ch == 13u){

 cmdBuffer[txtPos] = 0;
 if ( txtPos > 0u ){
 txtPos = 0;
 return 1u;
 }
 else{
 return 0u;
 }

 }
 else{
 Usart_write(ch);
 cmdBuffer[txtPos] = ch;
 txtPos++;
 return 0;
 }
}

void printMessage(const char* msg ){
 USART_Write(13);
 strConstCpy(txtBuffer,msg);
 USART_Send_String(txtBuffer);
}


void printFloat( char* name, float f ){
 char floatTxt[13];
 USART_Send_String(name);
 FloatToStr(f,floatTxt);
 USART_Send_String(floatTxt);
}

void delay5ClockCycles( unsigned short value ){
 unsigned short i = 0;
 for ( i=0;i< value;i++){
 }
 return;
}

void printStatus(){
 USART_Send_String("\nCNC Controller Status:\n");
 printFloat("err=", statusError );
 printFloat("xmult=",multipliers.x);
 printFloat("ymult=",multipliers.y);
 printFloat("zmult=",multipliers.z);
 printFloat("amult=",multipliers.a);
 printFloat("multiplier enable=",multipliers.enable);
 printFloat("steprate=",multipliers.delayCycles);
 USART_Send_String("\nCurrent Status: ");
 USART_Send_String(currentStatus);
}


unsigned short readMemory(){
 unsigned short size = sizeof ( multipliers );
 unsigned short versionId;


 Eeprom_Read_Obj(0, &versionId,1 );
 if ( versionId == EEPROM_VERSION_ID ){
 Eeprom_Read_Obj(1,&multipliers,size );
 return 1;
 }
 else{
 return 0;
 }
}

void writeMemory(){
 unsigned short size = sizeof( multipliers );

 EEprom_Write_Obj(0,&EEPROM_VERSION_ID,1);


 EEprom_Write_Obj(1,&multipliers,size);

}

void clearMemory(){


 unsigned short blank = 0xFF;
 EEprom_Write_Obj(0,&blank,1);
}






unsigned short commandMatches(const char *source ){
 char *cmdptr = cmdBuffer;
 while(*source && *cmdptr ){
 if ( *cmdptr != *source )
 return 0;
 cmdptr++;
 source++;
 }
 return 1;
}

float findFloatValue ( char *buffer){


 char *ptr = strchr(buffer,'=');
 return atof(++ptr);
}
int findIntValue ( char *buffer){


 char *ptr = strchr(buffer,'=');
 return atoi(++ptr);
}
long findLongValue ( char *buffer){


 char *ptr = strchr(buffer,'=');
 return atol(++ptr);
}

unsigned short findLargest ( unsigned short a, unsigned short b,
 unsigned short c, unsigned short d ){
 if ( a > b ){
 if ( a > c ){
 if ( a > d )
 return a;
 else
 return d;
 }
 else{
 if ( c > d )
 return c;
 else
 return d;
 }
 }
 else{
 if ( b > c ){
 if ( b > d ){
 return b;
 }
 else{
 return d;
 }
 }
 else{

 if ( c > d )
 return c;
 else
 return d;
 }
 }
}



void interrupt ( void ){
 if ( INTCON.RBIF ){


 unsigned short i;
 unsigned short maxmult;
 unsigned short stepX = 0;
 unsigned short stepY = 0;
 unsigned short stepZ = 0;
 unsigned short stepA = 0;



 if ( statusError == 1 ){
  PORTC.F0  = 0;
  PORTC.F1  = 0;
  PORTC.F2  = 0;
  PORTC.F3  = 0;
 INTCON.RBIF = 0;
 return;
 }



 if ( multipliers.enable == 0 ){

  PORTC.F0  =  PORTB.F7 ;
  PORTC.F1  =  PORTB.F6 ;
  PORTC.F2  =  PORTB.F5 ;
  PORTC.F3  =  PORTB.F4 ;
 INTCON.RBIF = 0;
 return;
 }
#line 376 "C:/data/emcfdm/src/pic/cncController/cnc_control.c"
 maxmult = findLargest(multipliers.x,multipliers.y,multipliers.z,multipliers.a );





 stepX = (  PORTB.F7  == 0 ) && ( currentSteps.F7 == 1 );
 stepY = (  PORTB.F6  == 0 ) && ( currentSteps.F6 == 1 );
 stepZ = (  PORTB.F5  == 0 ) && ( currentSteps.F5 == 1 );
 stepA = (  PORTB.F4  == 0 ) && ( currentSteps.F4 == 1 );


 for ( i=0;i<maxmult;i++ ){


 if ( (stepX == 1 ) && (i < multipliers.x ) ){
  PORTC.F0  = 1;
 }
 if ( (stepY == 1 ) && (i < multipliers.y ) ){
  PORTC.F1  = 1;
 }
 if ( (stepZ == 1 ) && (i < multipliers.z ) ){
  PORTC.F2  = 1;
 }
 if ( ( stepA == 1) && (i < multipliers.a ) ){
  PORTC.F3  = 1;
 }


 delay5ClockCycles(multipliers.stepLengthCycles);


 if ( (stepX == 1 ) && (i < multipliers.x ) ){
  PORTC.F0  = 0;
 }
 if ( (stepY == 1 ) && (i < multipliers.y ) ){
  PORTC.F1  = 0;
 }
 if ( (stepZ == 1 ) && (i < multipliers.z ) ){
  PORTC.F2  = 0;
 }
 if ( ( stepA == 1) && (i < multipliers.a ) ){
  PORTC.F3  = 0;
 }


 delay5ClockCycles(multipliers.delayCycles);
 }


 currentSteps = PORTB;
 INTCON.RBIF = 0;
 }
}

void interrupt_low ( void ) {
 if ( INTCON.TMR0IF ){



 unsigned short enable = 1;

 if (  PORTD.F7  == 0 ){
 enable = 0;
 *currentStatus = *msg_atLimits;
 }
 if (  PORTD.F6  == 0 ){
 enable = 0;
 *currentStatus = *msg_eStop;
 }

 if (  PORTD.F5  == 0 ){
 enable = 0;
 *currentStatus = *msg_disabled;
 }
 if (  PORTD.F4  == 0 ||  PORTD.F3  == 0 ||  PORTD.F2  == 0
 ||  PORTD.F1  == 0 ){
 enable = 0;
 *currentStatus = *msg_servoFault;
 }

 if ( enable == 1 ){
 *currentStatus = *msg_ok;
  PORTC.F4  = 0;
  PORTC.F5  = 1;
 }
 else{
  PORTC.F4  = 1;
  PORTC.F5  = 0;
 }
 statusError = enable;
 INTCON.TMR0IF = 0;
 }

}

void initRegisters(){

 TRISD = 0b11111111;
 TRISC = 0b10000000;
 TRISB = 0b11110000;
 TRISA = 0b00000000;

 currentSteps = PORTB;



 T0CON = 0b10000000;


 INTCON.RBIE = 1;
 INTCON2.RBIP=1;
 INTCON2.RBPU = 0;
 INTCON.TMR0IE = 1;
 INTCON2.TMR0IP = 0;
}




void main(){


 char x = 0;
 unsigned short i = 0;


 Usart_Init( 57600 );
 initRegisters();


 Delay_ms(100);
 printMessage(splash);


 if ( ! readMemory() ){
 printMessage(noEeprom);
 }
 printMessage(cmdPrompt);

 Delay_ms(200);


 RCON.IPEN = 1;
 INTCON.GIE = 1;

 while(1) {
 while ( Usart_Data_Ready() ){
 x = Usart_Read();





 if ( pushChar(x) ){

 if ( commandMatches(cmd_status ) ){
 printStatus();
 }
 else if ( commandMatches(cmd_saveEEprom )){
 writeMemory();
 }
 else if ( commandMatches(cmd_readEEprom )){
 readMemory();
 }
 else if ( commandMatches(cmd_defaults)){

 clearMemory();
 }
 else if ( commandMatches(cmd_step_multiplier_x)){
 multipliers.x = findIntValue(cmdBuffer);
 }
 else if ( commandMatches(cmd_step_multiplier_y)){
 multipliers.y = findIntValue(cmdBuffer);
 }
 else if ( commandMatches(cmd_step_multiplier_z)){
 multipliers.z = findIntValue(cmdBuffer);
 }
 else if ( commandMatches(cmd_step_multiplier_a)){
 multipliers.a = findIntValue(cmdBuffer);
 }
 else if ( commandMatches(cmd_enable_multiplers)){
 multipliers.enable = findIntValue(cmdBuffer);
 }
 else if ( commandMatches(cmd_delay_cycles)){
 multipliers.delayCycles = findIntValue(cmdBuffer);
 }
 else{
 printMessage(unknownCommand);
 USART_Send_String(cmdBuffer);
 }
 printMessage(cmdPrompt);
 }
 }


 };
}
