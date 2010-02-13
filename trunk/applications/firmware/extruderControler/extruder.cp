#line 1 "C:/data/emcfdm/src/pic/extruderControler/extruder.c"
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
#line 1 "c:/data/emcfdm/src/pic/extrudercontroler/duty.h"




void initDuty( unsigned short dtyperiod );
void dutyInterruptProc(void);
void setDuty( unsigned short setpnt );
#line 1 "c:/apps/mikroc/include/built_in.h"
#line 1 "c:/data/emcfdm/src/pic/extrudercontroler/eeprom.h"
typedef unsigned char byte;
typedef unsigned int word;


void Eeprom_Write_Obj(word addr,void *obj,byte size);
void Eeprom_Read_Obj(word addr,void *obj,byte size);
#line 1 "c:/data/emcfdm/src/pic/extrudercontroler/mflow.h"



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
#line 128 "C:/data/emcfdm/src/pic/extruderControler/extruder.c"
unsigned short buffer = 0xFF;
char txtBuffer[40];
char cmdBuffer[40];
unsigned short txtPos = 0;




unsigned short tempCount = 0u;
unsigned short stepMultiplier = 40u;
unsigned short heaterGlobalEnable = 0;
unsigned short motorGlobalEnable = 0;
unsigned short velocityControlMode = 0u;
unsigned short meltFlowComp = 0u;
unsigned short motorDirSwitches = 0u;
float qMf = 0.0;



int debugHeaterDuty = 0;
int debugMotorDuty = 0;
int debugMotorSpeed = 0;
long motorPulses = 0;
long motorTurns = 0;

struct PIDStruct pid_heater = {
 50,
 0,
 0,
 0,
 0,
 500,
 0,
 0,
 0,
 0,
 0,
 0,
 0,
 0,
 3.5,
 0.8,
 0,
 0.4,
 0,
 254,
 0,
 0,
 0
};

struct PIDStruct pid_motor = {
 0,
 0,
 0,
 200,
 50000000,
 2000,
 20000,
 0,
 0,
 0,
 0,
 0,
 0,
 0,
 0.9,
 0.08,
 0.2,
 0.0,
 2.0,
 1020,
 0,
 0,
 0
};


const char *splash = "Extruder v0.1\0";
const char *cmdPrompt = "\nCmd:>";
const char *noEeprom = "\nNo EEPROM Data.";
const char *unknownCommand = "\n?:";




unsigned short EEPROM_VERSION_ID = 172u;





const char *cmd_status = "s";
const char *cmd_saveEEprom = "v";
const char *cmd_readEEprom = "rd";
const char *cmd_defaults = "de";
const char *cmd_globalHeaterEnable = "hge";
const char *cmd_globalMotorEnable = "mge";
const char *cmd_heater_Kp = "hp";
const char *cmd_heater_Ki = "hi";
const char *cmd_heater_Kd = "hd";
const char *cmd_heater_kff0 = "hfg0";
const char *cmd_heater_duty = "hy";
const char *cmd_heater_SetTemp = "ht";
const char *cmd_heater_SetFeedback = "hf";
const char *cmd_motor_Kp = "mp";
const char *cmd_motor_Ki = "mi";
const char *cmd_motor_Kd = "md";
const char *cmd_motor_duty = "my";
const char *cmd_motor_Kff1 = "mfg";
const char *cmd_motor_SetPos = "mcmd";
const char *cmd_motor_Speed = "msp";
const char *cmd_motor_SetFeedback = "mf";







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



unsigned short readMemory(){
 unsigned short pidSize = sizeof ( pid_heater );
 unsigned short versionId;


 Eeprom_Read_Obj(0, &versionId,1 );
 if ( versionId == EEPROM_VERSION_ID ){
 Eeprom_Read_Obj(1,&pid_heater,pidSize );
 Eeprom_Read_Obj(pidSize+2,&pid_motor,pidSize);
 return 1;
 }
 else{
 return 0;
 }
}

void writeMemory(){
 unsigned short pidSize = sizeof( pid_heater );

 EEprom_Write_Obj(0,&EEPROM_VERSION_ID,1);


 EEprom_Write_Obj(1,&pid_heater,pidSize);
 EEprom_Write_Obj(pidSize+2,&pid_motor,pidSize);
}

void clearMemory(){


 unsigned short blank = 0xFF;
 EEprom_Write_Obj(0,&blank,1);
}
#line 332 "C:/data/emcfdm/src/pic/extruderControler/extruder.c"
void readTemp(){


 unsigned int res_temp =0;
 unsigned short res_h = 0;
 unsigned short res_l = 0;

  PORTD.F0  = 0;



 SSPBUF = 0xFF;
 while ( SSPSTAT.BF == 0u );
 res_h = SSPBUF;




 SSPBUF = 0xFF;
 while ( SSPSTAT.BF == 0u);
 res_l = SSPBUF;



  PORTD.F0  = 1;


 if ( res_l & 0x04 ){

 pid_heater.feedback = 2000;
 pid_heater.enable = 0;
 }
 else{
 if ( heaterGlobalEnable == 1 ){
 pid_heater.enable.F0 = 1;
 }
 else{
 pid_heater.enable.F0 = 0;
 }

 res_temp = ( res_h << 5 )+ ( res_l >> 3 );

 pid_heater.feedback = (res_temp >> 2);

 }

}
void setMotorDuty ( int newDuty){


 unsigned int tmp2 = 0;
 if ( newDuty < 0u ){
 if (  PORTB.F2  == 1 ){
 motorDirSwitches++;
 }
  PORTB.F2  = 0;
 tmp2 = -newDuty;
 }
 else{
 if (  PORTB.F2  == 0 ){
 motorDirSwitches++;
 }
  PORTB.F2  = 1;
 tmp2 = newDuty;
 }





 PDC0H =  ((char *)&tmp2)[1] ;
 PDC0L =  ((char *)&tmp2)[0] ;

}
#line 413 "C:/data/emcfdm/src/pic/extruderControler/extruder.c"
void calcMotorPosition(){
 long axis_adjust = 0;
 long turn_adjust = 0;

 if ( pid_motor.command >  2000000000L  || pid_motor.feedback >  2000000000L  ){

 axis_adjust = - 400000000L ;
 turn_adjust = - 200000 ;
 }
 if ( pid_motor.command <  -2000000000L  || pid_motor.feedback <  -2000000000L  ){

 axis_adjust =  400000000L ;
 turn_adjust =  200000 ;

 }
 if ( axis_adjust != 0 ){
 pid_motor.command += axis_adjust;
 pid_motor.feedback += axis_adjust;
 motorTurns += turn_adjust;
 }



 pid_motor.command += motorPulses;
 motorPulses = 0;

 pid_motor.feedback = (long)(motorTurns *  2000  ) +
 (long)(POSCNTH << 8 ) + (long)POSCNTL;

}
#line 474 "C:/data/emcfdm/src/pic/extruderControler/extruder.c"
void calcMotorVelocity(){
 long vOutput = motorPulses;
 float aIn = 0.0;
 float qTarget = 0.0;
 float qAdjusted = 0.0;




 if ( meltFlowComp == 1){
#line 496 "C:/data/emcfdm/src/pic/extruderControler/extruder.c"
 }



 pid_motor.command = motorPulses;

 pid_motor.feedback = (long)(motorTurns *  2000  ) +
 (long)(POSCNTH << 8 ) + (long)POSCNTL;

 motorPulses = 0;
 motorTurns = 0;

}

void resetPosition(){
 motorTurns = 0;
 POSCNTH = 0;
 POSCNTL = 0;
 pid_motor.feedback = 0;
 pid_motor.command = 0;
}
#line 529 "C:/data/emcfdm/src/pic/extruderControler/extruder.c"
void interrupt ( void ){



 if ( INTCON.INT0IF ){



 if (  PORTC.F0  ){
 motorPulses += stepMultiplier;
 }
 else{
 motorPulses -= stepMultiplier;
 }
 INTCON.INT0IF = 0;
 }
 if ( PIR3.IC2QEIF ){





 if ( QEICON.F5 = 1 ){

 motorTurns++;
 }
 else{

 motorTurns--;
 }
 PIR3.IC2QEIF = 0;
 }

}


void interrupt_low(void){

 unsigned short tmp = 0u;
 unsigned short tmp2 = 0u;


 if ( INTCON.TMR0IF ){


 dutyInterruptProc();






 if ( tempCount++ == 60u ){

 readTemp();

 tempCount = 0;

 calc_pid(&pid_heater);


 if ( pid_heater.output > 0 && heaterGlobalEnable == 1){
 setDuty(pid_heater.output);
 }
 else{
 setDuty(0);
 }
 }



 if ( debugHeaterDuty != 0u ){
 setDuty((unsigned short)debugHeaterDuty);
 }




 TMR0H = 0x5D;
 TMR0L = 0x3D;

 INTCON.TMR0IF = 0;
 }
 if (PIR1.TMR1IF ){







 if ( velocityControlMode == 1u){
 calcMotorVelocity();
 }
 else{
 calcMotorPosition();
 }



 if ( debugMotorSpeed != 0u && pid_motor.enable.F0 == 1 && motorGlobalEnable == 1 ) {
 pid_motor.command += debugMotorSpeed;
 }




 if ( motorGlobalEnable == 1 ){
 if ( pid_heater.feedback >  1  ){
 pid_motor.enable.F0 = 1;
  PORTC.F5  = 0;
 }
 else{
  PORTC.F5  = 1;
 }
 }
 else{
 pid_motor.enable.F0 = 0;
  PORTC.F5  = 1;
 }
 calc_pid(&pid_motor);
 setMotorDuty(pid_motor.output );
#line 662 "C:/data/emcfdm/src/pic/extruderControler/extruder.c"
 if ( velocityControlMode == 1 ){
 motorTurns = 0;
 pid_motor.feedback = 0;
 pid_motor.command = 0;
 }
#line 678 "C:/data/emcfdm/src/pic/extruderControler/extruder.c"
 TMR1H = 0xE0;
 TMR1L = 0x9F;
 PIR1.TMR1IF = 0;

 }

}



 void printFloat( char* name, float f ){
 char floatTxt[13];
 USART_Send_String(name);
 FloatToStr(f,floatTxt);
 USART_Send_String(floatTxt);
}


void printStatus(){
 printFloat("ht=",pid_heater.command);
 printFloat(",hv=",pid_heater.feedback);
 printFloat(",hp=",pid_heater.pgain);
 printFloat(",hi=",pid_heater.igain);
 printFloat(",hd=",pid_heater.dgain);
 printFloat(",hfg0=",pid_heater.ff0gain);
 printFloat(",hout=",pid_heater.output);
 printFloat(",herr=",pid_heater.error);
 printFloat(",herri=",pid_heater.error_i);
 printFloat(",herrd=",pid_heater.error_d);
 printFloat(",he=",pid_heater.enable);
 printFloat(",mcmd=",pid_motor.command);
 printFloat(",mv=",pid_motor.feedback);
 printFloat(",mp=",pid_motor.pgain);
 printFloat(",mi=",pid_motor.igain);
 printFloat(",md=",pid_motor.dgain);
 printFloat(",mfg=",pid_motor.ff1gain);
 printFloat(",mout=",pid_motor.output);
 printFloat(",merr=",pid_motor.error);
 printFloat(",merri=",pid_motor.error_i);
 printFloat(",merrd=",pid_motor.error_d);
 printFloat(",me=",pid_motor.enable);
 printFloat(",mTurns=",motorTurns);
 printFloat(",mge=",motorGlobalEnable);
 printFloat(",hge=",heaterGlobalEnable);
 printFloat(",dirswitchs=",motorDirSwitches);
}

void initRegisters(){


 PORTB = 0;





 TRISD = 0b00000100;
 TRISC = 0b10011011;
 TRISB = 0b00000000;
 TRISA = 0b11111111;





 T0CON = 0b10000000;




 T1CON = 0b10000101;


 ADCON1 = 0x00;
 ANSEL0 = 0b00000011;




 INTCON2.INTEDG0 = 0;


 INTCON.INT0IE = 1;










 INTCON.TMR0IE = 1;
 INTCON2.TMR0IP = 0;
 PIE1.TMR1IE = 1;
 IPR1.TMR1IP = 0;








 SSPSTAT = 0b01000000;



 SSPCON=0b00100001;

 SSPCON.SSPEN = 1;
 PIR1.SSPIF = 0;


 PIE1.SSPIE = 0;


 PIE3.PTIE = 0;





 QEICON = 0b10011000;
#line 834 "C:/data/emcfdm/src/pic/extruderControler/extruder.c"
 T5CON = 0b00011001;


 DFLTCON = 0b00000000;

 POSCNTH = 0x00;
 POSCNTL = 0x00;
 VELRL = 0x00;
 VELRH = 0x00;


 MAXCNTH = 0x07;
 MAXCNTL = 0xD0;





 PIE3.IC2QEIE = 1;
#line 872 "C:/data/emcfdm/src/pic/extruderControler/extruder.c"
 PTCON0 = 0b00000000;


 PTCON1 = 0b1000000;


 PWMCON0 = 0b00011111;



 PWMCON1 = 0b00000000;








 PTPERH = 0x00;
 PTPERL = 0xFF;







 PDC0H=0x00;
 PDC0L = 0x00;
 PTCON1.F7=1;

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


void delay_100_ms(){
 Delay_ms(100);
}

void main() {

 char x;
 unsigned short i;
 txtPos = 0;


 Usart_Init(57600);
 initDuty(120);
 initRegisters();
 resetPosition();


 Delay_ms(100);
 printMessage(splash);


 if ( ! readMemory() ){
 printMessage(noEeprom);
 }
 printMessage(cmdPrompt);

 Delay_ms(200);


 RCON.IPEN = 1;
 INTCON.GIE = 1;
 INTCON.PEIE = 1;
  PORTB.F0  = 0;
 while(1) {



 while ( Usart_Data_Ready() ) {
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
 else if ( commandMatches(cmd_globalMotorEnable)){
 motorGlobalEnable = findIntValue(cmdBuffer);






 }
 else if ( commandMatches(cmd_globalHeaterEnable)){
 heaterGlobalEnable = findIntValue(cmdBuffer);
 }
 else if ( commandMatches(cmd_defaults)){

 clearMemory();
 }
 else if ( commandMatches(cmd_heater_kff0)){
 pid_heater.ff0gain = findFloatValue(cmdBuffer);
 }
 else if ( commandMatches(cmd_heater_SetFeedback)){
 pid_heater.feedback = findIntValue(cmdBuffer);
 }
 else if ( commandMatches(cmd_heater_Kp )){
 pid_heater.pgain = findFloatValue(cmdBuffer);
 }
 else if ( commandMatches(cmd_heater_Ki )){
 pid_heater.igain = findFloatValue(cmdBuffer);
 }
 else if ( commandMatches(cmd_heater_Kd )){
 pid_heater.dgain = findFloatValue(cmdBuffer);
 }
 else if ( commandMatches(cmd_heater_duty )){
 debugHeaterDuty = findIntValue(cmdBuffer);
 }

 else if ( commandMatches(cmd_heater_SetTemp )){
 pid_heater.command = findIntValue(cmdBuffer);
 }
 else if ( commandMatches(cmd_motor_Kp )){
 pid_motor.pgain = findFloatValue(cmdBuffer);
 }
 else if ( commandMatches(cmd_motor_Ki )){
 pid_motor.igain = findFloatValue(cmdBuffer);
 }
 else if ( commandMatches(cmd_motor_Kd )){
 pid_motor.dgain = findFloatValue(cmdBuffer);
 }
 else if ( commandMatches(cmd_motor_Kff1 )){
 pid_motor.ff1gain = findFloatValue(cmdBuffer);
 }
 else if ( commandMatches(cmd_motor_duty )){
 debugMotorDuty = findIntValue(cmdBuffer);

 }
 else if ( commandMatches(cmd_motor_SetPos )){
 pid_motor.command = findLongValue(cmdBuffer);
 }
 else if ( commandMatches(cmd_motor_Speed )){
 debugMotorSpeed = findIntValue(cmdBuffer);
 }
 else if ( commandMatches(cmd_motor_SetFeedback)){
 pid_motor.feedback = findLongValue(cmdBuffer);

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
