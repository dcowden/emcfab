/*******
  CNC Controller project
  10/10/2009
  
  CNC Controller with these features:
    - estop and limit switch interlock
    - step multiplier on all channels
    - rs-232 commnication for setting options
    - eeprom saved values
    
    
    
  TODO:
       Compute cycle delay for generating steps ( currently hardcoded )
       allow dynamic change of controller max step rate
       save controller max step rate
       set logic inversion of limit switches and estop

*/

#include "eeprom.h"
#include "built_in.h"

#define RCV_BUFF_LEN 10
#define SERIAL_BAUD 57600


/* Input Pins */
#define LIMIT_IN      PORTD.F7
#define ESTOP_IN      PORTD.F6
#define PC_ENABLE_IN  PORTD.F5
#define A_ENABLE_IN   PORTD.F4
#define Z_ENABLE_IN   PORTD.F3
#define Y_ENABLE_IN   PORTD.F2
#define X_ENABLE_IN   PORTD.F1


/* these must be on port B, which has interrupt on change
   warning: x and y overlap with icsp pins */
#define X_STEP_IN   PORTB.F7
#define Y_STEP_IN   PORTB.F6
#define Z_STEP_IN   PORTB.F5
#define A_STEP_IN   PORTB.F4

/* Output Pins */

#define X_STEP_OUT          PORTC.F0
#define Y_STEP_OUT          PORTC.F1
#define Z_STEP_OUT          PORTC.F2
#define A_STEP_OUT          PORTC.F3
#define MACHINE_ERROR_OUT   PORTC.F4
#define SPINDLE_ENABLE_OUT  PORTC.F5




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

//save a version, so we know if data stored is compatible
//change this value if the data stored in EEPROM is not
//compatible with prior verions
unsigned short EEPROM_VERSION_ID = 112u;

// --- Command Summary ---
// These are serial commands that can be sent ( one per line )
// To the pic
// commands can be any lenght, but shorter saves ROM
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




//initialize defaults of step multipliers
struct  AXISShort multipliers = {
  1u, // x step multiplier
  1u, // y step multiplier
  1u, // z step multiplier
  1u,  // a step multiplier
  0u, //disable by default
  3u, //time between steps
  2u, //step length
  
  /**
     intervals are defined in somewhat weird terms
     the time will be 3 + 5*value clock cycles.
     
     so, a value of 2 means:
       3 + 5(2) = 13 clock cycles
       @8mhz = 13*.125 =  1.625 us
       
     the maxium value is 255, or
      3 + (255)*5 =  160us.
      
      this would mean the slowest step rate would be
      1 step every 320us or about 3khz
      
     for a target controller having step rate of 250khz,
     ( and assuming on time of 1/2 that rate ),
     we want both values to be about 2 us
     which is 16 clock cycles, or about (16-3)/5 = 3
  
  
  **/
};


unsigned short statusError = 0;
unsigned short currentSteps = 0x00;


//////////////////////////////////////////////////////////////////
//    Utility Functions
//
//
//////////////////////////////////////////////////////////////////
// --- Copying strings from ROM to RAM
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
       //txtPos++;
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
       Usart_write(ch);  //enable this line to echo input
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

//this routine is very expensive, if it can be avoided.
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

// Read PID values out of memory. This will save lots of headaches later
unsigned short readMemory(){
    unsigned short size = sizeof ( multipliers );
    unsigned short versionId;

    //read fingerprint and compare with current value
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
     //write header byte
     EEprom_Write_Obj(0,&EEPROM_VERSION_ID,1);

     //write data
     EEprom_Write_Obj(1,&multipliers,size);

}

void clearMemory(){
     //just blank out header byte
     //device will restore to defaults on next reset
     unsigned short blank = 0xFF;
     EEprom_Write_Obj(0,&blank,1);
}


//check to see if the specified buffer matches
//the template.
//the template is usually smaller than source
//characters are compared until there is no data in either buffer
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
   //find the value of a command
   //where the format is like COMMAND=<value>
   char *ptr = strchr(buffer,'=');
   return atof(++ptr);
}
int findIntValue ( char *buffer){
   //find the value of a command
   //where the format is like COMMAND=<value>
   char *ptr = strchr(buffer,'=');
   return atoi(++ptr);
}
long findLongValue ( char *buffer){
   //find the value of a command
   //where the format is like COMMAND=<value>
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
          else{ // c > a > b
                if ( c > d )
                   return c;
                else
                   return d;
          }
       }
       else{ //a < b
          if ( b > c ){
               if ( b > d ){
                  return b;
               }
               else{
                  return d;
               }
          }
          else{
             // c > b > a
             if ( c > d )
                return c;
             else
                 return d;
          }
       }
}

///Interrupt Handlers
// The only interrupt enabled should be portB change interrupt
void interrupt ( void ){
    if ( INTCON.RBIF ){
    
       //portb value changed-- generate output steps
       unsigned short i;
       unsigned short maxmult;
       unsigned short stepX = 0;
       unsigned short stepY = 0;
       unsigned short stepZ = 0;
       unsigned short stepA = 0;
       
       //if we are in a global error, stop all outputs
       //and drive all outputs low
       if ( statusError == 1 ){
           X_STEP_OUT = 0;
           Y_STEP_OUT = 0;
           Z_STEP_OUT = 0;
           A_STEP_OUT = 0;
           INTCON.RBIF = 0;
           return;
       }
       
       //if multpliers are disabled, simply set outputs
       //and return
       if ( multipliers.enable == 0 ){
       
          X_STEP_OUT = X_STEP_IN;
          Y_STEP_OUT = Y_STEP_IN;
          Z_STEP_OUT = Z_STEP_IN;
          A_STEP_OUT = A_STEP_IN;
          INTCON.RBIF = 0;
          return;
       }

       //this is a bit tricky-- we would like to generate
       //four separate signals with the proper timing
       //but we dont know which signals will need to change
       //what we can do though is assume that we'll
       //have enough clock cycles between interrupts to generate the
       //multiplier
       
       //note there is a potential constraint between the target
       //controller and the source pc. if the source pc times the
       //multiplier is greater than the target controller, we wont
       //have enough time to generate the steps
       
       //what is the largest multiplier?
       maxmult = findLargest(multipliers.x,multipliers.y,multipliers.z,multipliers.a );
       
       //determine which axes need to step
       //need to step when we transition from high to low


       stepX = ( X_STEP_IN == 0 ) && ( currentSteps.F7 == 1 );
       stepY = ( Y_STEP_IN == 0 ) && ( currentSteps.F6 == 1 );
       stepZ = ( Z_STEP_IN == 0 ) && ( currentSteps.F5 == 1 );
       stepA = ( A_STEP_IN == 0 ) && ( currentSteps.F4 == 1 );
       
       //generate steps
       for ( i=0;i<maxmult;i++ ){
       
           //paint fence. Up...
           if (  (stepX == 1 ) && (i < multipliers.x ) ){
               X_STEP_OUT = 1;
           }
           if ( (stepY == 1 ) && (i < multipliers.y ) ){
               Y_STEP_OUT = 1;
           }
           if ( (stepZ == 1 ) && (i < multipliers.z ) ){
               Z_STEP_OUT = 1;
           }
           if ( ( stepA == 1) && (i < multipliers.a ) ){
               A_STEP_OUT = 1;
           }

           //hold high for this length
           delay5ClockCycles(multipliers.stepLengthCycles);
           
           //down.
           if (  (stepX == 1 ) && (i < multipliers.x ) ){
               X_STEP_OUT = 0;
           }
           if ( (stepY == 1 ) && (i < multipliers.y ) ){
               Y_STEP_OUT = 0;
           }
           if ( (stepZ == 1 ) && (i < multipliers.z ) ){
               Z_STEP_OUT = 0;
           }
           if ( ( stepA == 1) && (i < multipliers.a ) ){
               A_STEP_OUT = 0;
           }
           
           //wait for time to generate the next step
           delay5ClockCycles(multipliers.delayCycles);
       }
       
       //save current value
       currentSteps = PORTB;
       INTCON.RBIF = 0;
    }
}

void interrupt_low ( void ) {
    if ( INTCON.TMR0IF ){
    
      //calculate saftey interlock value
      //limits and enables are high on error, low on ok
      unsigned short enable = 1;
      
      if ( LIMIT_IN == 0 ){
          enable = 0;
          *currentStatus = *msg_atLimits;
      }
      if ( ESTOP_IN == 0 ){
          enable = 0;
          *currentStatus = *msg_eStop;
      }
      
      if ( PC_ENABLE_IN == 0 ){
          enable = 0;
          *currentStatus = *msg_disabled;
      }
      if ( A_ENABLE_IN == 0 || Z_ENABLE_IN == 0 || Y_ENABLE_IN == 0
          || X_ENABLE_IN == 0 ){
          enable = 0;
          *currentStatus = *msg_servoFault;
      }
      
      if ( enable == 1 ){
         *currentStatus = *msg_ok;
         MACHINE_ERROR_OUT  = 0;
         SPINDLE_ENABLE_OUT  = 1;
      }
      else{
         MACHINE_ERROR_OUT  = 1;
         SPINDLE_ENABLE_OUT  = 0;
      }
      statusError = enable;
      INTCON.TMR0IF = 0;
    }

}

void initRegisters(){
  //0=output, 1=input
  TRISD = 0b11111111;  //inputs
  TRISC = 0b10000000; //rs-232, outputs
  TRISB = 0b11110000; //step inputs from pc
  TRISA = 0b00000000;  //not used

  currentSteps = PORTB;
  
  //timr0 interrupts to check inputs and outputs
  //these must override user input on the serial port
  T0CON = 0b10000000;
  
  //interrupts
  INTCON.RBIE = 1;   //interrupt on portb changes
  INTCON2.RBIP=1; //portb change interttupt high priority
  INTCON2.RBPU = 0; //enable portb pullups
  INTCON.TMR0IE = 1; //enable timer0 interrupts
  INTCON2.TMR0IP = 0; //timer 0 low priority
}



///////Main Routine
void main(){


     char x = 0;
     unsigned short i = 0;
    
    
     Usart_Init(SERIAL_BAUD);
     initRegisters();
     
     
     Delay_ms(100);
     printMessage(splash);
     
     //read memory...
     if ( ! readMemory() ){
        printMessage(noEeprom);
     }
     printMessage(cmdPrompt);

     Delay_ms(200);
     
     //enable interrupts
     RCON.IPEN = 1;
     INTCON.GIE = 1;
     
     while(1) {
         while ( Usart_Data_Ready() ){
             x = Usart_Read();
             
             //push character onto the command
             //buffer. if command is complete
             //(newline)
             //then process the command
             if ( pushChar(x) ){
                 //command is in txtBuffer
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
                      //restore to factory defaults
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

