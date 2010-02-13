/********************************************************
  Extruder control project.
  2/1/2008 dcowden

  Maintains temperature of a target extruder at desired
  setpoint
  
  Thermocouple measures temperature : result is an integer ( degrees C times 4 )
  plant is heater with cycle control enabled-- output is an integer from 0 to 255
  
  RS-232 interface for setting gains and temperature setpoints


  Interesting-- to implement velocity control, all that is required
  is to reset position counter and position feedback each servo loop.
  Then, though gains will be different, the
  control is matching desired velocity instead of desired position
  
*/


#include "pid.h"
#include "duty.h"
#include "built_in.h"
#include "eeprom.h"
#include "mflow.h"

//define pins
#define SPI_PIN PORTD.F0
#define SPI_DATA PORTD.F2
#define SPI_CLOCK PORTD.F3


#define SPI_6675_DELAY 220
#define MOTOR_DIR PORTB.F2
#define MOTOR_BRAKE_PIN PORTB.F0
#define SERIAL_BAUD 57600
#define CPU_BANDWIDTH_PIN PORTD.F6
#define RCV_BUFF_LEN 10
#define CNT_PER_TURN 2000


//program constants
//#define MOTOR_SAFE_TEMP 200
#define MOTOR_SAFE_TEMP 1
//code to rescale axis if we run past the end
//the motor would have to run at full speed for about 4 hours
//to get to these limits!
#define AXIS_MAX     2000000000L
#define AXIS_MIN    -2000000000L
#define AXIS_ADJUST  400000000L
#define  TURN_ADJUST 200000

//for input step/dir contrl
#define STEP_PIN PORTC.F3
#define DIR_PIN PORTC.F0

//for limit switches
#define LIMIT_1 PORTC.F2
#define LIMIT_2 PORTC.F1
#define FAULT_OUT PORTC.F5
#define ENABLE_IN PORTC.F4

//for melt flow compensation calcs
//these values change with extruder geometry and material
#define Tmf 5.0
#define Tex 0.08
#define pMF = 0.1 //percent melt flow
#define Af = 0.012 //filament cross section area
#define ppMf = 1.1 // 1 + pMf, used frequently
#define dT = 0.01 // time for the servo loop

//conditional compilation options to control
//program size

//TODO: Read current value from AD1 to allow current limiting
//TODO: use voltage divider to detect low supply voltage
//TODO: code limit switch inputs-- switches connect to ground
//TODO: code fault output and detection. (create fault output)
//TODO: code enable/disable pin

//TODO:startup with command same as feedback value
//TODO:startup with command and feedback zero
//TODO:implement overflow logic on position
//TODO:fix eeprom write logic

/*
     PIN SUMMARY --for board version 4.
     PORTA
          F0   LMD18200 Current Sense ( AD0 -IN)
          F1   LMD18200 SUpply Votage Sense ( AD1 - IN )
          F2   QEI Index ( IN )
          F3   QEI CH A ( IN )
          F4   QEI CH B ( IN )
          F5   N/C
          F6   N/C
          F7   N/C
     PORTB
          F0   LMD18200 BRAKE ( OUT )
          F1   LMD18200 PWM ( OUT )
          F2   LMD18200 DIR ( OUT )
          F3   N/C
          F4   N/C
          F5   ICSP PGM
          F6   ICSP PGC
          F7   ICSP PGD
     PORTC
          F0   Direction Input  ( IN )
          F1   Limit Switch       ( IN )
          F2   Limit Switch 1   ( IN )
          F3   Step Input       ( IN )
          F4   Drive Enable/disable ( IN )
          F5   FAULT output     ( OUT )
          F6   RS-232 TX        ( OUT )
          F7   RS-232 RX        ( IN )

     PORTD   --set config 3h to use spi on port D
          F0   Max6675 chip select ( OUT )
          F1   SPI data out (N/C)
          F2   SPI data in from max6675 ( IN )
          F3   SPI clock out (  OUT ) -- master mode
          F4   LMD18200 thermal flag ( IN )
          F5   Heater enable pin ( OUT )
          F6   N/C
          F7   N/C
*/

unsigned short buffer = 0xFF;
char txtBuffer[40];
char cmdBuffer[40];
unsigned short txtPos = 0;


//allows manual override of heater and motor duty

unsigned short tempCount = 0u;
unsigned short stepMultiplier = 40u;
unsigned short heaterGlobalEnable = 0;
unsigned short motorGlobalEnable = 0;
unsigned short velocityControlMode = 0u;
unsigned short meltFlowComp = 0u;
unsigned short motorDirSwitches = 0u;
float qMf = 0.0;      //melt flow rate


//allow manual override of duty cycles at any time
int debugHeaterDuty = 0;
int debugMotorDuty = 0;
int debugMotorSpeed = 0;
long motorPulses = 0;
long motorTurns = 0;

struct PIDStruct pid_heater = {
      50,          //command
      0,          //feedback
      0,          //error
      0,         //deadband
      0,         //maxerror
      500,      //maxerror_i
      0,         //maxerror_d
      0,         //maxcmd_d
      0,         //error_i
      0,         //prev_error
      0,         //error_d
      0,         //prev_cmd
      0,         //cmd_d
      0,         //bias
      3.5,         //pgain
      0.8,       //igain
      0,         //dgain
      0.4,         //ff0gain
      0,         //ff1gain
      254,       //maxoutput  int--max is 32000!
      0,         //output
      0,           //enable
      0           //limit_state
};

struct PIDStruct pid_motor = {
      0,           //command
      0,           //feedback
      0,           //error
      200,         //deadband
      50000000,         //maxerror
      2000,      //maxerror_i
      20000,         //maxerror_d
      0,         //maxcmd_d
      0,         //error_i
      0,         //prev_error
      0,         //error_d
      0,         //prev_cmd
      0,         //cmd_d
      0,         //bias
      0.9,         //pgain
      0.08,       //igain
      0.2,         //dgain
      0.0,         //ff0gain
      2.0,         //ff1gain
      1020,       //maxoutput int-- 32000 at most, dont use zero!
      0,         //output
      0,           //enable
      0           //limit_state
};


const char *splash = "Extruder v0.1\0";
const char *cmdPrompt = "\nCmd:>";
const char *noEeprom = "\nNo EEPROM Data.";
const char *unknownCommand = "\n?:";

//save a version, so we know if data stored is compatible
//change this value if the data stored in EEPROM is not
//compatible with prior verions
unsigned short EEPROM_VERSION_ID = 172u;

// --- Command Summary ---
// These are serial commands that can be sent ( one per line )
// To the pic
// commands can be any lenght, but shorter saves ROM
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
    //USART_Write(10);
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
       //Usart_write(ch);  //enable this line to echo input
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


// Read PID values out of memory. This will save lots of headaches later
unsigned short readMemory(){
    unsigned short pidSize = sizeof ( pid_heater );
    unsigned short versionId;
    
    //read fingerprint and compare with current value
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
     //write header byte
     EEprom_Write_Obj(0,&EEPROM_VERSION_ID,1);
     
     //write data
     EEprom_Write_Obj(1,&pid_heater,pidSize);
     EEprom_Write_Obj(pidSize+2,&pid_motor,pidSize);
}

void clearMemory(){
     //just blank out header byte
     //device will restore to defaults on next reset
     unsigned short blank = 0xFF;
     EEprom_Write_Obj(0,&blank,1);
}

/*
   read the temperature of the t-couple
   returns the temperature in degrees C
*/
void readTemp(){

   //variables for temperature measurement
   unsigned int res_temp =0;
   unsigned short res_h = 0;
   unsigned short res_l = 0;
   //a conversion should be ready
   SPI_PIN = 0;

   //read high byte
   //PIR1.SSPIF = 0;
   SSPBUF = 0xFF;
   while ( SSPSTAT.BF == 0u );
   res_h = SSPBUF;

   
   //read low byte
   //PIR1.SSPIF = 0;
   SSPBUF = 0xFF;
   while ( SSPSTAT.BF == 0u);
   res_l = SSPBUF;

   //PIR1.SSPIF = 0;
   //start another conversion
   SPI_PIN = 1;

   //check for open circuit
   if ( res_l & 0x04 ){
      //open thermocouple should turn off heater
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
     //convert to deg C
     res_temp =  ( res_h << 5 )+ (   res_l >> 3 );

     pid_heater.feedback = (res_temp >> 2);

   }

}
void setMotorDuty ( int newDuty){
     //note: the built-in PWM modules have duty cycle resolution
     //of 12 bits, or 4096.  we use an int to get all of that resoltion
     unsigned int tmp2 = 0;
     if ( newDuty < 0u ){
        if ( MOTOR_DIR == 1 ){
           motorDirSwitches++;
        }
        MOTOR_DIR = 0;
        tmp2 = -newDuty;
     }
     else{
        if ( MOTOR_DIR == 0 ){
           motorDirSwitches++;
        }
        MOTOR_DIR = 1;
        tmp2 = newDuty;
     }
     //set motor duty limits
     //if ( tmp2 > 255 ){
     //   tmp2 = 255;
     //}

     PDC0H = Hi(tmp2);
     PDC0L = Lo(tmp2);

}

/*
  Used to run the motor in position control mode.
  In this mode, the pid loop of the motor matches commanded position,
  according to the encoder.
  
*/
void calcMotorPosition(){
  long axis_adjust = 0;
  long turn_adjust = 0;

  if  ( pid_motor.command > AXIS_MAX || pid_motor.feedback > AXIS_MAX ){
      //adjust axis position down
      axis_adjust = -AXIS_ADJUST;
      turn_adjust = -TURN_ADJUST;
  }
  if ( pid_motor.command < AXIS_MIN || pid_motor.feedback < AXIS_MIN ){
      //adjust axis position up
      axis_adjust = AXIS_ADJUST;
      turn_adjust = TURN_ADJUST;

  }
  if ( axis_adjust != 0 ){
       pid_motor.command += axis_adjust;
       pid_motor.feedback += axis_adjust;
       motorTurns += turn_adjust;
  }
  //in position control, motor pulses are directly applied
  //to commanded position,
  //and encoder counts are directly applied to feedback.
  pid_motor.command += motorPulses;
  motorPulses = 0;
  
  pid_motor.feedback = (long)(motorTurns * CNT_PER_TURN ) +
     (long)(POSCNTH << 8 ) + (long)POSCNTL;

}


/*
   This routine computes the variables rquired for meltFlow compensation,
   etc.

   The general strategy is to run the motor pid loop in velocity control
   mode, meaning pid_motor.feedback and pid_motor.command are velocities
   (in units of counts per servo loop ).

   melt flow compensation is used to adjust the commanded velocity,
   according to exponential behavior of the extruder.

   INPUTS/preconditions:
           the motor encoder reflects the most recent position
           the pid_motor structure has the previously computed values:
              previous velocity, pid_motor.prev_cmd
           current value of the melt flow variables
              qmf, current melt flow ( which starts at zero )
           dt is a constant, equal to servo loop time.
   COMPUTED values:
           from inputs, we can compute the values of:
             new velocity= motor encoder counts since last
             acceleration= new velocity - previous velocity
             target speed= new velocity + (a constant ) * acceleration
             Qmf, new value of melt flow=complex equation from spreadsheet
             adjusted velocity = more complex stuff from spreadsheet

   OUTPUTS/actions:

*/
void calcMotorVelocity(){
  long vOutput = motorPulses;
  float aIn = 0.0;
  float qTarget = 0.0;
  float qAdjusted = 0.0;
  
  //if melt flow comp is on, adjust commanded
  //velocity to account for melt flow in the extruder.
  //if not, just pass open loop velocity commands.
  if ( meltFlowComp == 1){
    //aIn = (float)(pid_motor.command - vIn) / dT;

    //target meltflowrate is a function of velocity and acceleration
    //qtarget = Af * ( vIn + (Tex * aIn) );
    
    //compute change in melt flow rate
    
    //compute adjusted flow rate
    
    //compute adjusted command
    //vOutput = (long)( qAdjusted / Af );

  }

  //in velocity control, motor command is equal to number of pulses,
  //and encoder counts are directly applied to feedback.
  pid_motor.command = motorPulses;
  
  pid_motor.feedback = (long)(motorTurns * CNT_PER_TURN ) +
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


//timer 0 is handling output duty cycle for heater (approx 60 interrupts/sec )
//timer 1 is handling interrupts for pid calculation (approx 1khz )

//the max 6675 needs to be read after about 200ms.
//we dont want to implement a delay, so keep track and
//when ready, do a read

//high priority interrupts:
//receipt of step pulse for motor,
//receipt of motor index count for movement
void interrupt ( void ){
     //use portD.F6 to show bandwidth--brightness means more cpu used.
     //CPU_BANDWIDTH_PIN = 1;

     if ( INTCON.INT0IF ){
          //CPU_BANDWIDTH_PIN = ~CPU_BANDWIDTH_PIN;
          //got an edge on the step input
          //this is a change in commanded position
          if ( DIR_PIN ){
             motorPulses += stepMultiplier;
          }
          else{
             motorPulses -= stepMultiplier;
          }
          INTCON.INT0IF = 0;
     }
     if  ( PIR3.IC2QEIF ){
        //PIR3.IC3DRIF   --  direction has changed
        //PIR3.IC2QEIF   --  index/cuonter overflow
        //PIR3.IC1IF     --  velocity updated
        //index plus. look at direction to see
        //whether to remove or add a turn
        if ( QEICON.F5 = 1 ){
            //forward
            motorTurns++;
        }
        else{
            //backward
            motorTurns--;
        }
        PIR3.IC2QEIF = 0;
     }
     //CPU_BANDWIDTH_PIN = 0;
}

//reading temperature and computing pid are lower priority
void interrupt_low(void){

     unsigned short tmp = 0u;
     unsigned short tmp2 = 0u;
     

     if ( INTCON.TMR0IF ){  //timer0, 60 hz
        //CPU_BANDWIDTH_PIN = 1;
        //timer0 interrupt is calculation of output duty cycle
        dutyInterruptProc();

        //220 ms * 60 hhz is about 13 cycles. to make sure, wait 20
        //cycles
        //also compute the PID loop for the heater here
        //computing more often is pointless since we cannot
        //read the temp more often anyway
        if ( tempCount++ == 60u ){

           readTemp();

           tempCount = 0;

           calc_pid(&pid_heater);
           //heater is one-sided, we cannot cool
           //so we'll just turn it off
           if ( pid_heater.output > 0 && heaterGlobalEnable == 1){
              setDuty(pid_heater.output);
           }
           else{
              setDuty(0);
           }
        }
        
        
       //regardless, use override if specified
       if ( debugHeaterDuty != 0u ){
          setDuty((unsigned short)debugHeaterDuty);
       }
       
       
        //reset timer to value that results in 60hz cycle
        //tricky because it has to be timed to include cost of pid loop
        TMR0H = 0x5D;
        TMR0L = 0x3D;
        //CPU_BANDWIDTH_PIN = 0;
        INTCON.TMR0IF = 0;
     }
     if (PIR1.TMR1IF ){  //timer 1, 1khz
         //timer1 interrupt is calculation of PID loop

         //QEI interrupts only happen on period events,
         //so update position in the servo loop
         
         //this is also where we compute desired
         //melt flow adjustments.
         if ( velocityControlMode == 1u){
            calcMotorVelocity();
         }
         else{
            calcMotorPosition();
         }


        //apply motor speed if specified and enabled
          if ( debugMotorSpeed != 0u && pid_motor.enable.F0 == 1 && motorGlobalEnable == 1 ) {
              pid_motor.command += debugMotorSpeed;
          }
         //if ( velocityControlMode == 1u ){
         //    pid_motor.command = debugMotorSpeed;
         //}
         //check enable flag on motor stage
         if ( motorGlobalEnable == 1 ){
             if ( pid_heater.feedback > MOTOR_SAFE_TEMP ){
                 pid_motor.enable.F0 = 1;
                 FAULT_OUT = 0;
             }
             else{
                  FAULT_OUT = 1;
             }
         }
         else{
             pid_motor.enable.F0 = 0;
             FAULT_OUT = 1;
         }
         calc_pid(&pid_motor);
         setMotorDuty(pid_motor.output );
         //setMotorDuty(0);
         //if ( debugMotorDuty != 0u ){
         //     setMotorDuty(debugMotorDuty);
         //}

         //in velocity control mode, we're interested in the
         //change in position vs time. though gains might need to
         //be reset compared to position mode ( because we're not dividing by
         //delta time) ,
         //simply avoiding accumulation of position and position feedback
         //essential converts the drive to velocity control mode
         if ( velocityControlMode == 1 ){
             motorTurns = 0;
             pid_motor.feedback = 0;
             pid_motor.command = 0;
         }

         //TMR1H = 0xFF;
         //TMR1L = 0x00;   //?, dmm reports   1.8khz, 93% bandwidth used
         //TMR1H = 0xFC;
         //TMR1L = 0xE0;  //10khz   , dmm reports 83% bandwidth , 1.65 khz
         //TMR1H = 0xF9;
         //TMR1L = 0xC0;  //5khz    dmm reports 72% bandwidth, 1.4khz.
         //TMR1H = 0xF0;
         //TMR1L = 0x60;  //2khz    50% bandwidth (dmm reports 1khz though  )
         //this implies that computations are taking about 1ms?

         TMR1H = 0xE0;
         TMR1L = 0x9F; //1khz       dmm reports 750 hz, 33% bandwidth
         PIR1.TMR1IF = 0;

     }
     //CPU_BANDWIDTH_PIN = 0;
}


//TODO: pig, takes 3500 words
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

  //set registers
  PORTB = 0;

  //initialize spi
  //NOTE: CONFIG3H must be set to mux pins on RD0/1/2!!!
  
  //TRISE = 0b11111111;
  TRISD = 0b00000100;
  TRISC = 0b10011011; //RC3 is ext trigger, used for step input. RC0 is dir pin
  TRISB = 0b00000000;           // PORTB is output
  TRISA = 0b11111111;

  //0b   1      0     0               0          0          000
  //     enable 16bit internal clock  low-high   prescale   1:2 prescale
  //@32mhz, focc/4 = 8mhz, 8000000/2/65536 =  61 interrupts/sec
  //@40mhz, focc/4 = 10mhz, 10000000/4/65536    = 38 interrupts/sec
  T0CON = 0b10000000; //256:1 prescaler, internal clock, enable

  //set up TMR1 for use for PID loop
  //   0b  1      0            00                11              01
  //       16bit  sys clock    1:1 prescale      int. clock      enable
  T1CON = 0b10000101;


  ADCON1 = 0x00;
  ANSEL0 = 0b00000011;

  //enable edge capture on pin RC3/4/5 for step pulses
  //cannot use ccp2, as it requires TIMR1 as a resource
  //enable RC3 (INT0) as step pin for input
  INTCON2.INTEDG0 = 0; //interrupt on rising edge


  INTCON.INT0IE = 1; //enable int0 interrupts
  //int0 is always high priority. ( after an hour looking for it )
  
  //ccp2:     capture enabled, each falling edge 0000   0100
  //note: ccp2 is multiplexed with RC1,pin 16
  //CCP2CON = 0b00000100;
  //PIE2.CCP2IE = 1;
  //IPR2.CCP2IP= 1; //ccp2 interrupt high priority
  
  
  //interrupts
  INTCON.TMR0IE = 1; //enable timer0 interrupts
  INTCON2.TMR0IP = 0; //timer 0 low priority
  PIE1.TMR1IE = 1; //enable timer1 interrupts
  IPR1.TMR1IP = 0; //timer 1 low priority





  //configure spi port
  //enable spi
  //master mode, rising edge sample,
  SSPSTAT = 0b01000000;

  //idle low, fosc/64, sample middle
  //SSPCON = 0b00100000;
  SSPCON=0b00100001;
  
  SSPCON.SSPEN = 1; //0=input pins, 1=spi port
  PIR1.SSPIF = 0;
  
  //turn off ssp interrupts
  PIE1.SSPIE = 0;
  
  //no pwm timebase interrupts
  PIE3.PTIE = 0;
  
  
  //init QEI
  //0                 0         1         110                                 01
  //enable velocity   no oflow  forward   4x mode,interrupt on period match   1:1 velocity pulse
  QEICON = 0b10011000;
  //QEICON = 0b10011000

  //timer 5 is used to provide velocity info in conjunction with qei
  //synchronous timer mode
  //disable special events, disable during sleep, continuous mode,
  //1:1 prescaler , synchronize external, external clock, enabled
  
  //the VELR register will have the number of counts
  //that occur within that time
  //timer 5 is 16 bits, with 1:8 bit prescaler
  /**
      2000 encoder pulses per revolution
      1:4 velocity scaler ==> 500 pulses per revolution
      Fosc=40000000 hz
      Tcy = 10,000,000 hz
      TMR5 is a 16 bit timer, with 1:1-1:8 prescaler
      see timer clock spreadsheet,
      but with TMR5 prescale of 1:4, and 1:1 velocity prescale
      velocity pulse scale,
      motor speed ( rpm ) =
          10,000,000 / 8 * 60 / 2000 / cnts
          
      the max motor speed we could measure is
          10,000,000 / 8 * 60 / 2000 / 1 =  37,500 rpm
      the min motor speed we can measure is
          10,000,000 / 4 * 60 / 2000 / 65536 = 0.57 rpm

      4000 rpm ( creating 18 timer counts)
      and 1.25 rpm ( creating 60000 timer counts)
  
  **/
  T5CON = 0b00011001;

  //all noise filters disabled
  DFLTCON = 0b00000000;
  //DFLTCON = 0b00111001;
  POSCNTH = 0x00;
  POSCNTL = 0x00;
  VELRL = 0x00;
  VELRH = 0x00;

  //2000 counts per revolution
  MAXCNTH = 0x07;
  MAXCNTL = 0xD0;

  //enable qei interrupts
  //PIE3.IC3DRIE = 1;
  
  ///changed for debugging!
  PIE3.IC2QEIE = 1;
  //PIE3.IC1IE = 1;


  //init PWM
  /*
    PWM Frequency: (free running mode )
    f(hz) = fosc / ( PTPER + 1 ) * PTMR_PS
      
    PTMR_PS is prescale set in PTCON1
    PTPER is set in PTPERH|PTPERL

    PDCxH:|PDCxL set duty cycle.
    the resolution of the duty cycle register
    is based on clock
  */

   //lets try to set up PWM0 for 2khz period
   // 0000           00            00
   // 1:1 postscale  fosc/4 base   free running mode
   PTCON0 = 0b00000000;

   //    1 pwm timer enable 000000 not used
   PTCON1 = 0b1000000;
   // 0 010                  0000
   // 0 enable pwm0/pwm1     all pwm in independent mode
   PWMCON0 = 0b00011111;

   // 0000                          0                     0  1              1
   // 1:1 special trigger postscale special trigger on up 0  enable updates synchronized updates
   PWMCON1 = 0b00000000;

   //set duty cycle perdiod
   //freq = fosc/ ( 4 * PTMRPS * ( PTPER + 1 ) )
   //below is PTPER=0x03FF => 1023, so f @32mhz = 7.8khz
   //@40mhz -   9.775khz
   //use 156khz, so that the frequency is larger
   //by quite a bit than the servo loop ( at about 1khz )
   //this is evidently 10 bit resolution
   PTPERH = 0x00;
   PTPERL = 0xFF;
   
   //PTPERH = 0b00000011;
   //PTPERL = 0b11111111;

   //50% duty cycle
   //only least significant 12 bits are used
   //PDC0H = 0X08;
   PDC0H=0x00;
   PDC0L = 0x00;
   PTCON1.F7=1;

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


void delay_100_ms(){
     Delay_ms(100);
}
//main routine-- mainly user interface/serial input
void main() {

  char x;
  unsigned short i;
  txtPos = 0;

  
  Usart_Init(57600);
  initDuty(120);
  initRegisters();
  resetPosition();
  
  //delay_100_ms();
  Delay_ms(100);
  printMessage(splash);

  //read memory...
  if ( ! readMemory() ){
     printMessage(noEeprom);
  }
  printMessage(cmdPrompt);
  //delay_100_ms();
  Delay_ms(200);
  
  //enable interrupts
  RCON.IPEN = 1;
  INTCON.GIE = 1;
  INTCON.PEIE = 1;
  MOTOR_BRAKE_PIN = 0;
  while(1) {
  //CPU_BANDWIDTH_PIN = ~CPU_BANDWIDTH_PIN;
  //Delay_ms(1000);
  
    while ( Usart_Data_Ready() ) {
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
           else if ( commandMatches(cmd_globalMotorEnable)){
              motorGlobalEnable = findIntValue(cmdBuffer);
              //if ( motorGlobalEnable == 0 ){
              //   MOTOR_BRAKE_PIN = 0;
              //}
              //else{
              //   MOTOR_BRAKE_PIN = 1;
              //}
           }
           else if ( commandMatches(cmd_globalHeaterEnable)){
                heaterGlobalEnable = findIntValue(cmdBuffer);
           }
           else if ( commandMatches(cmd_defaults)){
                //restore to factory defaults
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
              //setMotorDuty(debugMotorDuty);
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
              //unrecognized command!
              printMessage(unknownCommand);
              USART_Send_String(cmdBuffer);

           }
           printMessage(cmdPrompt);
       }
       //else, character was not a newline

    }
  };
  
}//~!


