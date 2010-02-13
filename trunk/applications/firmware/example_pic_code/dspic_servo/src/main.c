//---------------------------------------------------------------------
//	File:		main.c
//
//	Written By:	Lawrence Glaister VE7IT
//
// Purpose: This program is used to control an single axis servo card
//			using a pic 30f2010 and a OPA549 power op am driver stage.
// 
// The following files should be included in the MPLAB project:
//
//		main.c		    -- Main source code file
//		capture.c		-- interface to pc quadrature cmd inputs using IC1 and IC2
//      timer1.c        -- timer 1 setup for 100us intervals
//      serial.c        -- interface to pc serial port for tuning - 9600n81
//      encoder.c       -- interface to quadature encoder
//		pwm.c			-- pwn ch for motor current control
//		pid.c			-- actual code for pid loop
//		save-res.c		-- routines to read/write configuration
//		p30f4012.gld	-- Linker script file
//		DataEEPROM.s	-- assembler file for read/write eeprom
//---------------------------------------------------------------------
//
// Revision History
//
// July 4 2006 -- first version 
// Aug 13 2006 -- fixed wrap around problem in servo loop
//             -- when servo is reenabled, the target posn is set to the current posn
//             -- if drive faults, it now stays faulted until pwr cycle
//				  or disable/reenable
// Aug 15 2006 -- added pulse multiplier option for high count motor encoders
// Sept 23 2006-  servo loop from 1ms to 500us (2khz rate)
// 
//---------------------------------------------------------------------- 
#include <stdio.h>
#include "dspicservo.h"
#include <math.h>

//--------------------------Device Configuration------------------------       
// pll16 needs a 30mhz part
_FOSC( XT_PLL16 );  // 6mhz * PLL16  / 4 = 24mips
_FWDT(WDT_OFF);                   // wdt off
/* enablebrownout @4.2v, 64ms powerup delay, MCLR pin active */
/* pwm pins in use, both active low to give 64ms delay on powerup */
//_FBORPOR(PBOR_ON & BORV_42 & PWRT_64 & MCLR_EN & RST_PWMPIN & PWMxL_ACT_LO );
_FBORPOR(MCLR_EN);
//----------------------------------------------------------------------
extern void setup_TMR1(void);
extern void setup_encoder(void);
extern void setup_uart(void);

extern volatile unsigned short int timer_test;
extern volatile unsigned short int cmd_posn;			// current posn cmd from PC
extern volatile short int rxrdy;

extern void setup_pwm(void);
extern void set_pwm(float amps);
extern void setup_adc10(void);
extern void setup_capture(void);
extern int restore_setup( void );
extern int calc_cksum(int sizew, int *adr);
extern void print_tuning( void );

extern void test_pc_interface( void );
extern void test_pid_interface( void );
extern void test_pwm_interface( void );
					
extern struct PID _EEDATA(32) pidEE;
extern volatile unsigned short int do_servo;
extern struct PID pid;
extern void calc_pid( void );
extern void init_pid(void);
extern void	process_serial_buffer();



void __attribute__((__interrupt__)) _StackError (void)
{
	PDC1 = (FCY/FPWM) - 1;	// cutoff output
	while (1)
	{
		STATUS_LED = 1;
		STATUS_LED = 0;			
		STATUS_LED = 0;		
		STATUS_LED = 0;			
	}
}

void __attribute__((__interrupt__)) _AddressError (void)
{
	PDC1 = (FCY/FPWM) - 1;	// cutoff output
	while (1)
	{
		STATUS_LED = 1;
		STATUS_LED = 1;			
		STATUS_LED = 0;			
		STATUS_LED = 0;			
	}
}

void __attribute__((__interrupt__)) _MathError (void)
{
	PDC1 = (FCY/FPWM) - 1;	// cutoff output
	while (1)
	{
		STATUS_LED = 1;
		STATUS_LED = 0;			
		STATUS_LED = 1;			
		STATUS_LED = 0;			
	}
}

/*********************************************************************
  Function:        void set-io(void)

  PreCondition:    None.
 
  Input:           None.

  Output:          None.

  Side Effects:    None.

  Overview:        Sets up io ports for in and out

  Note:            Some periperal configs may change them again
********************************************************************/
void setup_io(void)
{
	// PORT B
    ADPCFG = 0xffff;        // make sure analog doesnt grab encoder pins (all digital)
    ADPCFGbits.PCFG0 = 0;   // AN0 is analog in - spare
							// 0=output, 1=input
	_TRISB0 = 1;			// used as AN0 above
	_TRISB1 = 0;			// spare
	_TRISB2 = 0;			// spare
	_TRISB3 = 0;			// spare
	_TRISB4 = 1;			// used by quad encoder input ch A
	_TRISB5 = 1;			// used by quad encoder input ch B

	// PORT C				// 0=output, 1=input
	_TRISC13 = 0;			// serial port aux  tx data
	_TRISC14 = 1;			// serial port aux  rx data
	_TRISC15 = 1;			// spare (maybe future xtal)

	// PORT D				// 0=output, 1=input
	_TRISD0 = 1;			// quad signal from pc
	_TRISD1 = 1;			// quad signal from pc

	// PORT E				// 0=output, 1=input
	_TRISE0 = 1;			// used by pwm1L to drive servo(set as i/p for startup)
	_TRISE1 = 0;			// used by on board fault led
	_TRISE2 = 0;			// used to set fwd/rev on motor power stage
	_TRISE3 = 0;			// used to indicate when PID calc is active
	_TRISE4 = 0;			// spare 
	_TRISE5 = 0;			// spare 
	_TRISE8 = 1;			// goes low on amp overtemp/fault (also state of ext sw)

	// PORT F				// 0=output, 1=input
	_TRISF2 = 1;			// PGC used by icsp
	_TRISF3 = 1;			// PGD used by icsp
}

/*********************************************************************
  Function:        int main(void)

  PreCondition:    None.
 
  Input:           None.

  Output:          None.

  Side Effects:    None.

  Overview:        main function of the application. Peripherals are 
                   initialized.

  Note:            None.
********************************************************************/
int main(void) 
{
	int cs;
	// vars used for detection of incremental motion
	unsigned short new_cmd,last_cmd, new_fb,last_fb;

	setup_io();			// make all i/o pins go the right dir
	STATUS_LED = 0;		// led on

	setup_uart();		// setup the serial interface to the PC

    setup_TMR1();       // set up 1ms timer
	IEC0bits.T1IE = 1;  // Enable interrupts for timer 1
   						// needed for delays in following routines

	// 1/2 seconds startup delay 
	timer_test = 5000;		
	while ( timer_test );

	printf("\r\nPowerup..i/o...uart...timer...");	

	setup_pwm();		// start analog output
	set_pwm(0.0); 
	printf("pwm...");

	init_pid();
	printf("pid...");

    setup_encoder();    // 16 bit quadrature encoder module setup
	printf("encoder...");

    setup_capture();    // 2 pins with quadrature cmd from PC
	printf("capture...");

	printf("done\r\n");

	// some junk for the serial channel
	printf("%s%s\n\r",CPWRT,VERSION);

	STATUS_LED = 1;		// led off when init finished

	// restore config from eeprom
	// Read array named "setupEE" from DataEEPROM and place 
	// the result into array in RAM named, "setup" 
	restore_setup();
	cs = calc_cksum(sizeof(pid)/sizeof(int),(int*)&pid);
	if ( cs )
	{
		// opps, no valid setup detected
		// assume we are starting from a new box
		printf("No valid setup found in EEPROM\r\n");
		init_pid();
	}
	else
	{
		printf("Using setup from eeprom.. ? for help\r\n");
		print_tuning();
	}
    printf("using %fms servo loop interval\r\n",pid.ticksperservo * 0.1);

//	BLOCK OF TEST ROUTINES FOR HARDWARE DEBUGGING		
//	test_pwm_interface();		// play with opa549 hardware
//	test_pc_interface();		// echo cmded posn to serial port
//  test_pid_interface();		// test pid loop operation

	new_cmd = last_cmd = new_fb = last_fb = 0;

	while (1)
	{
		if ( do_servo )		// check and see if timer 1 has asked for servo calcs to be run
		{
			do_servo = 0;
			if (SVO_ENABLE)
			{
				if ( pid.enable == 0 )	// last loop, servo was off
				{
				    set_pwm( 0.0 );
					printf("servo-enabled\r\n>");
					pid.enable = 1;
					// make sure we dont move on enabling
					cmd_posn = POSCNT;		// make 16bit incr registers match
					pid.command = 0L;		// make 32 bit counter match
					pid.feedback = 0L;
					// make the 1ms loop temps match
					new_cmd = last_cmd = new_fb = last_fb = 0;
					pid.error_i = 0.0;		// reset integrator
				}
                // we can time the servo cycle calcs by scoping the PID_ACTIVE pin
			    PID_ACTIVE = 1;			// seems to take about 140us
			    new_cmd = cmd_posn;		// grab current cmd from pc
			    new_fb = POSCNT;		// grab current posn from encoder

			    pid.command  += (long int)((short)(new_cmd - last_cmd));
			    pid.feedback += (long int)((short)(new_fb  - last_fb ));
			    last_cmd = new_cmd;
			    last_fb = new_fb;

			    calc_pid();

			    // check for a drive fault ( posn error > allowed )
			    if (( pid.maxerror > 0.0 ) && 
				    ( fabs(pid.error) > pid.maxerror ))
			    {
				    short temp = SVO_ENABLE;
				    set_pwm( 0.0 );
				    while (1)	// trap here until svo disabled or pwr cycle
				    {
					    // reset integrator as it may have wound up
					    pid.error_i = 0.0;
					    printf("drive fault... maxerror exceeded\r\n");
					    STATUS_LED = 0;	timer_test = 2500; while ( timer_test );
					    STATUS_LED = 1;	timer_test = 2500; while ( timer_test );
					    STATUS_LED = 0;	timer_test = 2500; while ( timer_test );
					    STATUS_LED = 1;	timer_test = 2500; while ( timer_test );
					    if (temp != SVO_ENABLE) 
						    break;
				    }
			    }
			    else
			    {
				    set_pwm(pid.output);	// update motor drive
			    }
			    PID_ACTIVE = 0;			// i/o pin for timing pid calcs
			}
			else
			{
				if ( pid.enable == 1 )	// last loop servo was active
				{
				    set_pwm( 0.0 );
					pid.enable = 0;
					printf("servo-disabled\r\n>");
					// extra delay keeps us faulted for min 1 sec to let mechanicals settle
					STATUS_LED = 1;	timer_test = 10000; while ( timer_test );
				}
			}
		}

		// look for serial cmds
		// doing this while svo is enabled will cause bumps in servo loop
		// because of serial i/o time ( unless we get smart and move svo loop
		// into an isr )
		if ( rxrdy )
			process_serial_buffer();

		if (pid.limit_state)			// show we are at drive limit(error)
			STATUS_LED = 0;
		else
			STATUS_LED = 1;
	}
	// to keep compiler happy....
	return 0;
}
