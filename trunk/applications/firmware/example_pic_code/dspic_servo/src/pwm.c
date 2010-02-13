//---------------------------------------------------------------------
//	File:		pwm.c
//
//	Written By:	Lawrence Glaister
//
// Purpose: This set of routines to run pwn output
//      
// 
//---------------------------------------------------------------------
//
// Revision History
//
// Nov 21 2005 --    first version Lawrence Glaister
// Mar 18 2006 --    stripped to single output for servo card
//---------------------------------------------------------------------- 
#include "dspicservo.h"
#include <pwm.h>
#include <stdio.h>


/*********************************************************************
  Function:        void __attribute__((__interrupt__)) _PWMInterrupt(void)

  PreCondition:    None.
 
  Input:           None

  Output:          None.

  Side Effects:    None.

  Overview:        handles pwm interrupts 

  Note:            None.
********************************************************************/
void __attribute__((__interrupt__)) _PWMInterrupt(void)
{
    IFS2bits.PWMIF =0;
}


/*********************************************************************
  Function:        void setupPWM(void)

  PreCondition:    None.
 
  Input:           None

  Output:          None.

  Side Effects:    None.

  Overview:        port setup with deadtime, 
                   fault input disables pwm outputs on a per cycle basis

  Note:            None.
********************************************************************/
void setup_pwm(void)
{
    /* Holds the PWM interrupt configuration value*/
    unsigned int config;
    /* Configure pwm interrupt enable/disable and set interrupt priorties */
    config = (PWM_INT_DIS & PWM_FLTA_DIS_INT & PWM_INT_PR1 & PWM_FLTA_INT_PR0);

    /* clear the Interrupt flags */
    IFS2bits.PWMIF = 0;	
    IFS2bits.FLTAIF = 0;	

    /* Set priority for the period match */
    IPC9bits.PWMIP      = (0x0007 & config);

    /* Set priority for the Fault A */
    IPC10bits.FLTAIP    = (0x0070 & config)>> 4;

    /* enable /disable of interrupt Period match */
    IEC2bits.PWMIE      = (0x0008 & config) >> 3;

    /* enable /disable of interrupt Fault A.*/
    IEC2bits.FLTAIE     = (0x0080 & config) >> 7;


    /* Configure PWM to generate 0 current*/
    PWMCON2bits.UDIS = 0;
    PDC1 = (FCY/FPWM - 1);

    PTPER = (FCY/FPWM/2 - 1);      // set the pwm period register(/2 for cnt up/dwn)
    SEVTCMP = 0x00;
    /* 1 output is independant and enabled */
    PWMCON1 = (PWM_MOD1_IND & PWM_MOD2_IND & PWM_MOD3_IND &  /* independant i/o */
				PWM_PEN1L  & PWM_PDIS1H &                    /* use 1L as pwm, */
				PWM_PDIS2L & PWM_PDIS2H &					 /* rest as I/O */
				PWM_PDIS3L & PWM_PDIS3H 
				);
    /* set dead time options, scale = 1, 10*FCY (about 625ns) */
    DTCON1 = PWM_DTAPS1 & PWM_DTA10;
  
    /* set up the fault mode override bits and mode */
    FLTACON = PWM_FLTA_MODE_CYCLE &
              PWM_FLTA1_DIS &
              PWM_FLTA2_DIS &
              PWM_FLTA3_DIS &
              PWM_OVA1L_INACTIVE &
              PWM_OVA1H_INACTIVE &
              PWM_OVA2L_INACTIVE &
              PWM_OVA2H_INACTIVE &
              PWM_OVA3L_INACTIVE &
              PWM_OVA3H_INACTIVE ;

    /* set special event post scaler, output override sync select and pwm update enable */
    PWMCON2 = (PWM_SEVOPS1 & PWM_OSYNC_PWM & PWM_UEN);
    PTCON   = (PWM_EN & PWM_IDLE_CON & PWM_OP_SCALE1 & PWM_IPCLK_SCALE1 & PWM_MOD_UPDN);

}



/*********************************************************************
  Function:        void set_pwm(float amps)

  PreCondition:    None.
 
  Input:           current value in amps
				   (calculation specific to dspic-servo with 3.3k resistor
				   on current limit pin of the OPA549)

  Output:          None.

  Side Effects:    None.

  Overview:        This routine accepts an an amplitude value to
				   of -6.949 to + 6.949 amps. From this value,
                   the pwm channel is set as required.

  Note:            None.
********************************************************************/
void set_pwm(float amps)
{
	// driving the opa549, 4.75 volts is 0 current and 0 volts is 6.949 (by r12)
	// to tune this number... enable the call to pwmtest routine in main.c
    // put a low value resistor in place of the motor ( ie 1ohm 50w )
    // look at triangle waveform on scope, paying attention to the 0 crossover point
    // if you get a horizontal glitch in the waveform, slightly decrease the numerator
    // if you get a vertical glitch, slightly increase the numerator
    // make sure the fudge factor doesnt go over 1.0
    const long pwm_max = (FCY/FPWM)*(4.85/5.0) - 1;      // 95% pwm count ( gives 4.75v output )
    short temp_dir;
    static short last_dir = 0;
    long temp;

	// the direction of the current is controlled by a pic output pin
	if ( amps >= 0.0 )
        temp_dir = 1;
	else
	{
        temp_dir = 0;
		amps = -amps;		// flip minus values to an amplitude
	}
	// this limit is set by R12 on the dspic servo board
	if ( amps > 6.949 ) amps = 6.949;
	
	temp = (long)(((float)pwm_max * amps)/6.949);

    /* compute value for pwm register */
    temp = pwm_max - temp;


    /* limit inputs to expected ranges */ 
    if ( temp > pwm_max ) temp = pwm_max;
    if ( temp < 1 ) temp = 1;

//	printf("pwm amps cmd= %f maxcnt= %ld, writing %ld\r\n",
//			(double)amps,pwm_max,temp);

	// test code to see if we can reduce spiking at 0 drive position on direction changes
    if ( temp_dir != last_dir )
    {
        temp = pwm_max;		// force output to 0 for 1 cycle on dir change
    }
	last_dir = temp_dir;

    PDC1 = temp;
    SVO_DIR = temp_dir;
}

