//---------------------------------------------------------------------
//	File:		adc10.c
//
//	Written By:	Lawrence Glaister VE7IT
//
// Purpose: routines to setup and use 10 bit A/D ports
//      
// 
//---------------------------------------------------------------------
//
// Revision History
//
// 18 Mar 2006 -- first version
//---------------------------------------------------------------------- 
#include "dspicservo.h"
#include <adc10.h>
extern volatile unsigned short int timer_a2d;

/************************************************************************
* Function Name     : ADC10_read
* Description       : This function reads from ch0 AN0
* Parameters        : 
* Return Value      : unsigned int
*************************************************************************/
unsigned int ADC10_read(void)
{	
//	printf("adc10-read  starting\r\n");
	timer_a2d = 5;				// 2 ticks of 100us clock
	while ( timer_a2d );		// delay for sample time
//	printf("adc10-read after sample delay\r\n");
	ADCON1bits.SAMP = 0;		// start converting
	while (!ADCON1bits.DONE);	// wait for conversion to complete (12fcy)
//	printf("adc10-read after conversion complete\r\n");
	return ADCBUF0;
}

/*********************************************************************
* Function Name     : setup_ADC10
* Description       : Configures the ADC. This includes :
                    · Operating mode      // ADCON1<15> ADON bit
                    · Data o/p format     // ADCON1<9:8> FORM bits
                    · Sample Clk Source   // ADCON1<7:5> SSRC<2:0>bits
                    · Vref source         // ADCON2<15:13> VCFG<2:0> bits
                    . Channels utilized   // ADCON2<9:8> CHPS<1:0>bits
                    · No of samples/int   // ADCON2<4:2> SMPI<2:0>bits
                    · Buffer fill mode    // ADCON2<1> BUFM bit
                    · Alternate i/p sample mode // ADCON2<0> ALTS
                    · Auto sample time   //ADCON3<12:8> SAMC<4:0>bits
                    · Conv clock source  //ADCON3<6> ADRC
                    · Conv clock select bits //ADCON3<5:0> ADCS<5:0>
                    · Port config control bits.

* Parameters        : 
* Return Value      : None
*********************************************************************/
void setup_adc10(void)
{
    /* digital/analog mode selection on the port bits */
    // set by main setup_io()... ADPCFG = xxxx

    /* ADCON1 . shutdown sampling and a/d device for config */
	ADCON1 = 0x0004;	// asma bit=1, sample starts after convertion

	/* select ch 0 for reading RB0/AN0*/
	ADCHS = 0;						//ch0+ is an0, ch0- is aVss	

    /* configures the input scan selection bits */
    ADCSSL = 0;						// no inputs are scannned

    /* config ADCON3 */
    ADCON3 = 0x0008;				//manual sample tad 8 tcy

    /* config ADCON2 */
    ADCON2 = 0;						// Vref+ is AVdd and Vref- is AVss

	ADCON1bits.ADON = 1;			// turn on a/d module
}
