//---------------------------------------------------------------------
//	File:		encoder.c
//
//	Written By:	Lawrence Glaister VE7IT
//
// Purpose: routines to setup and use quadrature encoder
//      
// 
//---------------------------------------------------------------------
//
// Revision History
//
// Nov 5 2005 -- first version 
//---------------------------------------------------------------------- 
#include "dspicservo.h"

//#define ENC_MAX ((4*2000)-1)
//int global_enc_h;

/*********************************************************************
  Function:        void __attribute__((__interrupt__)) _QEIInterrupt(void)

  PreCondition:    None.
 
  Input:           None

  Output:          None.

  Side Effects:    None.

  Overview:        handles encoder interrupts 

  Note:            We can get intrs via index pulses, counting errors
********************************************************************/
void __attribute__((__interrupt__)) _QEIInterrupt(void)
{
    if (QEICONbits.CNTERR)
    {
        /* encoder rolled over */
//        if ( QEICONbits.UPDN )
//            global_enc_h++;
//        else
//            global_enc_h--;
        QEICONbits.CNTERR = 0;      // reset count error flag
    }

    IFS2bits.QEIIF = 0;         // reset the if flag

//    if ( STATUS_LED ) STATUS_LED = 0; else STATUS_LED = 1; // toggle led
}

/*********************************************************************
  Function:        void setupEncoder(void)

  PreCondition:    None.
 
  Input:           None

  Output:          None.

  Side Effects:    some analog I/O config bits are played with

  Overview:        handles encoder interrupts 

  Note:            We can get intrs via index pulses, counting errors
                   (counter rollovers). Setup is tricky.... we need
                    an intr on index pulse so that we can do commutation.
                    We also need an intr when the 16 bit internal 
                    counter rolls over so that we can keep track of multi
                    word position data. We setup counter for reset
                    on index, but dont enable the reset part (this still
                    generates an intr). By setting maxcnt = 0xffff, and
                    enabling intr on CNTERR, we also get an intr when the
                    counter rolls over (even though this is not really an
                    error).
********************************************************************/

void setup_encoder(void)
{
    ADPCFG = 0xffff;        // make sure analog doesnt grab encoder pins (all digital)
    ADPCFGbits.PCFG0 = 0;   // AN0 is analog (temp)
 
    MAXCNT = 0xffff;        // counts/rev (used as preset when index pulse seen)
    POSCNT = 0x0000;

    QEICON = 0;             // clr CNTERR bit (among others)
    QEICONbits.QEIM = 6;    // x4 reset by index pulse
//  QEICONbits.POSRES = 0;  // TRICK! dont allow index to reset counter
//  QEICONbits.SWPAB = 0;   // dont need to swap a/b phases


    // digital filter requires the input to be valid for 3 (scaled) clk pulses
    // note: default bit states show as comments in case you want to change modes
    DFLTCON = 0;            // digital filter set off
    DFLTCONbits.IMV = 3;    // in x4 mode, a and b and i have to be high for reset 
//  DFLTCONbits.QECK = 0;   // 1:1 clk
    DFLTCONbits.QEOUT = 1;  // enable digital filter on phase a,b,i
//  DFLTCONbits.CEID = 0;   // enable intr on count errors

    /* set up interrupts for encoder */
    IFS2bits.QEIIF = 0;         // clear Interrupt flag 
    IPC10bits.QEIIP = 1;        // bits <2:0> are the priority
    IEC2bits.QEIIE = 1;         // go live
}
