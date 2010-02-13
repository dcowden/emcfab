#include "dspicservo.h"
#include <stdio.h>
#include <uart.h>

// Select the desired UART baud rate here
#define THE_BAUD_RATE 9600
//#define THE_BAUD_RATE 19200
//#define THE_BAUD_RATE 57600
//#define THE_BAUD_RATE 115200 

char rxbuff[30];			// global rx buffer for serial data
char *rxbuffptr;		// local input ptr for storing data
short int rxrdy;			// flag to indicate a line of data is available in buffer

#if 0
//**************************************************************************
//* USART Test Code Here
//**************************************************************************

printf(" Hello World \r\n");

for (i=0;i<10;i++)
{
putchar('X');
putchar('Y');
putchar('Z');
printf(" This is a test %d times \r\n",i);
} 
//    char *temp_ptr = (char *) buffer;
//     while(!DataRdyUART1())
//    *temp_ptr++ = U1RXREG & 0xFF; /* data byte from HW buffer to SW buffer */

#endif

/*********************************************************************
  Function:        void __attribute__((__interrupt__)) _U1RXInterrupt (void)

  Input:           None.

  Output:          None.

  Side Effects:    None.

  Overview:        Handles uart rx intr

********************************************************************/

void __attribute__((__interrupt__)) _U1RXInterrupt (void)
{
	char ch;

    IFS0bits.U1RXIF = 0;

	while (U1STAbits.URXDA)
	{
		ch = U1RXREG & 0xFF;
		// save the character if there is room in the input buffer
		if ( ch == 0x0a )
			continue;			// strip LF

		if ( ch == 0x0d )
		{
			// end of input stream.. 
			// let user know if we have something to process
			rxrdy = 1;
			break;
		}

		// if we have room in the buffer, store the ch for later processing
		if (rxbuffptr < (&rxbuff[0] + sizeof(rxbuff) - 1 ))
		{
			// still working on filling buffer
			*rxbuffptr++ = ch;
			*rxbuffptr = 0;			// null terminate buffer
			putchar(ch);	
		}
	}
}


//**************************************************************************
//* Configure the USART
// the default stdin,stdout are directed to serial port 1
//**************************************************************************
void setup_uart( void )
{
	unsigned int uartmode;
	uartmode = UART_EN & UART_IDLE_CON & UART_DIS_WAKE &
               UART_DIS_LOOPBACK & UART_DIS_ABAUD &
               UART_NO_PAR_8BIT & UART_1STOPBIT &
               UART_ALTRX_ALTTX;	// use alternate i/o pins


    U1BRG  = (((FCY/THE_BAUD_RATE) /16) - 1);     /* baud rate */
    U1MODE = uartmode;  /* operation settings */
    U1STA = UART_TX_ENABLE & UART_TX_PIN_NORMAL;   /* TX & RX interrupt modes */
	U1STAbits.URXISEL = 0;						/* rx intr every ch */

	rxbuffptr = &rxbuff[0];
	rxbuff[0] = 0;
	rxrdy = 0;


	IFS0bits.U1RXIF = 0;
	IEC0bits.U1RXIE = 1;		// go live with serial rx intr
} 

