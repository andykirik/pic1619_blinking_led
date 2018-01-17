/*
 * File:   main_timer.c
 * Author: akirik
 *
 * Created on January 9, 2018, 3:52 PM
 * 
 * Using timers
 * 
 *  Board connection (Microchip Curiosity Board 8-bit, PIC16F1619):
 *   PIN                	Module                         				  
 * -------------------------------------------                        
 *    RA5 (D4)             LED
 *    RA1 (D5)             LED
 *    RA2 (D6)             LED
 *    RC5 (D7)             LED
 *
 */

/* The __delay_ms() function is provided by XC8. 
It requires you define _XTAL_FREQ as the frequency of your system clock. 
We are using the internal oscillator at its default 500 kHz, so _XTAL_FREQ is defined as 500000. 
The compiler then uses that value to calculate how many cycles are required to give the requested delay. 
There is also __delay_us() for microseconds and _delay() to delay for a specific number of clock cycles. 
Note that __delay_ms() and __delay_us() begin with a double underscore whereas _delay() 
begins with a single underscore.
*/
#define _XTAL_FREQ 500000

#include <xc.h>
#include <stdbool.h>
#include <stdint.h>


// CONFIG1
#pragma config FOSC     = INTOSC    // Oscillator Selection Bits->INTOSC oscillator: I/O function on CLKIN pin
#pragma config PWRTE    = OFF       // Power-up Timer Enable->PWRT disabled
#pragma config MCLRE    = ON        // MCLR Pin Function Select->MCLR/VPP pin function is MCLR
#pragma config CP       = OFF       // Flash Program Memory Code Protection->Program memory code protection is disabled
#pragma config BOREN    = ON        // Brown-out Reset Enable->Brown-out Reset enabled
#pragma config CLKOUTEN = OFF       // Clock Out Enable->CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin
#pragma config IESO     = ON        // Internal/External Switch Over->Internal External Switch Over mode is enabled
#pragma config FCMEN    = ON        // Fail-Safe Clock Monitor Enable->Fail-Safe Clock Monitor is enabled

// CONFIG2
#pragma config WRT      = OFF       // Flash Memory Self-Write Protection->Write protection off
#pragma config PPS1WAY  = ON        // Peripheral Pin Select one-way control->The PPSLOCK bit cannot be cleared once it is set by software
#pragma config ZCD      = OFF       // Zero Cross Detect Disable Bit->ZCD disable.  ZCD can be enabled by setting the ZCDSEN bit of ZCDCON
#pragma config PLLEN    = OFF       // PLL Enable Bit->4x PLL is enabled when software sets the SPLLEN bit
#pragma config STVREN   = ON        // Stack Overflow/Underflow Reset Enable->Stack Overflow or Underflow will cause a Reset
#pragma config BORV     = LO        // Brown-out Reset Voltage Selection->Brown-out Reset Voltage (Vbor), low trip point selected.
#pragma config LPBOR    = OFF       // Low-Power Brown Out Reset->Low-Power BOR is disabled
#pragma config LVP      = ON        // Low-Voltage Programming Enable->Low-voltage programming enabled

// CONFIG3
#pragma config WDTCPS   = WDTCPS1F  // WDT Period Select->Software Control (WDTPS)
#pragma config WDTE     = OFF       // Watchdog Timer Enable->WDT disabled
#pragma config WDTCWS   = WDTCWSSW  // WDT Window Select->Software WDT window size control (WDTWS bits)
#pragma config WDTCCS   = SWC       // WDT Input Clock Selector->Software control, controlled by WDTCS bits

#define LED_D4_LAT                LATAbits.LATA5
#define LED_D5_LAT                LATAbits.LATA1
#define LED_D6_LAT                LATAbits.LATA2
#define LED_D7_LAT                LATCbits.LATC5

volatile uint16_t timer1ReloadVal = 0;

// INIT
void system_init()
{
	// I/O
		// ANSELx registers
			ANSELA = 0x00;
			ANSELB = 0x00;
			ANSELC = 0x00;

		// TRISx registers (This register specifies the data direction of each pin)
			TRISA 	= 0x00;     // Set All on PORTA as Output
			TRISB 	= 0x00;     // Set All on PORTB as Output
			TRISC 	= 0x00; 	// Set All on PORTC as Output

		// LATx registers (used instead of PORTx registers to write (read could be done on PORTx))
			LATA = 0x00;        // Set PORTA all 0
			LATB = 0x00;        // Set PORTB all 0
			LATC = 0x00;        // Set PORTC all 0

    // WPUx registers (pull up resistors)
        WPUA = 0x00; 
        WPUB = 0x00; 
        WPUC = 0xFF; 
        OPTION_REGbits.nWPUEN = 0;

    // ODx registers
        ODCONA = 0x00;
        ODCONB = 0x00;
        ODCONC = 0x00;
    

	// Timer Setup
		T1CON = 0x00;					// T1CKPS 1:1; nT1SYNC synchronize; TMR1CS FOSC/4; TMR1ON off; 
		T1GCON = 0x00;					// T1GSS T1G_pin; TMR1GE disabled; T1GTM disabled; T1GPOL low; T1GGO_nDONE done; T1GSPM disabled; 
		TMR1H = 0x0B;					// TMR1H 11; 
		TMR1L = 0xDC;					// TMR1L 220; 
		timer1ReloadVal=(uint16_t)((TMR1H << 8) | TMR1L);// Load the TMR value to reload variable
		PIR1bits.TMR1IF = 0;        	// Clearing IF flag.
}

void main(void) 
{
    system_init();
    
    T1CONbits.TMR1ON = 1;		// Start the Timer by writing to TMRxON bit
    
    while(1)  
	{
        while (!TMR1IF); 					//Wait for Timer1 to overflow
        
        LATAbits.LATA5 = ~LATAbits.LATA5;   // Toggle the LED 
        LATAbits.LATA1 = ~LATAbits.LATA1;   // Toggle the LED
        LATAbits.LATA2 = ~LATAbits.LATA2;   // Toggle the LED
        LATCbits.LATC5 = ~LATCbits.LATC5;   // Toggle the LED        
        
        TMR1H = (timer1ReloadVal >> 8);		// Write to the Timer1 register
        TMR1L = timer1ReloadVal;
        
        
        TMR1IF = 0;							// Clear TMR1 Overflow flag
    }   

    T1CONbits.TMR1ON = 0; // Stop the Timer by writing to TMRxON bit (no need to do it, just to show that timer could be turned off)
    
    return;
}
