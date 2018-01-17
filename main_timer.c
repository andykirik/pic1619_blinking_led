/*
 * File:   main.c
 * Author: akirik
 *
 * Created on January 9, 2018, 3:52 PM
 * 
 * Blinking LED
 * 
 *  Board connection (Microchip Curiosity Board 8-bit, PIC16F1619):
 *   PIN                	Module                         				  
 * -------------------------------------------                        
 *    RA5 (D4)             LED
 *    RA1 (D5)             LED
 *    RA2 (D6)             LED
 *    RC5 (D7)             LED
 *
 *    RC4 (S1)             BUTTON
 *
 */

/* The __delay_ms() function is provided by XC8. 
It requires you define _XTAL_FREQ as the frequency of your system clock. 
We are using the internal oscillator at its default 500 kHz, so _XTAL_FREQ is defined as 4000000. 
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

#define S1_PORT                   PORTCbits.RC4

volatile uint16_t timer1ReloadVal;

// INIT
void sys_init()
{
    // LATx registers (used instead of PORTx registers to write (read could be done on PORTx))
    LATA = 0x00;        // Set PORTA all 0
    LATB = 0x00;        // Set PORTB all 0
    LATC = 0x00;        // Set PORTC all 0

    // TRISx registers (This register specifies the data direction of each pin)
    TRISA 	= 0x00;     // Set All on PORTA as Output
    TRISB 	= 0x00;     // Set All on PORTB as Output
    //TRISC 	= 0x00;     // Set All on PORTC as Output
    TRISC 	= 0b11011111; // All on PORTC as Output, RC4 as Input

    /*/ ANSELx registers
    ANSELA = 0x17; 0b00010111
    ANSELB = 0xF0; 0b11110000
    ANSELC = 0xCF; 0b11001111
    */

    /*/ WPUx registers
    WPUB = 0xF0; 0b11110000
    WPUA = 0x3F; 0b00111111
    WPUC = 0xFF; 0b11111111
    OPTION_REGbits.nWPUEN = 0;
    */

    // ODx registers
    ODCONA = 0x00;
    ODCONB = 0x00;
    ODCONC = 0x00;
    
    LATAbits.LATA5 = 0;
    LATAbits.LATA1 = 0;
    LATAbits.LATA2 = 0;
    LATCbits.LATC5 = 0;

//Set the Timer to the options selected in the GUI

    //T1CKPS 1:1; nT1SYNC synchronize; TMR1CS FOSC/4; TMR1ON off; 
    T1CON = 0x00;

    //T1GSS T1G_pin; TMR1GE disabled; T1GTM disabled; T1GPOL low; T1GGO_nDONE done; T1GSPM disabled; 
    T1GCON = 0x00;

    //TMR1H 11; 
    TMR1H = 0x0B;

    //TMR1L 220; 
    TMR1L = 0xDC;

    // Load the TMR value to reload variable
    timer1ReloadVal=(uint16_t)((TMR1H << 8) | TMR1L);

    // Clearing IF flag.
    PIR1bits.TMR1IF = 0;    
}

void main(void) 
{
    sys_init();
    
    // Start the Timer by writing to TMRxON bit
    T1CONbits.TMR1ON = 1;
    
    while(1)  
	{
        //Wait for Timer1 to overflow
        while (!TMR1IF);
        
        LATAbits.LATA5 = ~LATAbits.LATA5;   // flip 
        LATAbits.LATA1 = ~LATAbits.LATA1;   // flip
        LATAbits.LATA2 = ~LATAbits.LATA2;   // flip
        LATCbits.LATC5 = ~LATCbits.LATC5;   // flip
        
        //Write to the Timer1 register
        TMR1H = (timer1ReloadVal >> 8);
        TMR1L = timer1ReloadVal;
        
        //Clear TMR1 Overflow flag
        TMR1IF = 0;
    }   

    // Stop the Timer by writing to TMRxON bit
    T1CONbits.TMR1ON = 0;
    
    return;
}
