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
 *    RC0 (POT1)           POTENCIOMETER
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
#include <stdint.h>
#include <stdbool.h>


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

#define POT1                      0x4
#define ACQ_US_DELAY              5

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
    
    // set the ADC to the options selected in the User Interface
    
    // GO_nDONE stop; ADON enabled; CHS AN0; 
    ADCON0 = 0x01;
    
    // ADFM left; ADPREF VDD; ADCS FOSC/2; 
    ADCON1 = 0x00;
    
    // TRIGSEL no_auto_trigger; 
    ADCON2 = 0x00;
    
    // ADRESL 0; 
    ADRESL = 0x00;
    
    // ADRESH 0; 
    ADRESH = 0x00;
    
    // Set the PWM to the options selected in the PIC10 / PIC12 / PIC16 / PIC18 MCUs 

    // MODE PWM; EN enabled; FMT right_aligned; 
    CCP1CON = 0x8F;    

    // RH 3; 
    CCPR1H = 0x03;    

    // RL 50; 
    CCPR1L = 0x32;    

    // Selecting Timer 2
    CCPTMRSbits.CCP1TSEL = 0x0;
    
    //Set RC5 (LED_D7) as output of CCP1 using PPS
    RC5PPS = 0x0C;
    
    // Set TMR2 to the options selected in the User Interface

    // T2CKPS 1:1; T2OUTPS 1:1; TMR2ON off; 
    T2CON = 0x00;

    // T2CS FOSC/4; 
    T2CLKCON = 0x00;

    // T2PSYNC Not Synchronized; T2MODE Software control; T2CKPOL Rising Edge; T2CKSYNC Not Synchronized; 
    T2HLT = 0x00;

    // T2RSEL C1_OUT_SYNC; 
    T2RST = 0x01;

    // T2PR 255; 
    T2PR = 0xFF;

    // TMR2 0; 
    T2TMR = 0x00;

    // Clearing IF flag.
    PIR1bits.TMR2IF = 0;

}

uint16_t ADC_GetConversion(int channel)
{
    // select the A/D channel
    ADCON0bits.CHS = channel;    

    // Turn on the ADC module
    ADCON0bits.ADON = 1;
    // Acquisition time delay
    __delay_us(ACQ_US_DELAY);

    // Start the conversion
    ADCON0bits.GO_nDONE = 1;

    // Wait for the conversion to finish
    while (ADCON0bits.GO_nDONE);

    // Conversion finished, return the result
    return ((uint16_t)((ADRESH << 8) + ADRESL));
}

void main(void) 
{
    sys_init();
    
    // Start the Timer by writing to TMRxON bit
    T2CONbits.TMR2ON = 1;
    
    uint8_t adcResult;
    while(1)  
	{
        //Start ADC conversion
        adcResult = ADC_GetConversion(POT1) >> 6;

        adcResult &= 0x03FF;
    
        // Load duty cycle value
        if(CCP1CONbits.FMT)
        {
            adcResult <<= 6;
            CCPR1H = adcResult >> 8;
            CCPR1L = adcResult;
        }
        else
        {
            CCPR1H = adcResult >> 8;
            CCPR1L = adcResult;
        }
    }
    
    // Stop the Timer by writing to TMRxON bit
    T2CONbits.TMR2ON = 0;
            
    return;
}
