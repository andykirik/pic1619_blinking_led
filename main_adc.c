/*
 * File:   main_adc.c
 * Author: akirik
 *
 * Created on January 9, 2018, 3:52 PM
 * 
 * Analog to Digital Converter (ADC)
 * 
 *  Board connection (Microchip Curiosity Board 8-bit, PIC16F1619):
 *   PIN                	Module                         				  
 * -------------------------------------------                        
 *    RA5 (D4)             LED
 *    RA1 (D5)             LED
 *    RA2 (D6)             LED
 *    RC5 (D7)             LED
 * 
 *    RC0/AN4 (POT1)           POTENCIOMETER
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

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

#define LED_D4_LAT                LATAbits.LATA5
#define LED_D5_LAT                LATAbits.LATA1
#define LED_D6_LAT                LATAbits.LATA2
#define LED_D7_LAT                LATCbits.LATC5

#define POT1                      0b00100
#define ACQ_US_DELAY              5

// INIT
void system_init()
{
	// I/O
		// ANSELx registers
			ANSELA = 0x00;
			ANSELB = 0x00;
			ANSELC = 0x00;
            ANSELCbits.ANSC0 = 1;   // Set RC0/AN4 to analog mode

		// TRISx registers (This register specifies the data direction of each pin)
			TRISA 	= 0x00;     // Set All on PORTA as Output
			TRISB 	= 0x00;     // Set All on PORTB as Output
			TRISC 	= 0x00; 	// Set All on PORTC as Output
            TRISCbits.TRISC0 = 1; // Set RC0/AN4 as Input

		// LATx registers (used instead of PORTx registers to write (read could be done on PORTx))
			LATA = 0x00;        // Set PORTA all 0
			LATB = 0x00;        // Set PORTB all 0
			LATC = 0x00;        // Set PORTC all 0

    // WPUx registers (pull up resistors)
        WPUA = 0x00; 
        WPUB = 0x00; 
        WPUC = 0x00; 
        OPTION_REGbits.nWPUEN = 0;

    // ODx registers
		ODCONA = 0x00;
		ODCONB = 0x00;
		ODCONC = 0x00;
    
  
    // ADC setup
	// The PIC16F690 has a 10-bit ADC
	// It provides value 0 to 1023 as the input voltage rises from Vss to Vref
	// Vref is the analog reference voltage. 
	// It can either be Vdd or supplied externally to the RA1/AN1/Vref pin
	// The PIC16F1619 has twelve analog input pins (AN0 through AN11). 
	// In order to enable the analog function for a pin, 
	// the corresponding bit must be set in the ANSEL or ANSELH register (see above).
    // Steps to configure:
    //  1. voltage reference in the ADCON1 register
    //  2. select ADC conversion clock in the ADCON0 register
    //  3. select one input channel CH0CH13 in the ADCON0 register
    //  4. select data format in the ADCON1 register (ADFM)
    //  5. enable AD converter in the ADCON0 register (ADON)
    //  optional configure interrupt:
    //  6. clear ADIF bit
    //  7. set ADIE, PEIE and GIE bits (see interrupts for more instructions)
    /* 
     * -------------------ADCON0------------------------------------
     * Bit#:  ----7----6----5---4-----3----2----1-------0-----------
     * :      --|---|CHS4|CHS3|CHS2|CHS1|CHS0|GO/DONE|ADON|---------
     * -------------------------------------------------------------
        CHS<4:0> - channel select: This selects which voltage is fed into the ADC. 
            Setting this 0 to 11 selects AN0 to AN11 respectively. 
            Setting 12 feeds CVref (Comparator voltage reference). 
            Setting 13 connects the 0.6V fixed voltage reference from the
            Comparator module to the ADC.

        GO/DONE: 1 to start ADC conversion. 
            Be sure to wait at least 5us before doing this after changing 
            the input channel or performing a previous conversion.
            This bit is read to see if the current conversion is complete.
            The ADC will automatically reset this bit to 0 when conversion finishes.

        ADON - ADC enable bit: 0 - ADC disabled and consumes no current, 1 - ADC enabled.
      
     * -------------------ADCON1------------------------------------
     * Bit#:  ----7-----6-----5-----4-----3----2----1-------0-------
     * :      --|ADFM|ADCS2|ADCS1|ADCS0|----|----|ADPREF1|ADPREF0|--
     * -------------------------------------------------------------
        ADFM - result format: 0 - left justified (default), 1 - right justified:
            * -------------------------ADRESH------------------------|-------------------ADRESL-----------------------
            * Bit#:  ---7-----6-----5-----4-----3-----2-----1-----0--|--7-----6-----5-----4-----3-----2-----1-----0---
            * bit:   |  0  |  0  |  0  |  0  |  0  |  0  |bit-9|bit-8|bit-7|bit-6|bit-5|bit-4|bit-3|bit-2|bit-1|bit-0|
            * --------------------------------------------------------------------------------------------------------
            result = (ADRESH<<8)+ADRESL;

            * -------------------------ADRESH------------------------|-------------------ADRESL-----------------------
            * Bit#:  ---7-----6-----5-----4-----3-----2-----1-----0--|--7-----6-----5-----4-----3-----2-----1-----0---
            * bit:   |bit-9|bit-8|bit-7|bit-6|bit-5|bit-4|bit-3|bit-2|bit-1|bit-0|  0  |  0  |  0  |  0  |  0  |  0  |
            * --------------------------------------------------------------------------------------------------------
            This is useful if only the 8 most significant bits are required. 
            In this case, we just simply copy the contens of the ADRESH register.
      
        ADCS<2:0> - ADC Conversion Clock Select bits
      
        ADPREF<1:0>: ADC Positive Voltage Reference Configuration bits
     
     * -------------------ADCON2--------------------------------------------
     * Bit#:  -----7--------6--------5--------4--------3-------2---1---0----
     * :      --|TRIGSEL4|TRIGSEL3|TRIGSEL2|TRIGSEL1|TRIGSEL0|---|---|---|--
     * ---------------------------------------------------------------------
        TRIGSEL<4:0>: Auto-Conversion Trigger Selection bits
     
        * -------------------PIE1---------------------------------------
        * Bit#:  -------7-----6----5---4-----3-----2------1------0------
        * :      --|TMR1GIE|ADIE|RCIE|TXIE|SSPIE|CCP1IE|TMR2IE|TMR1IE|--
        * --------------------------------------------------------------
           ADIE - ADC interrupt enable. 
               Set to 1 to generate interrupts when ADC is completed
        
        * -------------------PIR1---------------------------------------
        * Bit#:  -------7-----6---5----4----3------2------1------0------
        * :      --|TMR1GIF|ADIF|RCIF|TXIF|SSPIF|CCP1IF|TMR2IF|TMR1IF|--
        * --------------------------------------------------------------
           ADIF - ADC interrupt flag. 
               This will be set to 1 by the ADC when a conversion completes. 
               This must be cleared by software in the interrupt service routine 
               to prepare the interrupt flag for the next conversion.
    */
		ADCON0 = 0x00;          // GO_nDONE stop; ADON off; CHS AN0; 
        ADCON1 = 0x00;          // ADFM left; ADPREF VDD; ADCS FOSC/2; 
		ADCON2 = 0x00;          // TRIGSEL no_auto_trigger; 
        ADRESL = 0x00;          // ADRESL 0; 
        ADRESH = 0x00;          // ADRESH 0; 
        ADCON1bits.ADFM = 1;    // right justified
        ADCON0bits.CHS = POT1;  // Select the A/D channel
        ADCON0bits.ADON = 1;    // Turn on the ADC module

}

uint16_t ADC_GetConversion()
{    
    ADCON0bits.ADON = 1;    // Turn on the ADC module
    __delay_us(ACQ_US_DELAY);					// Acquisition time delay
    ADCON0bits.GO_nDONE = 1;					// Start the conversion
    while (ADCON0bits.GO_nDONE);				// Wait for the conversion to finish
    ADCON0bits.ADON = 0;    // Turn off the ADC module
    return ((uint16_t)((ADRESH << 8) + ADRESL));// Conversion finished, return the result
}

void main(void) 
{
    system_init();
    
    while(1)  
	{
        uint8_t adcResult = ADC_GetConversion() >> 6; // Get the top 4 MSBs and display it on the LEDs

        //Determine which LEDs will light up
        LED_D4_LAT = adcResult & 1;
        LED_D5_LAT = (adcResult & 2) >> 1;
        LED_D6_LAT = (adcResult & 4) >> 2;
        LED_D7_LAT = (adcResult & 8) >> 3;
    }
            
    return;
}
