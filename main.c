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
 *    RA5 (D7)             LED
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

// INIT
void system_init()
{
	// To control Digital I/O use three registers: ANSEL, TRIS and LAT:
	
    // ANSELx registers
	// ANSEL and ANSELH control the mode of AN0 through AN11:
	// 0 sets the pin to digital mode and 1 sets the pin to analog mode.
    /* 
	 * -------------------ANSELA-------------------------------
     * Bit#:  ---7---6---5----4----3----2-----1-----0----------
     * ANS:   -|---|---|---|ANSA4|---|ANSA2|ANSA1|ANSA0|-------
     * --------------------------------------------------------
     * PORTA3 is Input only
     * 
	 * -------------------ANSELB-------------------------------
     * Bit#:  ----7---6----5-----4----3---2---1----0-----------
     * ANS:   --|---|---|ANSB5|ANSB4|---|---|---|---|----------
     * --------------------------------------------------------
     * -------------------ANSELC-------------------------------
     * Bit#:  ----7------6----5---4----3-----2----1-----0------
     * ANS:   --|ANSC7|ANSC6|---|---|ANSC3|ANSC2|ANSC1|ANSC0|--
     * --------------------------------------------------------
     */
        ANSELA = 0x00;
        ANSELB = 0x00;
        ANSELC = 0x00;

    // TRISx registers
	// This register specifies the data direction of each pin:
	// O - output, 1 - input
    /* 
	 * -------------------TRISA---------------------------------------------
     * Bit#:   ---7-------6------5------4------3------2------1------0-------
     * TRIS:   -|------|------|TRISA5|TRISA4|------|TRISA2|TRISA1|TRISA0|---
     * ---------------------------------------------------------------------
     * PORTA3 is Input only
     * 
	 * -------------------TRISB---------------------------------------------
     * Bit#:   ----7------6------5------4------3------2------1------0-------
     * TRIS:   -|TRISB7|TRISB6|TRISB5|TRISB4|------|------|------|------|---
     * ---------------------------------------------------------------------
     * -------------------TRISC---------------------------------------------
     * Bit#:   ----7------6------5------4------3------2------1------0-------
     * TRIS:   --|TRISC7|TRISC6|TRISC5|TRISC4|TRISC3|TRISC2|TRISC1|TRISC0|--
     * ---------------------------------------------------------------------
     */
        TRISA 	= 0x00;     // Set All on PORTA as Output
        TRISB 	= 0x00;     // Set All on PORTB as Output
        TRISC 	= 0x00;     // Set All on PORTC as Output

    // LATx registers 
    // output latch - used instead of PORTx registers to write (read could be done on PORTx)
    /* 
	 * -------------------LATA--------------------------------------
     * Bit#:   ---7-----6-----5-----4-----3-----2-----1-----0-------
     * LAT:   -|-----|-----|LATA5|LATA4|LATA3|LATA2|LATA1|LATA0|----
     * -------------------------------------------------------------
	 * -------------------LATB--------------------------------------
     * Bit#:   ----7----6----5-----4------3------2------1------0----
     * LAT:   -|LATB7|LATB6|LATB5|LATB4|------|------|------|------|
     * -------------------------------------------------------------
     * -------------------LATC--------------------------------------
     * Bit#:   ----7-----6-----5-----4-----3-----2-----1-----0------
     * LAT:   --|LATC7|LATC6|LATC5|LATC4|LATC3|LATC2|LATC1|LATC0|---
     * -------------------------------------------------------------
     */
        LATA = 0x00;        // Set PORTA all 0
        LATB = 0x00;        // Set PORTB all 0
        LATC = 0x00;        // Set PORTC all 0

    // WPUx registers (pull up resistors)
    /* 
	 * -------------------WPUA--------------------------------------
     * Bit#:   ---7-----6-----5-----4-----3-----2-----1-----0-------
     * WPU:   -|-----|-----|WPUA5|WPUA4|WPUA3|WPUA2|WPUA1|WPUA0|----
     * -------------------------------------------------------------
	 * -------------------WPUB--------------------------------------
     * Bit#:  ----7----6----5-----4------3------2-------1------0----
     * WPU:   -|WPUB7|WPUB6|WPUB5|WPUB4|------|------|------|------|
     * -------------------------------------------------------------
     * -------------------WPUC--------------------------------------
     * Bit#:   ----7-----6-----5-----4-----3-----2-----1-----0------
     * WPU:   --|WPUC7|WPUC6|WPUC5|WPUC4|WPUC3|WPUC2|WPUC1|WPUC0|---
     * -------------------------------------------------------------
     */
        WPUA = 0x00; 
        WPUB = 0x00; 
        WPUC = 0x00; 
        OPTION_REGbits.nWPUEN = 0;

    // ODCONx registers (open-drain)
    /* 
	 * -------------------ODCONA----------------------------------
     * Bit#: ---7------6-----5----4-----3-----2-----1-----0-------
     * OD:   -|-----|-----|ODA5|ODUA4|-----|ODUA2|ODUA1|ODUA0|----
     * -----------------------------------------------------------
     * PORTA3 is Input only
     * 
	 * -------------------ODCONAB----------------------------
     * Bit#: ----7----6----5----4-----3----2-----1-----0-----
     * OD:   -|ODB7|ODB6|ODB5|ODB4|-----|-----|-----|-----|--
     * ------------------------------------------------------
     * -------------------ODCONAC----------------------------
     * Bit#: ----7----6----5----4----3----2----1-----0-------
     * OD:   --|ODC7|ODC6|ODC5|ODC4|ODC3|ODC2|ODC1|ODC0|-----
     * ------------------------------------------------------
     */
        ODCONA = 0x00;
        ODCONB = 0x00;
        ODCONC = 0x00;

    // initial state of LEDs - OFF (redundant, just to show another method to set)
        LATAbits.LATA5 = 0;
        LATAbits.LATA1 = 0;
        LATAbits.LATA2 = 0;
        LATCbits.LATC5 = 0;    
}

static int mode = 1; // 0 - blink; 1 - rotate (need to be recompiled to change modes)

void main(void) 
{
    system_init();
    
    if(0 == mode)
    {
        // turn on and off
        while(1)  
        {
            LATAbits.LATA5 = ~LATAbits.LATA5;   // Toggle the LED 
            LATAbits.LATA1 = ~LATAbits.LATA1;   // Toggle the LED
            LATAbits.LATA2 = ~LATAbits.LATA2;   // Toggle the LED
            LATCbits.LATC5 = ~LATCbits.LATC5;   // Toggle the LED
            __delay_ms(2000);                   // sleep 2 seconds
        }
    }
    else
    {
        // rotate
        int index = 1;
        while(1)
        {
            LATAbits.LATA5 = index & 1;
            LATAbits.LATA1 = (index & 2) >> 1;
            LATAbits.LATA2 = (index & 4) >> 2;
            LATCbits.LATC5 = (index & 8) >> 3;

            index <<= 1;
            if(index >= 16)
                index = 1;

            __delay_ms(500);                   // sleep 0.5 second
        }
    }
    
    return;
}
