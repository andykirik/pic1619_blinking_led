/*
 * File:   main.c
 * Author: akirik
 *
 * Created on January 9, 2018, 3:52 PM
 * 
 * 7 Segment LED (ELS-321HDB, Common Cathode)
 * 
 *  Board connection (Microchip Curiosity Board 8-bit, PIC16F1619):
 *   PIN                	Module                         				  
 * -------------------------------------------                        
 *    RC0              segment A   (pin 14)
 *    RC1              segment B   (pin 13)
 *    RC2              segment C   (pin 8)
 *    RC3              segment D   (pin 7)
 *    RC4              segment E   (pin 6)
 *    RC5              segment F   (pin 1)
 *    RC6              segment G   (pin 2)
 *    RC7              segment RDP (pin 9)
 *                     segment LDP (-)
 *                     cathode     (pins 4, 12)
 *
 *          A           1   14
 *       F     B        2   13
 *          G               12
 *       E     C        4
 *          D           
 *     LDP     RDP      6    9
 *                      7    8   
 */

//            .GFEDCBA
#define _1_ 0b00000110 //   B C
#define _2_ 0b01011011 // A B   D E   G
#define _3_ 0b01001111 // A B C D     G
#define _4_ 0b01100110 //   B C     F G 
#define _5_ 0b01101101 // A   C D   F G
#define _6_ 0b01111101 // A   C D E F G 
#define _7_ 0b00000111 // A B C
#define _8_ 0b01111111 // A B C D E F G
#define _9_ 0b01101111 // A B C D   F G
#define _0_ 0b00111111 // A B C D E F

int digits[] = {
    0b00111111,//0
    0b00000110,//1
    0b01011011,//2
    0b01001111,//3
    0b01100110,//4
    0b01101101,//5
    0b01111101,//6
    0b00000111,//7
    0b01111111,//8
    0b01101111,//9
    0b10000000 //.
};

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

// INIT
void system_init()
{
	// To control Digital I/O use three registers: ANSEL, TRIS and LAT:
	
    // ANSELx registers
	// ANSEL and ANSELH control the mode of AN0 through AN11:
	// 0 sets the pin to digital mode and 1 sets the pin to analog mode.
        ANSELA = 0x00;
        ANSELB = 0x00;
        ANSELC = 0x00;

    // TRISx registers
	// This register specifies the data direction of each pin:
	// O - output, 1 - input
        TRISA 	= 0x00;     // Set All on PORTA as Output
        TRISB 	= 0x00;     // Set All on PORTB as Output
        TRISC 	= 0x00;     // Set All on PORTC as Output

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

}



void main(void) 
{
    system_init();

    LATCbits.LATC7 = 1;                     // dot
    __delay_ms(1000);                       // sleep 2 seconds
    
    while(1)
    {
        for(int n = 0; n < 10; ++n)
        {
            LATC = digits[n];              // next digit
            __delay_ms(1000);              // sleep 1 second
        }
    }
    
    /*while(1)
    {
        LATC = _1_;
        __delay_ms(1000);                   // sleep 1 second
        LATC = _2_;
        __delay_ms(1000);                   // sleep 1 second
        LATC = _3_;
        __delay_ms(1000);                   // sleep 1 second
        LATC = _4_;
        __delay_ms(1000);                   // sleep 1 second
        LATC = _5_;
        __delay_ms(1000);                   // sleep 1 second
        LATC = _6_;
        __delay_ms(1000);                   // sleep 1 second
        LATC = _7_;
        __delay_ms(1000);                   // sleep 1 second
        LATC = _8_;
        __delay_ms(1000);                   // sleep 1 second
        LATC = _9_;
        __delay_ms(1000);                   // sleep 1 second
        LATC = _0_;
        __delay_ms(1000);                   // sleep 1 second
    }*/
    
    return;
}
