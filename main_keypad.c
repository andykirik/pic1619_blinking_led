/*
 * File:   main.c
 * Author: akirik
 *
 * Created on January 19, 2018, 3:52 PM
 * 
 * "4x4 Universal 16 Key Keypad For Arduino"
 * 7 Segment LED (ELS-321HDB, Common Cathode)
 * 
 * =============================================================================
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
 *    A0                C4 (1)
 *    A1                C3 (2)
 *    A2                C2 (3)
 *    A4                C1 (4)
 *    B4                R4 (5)
 *    B5                R3 (6)
 *    B6                R2 (7)
 *    B7                R1 (8)
 * 
 * 
 * =============================================================================
 * 4x4 Universal 16 Key Keypad For Arduino
 * keypad pins:
 *      1   2   3   4   5   6   7   8
 * rows/columns:
 *      C4  C3  C2  C1  R4  R3  R2  R1
 * PIC16F1619:
 *      A0  A1  A2  A4  B4  B5  B6  B7
 *      --  --  --  --
 *      | PULL DOWN  |
 *        resistors
 *            |
 *           Vss
 * 
 *  keypad could be connected with Pull Up resistor or with Pull Down ones
 *  in this example we connect Pull Down to the ground
 *  if Pull Down are used then logic should be reversed
 * 
 *          
 * =============================================================================
 * 7 Segment LED
 *          A           1   14
 *       F     B        2   13
 *          G               12
 *       E     C        4
 *          D           
 *     LDP     RDP      6    9
 *                      7    8   
 * =============================================================================
 */

//4x4 Keypad
#define ROW1			LATBbits.LATB4		// Row 1
#define ROW2			LATBbits.LATB5		// Row 2
#define ROW3			LATBbits.LATB6		// Row 3
#define ROW4			LATBbits.LATB7		// Row 4
#define COL1			PORTAbits.RA4		// Column 1
#define COL2			PORTAbits.RA2		// Column 2
#define COL3			PORTAbits.RA1		// Column 3
#define COL4			PORTAbits.RA0		// Column 4

#define HIGH 1
#define LOW 0

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
    0b01110111,//A
    0b01111111,//B
    0b00111001,//C
    0b00111111,//D
    0b01000000,//*
    0b01001000,//#
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
        TRISA 	= 0xFF;     // PORTA 0, 1, 2, 4 Input
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

unsigned int keypad_readkey(void)
{
    // scan 1st row: 1, 2, 3, A
        ROW1 = HIGH;		
        ROW2 = LOW;
        ROW3 = LOW;
        ROW4 = LOW;
        __delay_us(30);
        if (COL1 == HIGH) return 0x1;		// Key '1' is pressed
        if (COL2 == HIGH) return 0x2;		// Key '2' is pressed
        if (COL3 == HIGH) return 0x3;		// Key '3' is pressed
        if (COL4 == HIGH) return 0xA;		// Key 'A' is pressed, we will store as 10
    
    // scan 2nd row: 4, 5, 6, B
        ROW1 = LOW;		
        ROW2 = HIGH;
        ROW3 = LOW;
        ROW4 = LOW;
        __delay_us(30);
        if (COL1 == HIGH) return 0x4;		// Key '4' is pressed
        if (COL2 == HIGH) return 0x5;		// Key '5' is pressed
        if (COL3 == HIGH) return 0x6;		// Key '6' is pressed
        if (COL4 == HIGH) return 0xB;		// Key 'B' is pressed, we will store as 11

	// scan 3rd row: 7, 8, 9, C
        ROW1 = LOW;		
        ROW2 = LOW;
        ROW3 = HIGH;
        ROW4 = LOW;
        __delay_us(30);
        if (COL1 == HIGH) return 0x7;		// Key '7' is pressed
        if (COL2 == HIGH) return 0x8;		// Key '8' is pressed
        if (COL3 == HIGH) return 0x9;		// Key '9' is pressed
        if (COL4 == HIGH) return 0xC;		// Key 'C' is pressed, we will store as 12

	// scan 4th row: *, 0, #, D
        ROW1 = LOW;		
        ROW2 = LOW;
        ROW3 = LOW;
        ROW4 = HIGH;
        __delay_us(30);
        if (COL1 == HIGH) return 0xE;		// Key '*' is pressed, we will store as 14
        if (COL2 == HIGH) return 0x0;		// Key '0' is pressed
        if (COL3 == HIGH) return 0xF;		// Key '#' is pressed, we will store as 15
        if (COL4 == HIGH) return 0xD;		// Key 'D' is pressed, we will store as 13
    
    ROW1 = LOW;		
	ROW2 = LOW;
	ROW3 = LOW;
	ROW4 = LOW;

	return 0xFF;					// if no key press, the register is 0xFF
}

unsigned int keypad_getkey(void)
{
	unsigned int key = 0xFF;	
	
	// wait until any key is pressed
	while(key == 0xFF)
		key = keypad_readkey();
	
	// wait until that key is released
	while(keypad_readkey() != 0xFF);
	
	return key;
}	

void main(void) 
{
    system_init();
            
    while(1)
    {
        unsigned int key = keypad_getkey(); // get pressed button (wait)
        if(key > 16) key = 16;              // safeguard
        LATC = digits[key];                 // show it on a LED display
        __delay_us(30);
    }
       
    return;
}
