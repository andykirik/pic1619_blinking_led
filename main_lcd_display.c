/*
 * File:   main_timer.c
 * Author: akirik
 *
 * Created on January 9, 2018, 3:52 PM
 * 
 * Using LCD Display (1602A-1)
 * 
 *  Board connection (Microchip Curiosity Board 8-bit, PIC16F1619):
 *   PIN                	Module                         				  
 * -------------------------------------------                        
 *    RC2                   RS (register select)
 *    RC3                   E  (enable signal)
 *    RC4                   DB4
 *    RC5                   DB5
 *    RC6                   DB6
 *    RC7                   DB7
 */

#ifndef D0
#define RS RC2
#define EN RC3
#define D4 RC4
#define D5 RC5
#define D6 RC6
#define D7 RC7
#endif

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
#include <stdbool.h>
#include <stdint.h>

void LCDSetData(char x)
{
	D4 = x & 1 ? 1 : 0;
	D5 = x & 2 ? 1 : 0;
	D6 = x & 4 ? 1 : 0;
	D7 = x & 8 ? 1 : 0;
}

void LCDSendCommand(char CMD)
{
	LCDSetData(CMD);
	RS = 0;              
	EN  = 1;             
    __delay_ms(4);
    EN  = 0;             
}

void LCDSendData(char n)
{
   char firstNibble = n & 0xF0;
   char secondNibble = n & 0x0F;
   RS = 1;             
   LCDSetData(firstNibble >> 4); 
   EN = 1;
   __delay_us(40);
   EN = 0;
   LCDSetData(secondNibble);
   EN = 1;
   __delay_us(40);
   EN = 0;
}

void LCDCleanScreen()
{
	LCDSendCommand(0);
	LCDSendCommand(1);
}

void LCDSetCursor(char row, char column)
{
	char offset = 1 == row ? 0x80 : 0xC0;
    char temp = offset + column - 1;
	LCDSendCommand(temp >> 4);
	LCDSendCommand(temp & 0x0F);
}

void LCDInitialize()
{
  LCDSetData(0x00);
   __delay_ms(20);
  LCDSendCommand(0x03);//send reset
	__delay_ms(5);
  LCDSendCommand(0x03);//NOTE this is not copy/paste error, we have to do it three times
	__delay_ms(11);
  LCDSendCommand(0x03);//NOTE this is not copy/paste error, we have to do it three times
  
  LCDSendCommand(0x02);
  LCDSendCommand(0x02);
  LCDSendCommand(0x08);
  LCDSendCommand(0x00);
  LCDSendCommand(0x0C);
  LCDSendCommand(0x00);
  LCDSendCommand(0x06);
}

void LCDPrint(char *pstr)
{
    for(; *pstr; ++pstr)
	   LCDSendData(*pstr);
}

void LCDShiftRight()
{
	LCDSendCommand(0x01);
	LCDSendCommand(0x0C);
}

void LCDShiftLeft()
{
	LCDSendCommand(0x01);
	LCDSendCommand(0x08);
}

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

	// LCD Display setup
    LCDInitialize();
}

void main(void) 
{
    system_init();
    
    //LCDCleanScreen();
    //LCDSetCursor(1, 1);
    //LCDPrint("  Jump Start");
    //LCDSetCursor(2, 1);
    //LCDPrint(" Programming");

    while(1)  
	{
            LCDCleanScreen();
            LCDSetCursor(1,1);
            LCDPrint("Jump Start");
            LCDSetCursor(2,1);
            LCDPrint("Programming");
            __delay_ms(2000);
            
            LCDCleanScreen();
            LCDSetCursor(1,1);
            LCDPrint("Developed By");
            LCDSetCursor(2,1);
            LCDPrint("Andrew Kirik");
            __delay_ms(2000);
            
            LCDCleanScreen();
            LCDSetCursor(1,1);
            LCDPrint("https://www.jumpstartprogramming.com");

            for(int i = 0; i < 20; ++i)
            {
                __delay_ms(500);
                LCDShiftLeft();
            }

            for(int i = 0; i < 20; ++i)
            {
                __delay_ms(400);
                LCDShiftRight();
            }
    }   

    return;
}
