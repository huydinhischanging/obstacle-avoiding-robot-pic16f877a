/*
 * main_original.c
 *
 * Original firmware as submitted for the course project.
 * Kept in the repository for reference — see README.md ("Debugging notes")
 * for the defects found in this version and how they were addressed
 * in src/main.c.
 *
 * Target : PIC16F877A @ 20 MHz
 * Toolchain: MPLAB X + XC8
 *
 * Reference only. This file defines its own main() and would collide with
 * src/main.c at link time, so its body is compiled out unless BUILD_ORIGINAL
 * is defined (e.g. xc8-cc -DBUILD_ORIGINAL ...). The code below is otherwise
 * unchanged from the original submission.
 */

#ifdef BUILD_ORIGINAL

#include <xc.h>

// Configuration bits setup
#pragma config FOSC  = HS       // High Speed Oscillator
#pragma config WDTE  = OFF      // Watchdog Timer disabled
#pragma config PWRTE = ON       // Power-up Timer enabled
#pragma config BOREN = ON       // Brown-out Reset enabled
#pragma config LVP   = OFF      // Low-Voltage Programming disabled
#pragma config CPD   = OFF      // Data EEPROM Code Protection off
#pragma config WRT   = OFF      // Flash Program Memory Write Enable off
#pragma config CP    = OFF      // Flash Program Memory Code Protection off

#define _XTAL_FREQ 20000000     // 20 MHz oscillator frequency

// Pin Definitions
#define Trigger RB2             // Ultrasonic sensor trigger pin
#define Echo    RB3             // Ultrasonic sensor echo pin
#define right   RB4             // Right IR sensor pin
#define left    RB0             // Left IR sensor pin

// Constants
#define THRESHOLD_CM 20         // Distance threshold in cm

// Global Variables
int time_taken;
int distance;

void back_off(void) {
    RC4 = 1; RC5 = 0;
    RC6 = 1; RC7 = 0;
    __delay_ms(1000);
}

void calculate_distance(void) {
    TMR1H = 0; TMR1L = 0;

    Trigger = 1;
    __delay_us(10);
    Trigger = 0;

    while (Echo == 0);          // no timeout
    TMR1ON = 1;
    while (Echo == 1);          // no timeout
    TMR1ON = 0;

    time_taken = (TMR1L | (TMR1H << 8));
    distance = (0.0272 * time_taken) / 2;
}

void main(void) {
    TRISB2 = 0;
    TRISB3 = 1;
    TRISD2 = 0;
    TRISB1 = 0; TRISB4 = 1;     // note: RB1 configured, but RB0 is the pin actually read

    TRISC4 = 0; TRISC5 = 0;
    TRISC6 = 0; TRISC7 = 0;

    T1CON = 0x20;               // Timer1, prescaler 1:4

    RD2 = 1;
    __delay_ms(1000);
    RD2 = 0;

    while (1) {
        calculate_distance();

        if (distance > THRESHOLD_CM) {
            RC4 = 0; RC5 = 1;
            RC6 = 0; RC7 = 1;
            RD2 = 1;
        }

        calculate_distance();
        if (left == 0 && right == 1 && distance <= THRESHOLD_CM) {
            back_off();
            RC4 = 1; RC5 = 1;
            RC6 = 1; RC7 = 0;
            __delay_ms(500);
        }

        calculate_distance();
        if (right == 0 && left == 1 && distance <= THRESHOLD_CM) {
            back_off();
            RC4 = 0; RC5 = 1;
            RC6 = 1; RC7 = 1;
            __delay_ms(500);
        }

        calculate_distance();
        if (right == 1 && left == 1 && distance <= THRESHOLD_CM) {
            back_off();
            RC4 = 0; RC5 = 1;
            RC6 = 1; RC7 = 1;
            __delay_ms(500);
        }

        calculate_distance();
        if (right == 0 && left == 0 && distance <= THRESHOLD_CM) {
            back_off();
            RC4 = 1; RC5 = 0;
            RC6 = 1; RC7 = 1;
            __delay_ms(1000);
        }
    }
}

#endif /* BUILD_ORIGINAL */
