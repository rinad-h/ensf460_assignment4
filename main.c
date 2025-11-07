#include <xc.h>
#include <p24F16KA101.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "clkChange.h"
#include "UART2.h"
#include "TimeDelay.h"
#include "ADC.h"
#include "IOs.h"

#pragma config BWRP = OFF
#pragma config BSS = OFF
#pragma config GWRP = OFF
#pragma config GCP = OFF
#pragma config FNOSC = FRC
#pragma config IESO = OFF
#pragma config POSCMOD = NONE
#pragma config OSCIOFNC = ON
#pragma config POSCFREQ = HS
#pragma config SOSCSEL = SOSCHP
#pragma config FCKSM = CSECMD
#pragma config WDTPS = PS32768
#pragma config FWPSA = PR128
#pragma config WINDIS = OFF
#pragma config FWDTEN = OFF
#pragma config BOREN = BOR3
#pragma config PWRTEN = ON
#pragma config I2C1SEL = PRI
#pragma config BORV = V18
#pragma config MCLRE = ON
#pragma config ICS = PGx2
#pragma config DSWDTPS = DSWDTPSF
#pragma config DSWDTOSC = LPRC
#pragma config RTCOSC = SOSC
#pragma config DSBOREN = ON
#pragma config DSWDTEN = ON


volatile SystemState current_mode = MODE_0_BARGRAPH;
volatile uint8_t mode_changed = 0;
volatile uint8_t pb1_event = 0;
volatile uint8_t timer_flag = 0;
volatile uint16_t sleep_flag = 0;

// Function prototypes
void setup_timer1(void);

// Timer1 setup
void setup_timer1(void) {
    T1CONbits.TON = 0;       // Turn timer off while setting it up
    T1CONbits.TCS = 0;       // Use internal clock
    T1CONbits.TCKPS = 0b01;  // Prescaler = 8
    TMR1 = 0;                // Clear timer count
    PR1 = 62499;             // Period register for roughly 100ms interval
    IFS0bits.T1IF = 0;       // Clear interrupt flag
    IEC0bits.T1IE = 1;       // Enable timer interrupt
    T1CONbits.TON = 1;       // Start timer
}

int main(void) {
    newClk(500);              // Run system at 500 kHz
    AD1PCFG = 0xFFFF;         // Set all pins to digital by default
    AD1PCFGbits.PCFG12 = 0;   // Make pin 15 analog input (potentiometer)
    IOinit();                 // Set up push buttons
    InitUART2();              // Initialize UART for serial communication
    init_ADC();               // Set up ADC
    setup_timer1();           // Start Timer1
    delay_ms(10);

    Disp2String("\r\n*** MODE 0: Bar Graph Display ***\r\n");

    // Main loop: constantly check for button press or ADC changes
    while (1) {
        IOCheck();            // Handles LED bar display and mode switching
        delay_ms(10);         // debounce
    }

    return 0;
}

//Timer1 Interrupt
void __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void) {
    IFS0bits.T1IF = 0;  // Clear interrupt flag
    timer_flag = 1;     // Signal that timer expired
}

//Timer2 Interrupt
void __attribute__((interrupt, no_auto_psv)) _T2Interrupt(void) {
    IFS0bits.T2IF = 0;
    T2CONbits.TON = 0;  // Stop Timer2
    sleep_flag = 1;     //Flag that sleep/delay period ended
}

//Detects button press and release on PB1
void __attribute__((interrupt, no_auto_psv)) _CNInterrupt(void) {
    static uint8_t button_was_pressed = 0;
    uint8_t current_state = PORTBbits.RB7;  // Read push button state

    // Check for release (button goes from low → high)
    if (button_was_pressed == 1 && current_state == 1) {
        pb1_event = 1;          // Button was released → event triggered
        button_was_pressed = 0;
    } 
    // Check for press (button goes from high → low)
    else if (current_state == 0) {
        button_was_pressed = 1;
    }

    IFS1bits.CNIF = 0;          // Clear CN interrupt flag
}
