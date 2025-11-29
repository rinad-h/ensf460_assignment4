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

// Config bits (same as before)
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

// -----------------------------------------------------------------------------
// GLOBAL VARIABLE DEFINITIONS (The 'Real' copies)
// -----------------------------------------------------------------------------
volatile uint8_t system_state = 0;
volatile uint8_t active_led = 1;
volatile uint8_t timer_flag = 0;
volatile uint16_t sleep_flag = 0;
volatile uint8_t uart_transmit_mode = 0;
volatile uint32_t millisecond_counter = 0;

// PWM and Blink Variables
volatile uint16_t pwm_duty_cycle = 0;
volatile uint16_t pwm_counter = 0;
volatile uint8_t current_led_state = 0;
volatile uint8_t blink_mode = 0;
volatile uint16_t blink_counter = 0;
volatile uint8_t blink_state = 0;
volatile uint16_t current_adc_value = 0;

// Macros for ISR usage
#define PWM_PERIOD 60
#define LED1 LATBbits.LATB9
#define LED2 LATAbits.LATA6

void setup_timer1(void);

void setup_timer1(void) {
    T1CONbits.TON = 0;
    T1CONbits.TCS = 0;
    T1CONbits.TCKPS = 0b01; // 1:8
    TMR1 = 0;
    PR1 = 62499;
    IFS0bits.T1IF = 0;
    IEC0bits.T1IE = 1;
    T1CONbits.TON = 1;
}

int main(void) {
    newClk(500);
    AD1PCFG = 0xFFFF;
    AD1PCFGbits.PCFG12 = 0;

    IOinit();
    InitUART2();
    init_ADC();
    setup_timer1();

    delay_ms(10);
    Disp2String("OFF MODE                    \r");

    while (1) {
        IOCheck();
        delay_ms(10);  
    }
    return 0;
}

// -----------------------------------------------------------------------------
// INTERRUPTS
// -----------------------------------------------------------------------------

void __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void) {
    IFS0bits.T1IF = 0;
    timer_flag = 1;
}

// Minimal CN Interrupt - Logic moved to IOCheck()
void __attribute__((interrupt, no_auto_psv)) _CNInterrupt(void) {
    // Read ports to clear mismatch
    volatile uint8_t dummyB = PORTB;
    volatile uint8_t dummyA = PORTA;
    IFS1bits.CNIF = 0;
}

void __attribute__((interrupt, no_auto_psv)) _T2Interrupt(void) {
    IFS0bits.T2IF = 0;
    T2CONbits.TON = 0;
    sleep_flag = 1;
}

// PWM Interrupt - Logic inline to avoid function calls
void __attribute__((interrupt, no_auto_psv)) _T3Interrupt(void) {
    IFS0bits.T3IF = 0;
    
    // 1. Millisecond Counting (4 ticks = 1ms approx)
    static uint8_t ms_scaler = 0;
    ms_scaler++;
    if (ms_scaler >= 4) {
        ms_scaler = 0;
        if (uart_transmit_mode) millisecond_counter++;
    }

    // 2. Blink Logic
    if (blink_mode) {
        blink_counter++;
        if (blink_counter >= 2000) { // ~500ms
            blink_counter = 0;
            blink_state = !blink_state;
        }
    }

    // 3. LED PWM Logic
    uint8_t led_on = 1;
    if (blink_mode && !blink_state) led_on = 0; // Off phase of blink

    if (system_state == 1) {
        // ON Mode: PWM
        if (led_on && pwm_counter < pwm_duty_cycle) {
            if (active_led == 1) { LED1 = 1; LED2 = 0; } 
            else { LED1 = 0; LED2 = 1; }
        } else {
            if (active_led == 1) LED1 = 0;
            else LED2 = 0;
        }
        
        pwm_counter++;
        if (pwm_counter >= PWM_PERIOD) pwm_counter = 0;
        
    } else {
        // OFF Mode: Blink only (100% brightness)
        if (blink_mode && blink_state) {
            if (active_led == 1) { LED1 = 1; LED2 = 0; }
            else { LED1 = 0; LED2 = 1; }
        } else {
            LED1 = 0; LED2 = 0;
        }
        pwm_counter = 0;
    }
}
