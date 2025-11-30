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


volatile uint8_t system_state = 0;
volatile uint8_t active_led = 1;
volatile uint8_t timer_flag = 0;
volatile uint16_t sleep_flag = 0;
volatile uint8_t uart_transmit_mode = 0;
volatile uint32_t millisecond_counter = 0;

volatile uint16_t pwm_duty_cycle = 0;
volatile uint16_t pwm_counter = 0;
volatile uint8_t current_led_state = 0;
volatile uint8_t blink_mode = 0;
volatile uint16_t blink_counter = 0;
volatile uint8_t blink_state = 0;
volatile uint16_t current_adc_value = 0;


volatile uint8_t pb1_is_pressed = 0;   
volatile uint16_t pb1_duration_timer = 0; 
volatile uint8_t pb1_release_event = 0; 

volatile uint8_t pb2_is_pressed = 0;
volatile uint8_t pb2_release_event = 0;

volatile uint8_t pb3_is_pressed = 0;
volatile uint8_t pb3_release_event = 0;

volatile uint8_t cn_pb1_prev = 1; 
volatile uint8_t cn_pb2_prev = 1;
volatile uint8_t cn_pb3_prev = 1;

#define PWM_PERIOD 45
#define LED1 LATBbits.LATB9
#define LED2 LATAbits.LATA6


void setup_timer1(void) {
    T1CONbits.TON = 0;
    T1CONbits.TCS = 0;
    T1CONbits.TCKPS = 0b01; // 1:8
    TMR1 = 0;
    PR1 = 3124;
    IFS0bits.T1IF = 0;
    IEC0bits.T1IE = 1;
    T1CONbits.TON = 1;
}

int main(void) {
    newClk(500); // 500kHz clock
    AD1PCFG = 0xFFFF;
    AD1PCFGbits.PCFG12 = 0; // AN12 for ADC

    IOinit();       // Initialize GPIO and CN Interrupts
    InitUART2();    // Initialize UART
    init_ADC();     // Initialize ADC
    setup_timer1(); // Initialize Timer 1

    Disp2String("\rSystem Ready: OFF MODE\r");

    while (1) {
        IOCheck();    
        delay_ms(5); 
        Idle();
                      
    }
    return 0;
}

// Timer 1: ADC triggering flag
void __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void) {
    IFS0bits.T1IF = 0;
    timer_flag = 1;
}

// Timer 2: Sleep flag for delay_ms
void __attribute__((interrupt, no_auto_psv)) _T2Interrupt(void) {
    IFS0bits.T2IF = 0;
    T2CONbits.TON = 0;
    sleep_flag = 1;
}

void __attribute__((interrupt, no_auto_psv)) _CNInterrupt(void) {
    IFS1bits.CNIF = 0;

    //Read Ports immediately to handle mismatch and get current state
    uint8_t cur_pb1 = PORTBbits.RB7;
    uint8_t cur_pb2 = PORTBbits.RB4;
    uint8_t cur_pb3 = PORTAbits.RA4;

    if (cur_pb1 == 0 && cn_pb1_prev == 1) { 
        pb1_is_pressed = 1;
        pb1_duration_timer = 0; 
    } 
    else if (cur_pb1 == 1 && cn_pb1_prev == 0) { 
        pb1_is_pressed = 0;
        pb1_release_event = 1; 
    }
    cn_pb1_prev = cur_pb1;

    if (cur_pb2 == 0 && cn_pb2_prev == 1) {
        pb2_is_pressed = 1;
    } 
    else if (cur_pb2 == 1 && cn_pb2_prev == 0) {
        pb2_is_pressed = 0;
        pb2_release_event = 1;
    }
    cn_pb2_prev = cur_pb2;

    if (cur_pb3 == 0 && cn_pb3_prev == 1) {
        pb3_is_pressed = 1;
    } 
    else if (cur_pb3 == 1 && cn_pb3_prev == 0) {
        pb3_is_pressed = 0;
        pb3_release_event = 1;
    }
    cn_pb3_prev = cur_pb3;
}

// Timer 3: PWM + Global Timebase + Button Timer
void __attribute__((interrupt, no_auto_psv)) _T3Interrupt(void) {
    IFS0bits.T3IF = 0;
    
    
    if (pb1_is_pressed) {
        // Prevent overflow
        if (pb1_duration_timer < 60000) pb1_duration_timer++;
    }

    static uint8_t ms_scaler = 0;
    ms_scaler++;
    if (ms_scaler >= 4) {
        ms_scaler = 0;
        if (uart_transmit_mode) millisecond_counter++;
    }

    if (blink_mode) {
        blink_counter++;
        if (blink_counter >= 1024) { // ~500ms
            blink_counter = 0;
            blink_state = !blink_state;
        }
    }

    uint8_t led_on = 1;
    if (blink_mode && !blink_state) led_on = 0; 

    if (system_state == 1) {
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
        if (blink_mode && blink_state) {
            if (active_led == 1) { LED1 = 1; LED2 = 0; }
            else { LED1 = 0; LED2 = 1; }
        } else {
            LED1 = 0; LED2 = 0;
        }
        pwm_counter = 0;
    }
}
