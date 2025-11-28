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

volatile uint8_t system_state = 0;  // 0 = OFF, 1 = ON
volatile uint8_t active_led = 1;     // 1 = LED1, 2 = LED2
volatile uint8_t pb1_event = 0;
volatile uint8_t pb1_long_press = 0;
volatile uint8_t pb2_event = 0;      // PB2 event flag
volatile uint8_t timer_flag = 0;
volatile uint16_t sleep_flag = 0;

extern void handle_pwm_interrupt(void);

void setup_timer1(void);

void setup_timer1(void) {
    T1CONbits.TON = 0;
    T1CONbits.TCS = 0;
    T1CONbits.TCKPS = 0b01;  // 1:8 prescaler
    TMR1 = 0;
    PR1 = 62499;  // ~500ms interrupt at 500kHz clock
    IFS0bits.T1IF = 0;
    IEC0bits.T1IE = 1;
    T1CONbits.TON = 1;
}

int main(void) {
    newClk(500);
    
    AD1PCFG = 0xFFFF;
    AD1PCFGbits.PCFG12 = 0;  // Enable AN12 for ADC
    
    IOinit();
    InitUART2();
    init_ADC();
    setup_timer1();
    
    delay_ms(10);
    
    Disp2String("\r\n*** LED Control System ***\r\n");
    Disp2String("PB1 Short press: Toggle ON/OFF\r\n");
    Disp2String("PB1 Long press (when ON): Swap LEDs\r\n");
    Disp2String("PB2: Toggle LED blinking\r\n");
    Disp2String("  - ON mode: Blink at current intensity\r\n");
    Disp2String("  - OFF mode: Blink at 100% intensity\r\n");
    Disp2String("*** SYSTEM OFF ***\r\n");
    
    while (1) {
        IOCheck();
        delay_ms(10);
    }
    
    return 0;
}

void __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void) {
    IFS0bits.T1IF = 0;
    timer_flag = 1;
}

void __attribute__((interrupt, no_auto_psv)) _CNInterrupt(void) {
    static uint8_t pb1_was_pressed = 0;
    static uint16_t pb1_press_start_time = 0;
    static uint8_t pb2_was_pressed = 0;
    static uint16_t main_loop_counter = 0;
    
    uint8_t pb1_current_state = PORTBbits.RB7;
    uint8_t pb2_current_state = PORTBbits.RB4;
    
    main_loop_counter++;  // Rough time counter (increments each CN event)
    
    // Handle PB1 (existing logic)
    if (pb1_was_pressed == 1 && pb1_current_state == 1) {
        // PB1 released
        uint16_t press_duration = main_loop_counter - pb1_press_start_time;
        
        if (press_duration > 200) {
            pb1_long_press = 1;
        } else {
            pb1_event = 1;
        }
        
        pb1_was_pressed = 0;
    } else if (pb1_current_state == 0) {
        // PB1 pressed
        pb1_was_pressed = 1;
        pb1_press_start_time = main_loop_counter;
    }
    
    // Handle PB2 (simple press/release detection)
    if (pb2_was_pressed == 1 && pb2_current_state == 1) {
        // PB2 released - trigger event
        pb2_event = 1;
        pb2_was_pressed = 0;
    } else if (pb2_current_state == 0) {
        // PB2 pressed
        pb2_was_pressed = 1;
    }
    
    IFS1bits.CNIF = 0;
}

void __attribute__((interrupt, no_auto_psv)) _T2Interrupt(void) {
    IFS0bits.T2IF = 0;
    T2CONbits.TON = 0;
    sleep_flag = 1;
}

void __attribute__((interrupt, no_auto_psv)) _T3Interrupt(void) {
    IFS0bits.T3IF = 0;
    handle_pwm_interrupt();  // Call PWM handler from IOs.c
}
