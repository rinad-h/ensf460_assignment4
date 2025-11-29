#include <xc.h>
#include <p24F16KA101.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "UART2.h"
#include "TimeDelay.h"
#include "ADC.h"
#include "IOs.h"

// -----------------------------------------------------------------------------
// MACROS (Must match main.c)
// -----------------------------------------------------------------------------
#define LED1 LATBbits.LATB9
#define LED2 LATAbits.LATA6
#define PWM_PERIOD 60 

// -----------------------------------------------------------------------------
// EXTERNAL VARIABLES (Defined in main.c)
// -----------------------------------------------------------------------------
extern volatile uint8_t system_state;
extern volatile uint8_t active_led;
extern volatile uint8_t uart_transmit_mode;
extern volatile uint32_t millisecond_counter;
extern volatile uint8_t timer_flag;

// PWM & Blink variables (Defined in main.c)
extern volatile uint16_t pwm_duty_cycle;
extern volatile uint8_t blink_mode;
extern volatile uint16_t blink_counter;
extern volatile uint8_t blink_state;
extern volatile uint16_t current_adc_value;

// -----------------------------------------------------------------------------
// HELPER FUNCTIONS
// -----------------------------------------------------------------------------
uint8_t get_current_intensity_percent(void) {
    if (blink_mode && blink_state == 0) return 0;
    
    if (system_state == 0) {
        if (blink_mode && blink_state == 1) return 100;
        return 0; 
    }
    
    return (uint8_t)((pwm_duty_cycle * 100UL) / PWM_PERIOD);
}

void update_status_line(void) {
    char buf[100];
    uint8_t intensity_percent = get_current_intensity_percent();
    
    if (system_state == 0) {
        if (blink_mode) {
            sprintf(buf, "\rOFF MODE (Blink) --- Intensity: %3u%%                     ", intensity_percent);
        } else {
            sprintf(buf, "\rOFF MODE --- Intensity: %3u%%                     ", intensity_percent);
        }
    } else {
        if (blink_mode) {
            sprintf(buf, "\rON MODE (Blink) LED%d --- ADC: %4u  Intensity: %3u%%          ", 
                    active_led, current_adc_value, intensity_percent);
        } else {
            sprintf(buf, "\rON MODE LED%d --- ADC: %4u  Intensity: %3u%%          ", 
                    active_led, current_adc_value, intensity_percent);
        }
    }
    Disp2String(buf);
}

void set_led_intensity(uint16_t adc_value) {
    pwm_duty_cycle = (uint16_t)((adc_value * (uint32_t)PWM_PERIOD) / 1023);
}

// -----------------------------------------------------------------------------
// INITIALIZATION
// -----------------------------------------------------------------------------
void IOinit(void) {
    // LEDs
    TRISBbits.TRISB9 = 0; LATBbits.LATB9 = 0;
    TRISAbits.TRISA6 = 0; LATAbits.LATA6 = 0;
    
    // PB1 (RB7)
    TRISBbits.TRISB7 = 1; CNPU2bits.CN23PUE = 1; CNEN2bits.CN23IE = 1;
    
    // PB2 (RB4)
    TRISBbits.TRISB4 = 1; CNPU1bits.CN1PUE = 1; CNEN1bits.CN1IE = 1;
    
    // PB3 (RA4)
    TRISAbits.TRISA4 = 1; CNPU1bits.CN0PUE = 1; CNEN1bits.CN0IE = 1;
    
    // Read ports
    (void)PORTB; (void)PORTA;
    
    // CN Interrupt
    IPC4bits.CNIP = 6; IFS1bits.CNIF = 0; IEC1bits.CNIE = 1;
    
    // Timer3 (PWM)
    T3CONbits.TON = 0; T3CONbits.TCS = 0; T3CONbits.TCKPS = 0b00;
    TMR3 = 0; PR3 = 122; 
    IFS0bits.T3IF = 0; IPC2bits.T3IP = 7; IEC0bits.T3IE = 1;
    T3CONbits.TON = 1;
}

// -----------------------------------------------------------------------------
// IO CHECK (POLLING LOGIC)
// -----------------------------------------------------------------------------
void IOCheck(void) {
    static uint16_t pb1_hold_timer = 0;
    static uint8_t pb1_prev = 1;
    static uint8_t pb2_prev = 1;
    static uint8_t pb3_prev = 1;
    static uint8_t long_press_handled = 0;
    
    uint16_t adc_value;
    char buf[80];

    // Read Inputs (Active Low)
    uint8_t pb1 = PORTBbits.RB7;
    uint8_t pb2 = PORTBbits.RB4;
    uint8_t pb3 = PORTAbits.RA4;

    // --- PB1 LOGIC (Short = On/Off, Long = Swap LED) ---
    if (pb1 == 0) { // Pressed
        pb1_hold_timer++;
        if (pb1_hold_timer > 150 && !long_press_handled && system_state == 1) {
            // Long Press Detected
            long_press_handled = 1;
            // Swap LEDs
            if (active_led == 1) { active_led = 2; LED1 = 0; }
            else { active_led = 1; LED2 = 0; }
            update_status_line();
        }
    } else { // Released
        if (pb1_prev == 0) { // Falling edge (release)
            if (!long_press_handled && pb1_hold_timer > 2) { 
                // Short Press Action
                if (system_state == 0) {
                    system_state = 1; active_led = 1;
                } else {
                    system_state = 0; LED1 = 0; LED2 = 0;
                    pwm_duty_cycle = 0; uart_transmit_mode = 0;
                }
                update_status_line();
            }
        }
        pb1_hold_timer = 0;
        long_press_handled = 0;
    }
    pb1_prev = pb1;

    // --- PB2 LOGIC (Toggle Blink) ---
    if (pb2 == 0 && pb2_prev == 1) {
        blink_mode = !blink_mode;
        blink_counter = 0;
        blink_state = blink_mode; // Start ON
        update_status_line();
        if (system_state == 0 && !blink_mode) { LED1 = 0; LED2 = 0; }
    }
    pb2_prev = pb2;

    // --- PB3 LOGIC (Toggle UART) ---
    if (pb3 == 0 && pb3_prev == 1 && system_state == 1) {
        if (uart_transmit_mode == 0) {
            uart_transmit_mode = 1;
            millisecond_counter = 0;
            Disp2String("\r\nSTART_DATA\r\n");
        } else {
            uart_transmit_mode = 0;
            Disp2String("\r\nSTOP_DATA\r\n");
        }
        update_status_line();
    }
    pb3_prev = pb3;

    // --- ADC & Status Update ---
    if (system_state == 1 && timer_flag) {
        timer_flag = 0;
        adc_value = do_ADC();
        current_adc_value = adc_value;
        set_led_intensity(adc_value);
        if (!uart_transmit_mode) update_status_line();
    }

    // --- UART Transmission ---
    if (uart_transmit_mode && system_state == 1) {
        uint8_t intensity = get_current_intensity_percent();
        sprintf(buf, "\r\n%lu,%u,%u", millisecond_counter, current_adc_value, intensity);
        Disp2String(buf);
        
        if (millisecond_counter >= 60000) {
            uart_transmit_mode = 0;
            Disp2String("\r\nSTOP_DATA\r\n");
            update_status_line();
        }
    }
}
