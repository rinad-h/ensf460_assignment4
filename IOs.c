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
// MACROS
// -----------------------------------------------------------------------------
#define LED1 LATBbits.LATB9
#define LED2 LATAbits.LATA6
#define PWM_PERIOD 60
#define LONG_PRESS_THRESHOLD 150

// -----------------------------------------------------------------------------
// EXTERNAL VARIABLES
// -----------------------------------------------------------------------------
extern volatile uint8_t system_state;
extern volatile uint8_t active_led;
extern volatile uint8_t uart_transmit_mode;
extern volatile uint32_t millisecond_counter;
extern volatile uint8_t timer_flag;
extern volatile uint16_t pwm_duty_cycle;
extern volatile uint8_t blink_mode;
extern volatile uint16_t blink_counter;
extern volatile uint8_t blink_state;
extern volatile uint16_t current_adc_value;

// -----------------------------------------------------------------------------
// STATIC VARIABLES FOR BUTTON HANDLING
// -----------------------------------------------------------------------------
static uint16_t pb1_hold_timer = 0;
static uint8_t pb1_prev = 1;
static uint8_t pb2_prev = 1;
static uint8_t pb3_prev = 1;
static uint8_t long_press_handled = 0;

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
// STATE MACHINE EVENT HANDLERS (Priority Order)
// -----------------------------------------------------------------------------

// Handle PB1: Short press = ON/OFF toggle, Long press = LED swap
static uint8_t handle_pb1_events(void) {
    uint8_t pb1 = PORTBbits.RB7;
    uint8_t action_taken = 0;
    
    if (pb1 == 0) { // Pressed
        pb1_hold_timer++;
        if (pb1_hold_timer > LONG_PRESS_THRESHOLD && !long_press_handled && system_state == 1) {
            // LONG PRESS: Swap Active LED
            long_press_handled = 1;
            if (active_led == 1) { 
                active_led = 2; 
                LED1 = 0; 
            } else { 
                active_led = 1; 
                LED2 = 0; 
            }
            update_status_line();
            action_taken = 1;
        }
    } else { // Released
        if (pb1_prev == 0) { // Falling edge (release)
            if (!long_press_handled && pb1_hold_timer > 2) { 
                // SHORT PRESS: Toggle System ON/OFF
                if (system_state == 0) {
                    // OFF → ON_LED1
                    system_state = 1; 
                    active_led = 1;
                } else {
                    // ON → OFF
                    system_state = 0; 
                    LED1 = 0; 
                    LED2 = 0;
                    pwm_duty_cycle = 0; 
                    uart_transmit_mode = 0;
                }
                update_status_line();
                action_taken = 1;
            }
        }
        pb1_hold_timer = 0;
        long_press_handled = 0;
    }
    pb1_prev = pb1;
    
    return action_taken;
}

// Handle PB2: Toggle Blink Mode
static uint8_t handle_pb2_events(void) {
    uint8_t pb2 = PORTBbits.RB4;
    uint8_t action_taken = 0;
    
    if (pb2 == 0 && pb2_prev == 1) { // Button pressed (edge detection)
        // Toggle blink mode
        blink_mode = !blink_mode;
        blink_counter = 0;
        blink_state = blink_mode; // Start ON if entering blink
        update_status_line();
        
        // If turning off blink in OFF mode, ensure LEDs are off
        if (system_state == 0 && !blink_mode) { 
            LED1 = 0; 
            LED2 = 0; 
        }
        action_taken = 1;
    }
    pb2_prev = pb2;
    
    return action_taken;
}

// Handle PB3: Toggle UART Transmission (only in ON mode)
static uint8_t handle_pb3_events(void) {
    uint8_t pb3 = PORTAbits.RA4;
    uint8_t action_taken = 0;
    
    if (pb3 == 0 && pb3_prev == 1 && system_state == 1) { // Button pressed in ON mode
        if (uart_transmit_mode == 0) {
            // Start UART transmission
            uart_transmit_mode = 1;
            millisecond_counter = 0;
            Disp2String("\r\nSTART_DATA\r\n");
        } else {
            // Stop UART transmission
            uart_transmit_mode = 0;
            Disp2String("\r\nSTOP_DATA\r\n");
        }
        update_status_line();
        action_taken = 1;
    }
    pb3_prev = pb3;
    
    return action_taken;
}

// Handle ADC reading and LED intensity update (only in ON mode)
static uint8_t handle_adc_update(void) {
    if (system_state == 1 && timer_flag) {
        timer_flag = 0;
        uint16_t adc_value = do_ADC();
        current_adc_value = adc_value;
        set_led_intensity(adc_value);
        
        // Only update status if not transmitting UART
        if (!uart_transmit_mode) {
            update_status_line();
        }
        return 1;
    }
    return 0;
}

// Handle UART data transmission
static uint8_t handle_uart_transmission(void) {
    if (uart_transmit_mode && system_state == 1) {
        char buf[80];
        uint8_t intensity = get_current_intensity_percent();
        
        sprintf(buf, "\r\n%lu,%u,%u", millisecond_counter, current_adc_value, intensity);
        Disp2String(buf);
        
        // Check if 60 seconds elapsed
        if (millisecond_counter >= 60000) {
            uart_transmit_mode = 0;
            Disp2String("\r\nSTOP_DATA\r\n");
            update_status_line();
        }
        return 1;
    }
    return 0;
}

// -----------------------------------------------------------------------------
// MAIN IO CHECK (State Machine Dispatcher)
// -----------------------------------------------------------------------------
void IOCheck(void) {
    // Process events in priority order
    // Note: All handlers can execute, they don't necessarily return on first action
    handle_pb1_events();
    handle_pb2_events();
    handle_pb3_events();
    handle_adc_update();
    handle_uart_transmission();
}
