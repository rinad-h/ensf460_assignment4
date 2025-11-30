#include <xc.h>
#include <p24F16KA101.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "IOs.h"
#include "UART2.h"
#include "ADC.h"

#define LED1 LATBbits.LATB9
#define LED2 LATAbits.LATA6
#define PWM_PERIOD 45
#define LONG_PRESS_THRESHOLD 600

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

// Button Flags from ISR
extern volatile uint8_t pb1_release_event;
extern volatile uint16_t pb1_duration_timer;
extern volatile uint8_t pb2_release_event;
extern volatile uint8_t pb3_release_event;

uint8_t get_current_intensity_percent(void) {
    if (system_state == 0) {
        if (blink_mode && blink_state == 1) {
            return 100;
        } else {
            return 0;
        }
    }
    
    if (blink_mode && blink_state == 0) {
        return 0;
    }
    
    return (uint8_t)((pwm_duty_cycle * 100UL) / PWM_PERIOD);
}

void update_status_line(void) {
    char buf[100];
    uint8_t intensity_percent = get_current_intensity_percent();
    
    if (system_state == 0) {
        if (blink_mode) {
            sprintf(buf, "\rOFF MODE (Blink) --- Intensity: %3u%%                      ",
                    intensity_percent);
        } else {
            sprintf(buf, "\rOFF MODE --- Intensity: %3u%%                      ",
                    intensity_percent);
        }
    } else {
        if (blink_mode) {
            if (blink_state == 1) {
                sprintf(buf,
                    "\rON MODE (Blink) LED%d --- ADC: %4u Intensity: %3u%%          ",
                    active_led, current_adc_value, intensity_percent);
            } else {
                sprintf(buf,
                    "\rON MODE (Blink) LED%d --- ADC: ---- Intensity:   0%%          ",
                    active_led);
            }
        } else {
            sprintf(buf,
                "\rON MODE LED%d --- ADC: %4u Intensity: %3u%%          ",
                active_led, current_adc_value, intensity_percent);
        }
    }

    Disp2String(buf);
}

static void handle_adc_update(void) {
    if (timer_flag) {
        timer_flag = 0;

        if (system_state == 1) {
            uint16_t adc_value = do_ADC();
            current_adc_value = adc_value;
            set_led_intensity(adc_value);
        } else {
            current_adc_value = 0;
        }
        
        if (!uart_transmit_mode || blink_mode) {
             update_status_line();
        }
    }
}

static void handle_uart_transmission(void) {
    if (uart_transmit_mode && system_state == 1) {
        char buf[80];
        uint8_t intensity = get_current_intensity_percent();
        
        sprintf(buf, "\r\n%lu,%u,%u", millisecond_counter, current_adc_value, intensity);
        Disp2String(buf);
        
        // Auto-Stop after 60 seconds
        if (millisecond_counter >= 60000) {
            uart_transmit_mode = 0;
            Disp2String("\r\nSTOP_DATA\r\n");
            update_status_line();
        }
    }
}

void set_led_intensity(uint16_t adc_value) {
    pwm_duty_cycle = (uint16_t)((adc_value * (uint32_t)PWM_PERIOD) / 1023);
}

void IOinit(void) {
    // LEDs
    TRISBbits.TRISB9 = 0; LATBbits.LATB9 = 0;
    TRISAbits.TRISA6 = 0; LATAbits.LATA6 = 0;
    
    // PB1 (RB7)
    TRISBbits.TRISB7 = 1;
    CNPU2bits.CN23PUE = 1;
    CNEN2bits.CN23IE = 1;
    
    // PB2 (RB4)
    TRISBbits.TRISB4 = 1;
    CNPU1bits.CN1PUE = 1;
    CNEN1bits.CN1IE = 1;
    
    // PB3 (RA4) 
    TRISAbits.TRISA4 = 1;
    CNPU1bits.CN0PUE = 1;
    CNEN1bits.CN0IE = 1;
    
    // Read ports to clear mismatch condition
    (void)PORTB; (void)PORTA;
    
    // CN Interrupt Priority and Enable
    IPC4bits.CNIP = 6;
    IFS1bits.CNIF = 0;
    IEC1bits.CNIE = 1;
    
    // Timer3 (PWM)
    T3CONbits.TON = 0;
    T3CONbits.TCS = 0;
    T3CONbits.TCKPS = 0b00;
    TMR3 = 0;
    PR3 = 122;
    IFS0bits.T3IF = 0;
    IPC2bits.T3IP = 7;
    IEC0bits.T3IE = 1;
    T3CONbits.TON = 1;
}

static uint8_t handle_pb1_events(void) {
    if (pb1_release_event) {
        pb1_release_event = 0; 
        
        if (system_state == 1 && pb1_duration_timer > LONG_PRESS_THRESHOLD) {
            if (active_led == 1) {
                active_led = 2;
                LED1 = 0;
            } else {
                active_led = 1;
                LED2 = 0;
            }
            update_status_line();
        } else {
            if (system_state == 0) {
                system_state = 1;
                active_led = 1;
            } else {
                system_state = 0;
                LED1 = 0;
                LED2 = 0;
                pwm_duty_cycle = 0;
                uart_transmit_mode = 0;
            }
            update_status_line();
        }
        
        return 1; // Handled
    }
    return 0; // Not handled
}

static uint8_t handle_pb2_events(void) {
    if (pb2_release_event) {
        pb2_release_event = 0; 
        
        blink_mode = !blink_mode;
        blink_counter = 0;
        blink_state = 1; 
        
        update_status_line();
        
        if (system_state == 0 && !blink_mode) {
            LED1 = 0;
            LED2 = 0;
            Idle();
        }
        
        return 1; // Handled
    }
    return 0; // Not handled
}

static uint8_t handle_pb3_events(void) {
    if (pb3_release_event) {
        pb3_release_event = 0;
        
        if (system_state == 1) {
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
        
        return 1; // Handled
    }
    return 0; // Not handled
}

void IOCheck(void) {
    if (handle_pb1_events()) return;
    if (handle_pb2_events()) return;
    if (handle_pb3_events()) return;
    
    //Always handle background tasks
    handle_adc_update();
    handle_uart_transmission();
}
