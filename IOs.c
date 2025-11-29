#include <xc.h>
#include <p24F16KA101.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "UART2.h"
#include "TimeDelay.h"
#include "ADC.h"
#include "IOs.h"

#define LED1 LATBbits.LATB9
#define LED2 LATAbits.LATA6

extern volatile uint8_t pb1_event;
extern volatile uint8_t pb1_long_press;
extern volatile uint8_t pb2_event;
extern volatile uint8_t pb3_event;
extern volatile uint8_t timer_flag;
extern volatile uint8_t system_state;  // 0 = OFF, 1 = ON
extern volatile uint8_t active_led;    // 1 = LED1, 2 = LED2
extern volatile uint8_t uart_transmit_mode;  // 0 = off, 1 = transmitting
extern volatile uint32_t millisecond_counter;  // For timestamping

// Software PWM variables
volatile uint16_t pwm_duty_cycle = 0;  // 0-PWM_PERIOD
volatile uint16_t pwm_counter = 0;
volatile uint8_t current_led_state = 0;  // Actual LED state (0 or 1)

#define PWM_PERIOD 60  // 60 timer interrupts = 1 PWM cycle (~67Hz PWM at 4kHz timer)

// Long press detection for PB1 in main loop
uint16_t button_hold_counter = 0;
uint8_t button_was_held = 0;

// Blinking state variables
volatile uint8_t blink_mode = 0;  // 0 = no blink, 1 = blinking
volatile uint16_t blink_counter = 0;  // Counts timer interrupts for blink timing
volatile uint8_t blink_state = 0;  // 0 = LED off phase, 1 = LED on phase

// Current ADC value (updated regularly)
volatile uint16_t current_adc_value = 0;

// Function prototypes
uint8_t get_current_intensity_percent(void);
void update_status_line(void);

// -----------------------------------------------------------------------------
// Helper: Update the single status line
// -----------------------------------------------------------------------------
void update_status_line(void) {
    char buf[100];
    uint8_t intensity_percent = get_current_intensity_percent();
    
    if (system_state == 0) {
        // OFF MODE
        if (blink_mode) {
            sprintf(buf, "\rOFF MODE (Blink) --- Intensity: %3u%%                    ", intensity_percent);
        } else {
            sprintf(buf, "\rOFF MODE --- Intensity: %3u%%                    ", intensity_percent);
        }
    } else {
        // ON MODE
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

// -----------------------------------------------------------------------------
void IOinit(void) {
    // Configure LED1 and LED2 as outputs
    TRISBbits.TRISB9 = 0;
    LATBbits.LATB9 = 0;
    TRISAbits.TRISA6 = 0;
    LATAbits.LATA6 = 0;
    
    // Configure PB1 (RB7) as input with pull-up
    TRISBbits.TRISB7 = 1;
    CNPU2bits.CN23PUE = 1;
    CNEN2bits.CN23IE = 1;
    
    // Configure PB2 (RB4) as input with pull-up
    TRISBbits.TRISB4 = 1;
    CNPU1bits.CN1PUE = 1;
    CNEN1bits.CN1IE = 1;
    
    // Configure PB3 (RA4) as input with pull-up
    TRISAbits.TRISA4 = 1;
    CNPU1bits.CN0PUE = 1;
    CNEN1bits.CN0IE = 1;
    
    // Read ports to clear mismatch
    (void)PORTB;
    (void)PORTA;
    
    // Configure CN interrupt
    IPC4bits.CNIP = 6;
    IFS1bits.CNIF = 0;
    IEC1bits.CNIE = 1;
    
    // Setup Timer3 for software PWM (fast interrupt for smooth PWM)
    T3CONbits.TON = 0;      // Turn off Timer3
    T3CONbits.TCS = 0;      // Use internal clock
    T3CONbits.TCKPS = 0b00; // 1:1 prescaler for high frequency PWM
    TMR3 = 0;
    PR3 = 122;              // ~4kHz timer interrupt at 500kHz clock
    IFS0bits.T3IF = 0;
    IPC2bits.T3IP = 7;      // Highest priority for smooth PWM
    IEC0bits.T3IE = 1;
    T3CONbits.TON = 1;      // Start Timer3
}

void set_led_intensity(uint16_t adc_value) {
    // Map ADC value (0-1023) to PWM duty cycle (0-PWM_PERIOD)
    pwm_duty_cycle = (uint16_t)((adc_value * (uint32_t)PWM_PERIOD) / 1023);
}

// Function to get current intensity as percentage (0-100%)
uint8_t get_current_intensity_percent(void) {
    if (blink_mode && blink_state == 0) {
        return 0;  // In blink off phase
    }
    
    if (system_state == 0) {
        // OFF mode: if blinking and in ON phase, show 100%
        if (blink_mode && blink_state == 1) {
            return 100;
        }
        return 0;  // System is off and not blinking
    }
    
    // ON mode: Calculate percentage based on current PWM duty cycle
    return (uint8_t)((pwm_duty_cycle * 100UL) / PWM_PERIOD);
}

void IOCheck(void) {
    uint16_t adc_value;
    uint8_t button_state = PORTBbits.RB7;
    char buf[80];
    static uint8_t long_press_detected = 0;
    
    // Monitor button hold time for PB1 (polled every 10ms)
    if (button_state == 0) {  // Button is pressed (active low)
        button_hold_counter++;
        
        // Check for long press threshold while button is held
        if (button_hold_counter >= 150 && system_state == 1) {
            long_press_detected = 1;
            pb1_event = 0;  // Clear any pending short press from CN interrupt
        }
    } else {
        // Button is released
        if (button_hold_counter > 0) {
            // Check if it was a long press
            if (long_press_detected) {
                // Long press - swap LEDs
                if (active_led == 1) {
                    active_led = 2;
                    LED1 = 0;  // Turn off LED1
                } else {
                    active_led = 1;
                    LED2 = 0;  // Turn off LED2
                }
                
                update_status_line();
                pb1_event = 0;  // Make sure short press doesn't trigger
            }
            // If it was a short press, pb1_event will be handled below
            
            // Reset counters
            button_hold_counter = 0;
            long_press_detected = 0;
        }
    }
    
    // Handle PB3 event - toggle UART transmission mode (ON MODE ONLY)
    if (pb3_event) {
        pb3_event = 0;
        
        // Only allow toggling in ON mode
        if (system_state == 1) {
            if (uart_transmit_mode == 0) {
                // Start UART transmission
                uart_transmit_mode = 1;
                millisecond_counter = 0;  // Reset timer
                
                // Print START_DATA on new line, then continue with status
                Disp2String("\r\nSTART_DATA\r\n");
            } else {
                // Stop UART transmission
                uart_transmit_mode = 0;
                Disp2String("\r\nSTOP_DATA\r\n");
            }
            update_status_line();
        }
    }
    
    // Handle PB2 event - toggle blink mode (BOTH ON AND OFF MODE)
    if (pb2_event) {
        pb2_event = 0;
        blink_mode = !blink_mode;
        blink_counter = 0;
        blink_state = blink_mode;  // Start with LED on if starting blink
        
        update_status_line();
        
        // Turn off LEDs if system is OFF and stopping blink
        if (system_state == 0 && !blink_mode) {
            LED1 = 0;
            LED2 = 0;
        }
    }
    
    // Handle long press event for PB1 (ON MODE ONLY - swap LEDs)
    // Note: Long press is now handled in the polling section above
    if (pb1_long_press) {
        pb1_long_press = 0;  // Clear the flag from CN interrupt if set
    }
    
    // Handle button press event for PB1 (short press - TOGGLE ON/OFF)
    if (pb1_event) {
        pb1_event = 0;
        
        // Toggle system state
        if (system_state == 0) {
            system_state = 1;  // Turn ON
            active_led = 1;    // Start with LED1
        } else {
            system_state = 0;  // Turn OFF
            LED1 = 0;
            LED2 = 0;
            pwm_duty_cycle = 0;
            uart_transmit_mode = 0;
        }
        update_status_line();
    }
    
    // Update LED intensity based on potentiometer when system is ON
    if (system_state == 1 && timer_flag) {
        timer_flag = 0;
        adc_value = do_ADC();
        current_adc_value = adc_value;
        set_led_intensity(adc_value);
        
        // Update status line (only if NOT transmitting data)
        if (!uart_transmit_mode) {
            update_status_line();
        }
    }
    
    // Transmit data over UART if in transmission mode (ON MODE ONLY)
    if (uart_transmit_mode && system_state == 1) {
        // Get current intensity percentage
        uint8_t intensity_percent = get_current_intensity_percent();
        
        // Format: TIMESTAMP,ADC_VALUE,INTENSITY_PERCENT
        // Print on new line to not interfere with status line
        sprintf(buf, "\r\n%lu,%u,%u", millisecond_counter, current_adc_value, intensity_percent);
        Disp2String(buf);
        
        // Check if 1 minute has elapsed (60000 ms)
        if (millisecond_counter >= 60000) {
            uart_transmit_mode = 0;
            Disp2String("\r\nSTOP_DATA\r\n");
            update_status_line();
        }
    }
}

// Software PWM interrupt handler (called by Timer3)
void handle_pwm_interrupt(void) {
    static uint16_t ms_counter = 0;
    
    // Increment millisecond counter (at 4kHz, every 4 interrupts = 1ms)
    ms_counter++;
    if (ms_counter >= 4) {
        ms_counter = 0;
        if (uart_transmit_mode) {
            millisecond_counter++;
        }
    }
    
    // Handle blinking timing (500ms on, 500ms off)
    if (blink_mode) {
        blink_counter++;
        if (blink_counter >= 2000) {  // 500ms elapsed
            blink_counter = 0;
            blink_state = !blink_state;
        }
    }
    
    // Determine if LED should be on based on blink state
    uint8_t led_should_be_on = 1;
    
    if (blink_mode && blink_state == 0) {
        led_should_be_on = 0;
    }
    
    if (system_state == 1) {
        // System ON - use current PWM duty cycle
        if (led_should_be_on && pwm_counter < pwm_duty_cycle) {
            // LED ON at current intensity
            if (active_led == 1) {
                LED1 = 1;
                LED2 = 0;
            } else {
                LED1 = 0;
                LED2 = 1;
            }
            current_led_state = 1;
        } else {
            // LED OFF
            if (active_led == 1) {
                LED1 = 0;
            } else {
                LED2 = 0;
            }
            current_led_state = 0;
        }
        
        // Increment and wrap counter
        pwm_counter++;
        if (pwm_counter >= PWM_PERIOD) {
            pwm_counter = 0;
        }
    } else {
        // System OFF - only blink at 100% if in blink mode
        if (blink_mode && blink_state == 1) {
            if (active_led == 1) {
                LED1 = 1;
                LED2 = 0;
            } else {
                LED1 = 0;
                LED2 = 1;
            }
            current_led_state = 1;
        } else {
            LED1 = 0;
            LED2 = 0;
            current_led_state = 0;
        }
        pwm_counter = 0;
    }
}
