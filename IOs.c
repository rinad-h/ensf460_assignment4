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
extern volatile uint8_t timer_flag;
extern volatile uint8_t system_state;  // 0 = OFF, 1 = ON
extern volatile uint8_t active_led;    // 1 = LED1, 2 = LED2

// Add PB2 event flag
extern volatile uint8_t pb2_event;

// Software PWM variables
volatile uint16_t pwm_duty_cycle = 0;  // 0-PWM_PERIOD
volatile uint16_t pwm_counter = 0;

#define PWM_PERIOD 60  // 50 timer interrupts = 1 PWM cycle (~80Hz PWM at 4kHz timer)

// Long press detection for PB1 in main loop
uint16_t button_hold_counter = 0;
uint8_t button_was_held = 0;

// Blinking state variables
volatile uint8_t blink_mode = 0;  // 0 = no blink, 1 = blinking
volatile uint16_t blink_counter = 0;  // Counts timer interrupts for blink timing
volatile uint8_t blink_state = 0;  // 0 = LED off phase, 1 = LED on phase

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
    
    // Read port to clear mismatch
    (void)PORTB;
    
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
    // Using 32-bit math to prevent overflow
    pwm_duty_cycle = (uint16_t)((adc_value * (uint32_t)PWM_PERIOD) / 1023);
}

void IOCheck(void) {
    uint16_t adc_value;
    uint8_t button_state = PORTBbits.RB7;
    
    // Monitor button hold time for PB1 (polled every 10ms)
    if (button_state == 0) {  // Button is pressed (active low)
        button_hold_counter++;
        button_was_held = 1;
    } else {
        // Button is released
        if (button_was_held) {
            // Check how long it was held
            if (button_hold_counter >= 150 && system_state == 1) {  // 150 * 10ms = 1.5 seconds
                // Long press detected - swap LEDs
                pb1_long_press = 1;
            }
            // If it was a short press, pb1_event will already be set by CN interrupt
            
            // Reset counters
            button_hold_counter = 0;
            button_was_held = 0;
        }
    }
    
    // Handle PB2 event - toggle blink mode
    if (pb2_event) {
        pb2_event = 0;
        
        // Toggle blink mode
        if (blink_mode == 0) {
            // Start blinking
            blink_mode = 1;
            blink_counter = 0;
            blink_state = 1;  // Start with LED on
            
            if (system_state == 0) {
                Disp2String("\r\n*** Blinking at 100% intensity ***\r\n");
            } else {
                Disp2String("\r\n*** Blinking at current intensity ***\r\n");
            }
        } else {
            // Stop blinking
            blink_mode = 0;
            blink_counter = 0;
            blink_state = 0;
            
            // Turn off LEDs if system is OFF
            if (system_state == 0) {
                LED1 = 0;
                LED2 = 0;
            }
            
            Disp2String("\r\n*** Blinking stopped ***\r\n");
        }
    }
    
    // Handle long press event for PB1
    if (pb1_long_press) {
        pb1_long_press = 0;
        
        // Swap LEDs
        if (active_led == 1) {
            active_led = 2;
            LED1 = 0;  // Turn off LED1
            Disp2String("\r\n*** Switched to LED2 ***\r\n");
        } else {
            active_led = 1;
            LED2 = 0;  // Turn off LED2
            Disp2String("\r\n*** Switched to LED1 ***\r\n");
        }
        
        // Clear any pending short press event
        pb1_event = 0;
    }
    
    // Handle button press event for PB1 (short press)
    if (pb1_event) {
        pb1_event = 0;
        
        // Only process if it wasn't a long press
        if (button_hold_counter < 150) {
            // Toggle system state
            if (system_state == 0) {
                system_state = 1;  // Turn ON
                active_led = 1;    // Start with LED1
                Disp2String("\r\n*** SYSTEM ON - LED1 Active ***\r\n");
            } else {
                system_state = 0;  // Turn OFF
                LED1 = 0;  // Turn off both LEDs
                LED2 = 0;
                pwm_duty_cycle = 0;  // Reset PWM
                blink_mode = 0;  // Stop blinking when turning off
                Disp2String("\r\n*** SYSTEM OFF ***\r\n");
            }
        }
    }
    
    // Update LED intensity based on potentiometer when system is ON
    if (system_state == 1 && timer_flag) {
        timer_flag = 0;
        adc_value = do_ADC();
        
        // Control active LED intensity based on ADC value
        set_led_intensity(adc_value);
        
        // Display current values for debugging
        char buf[50];
        sprintf(buf, "LED%d - ADC: %4u, PWM: %2u/%2u\r", active_led, adc_value, pwm_duty_cycle, PWM_PERIOD);
        Disp2String(buf);
    }
}

// Software PWM interrupt handler (called by Timer3)
void handle_pwm_interrupt(void) {
    // Handle blinking timing (500ms on, 500ms off)
    // At ~4kHz timer rate, 2000 interrupts = 500ms
    if (blink_mode) {
        blink_counter++;
        if (blink_counter >= 2000) {  // 500ms elapsed
            blink_counter = 0;
            blink_state = !blink_state;  // Toggle between on and off
        }
    }
    
    // Determine if LED should be on based on blink state
    uint8_t led_should_be_on = 1;
    
    if (blink_mode) {
        if (blink_state == 0) {
            // Blink OFF phase - LED should be off
            led_should_be_on = 0;
        }
        // If blink_state == 1, LED should be on (will go through PWM logic below)
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
        } else {
            // LED OFF
            if (active_led == 1) {
                LED1 = 0;
            } else {
                LED2 = 0;
            }
        }
        
        // Increment and wrap counter
        pwm_counter++;
        if (pwm_counter >= PWM_PERIOD) {
            pwm_counter = 0;
        }
    } else {
        // System OFF - only blink at 100% if in blink mode
        if (blink_mode && blink_state == 1) {
            // Blink ON phase at 100% intensity
            if (active_led == 1) {
                LED1 = 1;
                LED2 = 0;
            } else {
                LED1 = 0;
                LED2 = 1;
            }
        } else {
            // Turn off LEDs
            LED1 = 0;
            LED2 = 0;
        }
        pwm_counter = 0;
    }
}
