/*
 * File:   main.c
 * Author: Your Name
 * ADC Assignment - Mode 0 (Bar Graph) and Mode 1 (Data Streaming)
 * Created on: November 2025
 */

#include <xc.h>
#include <p24F16KA101.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "clkChange.h"
#include "UART2.h"
#include "TimeDelay.h"
#include "ADC.h"

// ========== CONFIGURATION BITS ==========
// FBS
#pragma config BWRP = OFF
#pragma config BSS = OFF
// FGS
#pragma config GWRP = OFF
#pragma config GCP = OFF
// FOSCSEL
#pragma config FNOSC = FRC
#pragma config IESO = OFF
// FOSC
#pragma config POSCMOD = NONE
#pragma config OSCIOFNC = ON
#pragma config POSCFREQ = HS
#pragma config SOSCSEL = SOSCHP
#pragma config FCKSM = CSECMD
// FWDT
#pragma config WDTPS = PS32768
#pragma config FWPSA = PR128
#pragma config WINDIS = OFF
#pragma config FWDTEN = OFF
// FPOR
#pragma config BOREN = BOR3
#pragma config PWRTEN = ON
#pragma config I2C1SEL = PRI
#pragma config BORV = V18
#pragma config MCLRE = ON
// FICD
#pragma config ICS = PGx2
// FDS
#pragma config DSWDTPS = DSWDTPSF
#pragma config DSWDTOSC = LPRC
#pragma config RTCOSC = SOSC
#pragma config DSBOREN = ON
#pragma config DSWDTEN = ON
// ========================================

// ----- State definitions -----
typedef enum {
    MODE_0_BARGRAPH = 0,
    MODE_1_STREAM = 1
} SystemState;

// ----- Global flags -----
volatile SystemState current_mode = MODE_0_BARGRAPH;
volatile uint8_t mode_changed = 0;  // Flag to indicate mode switch
volatile uint8_t pb1_event = 0;     // Flag for PB1 press (from CN ISR)
volatile uint8_t timer_flag = 0;  // Flag for 1Hz Timer (from Timer1 ISR)
volatile uint16_t sleep_flag = 0; // Flag for delay_ms (from Timer2 ISR)

// ========== FUNCTION PROTOTYPES ==========
void IOinit(void);
void setup_timer1(void);
void display_bargraph(uint16_t adc_value);

// ========== IMPLEMENTATIONS ==========

// ---- 1 Hz Timer ----
void setup_timer1(void) {
    T1CONbits.TON = 0;        // Disable Timer
    T1CONbits.TCS = 0;        // Select internal instruction cycle clock
    T1CONbits.TCKPS = 0b01;   // 1:8 prescaler (500kHz / 8 = 62.5 kHz)
    TMR1 = 0;                 // Clear timer register
    PR1 = 62499;              // Load period value for 1s interrupt (62500 - 1)
    IFS0bits.T1IF = 0;        // Clear Timer1 Interrupt Flag
    IEC0bits.T1IE = 1;        // Enable Timer1 interrupt
    T1CONbits.TON = 1;        // Start Timer
}

// ---- IO + Change Notification ----
void IOinit(void) {
    // Pin RA6 (PB1) is connected to RB7 (CN23) on the explorer board
    TRISBbits.TRISB7 = 1;        // Set RB7 (CN23) as input for PB1
    CNPU2bits.CN23PUE = 1;       // Enable pull-up resistor on CN23
    
    // Configure Change Notification for CN23
    CNEN2bits.CN23IE = 1;        // Enable CN interrupt for CN23
    (void)PORTB;                 // Read PORTB to clear mismatch
    
    // Configure CN Interrupt
    IPC4bits.CNIP = 6;           // Set CN interrupt priority (e.g., 6)
    IFS1bits.CNIF = 0;           // Clear CN interrupt flag
    IEC1bits.CNIE = 1;           // Enable CN interrupt
}

// ---- Display bar graph on one line ----
void display_bargraph(uint16_t adc_value) {
    char line[80];
    const int BAR_WIDTH = 32; // Width of the bar graph
    
    // Scale 10-bit ADC value (0-1023) to bar width
    int num_stars = (int)(((long)adc_value * BAR_WIDTH) / 1023); 
    
    // Clamp values to be safe
    if (num_stars < 0) num_stars = 0;
    if (num_stars > BAR_WIDTH) num_stars = BAR_WIDTH;

    int pos = 0;
    // \r returns to the start of the line without a new line
    line[pos++] = '\r'; 

    // Add prefix
    const char *prefix = "Mode 0: ";
    strcpy(&line[pos], prefix);
    pos += strlen(prefix);

    // Add stars
    for (int i = 0; i < num_stars && pos < sizeof(line) - 1; i++) {
        line[pos++] = '*';
    }
    // Add padding spaces
    for (int i = num_stars; i < BAR_WIDTH && pos < sizeof(line) - 1; i++) {
        line[pos++] = ' ';
    }

    // Add hex value, ensuring 3 digits (0x000 to 0x3FF)
    pos += snprintf(&line[pos], sizeof(line) - pos, " 0x%03X", adc_value);
    line[pos] = '\0'; // Null-terminate the string

    Disp2String(line); // Send the full line to UART
}

// IOCheck function --> move out later
// IOCheck function (FINAL VERSION)
void IOCheck(void) {
    // Retains state between calls. Needs to be static.
    static uint8_t stream_started = 0; 
    uint16_t adc_value;
    
    // ---- Handle PB1 presses (State TOGGLE logic) ----
    if (pb1_event) {
        pb1_event = 0; // Consume the event immediately
        
        if (current_mode == MODE_0_BARGRAPH) {
            // Mode 0 -> Mode 1 (Always changes mode)
            current_mode = MODE_1_STREAM;
            mode_changed = 1;
        } 
        else { // current_mode == MODE_1_STREAM
            // Mode 1: Pressing PB1 toggles the streaming state
            if (stream_started == 0) {
                // Not streaming: Start streaming
                stream_started = 1;
                Disp2String("STREAMING_START\r\n"); // <--- Print START here
            } else {
                // Already streaming: Stop stream and return to Mode 0
                stream_started = 0; // Explicitly stop streaming
                current_mode = MODE_0_BARGRAPH;
                mode_changed = 1;
                Disp2String("STREAMING_STOP\r\n"); // Optional: Add a stop message
            }
        }
    }

    // --- Handle Mode Transition (Prints only, no logic) ---
    if (mode_changed) {
        mode_changed = 0;
        // Crucial: Reset stream_started on mode change (if it wasn't done above)
        // In the new logic, it's done when we transition out of mode 1.
        stream_started = 0; 
        
        if (current_mode == MODE_0_BARGRAPH) {
            Disp2String("\r\n\n*** MODE 0: Bar Graph Display ***\r\n");
        } else {
            Disp2String("\r\n\n*** MODE 1: Data Streaming ***\r\n");
            Disp2String("Press PB1 to START data stream...\r\n");
        }
    }

    // ---- Mode 0: Bar Graph Logic ----
    if (current_mode == MODE_0_BARGRAPH) {
        if (timer_flag) {
            timer_flag = 0;
            adc_value = do_ADC();
            display_bargraph(adc_value);
        }
    }

    // ---- Mode 1: Streaming Logic (Simplified) ----
    else if (current_mode == MODE_1_STREAM) {
        // Only executes ADC and stream if stream_started is 1
        if (stream_started) {
            adc_value = do_ADC();
            char buf[20];
            sprintf(buf, "%u\r\n", adc_value); 
            Disp2String(buf);
            delay_ms(1000); 
        }
    }
}
// ========== MAIN ==========

int main(void) {
    
   
    // --- System Initialization ---
    newClk(500); // 500 kHz clock
    
    // --- Analog setup (AN12/RB15) ---
    AD1PCFG = 0xFFFF;
    AD1PCFGbits.PCFG12 = 0;
    
    IOinit();
    InitUART2();
    init_ADC();
    setup_timer1();
   
    delay_ms(10);
    Disp2String("\r\n*** MODE 0: Bar Graph Display ***\r\n");

    while (1) {
        Idle();      // Sleep until interrupt wakes us
        delay_ms(50);  // Handle debouncing if button pressed
        IOCheck();   // Process all pending events
    }

    return 0;
}

// ========== INTERRUPT SERVICE ROUTINES ==========

// Timer1 ISR (1 Hz)
void __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void) {
    IFS0bits.T1IF = 0; // Clear Timer1 interrupt flag
    timer_flag = 1;    // Set 1Hz flag for main loop
}

// Timer2 ISR (used by TimeDelay.c)
void __attribute__((interrupt, no_auto_psv)) _T2Interrupt(void) {
    IFS0bits.T2IF = 0; // Clear Timer2 interrupt flag
    T2CONbits.TON = 0; // Stop Timer2
    sleep_flag = 1;    // Set flag for delay_ms
}

// Change Notification ISR (PB1)
void __attribute__((interrupt, no_auto_psv)) _CNInterrupt(void) {
    IFS1bits.CNIF = 0; // Clear CN interrupt flag
    pb1_event = 1;     // Set button press flag for main loop
}
