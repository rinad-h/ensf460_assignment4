/*
 * File:   IOs.c
 * Author: Rinad Hamid, Simar Kandola, Abia Jahangir

 */

#include <xc.h>
#include <p24F16KA101.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "UART2.h"
#include "TimeDelay.h"
#include "ADC.h"
#include "IOs.h"

// Shared flags and variables from other modules
extern volatile uint8_t pb1_event;
extern volatile uint8_t timer_flag;
extern volatile uint8_t mode_changed;
extern volatile uint16_t sleep_flag;


//Draws a live bar graph based on ADC value
void display_bargraph(uint16_t adc_value) {
    char line[80];
    const int BAR_WIDTH = 32;
    int num_stars = (int)(((long)adc_value * BAR_WIDTH) / 1023);

    if (num_stars < 0) num_stars = 0;
    if (num_stars > BAR_WIDTH) num_stars = BAR_WIDTH;

    int pos = 0;
    line[pos++] = '\r';

    const char *prefix = "Mode 0: ";
    strcpy(&line[pos], prefix);
    pos += strlen(prefix);

    // Add stars for the filled portion
    for (int i = 0; i < num_stars && pos < sizeof(line) - 1; i++)
        line[pos++] = '*';
    
    // Add spaces for the remaining portion
    for (int i = num_stars; i < BAR_WIDTH && pos < sizeof(line) - 1; i++)
        line[pos++] = ' ';

    // Show ADC reading at the end
    pos += snprintf(&line[pos], sizeof(line) - pos, " 0x%03X", adc_value);
    line[pos] = '\0';

    Disp2String(line);
}

// Initialize input/output pins and enable CN interrupts for PB1
void IOinit(void) {
    TRISBbits.TRISB7 = 1;          // Set RB7 (PB1) as input
    CNPU2bits.CN23PUE = 1;         // Enable internal pull-up
    CNEN2bits.CN23IE = 1;          
    IPC4bits.CNIP = 6;             
    IFS1bits.CNIF = 0;             
    IEC1bits.CNIE = 1;             
}

//Handles all mode logic and user interactions
void IOCheck(void) {
    static uint8_t stream_started = 0;
    uint16_t adc_value;

    // Check if PB1 was pressed
    if (pb1_event) {
        pb1_event = 0;

        if (current_mode == MODE_0_BARGRAPH) {
            current_mode = MODE_1_STREAM;
            stream_started = 0;
            mode_changed = 1;
        } else {
            if (!stream_started) {
                stream_started = 1;
                Disp2String("STREAMING_START\n");
                delay_ms(100);
            } else {
                stream_started = 0;
                Disp2String("STREAMING_STOP\n");
                current_mode = MODE_0_BARGRAPH;
                mode_changed = 1;
            }
        }
    }

    //Handle transitions between modes
    if (mode_changed) {
        mode_changed = 0;

        if (current_mode == MODE_0_BARGRAPH) {
            Disp2String("\r\n\n*** MODE 0: Bar Graph Display ***\r\n");
        } else {
            Disp2String("\r\n\n*** MODE 1: Data Streaming ***\r\n");
            Disp2String("Press PB1 to START data stream...\r\n");
        }
    }

    //Mode 0: Display live ADC value as bar graph
    if (current_mode == MODE_0_BARGRAPH) {
        if (timer_flag) {
            timer_flag = 0;
            adc_value = do_ADC();
            display_bargraph(adc_value);
        }
    }

    //Mode 1: Stream ADC values continuously
    else if (current_mode == MODE_1_STREAM && stream_started) {
        while (stream_started && current_mode == MODE_1_STREAM) {
            adc_value = do_ADC();
            char buf[20];
            sprintf(buf, "%u\n", adc_value);
            Disp2String(buf);

            delay_ms(20);

            // Stop stream if button pressed again
            if (pb1_event) {
                pb1_event = 0;
                stream_started = 0;
                Disp2String("STREAMING_STOP\n");
                current_mode = MODE_0_BARGRAPH;
                mode_changed = 1;
                break;
            }
        }
    }
}
