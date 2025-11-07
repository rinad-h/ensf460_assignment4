#include "IOs.h"
#include <stdio.h>
#include <string.h>
#include "UART2.h"
#include "ADC.h"
#include "TimeDelay.h"

void IOinit(void) {
    // Pin RA6 (PB1) is connected to RB7 (CN23) on the explorer board
    TRISBbits.TRISB7 = 1;        // Set RB7 (CN23) as input for PB1
    CNPU2bits.CN23PUE = 1;       // Enable pull-up resistor on CN23

    // Configure Change Notification for CN23
    CNEN2bits.CN23IE = 1;        // Enable CN interrupt for CN23
    (void)PORTB;                 // Read PORTB to clear mismatch

    // Configure CN Interrupt
    IPC4bits.CNIP = 6;           // Set CN interrupt priority (6)
    IFS1bits.CNIF = 0;           // Clear CN interrupt flag
    IEC1bits.CNIE = 1;           // Enable CN interrupt
}

void IOCheck(void) {
    static uint8_t stream_started = 0;
    uint16_t adc_value;

    if (pb1_event) {
        pb1_event = 0;

        if (current_mode == MODE_0_BARGRAPH) {
            current_mode = MODE_1_STREAM;
            mode_changed = 1;
        } else {
            if (stream_started == 0) {
                stream_started = 1;
                Disp2String("STREAMING_START\r\n");
            } else {
                stream_started = 0;
                current_mode = MODE_0_BARGRAPH;
                mode_changed = 1;
                Disp2String("STREAMING_STOP\r\n");
            }
        }
    }

    if (mode_changed) {
        mode_changed = 0;
        stream_started = 0;

        if (current_mode == MODE_0_BARGRAPH) {
            Disp2String("\r\n\n*** MODE 0: Bar Graph Display ***\r\n");
        } else {
            Disp2String("\r\n\n*** MODE 1: Data Streaming ***\r\n");
            Disp2String("Press PB1 to START data stream...\r\n");
        }
    }

    if (current_mode == MODE_0_BARGRAPH) {
        if (timer_flag) {
            timer_flag = 0;
            adc_value = do_ADC();
            display_bargraph(adc_value);
        }
    } else if (current_mode == MODE_1_STREAM) {
        if (stream_started) {
            adc_value = do_ADC();
            char buf[20];
            sprintf(buf, "%u\r\n", adc_value);
            Disp2String(buf);
            delay_ms(1000);
        }
    }
}
