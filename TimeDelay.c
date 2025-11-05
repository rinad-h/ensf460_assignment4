    /*
     * File:   TimeDelay.c
     * Author: Rinad Hamid, Simar Kandola, Abia Jahangir
     *
     * Created on: October 2, 2025
     */

#include <xc.h>
#include <stdint.h>
#include "TimeDelay.h"
#include "clkChange.h"

extern uint16_t sleep_flag;
void delay_ms(uint16_t time_ms){
    
    
    //calculate the total number of instruction cycles for the delay
     uint32_t cycles = (uint32_t)time_ms * 250;  // 250 cycles per ms at 250kHz Fcy

    //select appropriate prescaler and PR2 value based on the number of cycles
    if (cycles <= 65535) {
        T2CONbits.TCKPS = 00;          // Prescaler 1:1
        PR2 = cycles - 1;
    }
    else if (cycles / 8 <= 65535) {
        T2CONbits.TCKPS = 01;          // Prescaler 1:8
        PR2 = (cycles / 8) - 1;
    }
    else if (cycles / 64 <= 65535) {
        T2CONbits.TCKPS = 10;          // Prescaler 1:64
        PR2 = (cycles / 64) - 1;
    }
    else {
        T2CONbits.TCKPS = 11;          // Prescaler 1:256
        PR2 = (cycles / 256) - 1;
    }

     //Timer2 configuration
    T2CONbits.TSIDL = 0;        //Continue timer in idle mode
    T2CONbits.T32 = 0;          // Use 16-bit timer mode
    T2CONbits.TCS = 0;          // Use internal clock
    
    TMR2 = 0;                   // Clear timer register
    
    T2CONbits.TON = 1;          // Turn on Timer2

     //configure interrupt
    IPC1bits.T2IP = 7;        //set highest interrupt priority       
    IFS0bits.T2IF = 0;        //clear interrupt flag
    IEC0bits.T2IE = 1;        //enable Timer2 interrupt

    //wait for ISR to set the flag
    while (!sleep_flag){
        Idle();
    }
    //Reset flag for next use
    sleep_flag = 0;
    return;
}
