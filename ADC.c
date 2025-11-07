/*
 * File:   ADC.c
 * Author: Rinad Hamid, Simar Kandola, Abia Jahangir
 * ADC driver functions for PIC24F
 */

#include <xc.h>
#include <p24F16KA101.h>
#include <stdint.h>
#include "ADC.h"
#include "TimeDelay.h"

extern volatile uint16_t sleep_flag;

// Initialize ADC module
void init_ADC(void) {
    // AD1CON1 register configuration
    AD1CON1bits.ADON = 0;       // Turn off ADC during configuration
    AD1CON1bits.ADSIDL = 0;     // Continue in idle mode
    AD1CON1bits.FORM = 0b00;    // Integer output format (0000 00dd dddd dddd)
    AD1CON1bits.SSRC = 0b111;   // Auto-convert (internal counter ends sampling)
    AD1CON1bits.ASAM = 0;       // Manual sampling start (we'll set SAMP bit)
    
    // AD1CON2 register configuration
    AD1CON2bits.VCFG = 0b000;   // Use AVdd and AVss as reference (VDD and VSS)
    AD1CON2bits.CSCNA = 0;      // Don't scan inputs, use channel selected in AD1CHS
    AD1CON2bits.BUFM = 0;       // Single 16-word buffer
    AD1CON2bits.ALTS = 0;       // Always use MUXA
    AD1CON2bits.SMPI = 0;       // Interrupt after every 1 sample
    
    // AD1CON3 register configuration
    AD1CON3bits.ADRC = 0;       // Use system clock
    AD1CON3bits.SAMC = 0b11111; // 31 TAD auto-sample time (longer = more stable)
    AD1CON3bits.ADCS = 0b111111; // 64 * TCY (TAD = TCY * 64)
                                // At 500kHz, TCY = 2us. TAD = 128us.
                                // Total conversion = (31+12)*TAD = 43*128us = ~5.5ms
    
    // AD1CHS register - Select input channel
    AD1CHSbits.CH0NA = 0;       // MUXA negative input = VREFL (VSS)
    AD1CHSbits.CH0SA = 0b01100; // MUXA positive input = AN12 (pin 15, RB15)
                                // 0b01100 is 12 decimal
    
    // Turn on ADC
    AD1CON1bits.ADON = 1;
    
    // Wait for ADC to stabilize (small delay)
    delay_ms(500);
}

// Perform ADC conversion and return result
uint16_t do_ADC(void) {
    uint16_t adc_result;
    
    // Ensure previous conversion is complete
    AD1CON1bits.DONE = 0;
    
    // Start sampling
    AD1CON1bits.SAMP = 1;
    
    // Wait for auto-sample time (SAMC) to complete
    // With SSRC=111, SAMP bit is cleared automatically
    while (AD1CON1bits.SAMP == 1);
    
    // Now conversion is happening automatically
    // Wait for conversion to complete (DONE bit set)
    while (AD1CON1bits.DONE == 0);
    
    // Read the conversion result from the buffer
    adc_result = ADC1BUF0;
    
    // Return the 10-bit result
    return adc_result;
}
