/*
 * File:   IOs.h
 * Author: Rinad Hamid, Simar Kandola, Abia Jahangir

 */

#ifndef IOS_H
#define IOS_H

#include <xc.h>
#include <stdint.h>

// --- External global variables ---
extern volatile uint8_t pb1_event;
extern volatile uint8_t timer_flag;
extern volatile uint8_t mode_changed;
extern volatile uint16_t sleep_flag;

// --- System mode enumeration ---
typedef enum {
    MODE_0_BARGRAPH = 0,
    MODE_1_STREAM = 1
} SystemState;

extern volatile SystemState current_mode;

// --- Function prototypes ---
void IOinit(void);
void IOCheck(void);
void display_bargraph(uint16_t adc_value);

#endif // IOS_H
