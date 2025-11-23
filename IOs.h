/*
 * File:   IOs.h
 * Author: Rinad Hamid, Simar Kandola, Abia Jahangir

 */

#ifndef IOS_H
#define IOS_H

#include <xc.h>
#include <stdint.h>

// --- External global variables from ios.c ---
extern volatile uint8_t pb1_event;
extern volatile uint8_t pb1_long_press;
extern volatile uint8_t timer_flag;
extern volatile uint8_t system_state;    // 0 = OFF, 1 = ON
extern volatile uint8_t active_led;      // 1 = LED1, 2 = LED2

// --- Function prototypes ---
void IOinit(void);
void IOCheck(void);
void set_led_intensity(uint16_t adc_value);
void handle_pwm_interrupt(void);

#endif  // IOS_H
