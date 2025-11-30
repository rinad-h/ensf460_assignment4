#ifndef IOS_H
#define IOS_H

#include <xc.h>
#include <p24F16KA101.h>
#include <stdint.h>

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

extern volatile uint8_t pb1_release_event;
extern volatile uint16_t pb1_duration_timer;
extern volatile uint8_t pb2_release_event;
extern volatile uint8_t pb3_release_event;

uint8_t get_current_intensity_percent(void);
void update_status_line(void);
void set_led_intensity(uint16_t adc_value);
void IOinit(void);
void IOCheck(void);

#endif
