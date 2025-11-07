#ifndef IOS_H
#define IOS_H

#include <xc.h>
#include <stdint.h>

extern volatile uint8_t pb1_event;
extern volatile uint8_t mode_changed;
extern volatile uint8_t timer_flag;
extern volatile uint16_t sleep_flag;

// Enumerations for system modes
typedef enum {
    MODE_0_BARGRAPH = 0,
    MODE_1_STREAM = 1
} SystemState;

extern volatile SystemState current_mode;

// ===== Function Prototypes =====
void IOinit(void);
void IOCheck(void);

#endif // IOS_H
