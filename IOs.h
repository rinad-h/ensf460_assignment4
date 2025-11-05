// IOs.h
#ifndef IOS_H
#define IOS_H

#include <xc.h>
#include <stdint.h>

// External variables defined in main.c
extern uint16_t PB1_event, PB2_event, PB3_event;



// Function prototypes
void IOinit(void);
void IOcheck(void);

#endif // IOS_H
