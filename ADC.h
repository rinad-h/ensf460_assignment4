/* * File:   ADC.h
 * Author: Your Name
 *
 * Created on November 2025
 */

#ifndef ADC_H
#define	ADC_H

#include <stdint.h> // For uint16_t

extern volatile uint16_t sleep_flag;

#ifdef	__cplusplus
extern "C" {
#endif

/**
 * @brief Initializes the ADC module.
 * Configures ADC for AN12 (Pin 15) with auto-conversion.
 */
void init_ADC(void);

/**
 * @brief Performs a single ADC conversion.
 * Starts sampling, waits for conversion, and returns the 10-bit result.
 * @return 10-bit ADC value (0-1023).
 */
uint16_t do_ADC(void);


#ifdef	__cplusplus
}
#endif

#endif	/* ADC_H */
