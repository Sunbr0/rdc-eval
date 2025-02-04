/*
 * demodulation.h
 *
 *  Created on: Jan 29, 2025
 *      Author: davis
 */

#ifndef INC_DEMODULATION_H_
#define INC_DEMODULATION_H_

#include "main.h"

#define DEMOD_BUFFER_SIZE 20  // Number of samples per wave cycle

typedef struct {
    uint16_t buffer_sin[DEMOD_BUFFER_SIZE];   // Buffer for demodulated sine component
    uint16_t buffer_cos[DEMOD_BUFFER_SIZE];   // Buffer for demodulated cosine component
    const uint16_t *excitation_lookup;        // Pointer to excitation lookup table
    uint16_t index;  // Current write index in buffers
} Demodulation;

void demodulation_init(Demodulation *demod, const uint16_t *excitation_lookup);
void demodulation_process(Demodulation *demod, uint16_t *adc_sin, uint16_t *adc_cos);
uint16_t *demodulation_get_sin_buffer(Demodulation *demod);
uint16_t *demodulation_get_cos_buffer(Demodulation *demod);


#endif /* INC_DEMODULATION_H_ */
