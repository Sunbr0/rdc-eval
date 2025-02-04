/*
 * demodulation.c
 *
 *  Created on: Jan 29, 2025
 *      Author: davis
 */

#include "demodulation.h"

#define SINE_LOOKUP_TABLE_SIZE 256  // Lookup table size (must match excitation table)


// Initialize demodulation structure
void demodulation_init(Demodulation *demod, const uint16_t *excitation_lookup) {
    demod->index = 0;
    demod->excitation_lookup = excitation_lookup;
}

// Perform demodulation on **entire ADC DMA buffers**
void demodulation_process(Demodulation *demod, uint16_t *adc_sin, uint16_t *adc_cos) {
    for (uint16_t i = 0; i < DEMOD_BUFFER_SIZE; i++) {
        uint16_t pos = (demod->index + i) % SINE_LOOKUP_TABLE_SIZE;  // Ensure valid lookup index
        uint16_t cos_pos = (pos + SINE_LOOKUP_TABLE_SIZE / 4) % SINE_LOOKUP_TABLE_SIZE;  // 90° phase shift

        int16_t ref_sin = demod->excitation_lookup[pos];
        int16_t ref_cos = demod->excitation_lookup[cos_pos];

        // Multiply resolver outputs with reference waves
        demod->buffer_sin[i] = (adc_sin[i] * ref_sin) >> 12;  // Scale down
        demod->buffer_cos[i] = (adc_cos[i] * ref_cos) >> 12;  // Scale down
    }

    // Update index for circular buffer
    demod->index = (demod->index + DEMOD_BUFFER_SIZE) % SINE_LOOKUP_TABLE_SIZE;
}


// Get buffer pointers (unchanged)
uint16_t *demodulation_get_sin_buffer(Demodulation *demod) { return demod->buffer_sin; }
uint16_t *demodulation_get_cos_buffer(Demodulation *demod) { return demod->buffer_cos; }
