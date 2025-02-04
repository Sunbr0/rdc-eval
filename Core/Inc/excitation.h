/*
 * excitation.h
 *
 *  Created on: Jan 29, 2025
 *      Author: davis
 */

#ifndef INC_EXCITATION_H_
#define INC_EXCITATION_H_

#include "main.h"

#define SINE_LOOKUP_TABLE_SIZE 256

// Define Excitation "class"
typedef struct Excitation {
    const uint16_t *lookup_table;  // Pointer to lookup table
    uint16_t current_position;     // Tracks lookup table position
    DAC_HandleTypeDef *hdac;       // Pointer to DAC handler
    TIM_HandleTypeDef *htim;       // Pointer to Timer handler

    // Methods (function pointers)
    uint16_t (*get_position)(struct Excitation *);
    uint16_t (*get_value)(struct Excitation *, uint16_t);
} Excitation;

// Public functions
void excitation_init(Excitation *self, DAC_HandleTypeDef *hdac, TIM_HandleTypeDef *htim);
uint16_t excitation_get_position(Excitation *self);
uint16_t excitation_get_value(Excitation *self, uint16_t position);

#endif /* INC_EXCITATION_H_ */
