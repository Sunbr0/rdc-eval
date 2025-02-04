/*
 * excitation.c
 *
 *  Created on: Jan 29, 2025
 *      Author: davis
 */

#include "excitation.h"

#define SINE_LOOKUP_TABLE_SIZE 256

// Private Variables
//static const uint16_t sine_lookup_table[SINE_LOOKUP_TABLE_SIZE] = {
//		0x73a, 0x75b, 0x77c, 0x79d, 0x7be, 0x7df, 0x800, 0x821,
//		0x841, 0x862, 0x882, 0x8a2, 0x8c2, 0x8e1, 0x901, 0x920,
//		0x93f, 0x95d, 0x97b, 0x999, 0x9b6, 0x9d3, 0x9f0, 0xa0c,
//		0xa28, 0xa43, 0xa5e, 0xa79, 0xa92, 0xaac, 0xac5, 0xadd,
//		0xaf5, 0xb0c, 0xb22, 0xb38, 0xb4e, 0xb62, 0xb76, 0xb8a,
//		0xb9c, 0xbaf, 0xbc0, 0xbd1, 0xbe1, 0xbf0, 0xbfe, 0xc0c,
//		0xc19, 0xc26, 0xc31, 0xc3c, 0xc46, 0xc4f, 0xc58, 0xc5f,
//		0xc66, 0xc6c, 0xc71, 0xc76, 0xc79, 0xc7c, 0xc7e, 0xc80,
//		0xc80, 0xc80, 0xc7e, 0xc7c, 0xc79, 0xc76, 0xc71, 0xc6c,
//		0xc66, 0xc5f, 0xc58, 0xc4f, 0xc46, 0xc3c, 0xc31, 0xc26,
//		0xc19, 0xc0c, 0xbfe, 0xbf0, 0xbe1, 0xbd1, 0xbc0, 0xbaf,
//		0xb9c, 0xb8a, 0xb76, 0xb62, 0xb4e, 0xb38, 0xb22, 0xb0c,
//		0xaf5, 0xadd, 0xac5, 0xaac, 0xa92, 0xa79, 0xa5e, 0xa43,
//		0xa28, 0xa0c, 0x9f0, 0x9d3, 0x9b6, 0x999, 0x97b, 0x95d,
//		0x93f, 0x920, 0x901, 0x8e1, 0x8c2, 0x8a2, 0x882, 0x862,
//		0x841, 0x821, 0x800, 0x7df, 0x7be, 0x79d, 0x77c, 0x75b,
//		0x73a, 0x719, 0x6f8, 0x6d7, 0x6b6, 0x695, 0x674, 0x653,
//		0x633, 0x612, 0x5f2, 0x5d2, 0x5b2, 0x593, 0x573, 0x554,
//		0x535, 0x517, 0x4f9, 0x4db, 0x4be, 0x4a1, 0x484, 0x468,
//		0x44c, 0x431, 0x416, 0x3fb, 0x3e2, 0x3c8, 0x3af, 0x397,
//		0x37f, 0x368, 0x352, 0x33c, 0x326, 0x312, 0x2fe, 0x2ea,
//		0x2d8, 0x2c5, 0x2b4, 0x2a3, 0x293, 0x284, 0x276, 0x268,
//		0x25b, 0x24e, 0x243, 0x238, 0x22e, 0x225, 0x21c, 0x215,
//		0x20e, 0x208, 0x203, 0x1fe, 0x1fb, 0x1f8, 0x1f6, 0x1f4,
//		0x1f4, 0x1f4, 0x1f6, 0x1f8, 0x1fb, 0x1fe, 0x203, 0x208,
//		0x20e, 0x215, 0x21c, 0x225, 0x22e, 0x238, 0x243, 0x24e,
//		0x25b, 0x268, 0x276, 0x284, 0x293, 0x2a3, 0x2b4, 0x2c5,
//		0x2d8, 0x2ea, 0x2fe, 0x312, 0x326, 0x33c, 0x352, 0x368,
//		0x37f, 0x397, 0x3af, 0x3c8, 0x3e2, 0x3fb, 0x416, 0x431,
//		0x44c, 0x468, 0x484, 0x4a1, 0x4be, 0x4db, 0x4f9, 0x517,
//		0x535, 0x554, 0x573, 0x593, 0x5b2, 0x5d2, 0x5f2, 0x612,
//		0x633, 0x653, 0x674, 0x695, 0x6b6, 0x6d7, 0x6f8, 0x719
//};

// Initialize the Excitation object
//void excitation_init(Excitation *self, DAC_HandleTypeDef *hdac, TIM_HandleTypeDef *htim) {
//    self->lookup_table = sine_lookup_table;
//    self->current_position = 0;
//    self->hdac = hdac;
//    self->htim = htim;
//
//    // Assign method pointers
//    self->get_position = &excitation_get_position;
//    self->get_value = &excitation_get_value;
//
//    // Start DAC with DMA
//    HAL_DAC_Start_DMA(self->hdac, DAC_CHANNEL_1, (uint32_t *)self->lookup_table,
//                      SINE_LOOKUP_TABLE_SIZE, DAC_ALIGN_12B_R);
//
//    // Start DAC timer (~10kHz)
//    HAL_TIM_Base_Start(self->htim);
//}

//// Get the next position in the lookup table and update the index
//uint16_t excitation_get_position(Excitation *self) {
//    self->current_position = (self->current_position + 1) % SINE_LOOKUP_TABLE_SIZE;
//    return self->current_position;
//}

// Get excitation value at a specific lookup table position
//uint16_t excitation_get_value(Excitation *self, uint16_t position) {
//    if (position < SINE_LOOKUP_TABLE_SIZE) {
//        return self->lookup_table[position];
//    }
//    return 0; // Return 0 if out of range
//}
