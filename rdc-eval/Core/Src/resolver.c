/*
 * resolver.c
 *
 *  Created on: Jan 29, 2025
 *      Author: davis
 */

#include "resolver.h"

void resolver_init(Resolver *self,
                   DAC_HandleTypeDef *hdac,
                   ADC_HandleTypeDef *hadc,
                   TIM_HandleTypeDef *htim_excitation,
                   TIM_HandleTypeDef *htim_adc,
                   const uint16_t *lookup_table) {
    self->hdac = hdac;
    self->hadc = hadc;
    self->htim_excitation = htim_excitation;
    self->htim_adc = htim_adc;
    self->lookup_table = lookup_table;
    self->freq_ratio = 1;  // Default frequency ratio, can be changed later

    // Initialize quadrature buffer
	self->resolver_quadrature.active_buffer = self->resolver_quadrature.adc_buffer_1;
	self->resolver_quadrature.idle_buffer = self->resolver_quadrature.adc_buffer_2;
	self->resolver_quadrature.state = ADC_IDLE;


    // Initialize DAC output
    HAL_DAC_Start_DMA(self->hdac, DAC_CHANNEL_1, (uint32_t *)self->lookup_table,
    				  LOOKUP_TABLE_SIZE, DAC_ALIGN_12B_R);

    // Initialize ADC input
    HAL_ADC_Start_DMA(self->hadc, (uint32_t *)self->resolver_quadrature.active_buffer, ADC_BUFFER_SIZE);
}


void resolver_start(Resolver *self) {
    // Start excitation signal
    HAL_TIM_Base_Start(self->htim_excitation);

    // Start ADC conversion
    HAL_TIM_Base_Start(self->htim_adc);
}

// Get the next position in the lookup table
uint16_t excitation_get_position(Resolver *self) {
	uint16_t remaining = __HAL_DMA_GET_COUNTER(self->hdac->DMA_Handle1);
	return (LOOKUP_TABLE_SIZE - remaining) % LOOKUP_TABLE_SIZE;
}

uint16_t excitation_get_value(Resolver *self, uint16_t position) {
    if (position < LOOKUP_TABLE_SIZE) {
        return self->lookup_table[position];
    }
    return 0; // Return 0 if out of range
}

uint16_t get_adc_position(Resolver *self) {
    uint16_t remaining = __HAL_DMA_GET_COUNTER(self->hadc->DMA_Handle);
    return (ADC_BUFFER_SIZE - remaining) % ADC_BUFFER_SIZE;
}

void resolver_handle_adc_callback(Resolver *self, uint8_t is_half_transfer) {
    if (!self) return;

    if (is_half_transfer) {
        // Half transfer: First half of the active buffer is being written
        self->resolver_quadrature.state = (self->resolver_quadrature.state == ADC_ACTIVE_1) ? ADC_ACTIVE_2 : ADC_ACTIVE_1;

    } else {
        // Full transfer: Second half of the active buffer is being written
        self->resolver_quadrature.state = (self->resolver_quadrature.state == ADC_ACTIVE_3) ? ADC_ACTIVE_4 : ADC_ACTIVE_3;

        // Swap buffers after full transfer
        HAL_ADC_Stop_DMA(self->hadc);
        uint16_t *temp = self->resolver_quadrature.active_buffer;
        self->resolver_quadrature.active_buffer = self->resolver_quadrature.idle_buffer;
        self->resolver_quadrature.idle_buffer = temp;
        HAL_ADC_Start_DMA(self->hadc, (uint32_t *)self->resolver_quadrature.active_buffer, ADC_BUFFER_SIZE);
    }
}
