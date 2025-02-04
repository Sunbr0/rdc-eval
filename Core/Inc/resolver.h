/*
 * resolver.h
 *
 *  Created on: Jan 29, 2025
 *      Author: davis
 */

#ifndef INC_RESOLVER_H_
#define INC_RESOLVER_H_

#include "main.h"

typedef enum {
    ADC_IDLE,       	// Stopped
    ADC_ACTIVE_1,    	// ADC writing 1st half
	ADC_ACTIVE_2,     // ADC writing 2nd half
} ADCState;

typedef struct {
    uint16_t adc_buffer_1[ADC_BUFFER_SIZE];
    uint16_t adc_buffer_2[ADC_BUFFER_SIZE];
    ADCState state;
    uint16_t *active_buffer;
    uint16_t *idle_buffer;
} ResolverQuadrature;

typedef struct {
	uint16_t sine_buffer[ADC_BUFFER_SIZE];
	uint16_t cosine_buffer[ADC_BUFFER_SIZE];
	uint16_t phase_offset;
} QuadratureSample;

typedef struct {
	DAC_HandleTypeDef *hdac;
	ADC_HandleTypeDef *hadc;
	TIM_HandleTypeDef *htim_excitation;
	TIM_HandleTypeDef *htim_adc;
	const uint16_t *lookup_table;
	ResolverQuadrature resolver_quadrature;
	QuadratureSample quadrature_sample;
	uint16_t freq_ratio;
}Resolver;

void resolver_init(Resolver *resolver,
				   DAC_HandleTypeDef *hdac,
				   ADC_HandleTypeDef *hadc,
				   TIM_HandleTypeDef *htim_excitation,
				   TIM_HandleTypeDef *htim_adc,
				   const uint16_t *lookup_table);
void resolver_start(Resolver *resolver);

void resolver_handle_adc_callback(Resolver *resolver, uint8_t is_half_transfer);
void resolver_get_buffer(Resolver *resolver, uint16_t *dest_buffer, uint16_t size);


#endif /* INC_RESOLVER_H_ */
