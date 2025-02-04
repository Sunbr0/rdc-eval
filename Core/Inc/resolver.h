/*
 * resolver.h
 *
 *  Created on: Jan 29, 2025
 *      Author: davis
 */

#ifndef INC_RESOLVER_H_
#define INC_RESOLVER_H_

#include "main.h"

typedef struct {
} ResolverQuadrature;

typedef struct {
	uint16_t sine_buffer[ADC_BUFFER_SIZE/2];
	uint16_t cosine_buffer[ADC_BUFFER_SIZE/2];
	uint16_t phase_offset;
} QuadratureSample;

typedef struct {
	DAC_HandleTypeDef *hdac;
	ADC_HandleTypeDef *hadc;
	TIM_HandleTypeDef *htim_excitation;
	TIM_HandleTypeDef *htim_adc;
	const uint16_t *lookup_table;
    uint16_t adc_buffer[ADC_BUFFER_SIZE];
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
void get_adc_buffer_safe(Resolver *self, uint16_t *buffer, uint16_t size);



#endif /* INC_RESOLVER_H_ */
