/*
 * serial_logger.h
 *
 *  Created on: Feb 5, 2025
 *      Author: davis
 */

#ifndef INC_SERIAL_LOGGER_H_
#define INC_SERIAL_LOGGER_H_

#include "main.h"
#include "resolver.h"

void serial_logger_init(UART_HandleTypeDef *huart);
void serial_logger_send(uint16_t sin_value, uint16_t cos_value, uint16_t excit_value);

#endif /* INC_SERIAL_LOGGER_H_ */
