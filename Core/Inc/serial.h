/*
 * serial.h
 *
 *  Created on: Feb 10, 2025
 *      Author: davis
 */

#ifndef INC_SERIAL_H_
#define INC_SERIAL_H_

#include "main.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

// Serial structure
typedef struct {
    UART_HandleTypeDef *huart;
} Serial_t;

// Function prototypes
void Serial_Init(Serial_t *serial, UART_HandleTypeDef *huart);
void Serial_SendString(Serial_t *serial, const char *str);
void Serial_SendData(Serial_t *serial, uint8_t *data, uint16_t len);
void Serial_SendString_IT(Serial_t *serial, const char *str);
void Serial_SendString_DMA(Serial_t *serial, const char *str);

// Error handling
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);



#endif /* INC_SERIAL_H_ */
