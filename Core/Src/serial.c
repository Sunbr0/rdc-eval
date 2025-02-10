/*
 * serial.c
 *
 *  Created on: Feb 10, 2025
 *      Author: davis
 */


#include "serial.h"

// Initialize the Serial structure
void Serial_Init(Serial_t *serial, UART_HandleTypeDef *huart) {
    serial->huart = huart;
    HAL_UART_Init(serial->huart);
}

// Send a string (Blocking Mode)
void Serial_SendString(Serial_t *serial, const char *str) {
    HAL_UART_Transmit(serial->huart, (uint8_t *)str, strlen(str), HAL_MAX_DELAY);
}

// Send raw data (Blocking Mode)
void Serial_SendData(Serial_t *serial, uint8_t *data, uint16_t len) {
    HAL_UART_Transmit(serial->huart, data, len, HAL_MAX_DELAY);
}

// Send a string using Interrupt Mode (Non-blocking)
void Serial_SendString_IT(Serial_t *serial, const char *str) {
    HAL_UART_Transmit_IT(serial->huart, (uint8_t *)str, strlen(str));
}

// Send a string using DMA Mode (Efficient for large logs)
void Serial_SendString_DMA(Serial_t *serial, const char *str) {
    HAL_UART_Transmit_DMA(serial->huart, (uint8_t *)str, strlen(str));
}
