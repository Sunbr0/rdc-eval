/*
 * serial_logger.c
 *
 *  Created on: Feb 5, 2025
 *      Author: davis
 */


#include "serial_logger.h"
#include <stdio.h>
#include <string.h>

static UART_HandleTypeDef *serial_huart;
static char uart_tx_buffer[32];  // Buffer for formatted UART data

void serial_logger_init(UART_HandleTypeDef *huart) {
    serial_huart = huart;
}

/**
 * @brief Formats and sends resolver values over UART in "X,Y,Z\r\n" format.
 *
 * @param sin_value  The sine component value
 * @param cos_value  The cosine component value
 * @param excit_value The excitation signal value
 */
void serial_logger_send(uint16_t sin_value, uint16_t cos_value, uint16_t excit_value) {
    int len = snprintf(uart_tx_buffer, sizeof(uart_tx_buffer), "%d,%d,%d\r\n",
                       sin_value, cos_value, excit_value);

    // Ensure we don't overflow and use DMA for efficient transfer
    HAL_UART_Transmit(serial_huart, (uint8_t *)uart_tx_buffer, len, HAL_MAX_DELAY);
}
