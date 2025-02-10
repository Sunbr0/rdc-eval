/*
 * serial.c
 *
 *  Created on: Feb 10, 2025
 *      Author: davis
 */


#include "serial.h"

// Initialize the Serial structure
void Serial_Init(Serial_t *self, UART_HandleTypeDef *huart) {
    self->huart = huart;
    self->tx_head = 0;
    self->tx_tail = 0;
    self->tx_busy = false;
    HAL_UART_Init(self->huart);
}

// Send a string using Interrupt Mode (Non-blocking)
void Serial_SendString(Serial_t *self, const char *str) {
    uint16_t len = strlen(str);

    // Copy data to the TX buffer
    for (uint16_t i = 0; i < len; i++) {
        self->tx_buffer[self->tx_head] = str[i];
        self->tx_head = (self->tx_head + 1) % SERIAL_TX_BUFFER_SIZE;

        // Prevent buffer overflow
        if (self->tx_head == self->tx_tail) {
            self->tx_head = (self->tx_head - 1) % SERIAL_TX_BUFFER_SIZE;  // Undo last write
            break;
        }
    }

    // Start transmission if not already busy
    if (!self->tx_busy) {
        self->tx_busy = true;
        uint8_t first_byte = self->tx_buffer[self->tx_tail];
        self->tx_tail = (self->tx_tail + 1) % SERIAL_TX_BUFFER_SIZE;
        HAL_UART_Transmit_IT(self->huart, &first_byte, 1);
    }
}

// Callback for UART Tx complete
void Serial_Transmit_Complete_Callback(Serial_t *self) {
    // Check if more data is waiting in the buffer
    if (self->tx_tail != self->tx_head) {
        uint8_t next_byte = self->tx_buffer[self->tx_tail];
        self->tx_tail = (self->tx_tail + 1) % SERIAL_TX_BUFFER_SIZE;
        HAL_UART_Transmit_IT(self->huart, &next_byte, 1);
    } else {
        self->tx_busy = false;  // Transmission complete
    }
}
