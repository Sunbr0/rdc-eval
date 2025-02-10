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

#define SERIAL_TX_BUFFER_SIZE 20

// Serial structure
typedef struct {
    UART_HandleTypeDef *huart;   // Pointer to the UART peripheral

    uint8_t tx_buffer[SERIAL_TX_BUFFER_SIZE];  // Transmission buffer
    volatile uint16_t tx_head;   // Index for adding new data to the buffer
    volatile uint16_t tx_tail;   // Index for removing data from the buffer
    volatile bool tx_busy;       // Flag to indicate if UART is currently transmitting
} Serial_t;


// Function prototypes
void Serial_Init(Serial_t *self, UART_HandleTypeDef *huart);
void Serial_SendString(Serial_t *self, const char *str);

// Callback prototypes
void Serial_Transmit_Complete_Callback(Serial_t *self); // Callback function


#endif /* INC_SERIAL_H_ */
