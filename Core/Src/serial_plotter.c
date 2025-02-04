/*
 * serial_plotter.c
 *
 *  Created on: Jan 29, 2025
 *      Author: davis
 */


#include "serial_plotter.h"
#include <stdio.h>

static SerialPlotter *active_plotter = NULL;
static char uart_tx_buffer[64];  // Buffer for UART transmission

// Initialize the Serial Plotter
void serial_plotter_init(SerialPlotter *plotter, UART_HandleTypeDef *huart, TIM_HandleTypeDef *htim) {
    plotter->huart = huart;
    plotter->htim = htim;
    plotter->num_channels = 0;
    plotter->capture_ready = 0;
    plotter->state = PLOTTER_IDLE;
    active_plotter = plotter;
}

// Add a data source (channel)
void serial_plotter_add_channel(SerialPlotter *plotter, const uint16_t *buffer) {
    if (plotter->num_channels < MAX_CHANNELS) {
        plotter->buffers[plotter->num_channels] = buffer;
        plotter->num_channels++;
    }
}


// Capture a full waveform cycle without blocking
void serial_plotter_start_capture(SerialPlotter *plotter) {
    if (plotter->state != PLOTTER_IDLE) return;  // Avoid overwriting active buffers

    // Assume data is already in the buffers (filled by ADC module)
    plotter->capture_ready = 1;
    plotter->state = PLOTTER_READY;
}

// Start non-blocking replay
void serial_plotter_start_replay(SerialPlotter *plotter) {
    if (plotter->state != PLOTTER_READY) return;  // Only start if data is ready

    plotter->transmit_index = 0;
    plotter->state = PLOTTER_TRANSMITTING;

    // Start first transmission (DMA/Interrupt mode)
    snprintf(uart_tx_buffer, sizeof(uart_tx_buffer), "%d,%d,%d\r\n",
             plotter->buffers[0][plotter->transmit_index],
             plotter->buffers[1][plotter->transmit_index],
             plotter->buffers[2][plotter->transmit_index]);

    HAL_UART_Transmit_IT(plotter->huart, (uint8_t *)uart_tx_buffer, strlen(uart_tx_buffer));
}

// Periodic processing function (called in `main.c` loop)
void serial_plotter_process(SerialPlotter *plotter) {
    if (plotter->state == PLOTTER_IDLE) {
        serial_plotter_start_capture(plotter);  // Start new capture
    }
    else if (plotter->state == PLOTTER_READY) {
        serial_plotter_start_replay(plotter);  // Start replaying if ready
    }
}

// UART transmission complete callback (called from `HAL_UART_TxCpltCallback`)
void serial_plotter_tx_complete_callback(UART_HandleTypeDef *huart) {
    if (active_plotter && huart == active_plotter->huart) {
        SerialPlotter *plotter = active_plotter;

        plotter->transmit_index++;

        if (plotter->transmit_index < WAVEFORM_SAMPLES) {
            // Transmit next sample
            snprintf(uart_tx_buffer, sizeof(uart_tx_buffer), "%d,%d,%d\r\n",
                     plotter->buffers[0][plotter->transmit_index],
                     plotter->buffers[1][plotter->transmit_index],
                     plotter->buffers[2][plotter->transmit_index]);

            HAL_UART_Transmit_IT(plotter->huart, (uint8_t *)uart_tx_buffer, strlen(uart_tx_buffer));
        } else {
            // Transmission complete
            plotter->state = PLOTTER_IDLE;
        }
    }
}
