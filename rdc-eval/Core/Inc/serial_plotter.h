/*
 * serial_plotter.h
 *
 *  Created on: Jan 29, 2025
 *      Author: davis
 */

#ifndef INC_SERIAL_PLOTTER_H_
#define INC_SERIAL_PLOTTER_H_

#include "main.h"

#define MAX_CHANNELS 5  // Maximum number of signals to capture
#define WAVEFORM_SAMPLES 20  // Points per wave cycle

typedef enum {
    PLOTTER_IDLE,       // Waiting for new data
    PLOTTER_CAPTURE,    // Capturing wave data
    PLOTTER_READY,      // Data is ready for transmission
    PLOTTER_TRANSMITTING // UART transmission in progress
} PlotterState;

typedef struct {
	const uint16_t *buffers[MAX_CHANNELS];
    uint8_t num_channels;
    volatile uint8_t capture_ready;
    volatile PlotterState state;
    UART_HandleTypeDef *huart;
    TIM_HandleTypeDef *htim;
    uint16_t transmit_index;
} SerialPlotter;

void serial_plotter_init(SerialPlotter *plotter, UART_HandleTypeDef *huart, TIM_HandleTypeDef *htim);
void serial_plotter_add_channel(SerialPlotter *plotter, const uint16_t *buffer);
void serial_plotter_start_capture(SerialPlotter *plotter);
void serial_plotter_start_replay(SerialPlotter *plotter);
void serial_plotter_process(SerialPlotter *plotter);
void serial_plotter_tx_complete_callback(UART_HandleTypeDef *huart);

#endif /* INC_SERIAL_PLOTTER_H_ */
