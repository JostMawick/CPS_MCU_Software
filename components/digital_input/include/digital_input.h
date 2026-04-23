/**
 * @file digital_input.h
 * @brief Digital Input handling component.
 */
#pragma once

#include <stdbool.h>
#include "esp_err.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

/**
 * @brief Structure holding the state of all digital inputs.
 *        This struct is returned by digital_input_get_data().
 */
typedef struct
{
    bool handguard_right;  // Right handguard switch (active-low)
    bool handguard_left;   // Left handguard switch (active-low)
    bool reset_btn;        // Reset button (active-low)
    bool lightgate_start;  // Lightgate start sensor (active-low)
    bool lightgate_end;    // Lightgate end sensor (active-low)
    bool emergency_btn;    // Emergency button (monitoring only, active-low)
    bool inductive_switch; // Inductive switch sensor (active-high, external pull-down)
} digital_inputs_t;

/**
 * @brief Initialize all digital inputs, ISRs, and the processing task.
 *
 * @note This configures GPIO pins and may install an ISR service.
 * @return esp_err_t
 *         - ESP_OK on success
 *         - ESP_FAIL if initialization fails
 */
esp_err_t digital_input_init(void);

/**
 * @brief Get the current snapshot of all digital input states.
 *        Thread-safe structure copy.
 *
 * @return digital_inputs_t Copy of the current state.
 */
digital_inputs_t digital_input_get_data(void);

/**
 * @brief Get the FreeRTOS queue handle for debounced input events.
 *        This queue receives updates whenever an input state changes after debouncing.
 *
 * @return QueueHandle_t Handle to the input event queue.
 */
QueueHandle_t digital_input_get_queue(void);
