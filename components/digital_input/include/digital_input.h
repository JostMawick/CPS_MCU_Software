#pragma once

#include <stdbool.h>
#include "esp_err.h"

/**
 * @brief Structure holding the state of all digital inputs.
 *        This struct is returned by digital_input_get_data().
 */
typedef struct
{
    bool btn_up;                 // User button UP
    bool btn_down;               // User button DOWN
    bool btn_stop;               // User button STOP
    bool light_barrier;          // Light barrier sensor (True if obstructed/triggered)
    bool emergency_switch_state; // MONITORING ONLY: Hardware Emergency Stop state (True if pressed/active)
} digital_inputs_t;

/**
 * @brief Initialize all digital inputs, ISRs, and the processing task.
 *
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t digital_input_init(void);

/**
 * @brief Get the current snapshot of all digital input states.
 *        Thread-safe structure copy.
 *
 * @return digital_inputs_t Copy of the current state.
 */
digital_inputs_t digital_input_get_data(void);
