#pragma once

#include "esp_err.h"

/**
 * @brief Initialize the Main Finite State Machine (FSM) component.
 *        Creates and starts the main FSM task which polls inputs and
 *        controls the system state.
 *
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t main_fsm_init(void);

/**
 * @brief Get a string representation of the given FSM state.
 *
 * @param state The state to convert to a string.
 * @return const char* String representation of the state.
 */
const char *main_fsm_get_state_string();
