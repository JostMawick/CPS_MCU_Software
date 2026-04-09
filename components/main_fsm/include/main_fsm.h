/**
 * @file main_fsm.h
 * @brief Main Finite State Machine component definition.
 */
#pragma once

#include "esp_err.h"

/**
 * @brief Initialize the Main Finite State Machine (FSM) component.
 *        Creates and starts the main FSM task which polls inputs and
 *        controls the system state.
 *
 * @note  Spawns a FreeRTOS task handling the state transitions.
 * @return esp_err_t
 *         - ESP_OK on success
 *         - ESP_FAIL if task creation fails
 */
esp_err_t main_fsm_init(void);

/**
 * @brief Get a string representation of the current FSM state.
 *
 * @return const char* String representation of the state.
 */
const char *main_fsm_get_state_string(void);
