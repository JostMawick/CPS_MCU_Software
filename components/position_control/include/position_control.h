/**
 * @file position_control.h
 * @brief Position Control component for controlling movement.
 */
#pragma once

#include <stdint.h>
#include "esp_err.h"

/**
 * @brief Initialize the position control component.
 *        Creates and starts the periodic timer for the position control loop.
 *
 * @return esp_err_t
 *         - ESP_OK on success
 *         - ESP_FAIL if timer creation fails
 */
esp_err_t position_control_init(void);

/**
 * @brief Set the target position in motor revolutions.
 *
 * @param revolutions Target number of revolutions.
 * @return esp_err_t
 *         - ESP_OK on success
 */
esp_err_t position_control_set_revolutions(float revolutions);

/**
 * @brief Set the target position in millimeters.
 *
 * @param position_mm Target position in millimeters.
 * @return esp_err_t
 *         - ESP_OK on success
 */
esp_err_t position_control_set_position_mm(float position_mm);

/**
 * @brief Set a target position to be reached precisely in a given time.
 *        This generates a linear movement profile (trajectory) to ensure the
 *        curtain arrives at the target exactly after the specified duration.
 *
 * @param position_mm Target position in millimeters.
 * @param duration_sec The total time in seconds the movement should take.
 * @return esp_err_t
 *         - ESP_OK on success
 *         - ESP_ERR_INVALID_ARG if the requested speed is too high for the motor.
 */
esp_err_t position_control_set_trajectory_mm(float position_mm, float duration_sec);

/**
 * @brief Stop the current movement and actively hold the current position.
 *
 * @return esp_err_t
 *         - ESP_OK on success
 */
esp_err_t position_control_hold_position(void);

/**
 * @brief Check if the current target position has been reached within tolerance.
 *
 * @return true if target is reached, false otherwise.
 */
bool position_control_is_target_reached(void);
