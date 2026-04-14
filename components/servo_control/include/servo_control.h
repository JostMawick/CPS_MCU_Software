/**
 * @file servo_control.h
 * @brief Servo Control Component.
 */
#pragma once

#include "esp_err.h"

/**
 * @brief Initialize the servo motor control.
 *        Sets up the MCPWM timer, operator, generator, and comparator based on Kconfig.
 *
 * @return esp_err_t
 *         - ESP_OK on success
 *         - ESP_FAIL or other error code on failure
 */
esp_err_t servo_control_init(void);

/**
 * @brief Set the servo to a specific angle.
 *
 * @param angle Target angle (must be between CONFIG_SERVO_MIN_DEGREE and CONFIG_SERVO_MAX_DEGREE).
 * @return esp_err_t
 *         - ESP_OK on success
 *         - ESP_ERR_INVALID_STATE if not initialized
 *         - ESP_ERR_INVALID_ARG if angle is out of range
 */
esp_err_t servo_control_set_angle(int angle);
