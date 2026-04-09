/**
 * @file led_control.h
 * @brief LED Control component definition.
 */
#pragma once

#include "esp_err.h"

/**
 * @brief Enum for LED operating modes.
 */
typedef enum
{
    LED_MODE_OFF,        // LED is turned off
    LED_MODE_ON,         // LED is continuously on
    LED_MODE_BLINK_FAST, // LED blinks fast
    LED_MODE_BLINK_SLOW  // LED blinks slow
} led_mode_t;

/**
 * @brief Initialize the LED control component.
 *
 * @note Initializes hardware and spawns the FreeRTOS task handling LED modes.
 * @return esp_err_t
 *         - ESP_OK on success
 *         - ESP_FAIL if initialization fails
 */
esp_err_t led_control_init(void);

/**
 * @brief Set the LED operating mode.
 *
 * @param mode The desired LED mode (from led_mode_t).
 * @return esp_err_t
 *         - ESP_OK on success
 *         - ESP_ERR_INVALID_STATE if not initialized
 *         - ESP_FAIL if queue is full
 */
esp_err_t led_control_set_mode(led_mode_t mode);
