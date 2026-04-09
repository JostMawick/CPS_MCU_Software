/**
 * @file bdc_motor_driver.h
 * @brief Brushed DC Motor Driver component definition.
 */
#pragma once

#include <stdint.h>
#include "esp_err.h"
#include <stdbool.h>

/**
 * @brief Initialize the BDC motor driver.
 *        Sets up MCPWM, PCNT (Encoder), PID Controller, and the control loop timer.
 *        If CONFIG_BDC_MOTOR_SERIAL_STUDIO_DEBUG is enabled, starts a debug task that prints the current pulse count for Serial Studio.
 *
 * @return esp_err_t
 *         - ESP_OK on success
 *         - ESP_FAIL if initialization fails
 */
esp_err_t bdc_motor_driver_init(void);

/**
 * @brief Set the target speed for the PID controller.
 *        Enables PID control mode.
 *
 * @param rps Target speed in Revolutions Per Second (RPS).
 *            Positive values for forward, negative for reverse.
 * @return esp_err_t
 *         - ESP_OK on success
 */
esp_err_t bdc_driver_set_pid_speed(float rps);

/**
 * @brief Set the motor PWM duty cycle directly (Open Loop).
 *        Disables PID control mode.
 *
 * @param duty_cycle Duty cycle between -1.0 (Full Reverse) and 1.0 (Full Forward).
 *                   Values outside this range are clamped.
 * @return esp_err_t
 *         - ESP_OK on success
 */
esp_err_t bdc_driver_set_pwm(float duty_cycle);

/**
 * @brief Get the current speed of the motor in Revolutions Per Second (RPS).
 *        Calculates speed based on encoder pulses and the PID loop period.
 *
 * @return float Current speed in RPS.
 */
float bdc_driver_get_speed_rps(void);

/**
 * @brief Get the total number of revolutions the motor has turned.
 *        Calculated from the total accumulated encoder pulse count.
 *
 * @return float Total revolutions (can be fractional).
 */
float bdc_driver_get_revolutions(void);

/**
 * @brief Get the total pulse count from the encoder.
 *        This is the raw count from the PCNT unit, which may be positive or negative based on direction.
 *
 * @return int Total pulse count.
 */
int bdc_driver_get_pulse_count(void);

/**
 * @brief Get the number of pulses counted in the last PID loop iteration.
 *        This represents the change in pulse count since the last PID loop callback.
 *
 * @return int Number of pulses counted in the last PID loop iteration.
 */
int bdc_driver_get_report_pulses(void);
