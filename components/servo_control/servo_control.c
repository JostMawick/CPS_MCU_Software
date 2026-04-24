/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/mcpwm_prelude.h"
#include "sdkconfig.h"
#include "servo_control.h"

static const char *TAG = "SERVO_CONTROL";

// Read configuration from Kconfig
#define SERVO_MIN_PULSEWIDTH_US CONFIG_SERVO_MIN_PULSEWIDTH_US
#define SERVO_MAX_PULSEWIDTH_US CONFIG_SERVO_MAX_PULSEWIDTH_US
#define SERVO_MIN_DEGREE CONFIG_SERVO_MIN_DEGREE
#define SERVO_MAX_DEGREE CONFIG_SERVO_MAX_DEGREE
#define SERVO_PULSE_GPIO CONFIG_SERVO_PULSE_GPIO

#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000 // 1MHz, 1us per tick
#define SERVO_TIMEBASE_PERIOD 20000          // 20000 ticks, 20ms

typedef struct
{
    mcpwm_timer_handle_t timer;
    mcpwm_oper_handle_t oper;
    mcpwm_cmpr_handle_t comparator;
    mcpwm_gen_handle_t generator;
    bool is_initialized;
} servo_control_context_t;

static servo_control_context_t s_servo_ctx = {0};

static inline uint32_t angle_to_compare(int angle)
{
    return (angle - SERVO_MIN_DEGREE) * (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) / (SERVO_MAX_DEGREE - SERVO_MIN_DEGREE) + SERVO_MIN_PULSEWIDTH_US;
}

esp_err_t servo_control_init(void)
{
    if (s_servo_ctx.is_initialized)
    {
        return ESP_OK; // Already initialized
    }

    ESP_LOGI(TAG, "Create timer and operator");
    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,
        .period_ticks = SERVO_TIMEBASE_PERIOD,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &s_servo_ctx.timer));

    mcpwm_operator_config_t operator_config = {
        .group_id = 0, // operator must be in the same group to the timer
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &s_servo_ctx.oper));

    ESP_LOGI(TAG, "Connect timer and operator");
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(s_servo_ctx.oper, s_servo_ctx.timer));

    ESP_LOGI(TAG, "Create comparator and generator from the operator");
    mcpwm_comparator_config_t comparator_config = {
        .flags.update_cmp_on_tez = true,
    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(s_servo_ctx.oper, &comparator_config, &s_servo_ctx.comparator));

    mcpwm_generator_config_t generator_config = {
        .gen_gpio_num = SERVO_PULSE_GPIO,
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(s_servo_ctx.oper, &generator_config, &s_servo_ctx.generator));

    // set the initial compare value, so that the servo will spin to the center position
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(s_servo_ctx.comparator, angle_to_compare(0)));

    ESP_LOGI(TAG, "Set generator action on timer and compare event");
    // go high on counter empty
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(s_servo_ctx.generator,
                                                              MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    // go low on compare threshold
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(s_servo_ctx.generator,
                                                                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, s_servo_ctx.comparator, MCPWM_GEN_ACTION_LOW)));

    ESP_LOGI(TAG, "Enable and start timer");
    ESP_ERROR_CHECK(mcpwm_timer_enable(s_servo_ctx.timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(s_servo_ctx.timer, MCPWM_TIMER_START_NO_STOP));

    s_servo_ctx.is_initialized = true;
    ESP_LOGI(TAG, "Servo Control Initialized");

    return ESP_OK;
}

esp_err_t servo_control_set_angle(int angle)
{
    if (!s_servo_ctx.is_initialized)
    {
        ESP_LOGE(TAG, "Servo Control not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (angle < SERVO_MIN_DEGREE || angle > SERVO_MAX_DEGREE)
    {
        ESP_LOGE(TAG, "Requested angle %d is out of bounds [%d, %d]", angle, SERVO_MIN_DEGREE, SERVO_MAX_DEGREE);
        return ESP_ERR_INVALID_ARG;
    }

    // ESP_LOGI(TAG, "Setting angle to %d", angle);
    esp_err_t err = mcpwm_comparator_set_compare_value(s_servo_ctx.comparator, angle_to_compare(angle));
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set comparator value");
        return err;
    }

    return ESP_OK;
}