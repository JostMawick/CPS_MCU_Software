#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "main_fsm.h"
#include "digital_input.h"
#include "position_control.h"
#include "bdc_motor_driver.h"
#include "led_control.h"
#include "servo_control.h"
#include "string.h"

#define SERVO_ANGLE_CLEAR 0
#define SERVO_ANGLE_PUSH -90

static const char *TAG = "MAIN_FSM";

typedef enum
{
    STATE_IDLE,
    STATE_WAIT_FOR_HANDGUARD,
    STATE_RESET_BOXCOUNT,
    STATE_MOVE_BAND,
    STATE_CHECK_BOXCOUNT,
    STATE_INVALID_BOXCOUNT,
    STATE_METAL_DETECTED,
    STATE_EMERGENCY,
    STATE_WAIT_FOR_CLEAR
} fsm_state_t;

static fsm_state_t current_state = STATE_IDLE;
static fsm_state_t previous_state = STATE_IDLE;
static QueueHandle_t input_event_queue = NULL;
static digital_inputs_t s_digital_inputs;
static int box_count = 0;
static bool previous_lightgate_start = false;

static void main_fsm_task(void *arg)
{
    ESP_LOGI(TAG, "Main FSM Task started");
    input_event_queue = digital_input_get_queue();
    esp_err_t err;

    while (1)
    {

        s_digital_inputs = digital_input_get_data();

        switch (current_state)
        {
        case STATE_IDLE:
            box_count = 0;
            led_control_set_mode(LED_MODE_OFF);
            bdc_driver_set_pwm(0.0f);
            servo_control_set_angle(SERVO_ANGLE_CLEAR);

            if (s_digital_inputs.lightgate_start)
            {
                box_count++;
                current_state = STATE_WAIT_FOR_HANDGUARD;
            }

            if (s_digital_inputs.emergency_btn)
                current_state = STATE_EMERGENCY;
            break;

        case STATE_WAIT_FOR_HANDGUARD:
            led_control_set_mode(LED_MODE_BLINK_SLOW);
            bdc_driver_set_pwm(0.0f);
            servo_control_set_angle(SERVO_ANGLE_CLEAR);
            if ((s_digital_inputs.lightgate_start == true) && (previous_lightgate_start == false))
            {
                box_count++;
                previous_lightgate_start = s_digital_inputs.lightgate_start;
            }

            if (s_digital_inputs.handguard_right && s_digital_inputs.handguard_left)
                current_state = STATE_MOVE_BAND;

            if (s_digital_inputs.emergency_btn)
                current_state = STATE_EMERGENCY;
            break;

        case STATE_RESET_BOXCOUNT:
            led_control_set_mode(LED_MODE_BLINK_SLOW);
            bdc_driver_set_pwm(0.0f);
            servo_control_set_angle(SERVO_ANGLE_CLEAR);
            box_count = 0;
            current_state = STATE_IDLE;

            break;

        case STATE_MOVE_BAND:
            led_control_set_mode(LED_MODE_ON);
            bdc_driver_set_pwm(1.0f);
            servo_control_set_angle(SERVO_ANGLE_CLEAR);

            if (s_digital_inputs.inductive_switch)
                current_state = STATE_METAL_DETECTED;
            else if (s_digital_inputs.lightgate_end)
                current_state = STATE_CHECK_BOXCOUNT;
            else if (s_digital_inputs.handguard_left && s_digital_inputs.handguard_right == false)
                current_state = STATE_WAIT_FOR_HANDGUARD;
            if (s_digital_inputs.emergency_btn)
                current_state = STATE_EMERGENCY;
            break;

        case STATE_CHECK_BOXCOUNT:
            led_control_set_mode(LED_MODE_ON);
            bdc_driver_set_pwm(1.0f);
            servo_control_set_angle(SERVO_ANGLE_CLEAR);

            if (box_count > 1)
            {
                box_count--;
                current_state = STATE_MOVE_BAND;
            }

            else if (box_count == 1)
            {
                box_count--;
                current_state = STATE_WAIT_FOR_CLEAR;
            }

            else if (box_count < 1)
                current_state = STATE_INVALID_BOXCOUNT;

            break;

        case STATE_WAIT_FOR_CLEAR:
            led_control_set_mode(LED_MODE_BLINK_SLOW);
            bdc_driver_set_pwm(1.0f);
            servo_control_set_angle(SERVO_ANGLE_CLEAR);
            vTaskDelay(pdMS_TO_TICKS(1000));
            current_state = STATE_IDLE;
            break;

        case STATE_INVALID_BOXCOUNT:
            led_control_set_mode(LED_MODE_BLINK_SLOW);
            bdc_driver_set_pwm(0.0f);
            servo_control_set_angle(SERVO_ANGLE_CLEAR);
            if (s_digital_inputs.reset_btn)
                current_state = STATE_RESET_BOXCOUNT;

            break;

        case STATE_METAL_DETECTED:
            led_control_set_mode(LED_MODE_BLINK_SLOW);
            bdc_driver_set_pwm(0.0f);
            servo_control_set_angle(SERVO_ANGLE_PUSH);
            vTaskDelay(pdMS_TO_TICKS(1000));
            current_state = STATE_MOVE_BAND;
            break;

        case STATE_EMERGENCY:
            led_control_set_mode(LED_MODE_BLINK_FAST);
            bdc_driver_set_pwm(0.0f);
            servo_control_set_angle(SERVO_ANGLE_CLEAR);
            if (s_digital_inputs.reset_btn)
                current_state = STATE_RESET_BOXCOUNT;
            break;

        default:
            break;
        }

        if (xQueueReceive(input_event_queue, &s_digital_inputs, 0) == pdPASS)
        {
            ESP_LOGI(TAG, "Current State: %s", main_fsm_get_state_string());
            ESP_LOGI(TAG, "Box Count: %d", box_count);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

esp_err_t main_fsm_init(void)
{
    // Crate FSM Task
    BaseType_t ret = xTaskCreate(main_fsm_task, "main_fsm_task",
                                 4096, NULL,
                                 5, NULL); // Priorität anpassen, falls nötig

    if (ret != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create Main FSM task");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Main FSM Component Initialized");
    return ESP_OK;
}

const char *main_fsm_get_state_string(void)
{
    switch (current_state)
    {
    case STATE_IDLE:
        return "IDLE";
    case STATE_WAIT_FOR_HANDGUARD:
        return "WAIT_FOR_HANDGUARD";
    case STATE_RESET_BOXCOUNT:
        return "RESET_BOXCOUNT";
    case STATE_MOVE_BAND:
        return "MOVE_BAND";
    case STATE_CHECK_BOXCOUNT:
        return "CHECK_BOXCOUNT";
    case STATE_INVALID_BOXCOUNT:
        return "INVALID_BOXCOUNT";
    case STATE_METAL_DETECTED:
        return "METAL_DETECTED";
    case STATE_EMERGENCY:
        return "EMERGENCY";
    case STATE_WAIT_FOR_CLEAR:
        return "WAIT_FOR_CLEAR";
    default:
        return "UNKNOWN";
    }
}