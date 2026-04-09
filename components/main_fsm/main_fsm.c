#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "main_fsm.h"
#include "digital_input.h"
#include "position_control.h"
#include "led_control.h"
#include "string.h"

/*---------Needs to be Replaced by LabView-----------*/
#define TOP_POSITION_MM 250
#define BOTTOM_POSITION_MM 0
#define TRAJECTORY_TIME_SEC 8
/*---------------------------------------------------*/
static const char *TAG = "MAIN_FSM";

typedef enum
{
    STATE_IDLE,
    STATE_MOVING_UP,
    STATE_REACHED_TOP,
    STATE_MOVING_DOWN,
    STATE_REACHED_BOTTOM,
    STATE_ERROR
} fsm_state_t;

static fsm_state_t current_state = STATE_IDLE;

static void main_fsm_task(void *arg)
{
    ESP_LOGI(TAG, "Main FSM Task started");

    while (1)
    {

        digital_inputs_t inputs = digital_input_get_data();

        // State Machine Logic
        switch (current_state)
        {
        case STATE_IDLE:
            position_control_hold_position();
            led_control_set_mode(LED_MODE_OFF);

            if (inputs.btn_up)
                current_state = STATE_MOVING_UP;
            else if (inputs.btn_down)
                current_state = STATE_MOVING_DOWN;
            else if (!inputs.light_barrier)
                current_state = STATE_ERROR;
            else if (!inputs.emergency_switch_state)
                current_state = STATE_ERROR;

            break;

        case STATE_MOVING_UP:
            position_control_set_trajectory_mm(TOP_POSITION_MM, TRAJECTORY_TIME_SEC);
            // position_control_set_position_mm(TOP_POSITION_MM);
            led_control_set_mode(LED_MODE_BLINK_SLOW);

            if (inputs.btn_stop)
                current_state = STATE_IDLE;
            else if (inputs.btn_down)
                current_state = STATE_MOVING_DOWN;
            else if (!inputs.light_barrier)
                current_state = STATE_ERROR;
            else if (!inputs.emergency_switch_state)
                current_state = STATE_ERROR;
            else if (position_control_is_target_reached())
                current_state = STATE_REACHED_TOP;
            break;

        case STATE_MOVING_DOWN:
            position_control_set_trajectory_mm(BOTTOM_POSITION_MM, TRAJECTORY_TIME_SEC);
            // position_control_set_position_mm(BOTTOM_POSITION_MM);
            led_control_set_mode(LED_MODE_BLINK_SLOW);

            if (inputs.btn_stop)
                current_state = STATE_IDLE;
            else if (inputs.btn_up)
                current_state = STATE_MOVING_UP;
            else if (!inputs.light_barrier)
                current_state = STATE_ERROR;
            else if (!inputs.emergency_switch_state)
                current_state = STATE_ERROR;
            else if (position_control_is_target_reached())
                current_state = STATE_REACHED_BOTTOM;
            break;

        case STATE_REACHED_TOP:
            position_control_hold_position();
            led_control_set_mode(LED_MODE_OFF);

            if (inputs.btn_down)
                current_state = STATE_MOVING_DOWN;
            else if (!inputs.emergency_switch_state)
                current_state = STATE_ERROR;
            break;

        case STATE_REACHED_BOTTOM:
            position_control_hold_position();
            led_control_set_mode(LED_MODE_OFF);

            if (inputs.btn_up)
                current_state = STATE_MOVING_UP;
            else if (!inputs.emergency_switch_state)
                current_state = STATE_ERROR;
            break;

        case STATE_ERROR:
            position_control_hold_position();
            led_control_set_mode(LED_MODE_BLINK_FAST);

            if (inputs.light_barrier && inputs.emergency_switch_state && inputs.btn_stop)
                current_state = STATE_IDLE;
            break;

        default:
            current_state = STATE_ERROR;
            ESP_LOGE(TAG, "Invalid state detected, transitioning to ERROR state");
            break;
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

const char *main_fsm_get_state_string()
{
    switch (current_state)
    {
    case STATE_IDLE:
        return "IDLE";
    case STATE_MOVING_UP:
        return "MOVING_UP";
    case STATE_REACHED_TOP:
        return "REACHED_TOP";
    case STATE_MOVING_DOWN:
        return "MOVING_DOWN";
    case STATE_REACHED_BOTTOM:
        return "REACHED_BOTTOM";
    case STATE_ERROR:
        return "ERROR";
    default:
        return "UNKNOWN";
    }
}