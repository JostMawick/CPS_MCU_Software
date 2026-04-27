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

static const char *TAG = "MAIN_FSM";

typedef enum
{
    STATE_IDLE,
    STATE_BELT_FORWARD,
    STATE_BELT_REVERSE,
    STATE_BELT_STOPPED,
    STATE_UNSECURE_HANDS,
    STATE_ERROR
} fsm_state_t;

static fsm_state_t current_state = STATE_IDLE;
static fsm_state_t previous_state = STATE_IDLE;
static QueueHandle_t input_event_queue = NULL;
static digital_inputs_t s_digital_inputs;

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
            bdc_driver_set_pwm(0);
            servo_control_set_angle(0);

            if (s_digital_inputs.lightgate_start)
                current_state = STATE_BELT_FORWARD;

            break;
        case STATE_BELT_FORWARD:
            bdc_driver_set_pwm(1.0f);
            servo_control_set_angle(90);

            if (s_digital_inputs.lightgate_end)
                current_state = STATE_IDLE;
            break;
        case STATE_BELT_REVERSE:
            break;
        case STATE_BELT_STOPPED:
            break;
        case STATE_UNSECURE_HANDS:
            break;
        case STATE_ERROR:
            break;
        default:
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

const char *main_fsm_get_state_string(void)
{
    switch (current_state)
    {
    case STATE_IDLE:
        return "IDLE";
    case STATE_BELT_FORWARD:
        return "BELT_FORWARD";
    case STATE_BELT_REVERSE:
        return "BELT_REVERSE";
    case STATE_BELT_STOPPED:
        return "BELT_STOPPED";
    case STATE_UNSECURE_HANDS:
        return "UNSECURE_HANDS";
    case STATE_ERROR:
        return "ERROR";
    default:
        return "UNKNOWN";
    }
}