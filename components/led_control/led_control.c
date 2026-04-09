#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "led_control.h"

#define LED_BLINK_FAST_DELAY CONFIG_LED_BLINK_INTERVAL_MS
#define LED_BLINK_SLOW_DELAY CONFIG_LED_BLINK_SLOW_INTERVAL_MS

static const char *TAG = "LED_CONTROL";

// Queue for mode commands
static QueueHandle_t led_queue = NULL;

// Current LED state
static led_mode_t current_mode = LED_MODE_OFF;
static led_mode_t last_mode = LED_MODE_OFF;

// Task handle
static TaskHandle_t led_task_handle = NULL;

static bool led_state = false;
static TickType_t timeout_ticks = portMAX_DELAY;

// LED task function
static void led_control_task(void *arg)
{
    while (1)
    {

        if (xQueueReceive(led_queue, &current_mode, timeout_ticks) == pdTRUE)
        {
            switch (current_mode)
            {
            case LED_MODE_OFF:
                led_state = false;
                gpio_set_level(CONFIG_LED_CONTROL_PIN, 0);
                break;
            case LED_MODE_ON:
                led_state = true;
                gpio_set_level(CONFIG_LED_CONTROL_PIN, 1);
                break;
            case LED_MODE_BLINK_FAST:
                led_state = !led_state;
                gpio_set_level(CONFIG_LED_CONTROL_PIN, led_state);
                timeout_ticks = pdMS_TO_TICKS(LED_BLINK_FAST_DELAY);
                break;
            case LED_MODE_BLINK_SLOW:
                led_state = !led_state;
                gpio_set_level(CONFIG_LED_CONTROL_PIN, led_state);
                timeout_ticks = pdMS_TO_TICKS(LED_BLINK_SLOW_DELAY);
                break;
            }
        }
        else
        {
            // Timeout occurred, toggle LED if in blink mode
            if (current_mode == LED_MODE_BLINK_FAST || current_mode == LED_MODE_BLINK_SLOW)
            {
                led_state = !led_state;
                gpio_set_level(CONFIG_LED_CONTROL_PIN, led_state);
            }
        }
    }
}

esp_err_t led_control_init(void)
{
    // Configure GPIO
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << CONFIG_LED_CONTROL_PIN);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;

    esp_err_t err = gpio_config(&io_conf);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "GPIO config failed: %s", esp_err_to_name(err));
        return err;
    }

    // Create queue of length 1 for immediate overwriting
    led_queue = xQueueCreate(1, sizeof(led_mode_t));
    if (led_queue == NULL)
    {
        ESP_LOGE(TAG, "Queue creation failed");
        return ESP_ERR_NO_MEM;
    }

    // Create task
    BaseType_t ret = xTaskCreate(led_control_task, "led_control_task",
                                 CONFIG_LED_TASK_STACK_SIZE, NULL,
                                 CONFIG_LED_TASK_PRIORITY, &led_task_handle);
    if (ret != pdPASS)
    {
        ESP_LOGE(TAG, "Task creation failed");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "LED Control initialized");
    return ESP_OK;
}

esp_err_t led_control_set_mode(led_mode_t mode)
{

    if (mode == last_mode)
            {
                return ESP_OK; 
            }
     last_mode = mode;

    if (led_queue == NULL)
    {
        ESP_LOGE(TAG, "LED Control not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    // Overwrite the current mode in the queue (length is 1)
    if (xQueueOverwrite(led_queue, &mode) == pdTRUE)
    {
        return ESP_OK;
    }
    else
    {
        ESP_LOGE(TAG, "Failed to send to LED queue");
        return ESP_FAIL;
    }
}
