#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "sdkconfig.h"
#include "digital_input.h"

// --- Configuration Mapping ---
#define PIN_BTN_UP CONFIG_DIGITAL_INPUT_PIN_BTN_UP
#define PIN_BTN_DOWN CONFIG_DIGITAL_INPUT_PIN_BTN_DOWN
#define PIN_BTN_STOP CONFIG_DIGITAL_INPUT_PIN_BTN_STOP
#define PIN_LIGHT_BARRIER CONFIG_DIGITAL_INPUT_PIN_LIGHT_BARRIER
#define PIN_EMERGENCY CONFIG_DIGITAL_INPUT_PIN_EMERGENCY
#define PIN_INDUCTIVE_SWITCH CONFIG_DIGITAL_INPUT_PIN_INDUCTIVE_SWITCH

#define DEBOUNCE_TIME_US 50000 // 50ms

static const char *TAG = "DIG_INPUT";

// --- Private Types ---
typedef struct
{
    int pin;
    int level;
} isr_event_t;

// --- Private Globals ---
static digital_inputs_t s_input_values = {0};
static QueueHandle_t s_input_queue = NULL;
static QueueHandle_t s_input_queue_debounced = NULL;
static SemaphoreHandle_t s_input_mutex = NULL;

static int64_t last_isr_time[GPIO_NUM_MAX] = {0};

// --- ISR Handler ---
static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    uint32_t gpio_num = (uint32_t)arg;
    isr_event_t evt;
    evt.pin = gpio_num;
    evt.level = gpio_get_level(gpio_num);
    xQueueSendFromISR(s_input_queue, &evt, NULL);
}

// --- Task ---
static void digital_input_task(void *arg)
{
    isr_event_t evt;
    while (1)
    {
        if (xQueueReceive(s_input_queue, &evt, portMAX_DELAY))
        {
            int64_t now = esp_timer_get_time();

            // 1. Check if enough time has passed since last processing for this pin
            if ((now - last_isr_time[evt.pin]) < DEBOUNCE_TIME_US)
            {
                continue; // Ignore bouncing events inside the window
            }
            last_isr_time[evt.pin] = now;

            // 2. Wait a little bit to let the bouncing settle physically
            vTaskDelay(pdMS_TO_TICKS(10));

            // 3. Read the ACTUAL current hardware state
            int current_level = gpio_get_level(evt.pin);

            if (xSemaphoreTake(s_input_mutex, portMAX_DELAY))
            {

                switch (evt.pin)
                {
                case PIN_BTN_UP:
                    if (s_input_values.btn_up == !current_level)
                        break; // Check against ACTUAL level
                    s_input_values.btn_up = !current_level;
                    xQueueSend(s_input_queue_debounced, &s_input_values, 0);
                    ESP_LOGI(TAG, "BTN UP: %d", s_input_values.btn_up);
                    break;

                case PIN_BTN_DOWN:
                    if (s_input_values.btn_down == !current_level)
                        break;
                    s_input_values.btn_down = !current_level;
                    xQueueSend(s_input_queue_debounced, &s_input_values, 0);
                    ESP_LOGI(TAG, "BTN DOWN: %d", s_input_values.btn_down);
                    break;

                case PIN_BTN_STOP:
                    if (s_input_values.btn_stop == !current_level)
                        break;
                    s_input_values.btn_stop = !current_level;
                    xQueueSend(s_input_queue_debounced, &s_input_values, 0);
                    ESP_LOGI(TAG, "BTN STOP: %d", s_input_values.btn_stop);
                    break;

                case PIN_LIGHT_BARRIER:
                    // Assuming Light Barrier is also Active Low (Pull-Up)
                    if (s_input_values.light_barrier == !current_level)
                        break;
                    s_input_values.light_barrier = !current_level;
                    xQueueSend(s_input_queue_debounced, &s_input_values, 0);
                    ESP_LOGI(TAG, "LIGHT BARRIER: %d", s_input_values.light_barrier);
                    break;

                case PIN_EMERGENCY:
                    // Monitoring only
                    if (s_input_values.emergency_switch_state == !current_level)
                        break;
                    s_input_values.emergency_switch_state = !current_level;
                    xQueueSend(s_input_queue_debounced, &s_input_values, 0);
                    ESP_LOGW(TAG, "EMERGENCY SWITCH STATE: %d", s_input_values.emergency_switch_state);
                    break;

                case PIN_INDUCTIVE_SWITCH:
                    // Active High with external pull-down
                    if (s_input_values.inductive_switch == current_level)
                        break;
                    s_input_values.inductive_switch = current_level;
                    xQueueSend(s_input_queue_debounced, &s_input_values, 0);
                    ESP_LOGI(TAG, "INDUCTIVE SWITCH: %d", s_input_values.inductive_switch);
                    break;

                default:
                    break;
                }

                xSemaphoreGive(s_input_mutex);
            }
        }
    }
}

// --- Public API ---

esp_err_t digital_input_init(void)
{
    // 1. Create Synchronization Objects
    s_input_queue = xQueueCreate(10, sizeof(isr_event_t));
    if (s_input_queue == NULL)
        return ESP_ERR_NO_MEM;

    s_input_queue_debounced = xQueueCreate(10, sizeof(digital_inputs_t));
    if (s_input_queue_debounced == NULL)
        return ESP_ERR_NO_MEM;

    s_input_mutex = xSemaphoreCreateMutex();
    if (s_input_mutex == NULL)
        return ESP_ERR_NO_MEM;

    // 2. Configure GPIOs - Block 1: Pins with Pull-Up (Active Low)
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_ANYEDGE; // Interrupt on both edges
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1; // Internal Pull-up enabled

    // Bitmask creation using OR (|) and Shift (<<)
    io_conf.pin_bit_mask = (1ULL << PIN_BTN_UP) |
                           (1ULL << PIN_BTN_DOWN) |
                           (1ULL << PIN_BTN_STOP) |
                           (1ULL << PIN_LIGHT_BARRIER) |
                           (1ULL << PIN_EMERGENCY);

    esp_err_t err = gpio_config(&io_conf);
    if (err != ESP_OK)
        return err;

    // 2b. Configure GPIOs - Block 2: Inductive Switch with external Pull-Down
    gpio_config_t io_conf_inductive = {};
    io_conf_inductive.intr_type = GPIO_INTR_ANYEDGE; // Interrupt on both edges
    io_conf_inductive.mode = GPIO_MODE_INPUT;
    io_conf_inductive.pull_down_en = 0; // External pull-down used
    io_conf_inductive.pull_up_en = 0;   // No internal pull-up
    io_conf_inductive.pin_bit_mask = (1ULL << PIN_INDUCTIVE_SWITCH);

    err = gpio_config(&io_conf_inductive);
    if (err != ESP_OK)
        return err;

    // 3. Install ISR Service
    err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
    {
        // ESP_ERR_INVALID_STATE means service already installed (ok if called from other component)
        return err;
    }

    // 4. Add ISR Helpers
    gpio_isr_handler_add(PIN_BTN_UP, gpio_isr_handler, (void *)PIN_BTN_UP);
    gpio_isr_handler_add(PIN_BTN_DOWN, gpio_isr_handler, (void *)PIN_BTN_DOWN);
    gpio_isr_handler_add(PIN_BTN_STOP, gpio_isr_handler, (void *)PIN_BTN_STOP);
    gpio_isr_handler_add(PIN_LIGHT_BARRIER, gpio_isr_handler, (void *)PIN_LIGHT_BARRIER);
    gpio_isr_handler_add(PIN_EMERGENCY, gpio_isr_handler, (void *)PIN_EMERGENCY);
    gpio_isr_handler_add(PIN_INDUCTIVE_SWITCH, gpio_isr_handler, (void *)PIN_INDUCTIVE_SWITCH);

    // 5. Create Task
    BaseType_t ret = xTaskCreate(digital_input_task, "digital_input_task",
                                 CONFIG_DIGITAL_INPUT_TASK_STACK_SIZE, NULL,
                                 CONFIG_DIGITAL_INPUT_TASK_PRIORITY, NULL);

    if (ret != pdPASS)
        return ESP_FAIL;

    ESP_LOGI(TAG, "Digital Input Component Initialized");
    return ESP_OK;
}

digital_inputs_t digital_input_get_data(void)
{
    digital_inputs_t data_copy = {0};

    if (s_input_mutex)
    {
        if (xSemaphoreTake(s_input_mutex, portMAX_DELAY))
        {
            data_copy = s_input_values;
            xSemaphoreGive(s_input_mutex);
        }
    }
    return data_copy;
}

QueueHandle_t digital_input_get_queue(void)
{
    return s_input_queue_debounced;
}
