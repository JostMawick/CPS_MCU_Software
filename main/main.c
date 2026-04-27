#include <stdio.h>
#include <fcntl.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "digital_input.h"
#include "esp_err.h"
#include "bdc_motor_driver.h"
#include "led_control.h"
#include "position_control.h"
#include "servo_control.h"
#include "main_fsm.h"

void app_main(void)
{
    // 1. Initialize Components
    printf("Starting initialization...\n");

    // Init Digital Inputs
    esp_err_t err = digital_input_init();
    if (err == ESP_OK)
    {
        printf("Digital Input init successful.\n");
    }
    else
    {
        printf("Digital Input init FAILED: %s\n", esp_err_to_name(err));
    }

    // Init Motor Driver
    err = bdc_motor_driver_init();
    if (err == ESP_OK)
    {
        printf("BDC Motor Driver init successful.\n");
    }
    else
    {
        printf("BDC Motor Driver init FAILED: %s\n", esp_err_to_name(err));
    }

    // Init Position Control
    err = position_control_init();
    if (err == ESP_OK)
    {
        printf("Position Control init successful.\n");
    }
    else
    {
        printf("Position Control init FAILED: %s\n", esp_err_to_name(err));
    }

    // Init LED Control
    err = led_control_init();
    if (err == ESP_OK)
    {
        printf("LED Control init successful.\n");
    }
    else
    {
        printf("LED Control init FAILED: %s\n", esp_err_to_name(err));
    }

    // Init Servo Control
    err = servo_control_init();
    if (err == ESP_OK)
    {
        printf("Servo Control init successful.\n");
    }
    else
    {
        printf("Servo Control init FAILED: %s\n", esp_err_to_name(err));
    }

    err = main_fsm_init();
    if (err == ESP_OK)
    {
        printf("Main FSM init successful.\n");
    }
    else
    {
        printf("Main FSM init FAILED: %s\n", esp_err_to_name(err));
    }

    printf("V3\n");

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}