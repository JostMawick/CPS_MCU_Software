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

    // 2. Main Loop
    while (1)
    {
        // For testing: Get data and print it every second
         /*
         digital_inputs_t inputs = digital_input_get_data();

         // Updated debug fields to match refactored digital_inputs_t
         printf("Inputs: HandguardR: %d | HandguardL: %d | Reset: %d | LightgateStart: %d | LightgateEnd: %d | Emergency: %d | Inductive: %d\n",
             inputs.handguard_right,
             inputs.handguard_left,
             inputs.reset_btn,
             inputs.lightgate_start,
             inputs.lightgate_end,
             inputs.emergency_btn,
             inputs.inductive_switch);

         printf("Revolutions %lf\n", bdc_driver_get_revolutions());
         printf("Speed RPS %f\n", bdc_driver_get_speed_rps());

         printf("total ticks: %d\n", bdc_driver_get_pulse_count());
         printf("current ticks: %d\n", bdc_driver_get_report_pulses());
         printf("Current FSM State: %s\n", main_fsm_get_state_string());

         */

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}