#include <stdio.h>
#include <math.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/pulse_cnt.h"
#include "bdc_motor.h"
#include "pid_ctrl.h"
#include "bdc_motor_driver.h"

static const char *TAG = "BDC_MOTOR_DRIVER";

// Enable this config to print debug formated string for Serial-Studio
#define SERIAL_STUDIO_DEBUG CONFIG_BDC_MOTOR_SERIAL_STUDIO_DEBUG
#define QUADRATURE_ENCODER_ENABLE CONFIG_BDC_MOTOR_ENABLE_QUADRATURE_ENCODER

#define BDC_MCPWM_TIMER_RESOLUTION_HZ 10000000 // 10MHz, 1 tick = 0.1us
#define BDC_MCPWM_FREQ_HZ CONFIG_BDC_MOTOR_PWM_FREQ_HZ
#define BDC_MCPWM_DUTY_TICK_MAX (BDC_MCPWM_TIMER_RESOLUTION_HZ / BDC_MCPWM_FREQ_HZ)

#define BDC_MCPWM_GPIO_A CONFIG_BDC_MOTOR_PWM_GPIO_A
#define BDC_MCPWM_GPIO_B CONFIG_BDC_MOTOR_PWM_GPIO_B

#define BDC_ENCODER_GPIO_A CONFIG_BDC_MOTOR_ENCODER_GPIO_A
#define BDC_ENCODER_GPIO_B CONFIG_BDC_MOTOR_ENCODER_GPIO_B
#define BDC_ENCODER_PCNT_HIGH_LIMIT CONFIG_BDC_MOTOR_PCNT_HIGH_LIMIT
#define BDC_ENCODER_PCNT_LOW_LIMIT CONFIG_BDC_MOTOR_PCNT_LOW_LIMIT
#define BDC_ENCODER_RESOLUTION CONFIG_BDC_MOTOR_ENCODER_RESOLUTION
#define BDC_ENCODER_ENABLE_QUADRATURE CONFIG_BDC_MOTOR_ENABLE_QUADRATURE_ENCODER

#define BDC_PID_LOOP_PERIOD_MS CONFIG_BDC_MOTOR_PID_LOOP_PERIOD_MS

#define PID_KP 0.6
#define PID_TI 0.4
#define PID_TD 0.2

typedef struct
{
    bdc_motor_handle_t motor;
    pcnt_unit_handle_t pcnt_encoder;
    pid_ctrl_block_handle_t pid_ctrl;
    int report_pulses;
    float report_speed_rps;
    bool pid_enabled;
    bool direction_forward;
    int pid_expect_speed_in_ticks;
} motor_control_context_t;

static motor_control_context_t motor_ctrl_ctx = {
    .pid_expect_speed_in_ticks = BDC_ENCODER_RESOLUTION * BDC_PID_LOOP_PERIOD_MS * 0.001f,
};

#if SERIAL_STUDIO_DEBUG
static void serial_studio_task(void *arg)
{
    while (1)
    {
        printf("/*%f*/\r\n", bdc_driver_get_speed_rps());
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
#endif

static void pid_loop_cb(void *args)
{
    static int last_pulse_count = 0;
    motor_control_context_t *ctx = (motor_control_context_t *)args;
    pcnt_unit_handle_t pcnt_unit = ctx->pcnt_encoder;
    pid_ctrl_block_handle_t pid_ctrl = ctx->pid_ctrl;
    bdc_motor_handle_t motor = ctx->motor;

    // get the result from rotary encoder
    int cur_pulse_count = 0;
    pcnt_unit_get_count(pcnt_unit, &cur_pulse_count);
    int real_pulses = cur_pulse_count - last_pulse_count;
    last_pulse_count = cur_pulse_count;

#if !QUADRATURE_ENCODER_ENABLE
    if (!ctx->direction_forward)
    {
        real_pulses = -real_pulses;
    }
#endif

    ctx->report_pulses = real_pulses;
    ctx->report_speed_rps = (real_pulses / (BDC_PID_LOOP_PERIOD_MS / 1000.0f)) / BDC_ENCODER_RESOLUTION;

    // calculate the speed error
    float error = ctx->pid_expect_speed_in_ticks - abs(real_pulses); // expected speed and real pulses must be positive
    float new_speed = 0;

    /*
        // set the new speed
        if (ctx->pid_enabled)
        {
            pid_compute(pid_ctrl, error, &new_speed);
            bdc_motor_set_speed(motor, (uint32_t)new_speed);
        }
    */
}

esp_err_t bdc_motor_driver_init(void)
{
    ESP_LOGI(TAG, "Create DC motor");
    bdc_motor_config_t motor_config = {
        .pwm_freq_hz = BDC_MCPWM_FREQ_HZ,
        .pwma_gpio_num = BDC_MCPWM_GPIO_A,
        .pwmb_gpio_num = BDC_MCPWM_GPIO_B,
    };
    bdc_motor_mcpwm_config_t mcpwm_config = {
        .group_id = 0,
        .resolution_hz = BDC_MCPWM_TIMER_RESOLUTION_HZ,
    };
    bdc_motor_handle_t motor = NULL;
    ESP_ERROR_CHECK(bdc_motor_new_mcpwm_device(&motor_config, &mcpwm_config, &motor));
    motor_ctrl_ctx.motor = motor;

    ESP_LOGI(TAG, "Init pcnt driver to decode rotary signal");
    pcnt_unit_config_t unit_config = {
        .high_limit = BDC_ENCODER_PCNT_HIGH_LIMIT,
        .low_limit = BDC_ENCODER_PCNT_LOW_LIMIT,
        .flags.accum_count = true, // enable counter accumulation
    };
    pcnt_unit_handle_t pcnt_unit = NULL;
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));
    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 1000,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config));

#if BDC_ENCODER_ENABLE_QUADRATURE
    ESP_LOGI(TAG, "Configure PCNT for quadrature encoder");
    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num = BDC_ENCODER_GPIO_A,
        .level_gpio_num = BDC_ENCODER_GPIO_B,
    };
    pcnt_channel_handle_t pcnt_chan_a = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_a_config, &pcnt_chan_a));
    pcnt_chan_config_t chan_b_config = {
        .edge_gpio_num = BDC_ENCODER_GPIO_B,
        .level_gpio_num = BDC_ENCODER_GPIO_A,
    };
    pcnt_channel_handle_t pcnt_chan_b = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_b_config, &pcnt_chan_b));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

#else
    ESP_LOGI(TAG, "Configure PCNT for single channel encoder");
    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num = BDC_ENCODER_GPIO_A,
        .level_gpio_num = -1,
    };
    pcnt_channel_handle_t pcnt_chan_a = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_a_config, &pcnt_chan_a));

    // Count up on rising edge, do nothing on falling edge
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD));
    // Ignore level
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_KEEP));
#endif

    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, BDC_ENCODER_PCNT_HIGH_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, BDC_ENCODER_PCNT_LOW_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));

    motor_ctrl_ctx.pcnt_encoder = pcnt_unit;

    ESP_LOGI(TAG, "Create PID control block");
    pid_ctrl_parameter_t pid_runtime_param = {
        .kp = PID_KP,
        .ki = PID_TI,
        .kd = PID_TD,
        .cal_type = PID_CAL_TYPE_INCREMENTAL,
        .max_output = BDC_MCPWM_DUTY_TICK_MAX - 1,
        .min_output = 0,
        .max_integral = 1000,
        .min_integral = -1000,
    };
    pid_ctrl_block_handle_t pid_ctrl = NULL;
    pid_ctrl_config_t pid_config = {
        .init_param = pid_runtime_param,
    };
    ESP_ERROR_CHECK(pid_new_control_block(&pid_config, &pid_ctrl));
    motor_ctrl_ctx.pid_ctrl = pid_ctrl;

    ESP_LOGI(TAG, "Create a timer to do PID calculation periodically");
    const esp_timer_create_args_t periodic_timer_args = {
        .callback = pid_loop_cb,
        .arg = &motor_ctrl_ctx,
        .name = "pid_loop"};
    esp_timer_handle_t pid_loop_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &pid_loop_timer));

    ESP_LOGI(TAG, "Enable motor");
    ESP_ERROR_CHECK(bdc_motor_enable(motor));
    ESP_LOGI(TAG, "coast motor");
    ESP_ERROR_CHECK(bdc_motor_coast(motor));

    ESP_LOGI(TAG, "Start motor speed loop");
    motor_ctrl_ctx.pid_enabled = true;
    ESP_ERROR_CHECK(esp_timer_start_periodic(pid_loop_timer, BDC_PID_LOOP_PERIOD_MS * 1000));

#if SERIAL_STUDIO_DEBUG
    xTaskCreate(serial_studio_task, "serial_studio_task", 4096, NULL, 5, NULL);
#endif

    return ESP_OK;
}

esp_err_t bdc_driver_set_pid_speed(float rps)
{
    motor_ctrl_ctx.pid_enabled = true;
    if (rps > 0)
    {
        ESP_ERROR_CHECK(bdc_motor_forward(motor_ctrl_ctx.motor));
        motor_ctrl_ctx.direction_forward = true;
    }
    else if (rps < 0)
    {
        ESP_ERROR_CHECK(bdc_motor_reverse(motor_ctrl_ctx.motor));
        motor_ctrl_ctx.direction_forward = false;
    }
    rps = fabsf(rps);
    float target_pulses_per_sec = rps * BDC_ENCODER_RESOLUTION;
    float pulses_per_loop = target_pulses_per_sec * (BDC_PID_LOOP_PERIOD_MS / 1000.0f);
    motor_ctrl_ctx.pid_expect_speed_in_ticks = (int)pulses_per_loop;
    return ESP_OK;
}

esp_err_t bdc_driver_set_pwm(float duty_cycle)
{
    motor_ctrl_ctx.pid_enabled = false;
    if (duty_cycle > 0)
    {
        ESP_ERROR_CHECK(bdc_motor_forward(motor_ctrl_ctx.motor));
        motor_ctrl_ctx.direction_forward = true;
    }

    if (duty_cycle < 0)
    {
        ESP_ERROR_CHECK(bdc_motor_reverse(motor_ctrl_ctx.motor));
        motor_ctrl_ctx.direction_forward = false;
    }

    if (duty_cycle == 0)
    {
        ESP_ERROR_CHECK(bdc_motor_brake(motor_ctrl_ctx.motor));
    }

    if (duty_cycle > 1.0f)
    {
        duty_cycle = 1.0f;
    }

    if (duty_cycle < -1.0f)
    {
        duty_cycle = -1.0f;
    }

    duty_cycle = fabsf(duty_cycle);
    float pwm_duty_as_ticks = duty_cycle * BDC_MCPWM_DUTY_TICK_MAX;
    bdc_motor_set_speed(motor_ctrl_ctx.motor, (uint32_t)pwm_duty_as_ticks);
    return ESP_OK;
}

float bdc_driver_get_speed_rps(void)
{
    return motor_ctrl_ctx.report_speed_rps;
}
#if !QUADRATURE_ENCODER_ENABLE
static int last_count = 0;
#endif
float bdc_driver_get_revolutions(void)
{
    int count = 0;
    ESP_ERROR_CHECK(pcnt_unit_get_count(motor_ctrl_ctx.pcnt_encoder, &count));

#if !QUADRATURE_ENCODER_ENABLE
    if (motor_ctrl_ctx.direction_forward)
    {
        last_count = count;
    }
    count = last_count - (count - last_count);
#endif

    float revolutions = (float)count / BDC_ENCODER_RESOLUTION;
    return revolutions;
}

int bdc_driver_get_pulse_count(void)
{
    int count = 0;
    ESP_ERROR_CHECK(pcnt_unit_get_count(motor_ctrl_ctx.pcnt_encoder, &count));
    return count;
}

int bdc_driver_get_report_pulses(void)
{
    return motor_ctrl_ctx.report_pulses;
}