#include <stdio.h>
#include <math.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "bdc_motor_driver.h"
#include "position_control.h"

static const char *TAG = "POS_CTRL";

#define PULLEY_RADIUS_MM CONFIG_POS_CTRL_PULLEY_RADIUS_MM
#define LAYER_THICKNESS_MM CONFIG_POS_CTRL_LAYER_THICKNESS_MM
#define MAX_POSITION_MM CONFIG_POS_CTRL_MAX_POSITION_MM
#define MIN_POSITION_MM CONFIG_POS_CTRL_MIN_POSITION_MM
#define MAX_REVOLUTIONS CONFIG_POS_CTRL_MAX_REVOLUTIONS
#define MIN_REVOLUTIONS CONFIG_POS_CTRL_MIN_REVOLUTIONS
#define TOLERANCE_MM CONFIG_POS_CTRL_TOLERANCE_MM
#define LOOP_PERIOD_MS CONFIG_POS_CTRL_LOOP_PERIOD_MS
#define POS_CTRL_KP 1.5f

typedef struct
{
    float target_revolutions;
    bool holding_position;
    bool target_reached;

    // Trajectory variables
    bool trajectory_active;
    float current_trajectory_revs;
    float trajectory_step_revs_per_loop;
    float trajectory_end_revs;
} position_control_context_t;

static position_control_context_t pos_ctrl_ctx = {
    .target_revolutions = 0.0f,
    .holding_position = false,
    .trajectory_active = false,
    .current_trajectory_revs = 0.0f,
    .trajectory_step_revs_per_loop = 0.0f,
    .trajectory_end_revs = 0.0f};

/*------------Prototype Declarations------------*/
static void position_control_loop_cb(void *arg);
/*----------------------------------------------*/

esp_err_t position_control_init(void)
{
    ESP_LOGI(TAG, "Initializing Position Control...");

    ESP_LOGI(TAG, "Creating position control loop timer");
    const esp_timer_create_args_t periodic_timer_args = {
        .callback = &position_control_loop_cb,
        .arg = &pos_ctrl_ctx,
        .name = "pos_ctrl_loop"};

    esp_timer_handle_t pos_ctrl_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &pos_ctrl_timer));

    ESP_LOGI(TAG, "Starting position control loop timer (%d ms)", LOOP_PERIOD_MS);
    ESP_ERROR_CHECK(esp_timer_start_periodic(pos_ctrl_timer, LOOP_PERIOD_MS * 1000));

    ESP_LOGI(TAG, "Position Control initialized successfully");
    return ESP_OK;
}

static void position_control_loop_cb(void *arg)
{
    position_control_context_t *ctx = (position_control_context_t *)arg;

    bool interpolation_reached = false;

    // --- Trajectory Generation ---
    if (ctx->trajectory_active)
    {
        // Move the ghost target by the step size
        ctx->current_trajectory_revs += ctx->trajectory_step_revs_per_loop;

        // Check if we reached or overshot the final target
        interpolation_reached = false;
        if (ctx->trajectory_step_revs_per_loop > 0 && ctx->current_trajectory_revs >= ctx->trajectory_end_revs)
        {
            interpolation_reached = true;
        }
        else if (ctx->trajectory_step_revs_per_loop < 0 && ctx->current_trajectory_revs <= ctx->trajectory_end_revs)
        {
            interpolation_reached = true;
        }

        if (interpolation_reached)
        {
            ctx->trajectory_active = false;
            ctx->target_revolutions = ctx->trajectory_end_revs; // Snap cleanly to the end target
        }
        else
        {
            ctx->target_revolutions = ctx->current_trajectory_revs; // Feed the moving target to the P-Controller
        }
    }
    // -----------------------------

    float current_revs = bdc_driver_get_revolutions();
    float error = ctx->target_revolutions - current_revs;

    // Calculate the deadband tolerance in revolutions (maybe it needs some extra margin to account for the layered curtain thickness)
    float circumference_mm = 2.0f * M_PI * PULLEY_RADIUS_MM;
    float tolerance_revs = TOLERANCE_MM / circumference_mm;

    // Check if we reached the target position
    if (fabsf(error) <= tolerance_revs)
    {
        bdc_driver_set_pid_speed(0.0f);
        ctx->target_reached = true;
        return;
    }

    // P-Controller logic
    float target_speed_rps = error * POS_CTRL_KP;
    bdc_driver_set_pid_speed(target_speed_rps);
}

esp_err_t position_control_set_revolutions(float revolutions)
{

    if (revolutions < MIN_REVOLUTIONS || revolutions > MAX_REVOLUTIONS)
    {
        ESP_LOGE(TAG, "Target revolutions %.2f is out of bounds [%d, %d]", revolutions, MIN_REVOLUTIONS, MAX_REVOLUTIONS);
        return ESP_ERR_INVALID_ARG;
    }

    pos_ctrl_ctx.target_revolutions = revolutions;
    // pos_ctrl_ctx.trajectory_end_revs = revolutions;
    pos_ctrl_ctx.target_reached = false;
    pos_ctrl_ctx.holding_position = false;
    pos_ctrl_ctx.trajectory_active = false; // Cancel any active trajectory if a hard position is set

    ESP_LOGI(TAG, "New Target Set: %.2f revolutions", revolutions);
    return ESP_OK;
}

esp_err_t position_control_set_position_mm(float position_mm)
{

    if (position_mm < MIN_POSITION_MM || position_mm > MAX_POSITION_MM)
    {
        ESP_LOGE(TAG, "Target position %.2f mm is out of bounds [%d, %d]", position_mm, MIN_POSITION_MM, MAX_POSITION_MM);
        return ESP_ERR_INVALID_ARG;
    }

    // Basic calculation for now: constant pulley radius
    // Circumference C = 2 * pi * r
    // revolutions = distance / C
    // add complex layered logic here if needed in the future
    float circumference_mm = 2.0f * M_PI * PULLEY_RADIUS_MM;

    float target_revs = position_mm / circumference_mm;

    pos_ctrl_ctx.target_revolutions = target_revs;
    pos_ctrl_ctx.trajectory_end_revs = target_revs;
    pos_ctrl_ctx.target_reached = false;
    pos_ctrl_ctx.holding_position = false;
    pos_ctrl_ctx.trajectory_active = false; // Cancel any active trajectory if a hard position is set

    return ESP_OK;
}
typedef struct
{
    float last_mm;
    float last_sec;
} last_trajectory_traget_t;

static last_trajectory_traget_t last_trajectory_target = {
    .last_mm = 0.0f,
    .last_sec = 0.0f};

esp_err_t position_control_set_trajectory_mm(float position_mm, float duration_sec)
{

    pos_ctrl_ctx.trajectory_active = true;
    pos_ctrl_ctx.holding_position = false;
    if (position_mm == last_trajectory_target.last_mm && duration_sec == last_trajectory_target.last_sec)
    {
        // ESP_LOGW(TAG, "Requested trajectory is identical to the last one. Ignoring redundant command.");
        return ESP_OK;
    }

    last_trajectory_target.last_mm = position_mm;
    last_trajectory_target.last_sec = duration_sec;

    if (position_mm < MIN_POSITION_MM || position_mm > MAX_POSITION_MM)
    {
        ESP_LOGE(TAG, "Target position %.2f mm is out of bounds [%d, %d]", position_mm, MIN_POSITION_MM, MAX_POSITION_MM);
        return ESP_ERR_INVALID_ARG;
    }

    if (duration_sec <= 0.0f)
    {
        ESP_LOGE(TAG, "Duration must be greater than 0");
        return ESP_ERR_INVALID_ARG;
    }

    float circumference_mm = 2.0f * M_PI * PULLEY_RADIUS_MM;
    float final_target_revs = position_mm / circumference_mm;
    float current_revs = bdc_driver_get_revolutions();

    float delta_revs = final_target_revs - current_revs;

    // Check required speed
    float speed_rps = fabsf(delta_revs) / duration_sec;
    if (speed_rps > 0.6f)
    {
        ESP_LOGE(TAG, "Requested trajectory requires %.2f rps, which exceeds the maximum of 0.6 rps! Lengthen duration.", speed_rps);
        return ESP_ERR_INVALID_ARG;
    }

    // Calculate step per loop
    float total_loops = duration_sec / (LOOP_PERIOD_MS / 1000.0f);
    float step_per_loop = delta_revs / total_loops;

    // Apply to context
    pos_ctrl_ctx.trajectory_end_revs = final_target_revs;
    pos_ctrl_ctx.target_reached = false;
    pos_ctrl_ctx.trajectory_step_revs_per_loop = step_per_loop;
    pos_ctrl_ctx.current_trajectory_revs = current_revs;

    pos_ctrl_ctx.trajectory_active = true;
    pos_ctrl_ctx.holding_position = false;

    ESP_LOGI(TAG, "Trajectory started: Target: %.2f revs, Duration: %.2fs, Speed: %.3f rps, Step/Loop: %.5f",
             final_target_revs, duration_sec, speed_rps, step_per_loop);

    return ESP_OK;
}

esp_err_t position_control_hold_position(void)
{
    if (!pos_ctrl_ctx.holding_position)
    {
        bdc_driver_set_pwm(0.0f);
        // bdc_driver_set_pid_speed(0.0f);
        position_control_set_revolutions(bdc_driver_get_revolutions());
        pos_ctrl_ctx.holding_position = true;
        pos_ctrl_ctx.trajectory_active = false; // Cancel active trajectories
    }
    return ESP_OK;
}

bool position_control_is_target_reached(void)
{

    float circumference_mm = 2.0f * M_PI * PULLEY_RADIUS_MM;
    float tolerance_revs = TOLERANCE_MM / circumference_mm;

    // Check if we reached the target position
    if (fabsf(bdc_driver_get_revolutions() - pos_ctrl_ctx.trajectory_end_revs) <= tolerance_revs)
    {
        return true;
    }
    return false;
}