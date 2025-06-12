/**
 * @file motor_hal.c
 * @brief Hardware Abstraction Layer implementation
 */

#include "motor_hal.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "motor_hal";

// Initialize PWM timer (shared between channels)
static bool pwm_timer_initialized = false;

static esp_err_t init_pwm_timer(ledc_timer_t timer)
{
    if (pwm_timer_initialized) {
        return ESP_OK;
    }

    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = timer,
        .duty_resolution = PWM_RESOLUTION,
        .freq_hz = PWM_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK
    };

    esp_err_t ret = ledc_timer_config(&ledc_timer);
    if (ret == ESP_OK) {
        pwm_timer_initialized = true;
    }
    return ret;
}

esp_err_t motor_hal_init(motor_hal_t *hal, const motor_hal_config_t *config)
{
    if (!hal || !config) {
        return ESP_ERR_INVALID_ARG;
    }

    // Copy configuration
    memcpy(&hal->config, config, sizeof(motor_hal_config_t));

    // Configure direction control pins
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << config->in1_gpio) | (1ULL << config->in2_gpio),
        .pull_down_en = 0,
        .pull_up_en = 0
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    // Initialize PWM timer
    ESP_ERROR_CHECK(init_pwm_timer(config->pwm_timer));

    // Configure PWM channel
    ledc_channel_config_t ledc_channel = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = config->pwm_channel,
        .timer_sel = config->pwm_timer,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = config->enable_gpio,
        .duty = 0,
        .hpoint = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    // Initialize to stopped state
    hal->initialized = true;
    hal->current_duty = 0;
    hal->current_direction = MOTOR_DIR_FORWARD;
    
    motor_hal_stop(hal);
    
    ESP_LOGI(TAG, "Motor HAL initialized - IN1:%d, IN2:%d, EN:%d", 
             config->in1_gpio, config->in2_gpio, config->enable_gpio);
             
    return ESP_OK;
}

esp_err_t motor_hal_deinit(motor_hal_t *hal)
{
    if (!hal || !hal->initialized) {
        return MOTOR_ERR_NOT_INIT;
    }

    motor_hal_stop(hal);
    hal->initialized = false;
    
    return ESP_OK;
}

esp_err_t motor_hal_set_direction(motor_hal_t *hal, motor_direction_t direction)
{
    if (!hal || !hal->initialized) {
        return MOTOR_ERR_NOT_INIT;
    }

    switch (direction) {
        case MOTOR_DIR_FORWARD:
            gpio_set_level(hal->config.in1_gpio, 1);
            gpio_set_level(hal->config.in2_gpio, 0);
            break;
            
        case MOTOR_DIR_BACKWARD:
            gpio_set_level(hal->config.in1_gpio, 0);
            gpio_set_level(hal->config.in2_gpio, 1);
            break;
            
        case MOTOR_DIR_BRAKE:
            gpio_set_level(hal->config.in1_gpio, 1);
            gpio_set_level(hal->config.in2_gpio, 1);
            break;
            
        default:
            return ESP_ERR_INVALID_ARG;
    }

    hal->current_direction = direction;
    return ESP_OK;
}

esp_err_t motor_hal_set_speed(motor_hal_t *hal, uint32_t duty_cycle)
{
    if (!hal || !hal->initialized) {
        return MOTOR_ERR_NOT_INIT;
    }

    if (duty_cycle > PWM_MAX_DUTY) {
        duty_cycle = PWM_MAX_DUTY;
    }

    esp_err_t ret = ledc_set_duty(LEDC_LOW_SPEED_MODE, 
                                  hal->config.pwm_channel, 
                                  duty_cycle);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = ledc_update_duty(LEDC_LOW_SPEED_MODE, hal->config.pwm_channel);
    if (ret == ESP_OK) {
        hal->current_duty = duty_cycle;
    }

    return ret;
}

esp_err_t motor_hal_stop(motor_hal_t *hal)
{
    if (!hal || !hal->initialized) {
        return MOTOR_ERR_NOT_INIT;
    }

    // Set speed to 0
    motor_hal_set_speed(hal, 0);
    
    // Set both pins low (coast to stop)
    gpio_set_level(hal->config.in1_gpio, 0);
    gpio_set_level(hal->config.in2_gpio, 0);
    
    return ESP_OK;
}

esp_err_t motor_hal_emergency_stop(motor_hal_t *hal)
{
    if (!hal || !hal->initialized) {
        return MOTOR_ERR_NOT_INIT;
    }

    // Set speed to 0
    motor_hal_set_speed(hal, 0);
    
    // Brake mode
    motor_hal_set_direction(hal, MOTOR_DIR_BRAKE);
    
    return ESP_OK;
}

uint32_t motor_hal_get_speed(motor_hal_t *hal)
{
    if (!hal || !hal->initialized) {
        return 0;
    }
    
    return hal->current_duty;
}