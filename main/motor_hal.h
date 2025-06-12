/**
 * @file motor_hal.h
 * @brief Hardware Abstraction Layer for motor control
 */

#ifndef MOTOR_HAL_H
#define MOTOR_HAL_H

#include "motor_control.h"
#include "driver/ledc.h"

#ifdef __cplusplus
extern "C" {
#endif

// HAL Configuration Structure
typedef struct {
    gpio_num_t in1_gpio;
    gpio_num_t in2_gpio;
    gpio_num_t enable_gpio;
    ledc_channel_t pwm_channel;
    ledc_timer_t pwm_timer;
} motor_hal_config_t;

// HAL Handle Structure
typedef struct {
    motor_hal_config_t config;
    bool initialized;
    uint32_t current_duty;
    motor_direction_t current_direction;
} motor_hal_t;

// HAL Function Prototypes
esp_err_t motor_hal_init(motor_hal_t *hal, const motor_hal_config_t *config);
esp_err_t motor_hal_deinit(motor_hal_t *hal);
esp_err_t motor_hal_set_direction(motor_hal_t *hal, motor_direction_t direction);
esp_err_t motor_hal_set_speed(motor_hal_t *hal, uint32_t duty_cycle);
esp_err_t motor_hal_stop(motor_hal_t *hal);
esp_err_t motor_hal_emergency_stop(motor_hal_t *hal);
uint32_t motor_hal_get_speed(motor_hal_t *hal);

#ifdef __cplusplus
}
#endif

#endif // MOTOR_HAL_H