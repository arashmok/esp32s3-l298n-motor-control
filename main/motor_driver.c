/**
 * @file motor_driver.c
 * @brief Motor driver implementation with state machine
 */

#include "motor_driver.h"
#include "esp_log.h"
#include <stdlib.h>

static const char *TAG = "motor_driver";

// State names for logging
static const char *state_names[] = {
    "IDLE", "FORWARD", "BACKWARD", "ACCELERATING", 
    "DECELERATING", "EMERGENCY_STOP", "FAULT"
};

static esp_err_t transition_state(motor_driver_t *motor, motor_state_t new_state)
{
    if (motor->state == MOTOR_STATE_FAULT && new_state != MOTOR_STATE_IDLE) {
        ESP_LOGW(TAG, "Motor %d: Cannot transition from FAULT to %s", 
                 motor->id, state_names[new_state]);
        return MOTOR_ERR_INVALID_STATE;
    }
    
    ESP_LOGD(TAG, "Motor %d: %s -> %s", motor->id, 
             state_names[motor->state], state_names[new_state]);
             
    motor->state = new_state;
    return ESP_OK;
}

esp_err_t motor_driver_init(motor_driver_t *motor, motor_id_t id, 
                           const motor_hal_config_t *hal_config)
{
    if (!motor || !hal_config) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Initialize structure
    motor->id = id;
    motor->state = MOTOR_STATE_IDLE;
    motor->direction = MOTOR_DIR_FORWARD;
    motor->target_speed = 0;
    motor->current_speed = 0;
    motor->acceleration_rate = ACCELERATION_RATE;
    motor->update_count = 0;
    motor->last_update_time = 0;
    
    // Create mutex
    motor->mutex = xSemaphoreCreateMutex();
    if (!motor->mutex) {
        ESP_LOGE(TAG, "Failed to create mutex for motor %d", id);
        return ESP_ERR_NO_MEM;
    }
    
    // Initialize HAL
    esp_err_t ret = motor_hal_init(&motor->hal, hal_config);
    if (ret != ESP_OK) {
        vSemaphoreDelete(motor->mutex);
        return ret;
    }
    
    ESP_LOGI(TAG, "Motor %d initialized", id);
    return ESP_OK;
}

esp_err_t motor_driver_deinit(motor_driver_t *motor)
{
    if (!motor) {
        return ESP_ERR_INVALID_ARG;
    }
    
    xSemaphoreTake(motor->mutex, portMAX_DELAY);
    
    motor_hal_stop(&motor->hal);
    motor_hal_deinit(&motor->hal);
    
    xSemaphoreGive(motor->mutex);
    vSemaphoreDelete(motor->mutex);
    
    return ESP_OK;
}

esp_err_t motor_driver_set_speed(motor_driver_t *motor, int16_t speed)
{
    if (!motor) {
        return ESP_ERR_INVALID_ARG;
    }
    
    xSemaphoreTake(motor->mutex, portMAX_DELAY);
    
    // Clamp speed to valid range
    if (speed > PWM_MAX_DUTY) speed = PWM_MAX_DUTY;
    if (speed < -PWM_MAX_DUTY) speed = -PWM_MAX_DUTY;
    
    motor->target_speed = speed;
    
    // Determine direction
    if (speed > 0) {
        motor->direction = MOTOR_DIR_FORWARD;
    } else if (speed < 0) {
        motor->direction = MOTOR_DIR_BACKWARD;
    }
    
    // Update state
    if (abs(motor->target_speed) > 0) {
        if (motor->current_speed == 0) {
            transition_state(motor, MOTOR_STATE_ACCELERATING);
        } else if (abs(motor->target_speed) < abs(motor->current_speed)) {
            transition_state(motor, MOTOR_STATE_DECELERATING);
        } else {
            motor_state_t running_state = (motor->direction == MOTOR_DIR_FORWARD) ? 
                                         MOTOR_STATE_RUNNING_FORWARD : 
                                         MOTOR_STATE_RUNNING_BACKWARD;
            transition_state(motor, running_state);
        }
    } else {
        transition_state(motor, MOTOR_STATE_DECELERATING);
    }
    
    xSemaphoreGive(motor->mutex);
    return ESP_OK;
}

esp_err_t motor_driver_stop(motor_driver_t *motor)
{
    if (!motor) {
        return ESP_ERR_INVALID_ARG;
    }
    
    xSemaphoreTake(motor->mutex, portMAX_DELAY);
    
    motor->target_speed = 0;
    motor->current_speed = 0;
    motor_hal_stop(&motor->hal);
    transition_state(motor, MOTOR_STATE_IDLE);
    
    xSemaphoreGive(motor->mutex);
    return ESP_OK;
}

esp_err_t motor_driver_emergency_stop(motor_driver_t *motor)
{
    if (!motor) {
        return ESP_ERR_INVALID_ARG;
    }
    
    xSemaphoreTake(motor->mutex, portMAX_DELAY);
    
    motor->target_speed = 0;
    motor->current_speed = 0;
    motor_hal_emergency_stop(&motor->hal);
    transition_state(motor, MOTOR_STATE_EMERGENCY_STOP);
    
    xSemaphoreGive(motor->mutex);
    
    ESP_LOGW(TAG, "Motor %d: Emergency stop activated", motor->id);
    return ESP_OK;
}

esp_err_t motor_driver_set_acceleration(motor_driver_t *motor, uint8_t rate)
{
    if (!motor) {
        return ESP_ERR_INVALID_ARG;
    }
    
    xSemaphoreTake(motor->mutex, portMAX_DELAY);
    motor->acceleration_rate = (rate > 0) ? rate : 1;
    xSemaphoreGive(motor->mutex);
    
    return ESP_OK;
}

esp_err_t motor_driver_update(motor_driver_t *motor)
{
    if (!motor) {
        return ESP_ERR_INVALID_ARG;
    }
    
    xSemaphoreTake(motor->mutex, portMAX_DELAY);
    
    motor->update_count++;
    motor->last_update_time = xTaskGetTickCount();
    
    switch (motor->state) {
        case MOTOR_STATE_ACCELERATING:
            if (abs(motor->current_speed) < abs(motor->target_speed)) {
                // Accelerate
                if (motor->target_speed > 0) {
                    motor->current_speed += motor->acceleration_rate;
                    if (motor->current_speed > motor->target_speed) {
                        motor->current_speed = motor->target_speed;
                    }
                } else {
                    motor->current_speed -= motor->acceleration_rate;
                    if (motor->current_speed < motor->target_speed) {
                        motor->current_speed = motor->target_speed;
                    }
                }
                
                // Update hardware
                motor_hal_set_direction(&motor->hal, motor->direction);
                motor_hal_set_speed(&motor->hal, abs(motor->current_speed));
                
                // Check if target reached
                if (motor->current_speed == motor->target_speed) {
                    motor_state_t running_state = (motor->direction == MOTOR_DIR_FORWARD) ? 
                                                 MOTOR_STATE_RUNNING_FORWARD : 
                                                 MOTOR_STATE_RUNNING_BACKWARD;
                    transition_state(motor, running_state);
                }
            }
            break;
            
        case MOTOR_STATE_DECELERATING:
            if (motor->current_speed != motor->target_speed) {
                // Decelerate
                if (motor->current_speed > 0) {
                    motor->current_speed -= motor->acceleration_rate;
                    if (motor->current_speed < motor->target_speed) {
                        motor->current_speed = motor->target_speed;
                    }
                } else {
                    motor->current_speed += motor->acceleration_rate;
                    if (motor->current_speed > motor->target_speed) {
                        motor->current_speed = motor->target_speed;
                    }
                }
                
                // Update hardware
                motor_hal_set_speed(&motor->hal, abs(motor->current_speed));
                
                // Check if stopped
                if (motor->current_speed == 0) {
                    motor_hal_stop(&motor->hal);
                    transition_state(motor, MOTOR_STATE_IDLE);
                } else if (motor->current_speed == motor->target_speed) {
                    motor_state_t running_state = (motor->direction == MOTOR_DIR_FORWARD) ? 
                                                 MOTOR_STATE_RUNNING_FORWARD : 
                                                 MOTOR_STATE_RUNNING_BACKWARD;
                    transition_state(motor, running_state);
                }
            }
            break;
            
        case MOTOR_STATE_RUNNING_FORWARD:
        case MOTOR_STATE_RUNNING_BACKWARD:
            // Check if speed target changed
            if (motor->current_speed != motor->target_speed) {
                if (abs(motor->current_speed) < abs(motor->target_speed)) {
                    transition_state(motor, MOTOR_STATE_ACCELERATING);
                } else {
                    transition_state(motor, MOTOR_STATE_DECELERATING);
                }
            }
            break;
            
        case MOTOR_STATE_IDLE:
        case MOTOR_STATE_EMERGENCY_STOP:
        case MOTOR_STATE_FAULT:
            // No automatic updates in these states
            break;
    }
    
    xSemaphoreGive(motor->mutex);
    return ESP_OK;
}

motor_state_t motor_driver_get_state(motor_driver_t *motor)
{
    if (!motor) {
        return MOTOR_STATE_FAULT;
    }
    
    xSemaphoreTake(motor->mutex, portMAX_DELAY);
    motor_state_t state = motor->state;
    xSemaphoreGive(motor->mutex);
    
    return state;
}

int16_t motor_driver_get_speed(motor_driver_t *motor)
{
    if (!motor) {
        return 0;
    }
    
    xSemaphoreTake(motor->mutex, portMAX_DELAY);
    int16_t speed = motor->current_speed;
    xSemaphoreGive(motor->mutex);
    
    return speed;
}