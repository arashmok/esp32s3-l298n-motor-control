/**
 * @file motor_driver.h
 * @brief High-level motor driver with state management
 */

#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include "motor_control.h"
#include "motor_hal.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#ifdef __cplusplus
extern "C" {
#endif

// Motor Control Structure
typedef struct {
    motor_id_t id;
    motor_hal_t hal;
    motor_state_t state;
    motor_direction_t direction;
    
    // Speed control with acceleration
    int16_t target_speed;      // -1023 to +1023
    int16_t current_speed;     // Actual current speed
    uint8_t acceleration_rate;
    
    // Synchronization
    SemaphoreHandle_t mutex;
    
    // Statistics
    uint32_t update_count;
    TickType_t last_update_time;
} motor_driver_t;

// Driver Function Prototypes
esp_err_t motor_driver_init(motor_driver_t *motor, motor_id_t id, 
                           const motor_hal_config_t *hal_config);
esp_err_t motor_driver_deinit(motor_driver_t *motor);
esp_err_t motor_driver_set_speed(motor_driver_t *motor, int16_t speed);
esp_err_t motor_driver_stop(motor_driver_t *motor);
esp_err_t motor_driver_emergency_stop(motor_driver_t *motor);
esp_err_t motor_driver_set_acceleration(motor_driver_t *motor, uint8_t rate);
esp_err_t motor_driver_update(motor_driver_t *motor);
motor_state_t motor_driver_get_state(motor_driver_t *motor);
int16_t motor_driver_get_speed(motor_driver_t *motor);

#ifdef __cplusplus
}
#endif

#endif // MOTOR_DRIVER_H