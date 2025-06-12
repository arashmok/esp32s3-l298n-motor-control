/**
 * @file motor_control.h
 * @brief Common definitions for ESP32-S3 motor control system
 */

#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdint.h>
#include "esp_err.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

// GPIO Pin Definitions - Safe pins for ESP32-S3 Feather
// Using pins that don't conflict with boot, flash, or onboard peripherals
#define MOTOR_A_IN1_GPIO    GPIO_NUM_5   // D5 - Safe
#define MOTOR_A_IN2_GPIO    GPIO_NUM_6   // D6 - Safe
#define MOTOR_A_ENA_GPIO    GPIO_NUM_9   // D9 - Safe

#define MOTOR_B_IN3_GPIO    GPIO_NUM_10  // D10 - Safe
#define MOTOR_B_IN4_GPIO    GPIO_NUM_11  // D11 - Safe
#define MOTOR_B_ENB_GPIO    GPIO_NUM_12  // D12 - Safe

// PWM Configuration
#define PWM_FREQUENCY       5000    // 5 kHz
#define PWM_RESOLUTION      LEDC_TIMER_10_BIT  // 10-bit resolution (0-1023)
#define PWM_MAX_DUTY        ((1 << 10) - 1)   // 1023 for 10-bit

// Task Configuration
#define MOTOR_TASK_PRIORITY     (configMAX_PRIORITIES - 1)
#define MOTOR_TASK_STACK_SIZE   4096
#define MOTOR_TASK_CORE         1  // Pin to Core 1

// Control Loop Configuration
#define CONTROL_LOOP_PERIOD_MS  20  // 50Hz control loop
#define ACCELERATION_RATE       5   // Units per update

// Motor States
typedef enum {
    MOTOR_STATE_IDLE,
    MOTOR_STATE_RUNNING_FORWARD,
    MOTOR_STATE_RUNNING_BACKWARD,
    MOTOR_STATE_ACCELERATING,
    MOTOR_STATE_DECELERATING,
    MOTOR_STATE_EMERGENCY_STOP,
    MOTOR_STATE_FAULT
} motor_state_t;

// Motor Direction
typedef enum {
    MOTOR_DIR_FORWARD,
    MOTOR_DIR_BACKWARD,
    MOTOR_DIR_BRAKE
} motor_direction_t;

// Motor Identifier
typedef enum {
    MOTOR_A,
    MOTOR_B
} motor_id_t;

// Error codes
typedef enum {
    MOTOR_OK = ESP_OK,
    MOTOR_ERR_INVALID_ARG = ESP_ERR_INVALID_ARG,
    MOTOR_ERR_INVALID_STATE = ESP_ERR_INVALID_STATE,
    MOTOR_ERR_NOT_INIT = ESP_ERR_INVALID_STATE + 1,
    MOTOR_ERR_TIMEOUT,
    MOTOR_ERR_HARDWARE
} motor_err_t;

#ifdef __cplusplus
}
#endif

#endif // MOTOR_CONTROL_H