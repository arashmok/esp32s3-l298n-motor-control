/**
 * @file main.c
 * @brief ESP32-S3 Dual Motor Control Application
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_task_wdt.h"  // Added: Required header for task watchdog
#include "motor_driver.h"

static const char *TAG = "motor_app";

// Motor instances
static motor_driver_t motor_a;
static motor_driver_t motor_b;

// Task handle
static TaskHandle_t motor_control_task_handle = NULL;

// Watchdog configuration
#define TWDT_TIMEOUT_SEC    5
#define MOTOR_WDT_CHECK_INTERVAL_MS  100  // Check watchdog more frequently than control loop

/**
 * Motor control task - runs at high priority on Core 1
 */
static void motor_control_task(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(CONTROL_LOOP_PERIOD_MS);
    uint32_t loop_count = 0;
    
    ESP_LOGI(TAG, "Motor control task started on core %d", xPortGetCoreID());
    
    // Add this task to the watchdog
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
    
    while (1) {
        // Wait for next control period
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        
        // Update both motors
        esp_err_t motor_a_result = motor_driver_update(&motor_a);
        esp_err_t motor_b_result = motor_driver_update(&motor_b);
        
        // Only reset watchdog if both motors updated successfully
        if (motor_a_result == ESP_OK && motor_b_result == ESP_OK) {
            // Check motor states are safe before feeding watchdog
            motor_state_t state_a = motor_driver_get_state(&motor_a);
            motor_state_t state_b = motor_driver_get_state(&motor_b);
            
            // Don't feed watchdog if motors are in fault state
            if (state_a != MOTOR_STATE_FAULT && state_b != MOTOR_STATE_FAULT) {
                esp_task_wdt_reset();
            } else {
                ESP_LOGE(TAG, "Motor fault detected, not feeding watchdog");
            }
        } else {
            ESP_LOGE(TAG, "Motor update failed, not feeding watchdog");
        }
        
        // Periodic status logging
        if (++loop_count % 50 == 0) {  // Every second at 50Hz
            ESP_LOGD(TAG, "Control loop running, count: %lu", loop_count);
        }
    }
    
    // Should never reach here, but if it does, remove from watchdog
    esp_task_wdt_delete(NULL);
}

/**
 * Demo task - runs at lower priority on Core 0
 */
static void demo_task(void *pvParameters)
{
    int demo_state = 0;
    const TickType_t demo_delay = pdMS_TO_TICKS(3000);
    
    ESP_LOGI(TAG, "Demo task started on core %d", xPortGetCoreID());
    
    // Optional: Add demo task to watchdog with longer timeout
    // esp_task_wdt_add(NULL);
    
    while (1) {
        switch (demo_state) {
            case 0:
                ESP_LOGI(TAG, "Demo: Both motors forward at 50%% speed");
                motor_driver_set_speed(&motor_a, PWM_MAX_DUTY / 2);
                motor_driver_set_speed(&motor_b, PWM_MAX_DUTY / 2);
                break;
                
            case 1:
                ESP_LOGI(TAG, "Demo: Motor A forward 75%%, Motor B backward 50%%");
                motor_driver_set_speed(&motor_a, (PWM_MAX_DUTY * 3) / 4);
                motor_driver_set_speed(&motor_b, -(PWM_MAX_DUTY / 2));
                break;
                
            case 2:
                ESP_LOGI(TAG, "Demo: Both motors full speed forward");
                motor_driver_set_speed(&motor_a, PWM_MAX_DUTY);
                motor_driver_set_speed(&motor_b, PWM_MAX_DUTY);
                break;
                
            case 3:
                ESP_LOGI(TAG, "Demo: Gradual stop");
                motor_driver_set_speed(&motor_a, 0);
                motor_driver_set_speed(&motor_b, 0);
                break;
                
            case 4:
                ESP_LOGI(TAG, "Demo: Motor A backward 60%%, Motor B forward 80%%");
                motor_driver_set_speed(&motor_a, -(PWM_MAX_DUTY * 3) / 5);
                motor_driver_set_speed(&motor_b, (PWM_MAX_DUTY * 4) / 5);
                break;
                
            case 5:
                ESP_LOGI(TAG, "Demo: Emergency stop!");
                motor_driver_emergency_stop(&motor_a);
                motor_driver_emergency_stop(&motor_b);
                vTaskDelay(pdMS_TO_TICKS(1000));
                ESP_LOGI(TAG, "Demo: Resetting motors...");
                motor_driver_stop(&motor_a);
                motor_driver_stop(&motor_b);
                break;
        }
        
        // Log motor states
        ESP_LOGI(TAG, "Motor A speed: %d, Motor B speed: %d",
                 motor_driver_get_speed(&motor_a),
                 motor_driver_get_speed(&motor_b));
        
        // Cycle through demo states
        demo_state = (demo_state + 1) % 6;
        vTaskDelay(demo_delay);
        
        // Optional: Reset watchdog if demo task is monitored
        // esp_task_wdt_reset();
    }
}

/**
 * Initialize motor hardware
 */
static esp_err_t init_motors(void)
{
    // Motor A configuration
    motor_hal_config_t motor_a_config = {
        .in1_gpio = MOTOR_A_IN1_GPIO,
        .in2_gpio = MOTOR_A_IN2_GPIO,
        .enable_gpio = MOTOR_A_ENA_GPIO,
        .pwm_channel = LEDC_CHANNEL_0,
        .pwm_timer = LEDC_TIMER_0
    };
    
    // Motor B configuration
    motor_hal_config_t motor_b_config = {
        .in1_gpio = MOTOR_B_IN3_GPIO,
        .in2_gpio = MOTOR_B_IN4_GPIO,
        .enable_gpio = MOTOR_B_ENB_GPIO,
        .pwm_channel = LEDC_CHANNEL_1,
        .pwm_timer = LEDC_TIMER_0  // Share timer
    };
    
    // Initialize motors
    ESP_ERROR_CHECK(motor_driver_init(&motor_a, MOTOR_A, &motor_a_config));
    ESP_ERROR_CHECK(motor_driver_init(&motor_b, MOTOR_B, &motor_b_config));
    
    // Set acceleration rates
    motor_driver_set_acceleration(&motor_a, 10);
    motor_driver_set_acceleration(&motor_b, 10);
    
    return ESP_OK;
}

/**
 * Check reset reason and handle accordingly
 */
static void check_reset_reason(void)
{
    esp_reset_reason_t reset_reason = esp_reset_reason();
    
    switch (reset_reason) {
        case ESP_RST_POWERON:
            ESP_LOGI(TAG, "Power-on reset");
            break;
        case ESP_RST_TASK_WDT:
            ESP_LOGW(TAG, "Task watchdog reset detected!");
            // Could implement special recovery logic here
            break;
        case ESP_RST_PANIC:
            ESP_LOGE(TAG, "Software panic reset detected!");
            break;
        default:
            ESP_LOGI(TAG, "Reset reason: %d", reset_reason);
            break;
    }
}

/**
 * Application main entry point
 */
void app_main(void)
{
    ESP_LOGI(TAG, "ESP32-S3 Dual Motor Control");
    ESP_LOGI(TAG, "=========================");
    
    // Check why we reset
    check_reset_reason();
    
    // Initialize motors
    ESP_ERROR_CHECK(init_motors());
    
    // Configure and initialize task watchdog
    esp_task_wdt_config_t twdt_config = {
        .timeout_ms = TWDT_TIMEOUT_SEC * 1000,
        .idle_core_mask = 0,  // Don't watch idle tasks
        .trigger_panic = true  // Panic on timeout for safety
    };
    ESP_ERROR_CHECK(esp_task_wdt_init(&twdt_config));
    ESP_LOGI(TAG, "Task watchdog initialized with %d second timeout", TWDT_TIMEOUT_SEC);
    
    // Create motor control task - high priority on Core 1
    xTaskCreatePinnedToCore(motor_control_task,
                           "motor_ctrl",
                           MOTOR_TASK_STACK_SIZE,
                           NULL,
                           MOTOR_TASK_PRIORITY,
                           &motor_control_task_handle,
                           MOTOR_TASK_CORE);
    
    // Create demo task - lower priority on Core 0
    xTaskCreatePinnedToCore(demo_task,
                           "demo",
                           4096,
                           NULL,
                           5,
                           NULL,
                           0);
    
    ESP_LOGI(TAG, "System initialized successfully");
}