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
#include "esp_task_wdt.h"
#include "esp_chip_info.h"
#include "motor_driver.h"

static const char *TAG = "motor_app";

// Motor instances
static motor_driver_t motor_a;
static motor_driver_t motor_b;

// Task handles
static TaskHandle_t motor_control_task_handle = NULL;
static TaskHandle_t demo_task_handle = NULL;

// Watchdog configuration
#define TWDT_TIMEOUT_MS     5000
#define MOTOR_WDT_CHECK_INTERVAL_MS  100

// Watchdog state tracking
static bool watchdog_initialized = false;

/**
 * Initialize or reconfigure task watchdog
 */
static esp_err_t init_task_watchdog(void)
{
    esp_err_t ret;
    
    // Try to deinit first in case it's already initialized
    ret = esp_task_wdt_deinit();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to deinit watchdog: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Configure and initialize task watchdog
    esp_task_wdt_config_t twdt_config = {
        .timeout_ms = TWDT_TIMEOUT_MS,
        .idle_core_mask = 0,  // Don't watch idle tasks
        .trigger_panic = true  // Panic on timeout for safety
    };
    
    ret = esp_task_wdt_init(&twdt_config);
    if (ret == ESP_OK) {
        watchdog_initialized = true;
        ESP_LOGI(TAG, "Task watchdog initialized with %d ms timeout", TWDT_TIMEOUT_MS);
    } else if (ret == ESP_ERR_INVALID_STATE) {
        // Already initialized - try to just use it
        watchdog_initialized = true;
        ESP_LOGW(TAG, "Watchdog already initialized - continuing");
        ret = ESP_OK;
    } else {
        ESP_LOGE(TAG, "Failed to initialize watchdog: %s", esp_err_to_name(ret));
    }
    
    return ret;
}

/**
 * Motor control task - runs at high priority on Core 1
 */
static void motor_control_task(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(CONTROL_LOOP_PERIOD_MS);
    uint32_t loop_count = 0;
    bool task_added_to_wdt = false;
    
    ESP_LOGI(TAG, "Motor control task started on core %d", xPortGetCoreID());
    
    // Add this task to the watchdog if watchdog is initialized
    if (watchdog_initialized) {
        esp_err_t ret = esp_task_wdt_add(NULL);
        if (ret == ESP_OK) {
            task_added_to_wdt = true;
            ESP_LOGI(TAG, "Motor control task added to watchdog");
        } else {
            ESP_LOGE(TAG, "Failed to add task to watchdog: %s", esp_err_to_name(ret));
        }
    }
    
    while (1) {
        // Wait for next control period
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        
        // Update both motors
        esp_err_t motor_a_result = motor_driver_update(&motor_a);
        esp_err_t motor_b_result = motor_driver_update(&motor_b);
        
        // Only reset watchdog if task was successfully added
        if (task_added_to_wdt) {
            // Only reset if both motors updated successfully
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
                ESP_LOGE(TAG, "Motor update failed (A:%d, B:%d), not feeding watchdog", 
                         motor_a_result, motor_b_result);
            }
        }
        
        // Periodic status logging
        if (++loop_count % 50 == 0) {  // Every second at 50Hz
            ESP_LOGD(TAG, "Control loop running, count: %lu", loop_count);
        }
    }
    
    // Should never reach here, but if it does, remove from watchdog
    if (task_added_to_wdt) {
        esp_task_wdt_delete(NULL);
    }
}

/**
 * Demo task - runs at lower priority on Core 0
 */
static void demo_task(void *pvParameters)
{
    int demo_state = 0;
    const TickType_t demo_delay = pdMS_TO_TICKS(3000);
    
    ESP_LOGI(TAG, "Demo task started on core %d", xPortGetCoreID());
    
    // Let motor control task stabilize first
    vTaskDelay(pdMS_TO_TICKS(500));
    
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
        
        // Log motor states and speeds
        ESP_LOGI(TAG, "Motor A - Speed: %d, State: %d | Motor B - Speed: %d, State: %d",
                 motor_driver_get_speed(&motor_a),
                 motor_driver_get_state(&motor_a),
                 motor_driver_get_speed(&motor_b),
                 motor_driver_get_state(&motor_b));
        
        // Cycle through demo states
        demo_state = (demo_state + 1) % 6;
        vTaskDelay(demo_delay);
    }
}

/**
 * Initialize motor hardware
 */
static esp_err_t init_motors(void)
{
    ESP_LOGI(TAG, "Initializing motor hardware...");
    
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
    esp_err_t ret = motor_driver_init(&motor_a, MOTOR_A, &motor_a_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init Motor A: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = motor_driver_init(&motor_b, MOTOR_B, &motor_b_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init Motor B: %s", esp_err_to_name(ret));
        motor_driver_deinit(&motor_a);  // Clean up motor A
        return ret;
    }
    
    // Set acceleration rates for smooth operation
    motor_driver_set_acceleration(&motor_a, 10);
    motor_driver_set_acceleration(&motor_b, 10);
    
    ESP_LOGI(TAG, "Motors initialized successfully");
    ESP_LOGI(TAG, "  Motor A: IN1=%d, IN2=%d, ENA=%d", 
             MOTOR_A_IN1_GPIO, MOTOR_A_IN2_GPIO, MOTOR_A_ENA_GPIO);
    ESP_LOGI(TAG, "  Motor B: IN3=%d, IN4=%d, ENB=%d", 
             MOTOR_B_IN3_GPIO, MOTOR_B_IN4_GPIO, MOTOR_B_ENB_GPIO);
    
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
        case ESP_RST_SW:
            ESP_LOGI(TAG, "Software reset");
            break;
        case ESP_RST_PANIC:
            ESP_LOGW(TAG, "Software panic reset detected - previous crash!");
            // Add delay to allow serial monitor to catch up
            vTaskDelay(pdMS_TO_TICKS(100));
            break;
        case ESP_RST_TASK_WDT:
            ESP_LOGE(TAG, "Task watchdog reset detected!");
            vTaskDelay(pdMS_TO_TICKS(100));
            break;
        case ESP_RST_WDT:
            ESP_LOGE(TAG, "Other watchdog reset detected!");
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
    ESP_LOGI(TAG, "================================================");
    ESP_LOGI(TAG, "ESP32-S3 Dual Motor Control - L298N Driver");
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    ESP_LOGI(TAG, "Chip model: %d, revision: %d", chip_info.model, chip_info.revision);
    
    // Check why we reset
    check_reset_reason();
    
    // Initialize motors first
    esp_err_t ret = init_motors();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize motors, entering safe mode");
        // Could implement safe mode here
        return;
    }
    
    // Initialize or reconfigure task watchdog
    ret = init_task_watchdog();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Continuing without watchdog protection");
        // Continue anyway - motors are more important than watchdog
    }
    
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
                           &demo_task_handle,
                           0);
    
    ESP_LOGI(TAG, "System initialized successfully");
    ESP_LOGI(TAG, "Motor control running on Core %d", MOTOR_TASK_CORE);
    ESP_LOGI(TAG, "Demo running on Core 0");
    ESP_LOGI(TAG, "Starting motor control...");
}