// ═══════════════════════════════════════════════
// STM32F411 - MOTOR CONTROLLER
// main.cpp (STM32 HAL + FreeRTOS)
// ═══════════════════════════════════════════════

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

// ── MOTOR STRUCTURES ─────────────────────────────
typedef struct {
    TIM_HandleTypeDef* htim;
    uint32_t channel;
    GPIO_TypeDef* dir_port;
    uint16_t dir_pin_a;
    uint16_t dir_pin_b;
    int32_t encoder_count;
    float speed_rpm;
    float target_speed;
    
    // PID
    float kp, ki, kd;
    float error_integral;
    float prev_error;
} Motor_t;

// ── SYSTEM STATE ─────────────────────────────────
typedef enum {
    STATE_IDLE,
    STATE_RUNNING,
    STATE_EMERGENCY_STOP,
    STATE_ERROR
} SystemState_t;

SystemState_t system_state = STATE_IDLE;

// ── MOTORS ───────────────────────────────────────
Motor_t motor_main;     // Main conveyor
Motor_t motor_belt1;    // Acceleration belt 1
Motor_t motor_belt2;    // Acceleration belt 2

// ── QUEUES ───────────────────────────────────────
QueueHandle_t xCommandQueue;
QueueHandle_t xStatusQueue;

// ── COMMAND PARSER ───────────────────────────────
typedef struct {
    char command[16];
    char motor[32];
    float value;
} Command_t;

// ═════════════════════════════════════════════════
// MOTOR CONTROL
// ═════════════════════════════════════════════════

void Motor_Init(Motor_t* motor, float kp, float ki, float kd) {
    motor->kp = kp;
    motor->ki = ki;
    motor->kd = kd;
    motor->error_integral = 0;
    motor->prev_error = 0;
    motor->encoder_count = 0;
    motor->speed_rpm = 0;
    motor->target_speed = 0;
}

void Motor_SetSpeed(Motor_t* motor, float speed_percent) {
    // speed_percent: -100 to +100
    motor->target_speed = speed_percent;
    
    uint32_t pwm_val = (uint32_t)(
        fabs(speed_percent) / 100.0f * 
        motor->htim->Init.Period
    );
    
    // Direction
    if (speed_percent > 0) {
        HAL_GPIO_WritePin(motor->dir_port, 
                          motor->dir_pin_a, GPIO_PIN_SET);
        HAL_GPIO_WritePin(motor->dir_port, 
                          motor->dir_pin_b, GPIO_PIN_RESET);
    } else if (speed_percent < 0) {
        HAL_GPIO_WritePin(motor->dir_port, 
                          motor->dir_pin_a, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor->dir_port, 
                          motor->dir_pin_b, GPIO_PIN_SET);
    } else {
        // Brake
        HAL_GPIO_WritePin(motor->dir_port, 
                          motor->dir_pin_a, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor->dir_port, 
                          motor->dir_pin_b, GPIO_PIN_RESET);
    }
    
    __HAL_TIM_SET_COMPARE(motor->htim, 
                           motor->channel, pwm_val);
}

// PID Speed Control
void Motor_PID_Update(Motor_t* motor, float dt) {
    float error = motor->target_speed - motor->speed_rpm;
    
    motor->error_integral += error * dt;
    motor->error_integral = 
        fmax(-100, fmin(100, motor->error_integral)); // anti-windup
    
    float derivative = (error - motor->prev_error) / dt;
    
    float output = motor->kp * error + 
                   motor->ki * motor->error_integral + 
                   motor->kd * derivative;
    
    output = fmax(-100, fmin(100, output));
    Motor_SetSpeed(motor, output);
    
    motor->prev_error = error;
}

// ═════════════════════════════════════════════════
// FREERTOS TASKS
// ═════════════════════════════════════════════════

// ── MOTOR CONTROL TASK (1kHz) ─────────────────
void vMotorControlTask(void* pvParameters) {
    
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(1); // 1ms = 1kHz
    
    while(1) {
        if (system_state == STATE_RUNNING) {
            // Update PID for all motors
            Motor_PID_Update(&motor_main, 0.001f);
            Motor_PID_Update(&motor_belt1, 0.001f);
            Motor_PID_Update(&motor_belt2, 0.001f);
        } else if (system_state == STATE_EMERGENCY_STOP) {
            // Stop all motors immediately
            Motor_SetSpeed(&motor_main, 0);
            Motor_SetSpeed(&motor_belt1, 0);
            Motor_SetSpeed(&motor_belt2, 0);
        }
        
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}

// ── COMMAND TASK ──────────────────────────────
void vCommandTask(void* pvParameters) {
    
    Command_t cmd;
    char rx_buffer[128];
    uint8_t rx_byte;
    uint8_t buf_idx = 0;
    
    while(1) {
        // Read UART byte by byte
        if (HAL_UART_Receive(&huart1, &rx_byte, 1, 10) == HAL_OK) {
            
            if (rx_byte == '\n') {
                // Parse JSON command
                rx_buffer[buf_idx] = '\0';
                parse_command(rx_buffer, &cmd);
                
                // Execute command
                if (strcmp(cmd.command, "set_speed") == 0) {
                    if (strcmp(cmd.motor, "main_belt") == 0)
                        motor_main.target_speed = cmd.value;
                    else if (strcmp(cmd.motor, "belt1") == 0)
                        motor_belt1.target_speed = cmd.value;
                    else if (strcmp(cmd.motor, "belt2") == 0)
                        motor_belt2.target_speed = cmd.value;
                    
                    system_state = STATE_RUNNING;
                }
                else if (strcmp(cmd.command, "stop_all") == 0) {
                    system_state = STATE_EMERGENCY_STOP;
                }
                else if (strcmp(cmd.command, "start") == 0) {
                    system_state = STATE_RUNNING;
                }
                else if (strcmp(cmd.command, "vibrate") == 0) {
                    // Set vibration PWM
                    set_vibration(cmd.value);
                }
                
                buf_idx = 0;
                
            } else if (buf_idx < 127) {
                rx_buffer[buf_idx++] = rx_byte;
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// ── STATUS REPORT TASK (100ms) ────────────────
void vStatusTask(void* pvParameters) {
    
    char status_buf[256];
    
    while(1) {
        // Send status to Jetson
        snprintf(status_buf, sizeof(status_buf),
            "{\"state\":%d,"
            "\"belt_main\":%.1f,"
            "\"belt1\":%.1f,"
            "\"belt2\":%.1f,"
            "\"enc_main\":%ld}\n",
            system_state,
            motor_main.speed_rpm,
            motor_belt1.speed_rpm,
            motor_belt2.speed_rpm,
            motor_main.encoder_count
        );
        
        HAL_UART_Transmit(&huart1, 
                          (uint8_t*)status_buf,
                          strlen(status_buf), 100);
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// ─────────────────────────────────────────────────
// ENCODER INTERRUPT (must be fast!)
// ─────────────────────────────────────────────────
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == ENCODER_A_PIN) {
        if (HAL_GPIO_ReadPin(ENCODER_B_PORT, ENCODER_B_PIN))
            motor_main.encoder_count++;
        else
            motor_main.encoder_count--;
    }
}
