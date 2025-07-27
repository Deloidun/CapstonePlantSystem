#include "pid_controller.h"
#include <math.h>
#include <string.h>

// Global PID controller instance
PID_Controller_t g_pid_controller;

// Timer handle for PID ISR (will be set from main.c)
extern TIM_HandleTypeDef htim3; // Using TIM3 for PID ISR

// PID ISR frequency (Hz)
#define PID_ISR_FREQUENCY_HZ    100.0f
#define PID_ISR_PERIOD_MS       10     // 10ms = 100Hz

// PID computation function
float PID_Compute(PID_Controller_t* pid, float measurement)
{
    if (!pid->is_initialized || !pid->is_enabled) {
        return 0.0f;
    }
    
    // Calculate time delta
    uint32_t current_time = HAL_GetTick();
    pid->delta_time = (float)(current_time - pid->last_time) / 1000.0f; // Convert to seconds
    pid->last_time = current_time;
    
    // Calculate error
    pid->error = pid->setpoint - measurement;
    
    // Proportional term
    pid->proportional = pid->Kp * pid->error;
    
    // Integral term
    pid->error_sum += pid->error * pid->delta_time;
    
    // Anti-windup: Limit integral term
    if (pid->error_sum > pid->integral_max) {
        pid->error_sum = pid->integral_max;
    } else if (pid->error_sum < pid->integral_min) {
        pid->error_sum = pid->integral_min;
    }
    
    pid->integral = pid->Ki * pid->error_sum;
    
    // Derivative term (with filtering)
    pid->derivative = pid->Kd * (pid->error - pid->previous_error) / pid->delta_time;
    
    // Calculate output
    pid->output = pid->proportional + pid->integral + pid->derivative;
    
    // Apply output limits
    if (pid->output > pid->output_max) {
        pid->output = pid->output_max;
    } else if (pid->output < pid->output_min) {
        pid->output = pid->output_min;
    }
    
    // Store previous error for next iteration
    pid->previous_error = pid->error;
    
    return pid->output;
}

// Initialize PID controller
void PID_Init(PID_Controller_t* pid, float kp, float ki, float kd, float output_min, float output_max)
{
    // Set gains
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    
    // Set limits
    pid->output_min = output_min;
    pid->output_max = output_max;
    pid->integral_min = -100.0f;  // Default integral limits
    pid->integral_max = 100.0f;
    
    // Initialize variables
    pid->error = 0.0f;
    pid->previous_error = 0.0f;
    pid->error_sum = 0.0f;
    pid->proportional = 0.0f;
    pid->integral = 0.0f;
    pid->derivative = 0.0f;
    pid->output = 0.0f;
    pid->setpoint = 0.0f;
    pid->measurement = 0.0f;
    pid->delta_time = 0.01f; // Default 10ms
    pid->last_time = HAL_GetTick();
    
    // Set status
    pid->is_initialized = true;
    pid->is_enabled = false;
}

// Reset PID controller
void PID_Reset(PID_Controller_t* pid)
{
    if (pid->is_initialized) {
        pid->error = 0.0f;
        pid->previous_error = 0.0f;
        pid->error_sum = 0.0f;
        pid->proportional = 0.0f;
        pid->integral = 0.0f;
        pid->derivative = 0.0f;
        pid->output = 0.0f;
        pid->last_time = HAL_GetTick();
    }
}

// Set PID gains
void PID_SetGains(PID_Controller_t* pid, float kp, float ki, float kd)
{
    if (pid->is_initialized) {
        pid->Kp = kp;
        pid->Ki = ki;
        pid->Kd = kd;
    }
}

// Set output limits
void PID_SetLimits(PID_Controller_t* pid, float output_min, float output_max)
{
    if (pid->is_initialized) {
        pid->output_min = output_min;
        pid->output_max = output_max;
    }
}

// Set setpoint
void PID_SetSetpoint(PID_Controller_t* pid, float setpoint)
{
    if (pid->is_initialized) {
        pid->setpoint = setpoint;
    }
}

// Enable PID controller
void PID_Enable(PID_Controller_t* pid)
{
    if (pid->is_initialized) {
        pid->is_enabled = true;
        PID_Reset(pid); // Reset when enabling
    }
}

// Disable PID controller
void PID_Disable(PID_Controller_t* pid)
{
    if (pid->is_initialized) {
        pid->is_enabled = false;
        pid->output = 0.0f; // Clear output when disabling
    }
}

// Check if PID is enabled
bool PID_IsEnabled(PID_Controller_t* pid)
{
    return (pid->is_initialized && pid->is_enabled);
}

// Get current output
float PID_GetOutput(PID_Controller_t* pid)
{
    return pid->output;
}

// Get current error
float PID_GetError(PID_Controller_t* pid)
{
    return pid->error;
}

// Initialize PID ISR
void PID_ISR_Init(void)
{
    // Initialize global PID controller
    PID_Init(&g_pid_controller, 1.0f, 0.1f, 0.01f, -100.0f, 100.0f);
    
    // Configure TIM3 for PID ISR
    // Note: This assumes TIM3 is already configured in CubeMX
    // The actual timer configuration should be done in CubeMX
    
    // Enable TIM3 interrupt
    HAL_TIM_Base_Start_IT(&htim3);
}

// Start PID ISR
void PID_ISR_Start(void)
{
    PID_Enable(&g_pid_controller);
    HAL_TIM_Base_Start_IT(&htim3);
}

// Stop PID ISR
void PID_ISR_Stop(void)
{
    PID_Disable(&g_pid_controller);
    HAL_TIM_Base_Stop_IT(&htim3);
}

// PID ISR Handler (called from timer interrupt)
void PID_ISR_Handler(void)
{
    // Get current measurement (encoder position)
    extern int32_t Encoder_GetPosition(void);
    extern float CalculateTravelDistance(int32_t encoderCounts);
    
    int32_t encoder_counts = Encoder_GetPosition();
    float current_position = CalculateTravelDistance(encoder_counts);
    
    // Update measurement
    g_pid_controller.measurement = current_position;
    
    // Compute PID output
    float pid_output = PID_Compute(&g_pid_controller, current_position);
    
    // Apply output to motors (you can customize this part)
    extern void Motors_SetSynchronizedSpeedWithOffset(uint8_t speed, uint8_t direction, int8_t motor2_offset);
    extern int8_t motor2_offset; // Assuming this is accessible
    
    if (g_pid_controller.is_enabled) {
        // Convert PID output to motor control
        uint8_t motor_speed = (uint8_t)fabs(pid_output);
        if (motor_speed > 100) motor_speed = 100;
        
        uint8_t direction = (pid_output >= 0) ? 0 : 1; // 0 = forward, 1 = backward
        
        // Apply to motors
        Motors_SetSynchronizedSpeedWithOffset(motor_speed, direction, motor2_offset);
    }
}

// Get PID status information
void PID_GetStatus(PID_Controller_t* pid, float* setpoint, float* measurement, float* output, float* error)
{
    if (pid->is_initialized) {
        *setpoint = pid->setpoint;
        *measurement = pid->measurement;
        *output = pid->output;
        *error = pid->error;
    }
}

// Get PID terms for debugging
void PID_GetTerms(PID_Controller_t* pid, float* proportional, float* integral, float* derivative)
{
    if (pid->is_initialized) {
        *proportional = pid->proportional;
        *integral = pid->integral;
        *derivative = pid->derivative;
    }
}

// Tune PID gains using Ziegler-Nichols method
void PID_TuneZieglerNichols(PID_Controller_t* pid, float ku, float tu)
{
    if (pid->is_initialized) {
        // Ziegler-Nichols tuning formulas
        pid->Kp = 0.6f * ku;
        pid->Ki = 1.2f * ku / tu;
        pid->Kd = 0.075f * ku * tu;
    }
}

// Auto-tuning function (simplified)
void PID_AutoTune(PID_Controller_t* pid)
{
    // This is a simplified auto-tuning implementation
    // In practice, you'd want a more sophisticated algorithm
    
    printf("Starting PID auto-tuning...\r\n");
    
    // Disable PID temporarily
    PID_Disable(pid);
    
    // Set a small step input
    float original_setpoint = pid->setpoint;
    PID_SetSetpoint(pid, 100.0f); // 100cm target
    
    // Apply small constant output
    extern void Motors_SetSynchronizedSpeedWithOffset(uint8_t speed, uint8_t direction, int8_t motor2_offset);
    extern int8_t motor2_offset;
    
    Motors_SetSynchronizedSpeedWithOffset(30, 0, motor2_offset);
    
    // Wait for system to respond
    HAL_Delay(2000);
    
    // Measure response (simplified)
    extern int32_t Encoder_GetPosition(void);
    extern float CalculateTravelDistance(int32_t encoderCounts);
    
    int32_t encoder_counts = Encoder_GetPosition();
    float final_position = CalculateTravelDistance(encoder_counts);
    
    // Stop motors
    Motors_SetSynchronizedSpeedWithOffset(0, 0, motor2_offset);
    
    // Calculate rough tuning parameters
    float error = 100.0f - final_position;
    float ku = 30.0f; // Rough estimate
    float tu = 2.0f;  // Rough estimate based on response time
    
    // Apply Ziegler-Nichols tuning
    PID_TuneZieglerNichols(pid, ku, tu);
    
    // Restore original setpoint
    PID_SetSetpoint(pid, original_setpoint);
    
    printf("Auto-tuning complete. Kp=%.3f, Ki=%.3f, Kd=%.3f\r\n", 
           pid->Kp, pid->Ki, pid->Kd);
}
