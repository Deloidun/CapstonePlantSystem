#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

// PID Controller Structure
typedef struct {
    // PID Gains
    float Kp;           // Proportional gain
    float Ki;           // Integral gain
    float Kd;           // Derivative gain
    
    // PID Terms
    float proportional;
    float integral;
    float derivative;
    
    // Error tracking
    float error;
    float previous_error;
    float error_sum;
    
    // Output limits
    float output_min;
    float output_max;
    float integral_min;
    float integral_max;
    
    // Timing
    float delta_time;   // Time between updates (seconds)
    uint32_t last_time; // Last update time
    
    // Status
    bool is_initialized;
    bool is_enabled;
    
    // Output
    float output;
    float setpoint;
    float measurement;
    
} PID_Controller_t;

// Function prototypes
void PID_Init(PID_Controller_t* pid, float kp, float ki, float kd, float output_min, float output_max);
void PID_Reset(PID_Controller_t* pid);
void PID_SetGains(PID_Controller_t* pid, float kp, float ki, float kd);
void PID_SetLimits(PID_Controller_t* pid, float output_min, float output_max);
void PID_SetSetpoint(PID_Controller_t* pid, float setpoint);
float PID_Compute(PID_Controller_t* pid, float measurement);
void PID_Enable(PID_Controller_t* pid);
void PID_Disable(PID_Controller_t* pid);
bool PID_IsEnabled(PID_Controller_t* pid);
float PID_GetOutput(PID_Controller_t* pid);
float PID_GetError(PID_Controller_t* pid);

// ISR-based PID functions
void PID_ISR_Init(void);
void PID_ISR_Start(void);
void PID_ISR_Stop(void);
void PID_ISR_Handler(void);

// Global PID controller instance (for ISR access)
extern PID_Controller_t g_pid_controller;

#endif // PID_CONTROLLER_H
