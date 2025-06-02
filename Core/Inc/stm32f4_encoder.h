#ifndef ENCODER_H
#define ENCODER_H
#include "stm32f4xx_hal.h"


extern TIM_HandleTypeDef htim3;
// Define the timer used for the encoder
#define ENCODER_TIMER htim3

// Define the number of pulses per revolution (PPR) of the encoder
#define ENCODER_PPR 360

// Define the gear ratio of the motor
#define GEAR_RATIO 50

// Function prototypes
void Encoder_Init(void);
int32_t Encoder_GetPosition(void);
int32_t Encoder_GetSpeed(void);

#endif // ENCODER_H
