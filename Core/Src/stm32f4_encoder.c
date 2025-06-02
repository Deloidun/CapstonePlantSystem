#include "stm32f4_encoder.h"

// Static variables to store encoder data
static int32_t encoder_position = 0;
static int32_t last_position = 0;
static int32_t encoder_speed = 0;

// Initialize the encoder
void Encoder_Init(void)
{
    // Start the timer in encoder mode
    HAL_TIM_Encoder_Start(&ENCODER_TIMER, TIM_CHANNEL_ALL);
    // Reset the counter to zero
    __HAL_TIM_SET_COUNTER(&ENCODER_TIMER, 0);
}

// Get the current position of the encoder
int32_t Encoder_GetPosition(void)
{
    // Read the current counter value
    int32_t current_count = __HAL_TIM_GET_COUNTER(&ENCODER_TIMER);

    // Adjust for overflow/underflow
    if (current_count > 32767)
    {
        current_count -= 65536;
    }

    // Update the encoder position
    encoder_position += current_count;
    __HAL_TIM_SET_COUNTER(&ENCODER_TIMER, 0);

    return encoder_position;
}

// Get the speed of the motor in RPM
int32_t Encoder_GetSpeed(void)
{
    // Calculate the change in position
    int32_t current_position = Encoder_GetPosition();
    int32_t delta_position = current_position - last_position;

    // Calculate the speed in RPM
    encoder_speed = (delta_position * 60) / (ENCODER_PPR * GEAR_RATIO);

    // Update the last position
    last_position = current_position;

    return encoder_speed;
}