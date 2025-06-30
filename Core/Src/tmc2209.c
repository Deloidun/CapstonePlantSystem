#include "tmc2209.h"
#include "stm32f4xx_hal.h"

// Link to CubeMX-generated UART handle
extern UART_HandleTypeDef huart3;

static uint8_t tmc_crc(uint8_t *data, uint8_t len) {
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80)
                crc = (crc << 1) ^ 0x07;
            else
                crc <<= 1;
        }
    }
    return crc;
}

void TMC2209_WriteRegister(uint8_t reg, uint32_t data) {
    uint8_t tx[8];

    tx[0] = TMC2209_SLAVE_ADDR;
    tx[1] = 0x80 | reg;
    tx[2] = (data >>  0) & 0xFF;
    tx[3] = (data >>  8) & 0xFF;
    tx[4] = (data >> 16) & 0xFF;
    tx[5] = (data >> 24) & 0xFF;
    tx[6] = tmc_crc(tx, 6);

    HAL_UART_Transmit(&huart3, tx, 7, HAL_MAX_DELAY);
}

void TMC2209_Init() {
    // Set motor current: IHOLD=8, IRUN=16, delay=4
    TMC2209_WriteRegister(TMC2209_REG_IHOLD_IRUN, (8 << 0) | (16 << 8) | (4 << 16));

    // Enable stealthChop, interpolation, 16 microsteps
    TMC2209_WriteRegister(TMC2209_REG_CHOPCONF, 0x000100C3);

    // Enable global config with stealthChop
    TMC2209_WriteRegister(TMC2209_REG_GCONF, 0x00000004);
}
