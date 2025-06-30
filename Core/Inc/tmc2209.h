#ifndef TMC2209_H
#define TMC2209_H

#include <stdint.h>

// Slave address (default TMC2209)
#define TMC2209_SLAVE_ADDR       0x05

// TMC2209 Register addresses
#define TMC2209_REG_GCONF        0x00
#define TMC2209_REG_IHOLD_IRUN   0x10
#define TMC2209_REG_CHOPCONF     0x6C

void TMC2209_WriteRegister(uint8_t reg, uint32_t data);
void TMC2209_Init(void);

#endif
