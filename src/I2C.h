#include "stm32f0xx.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

void init_I2C1(void);
void I2C_Start(void);
void I2C_Stop(void);
void I2C_SendByte(uint8_t data);
uint8_t I2C_ReadByte(void);
