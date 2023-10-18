#include "stm32f0xx.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

void init_usart5(void);
void USART5_SendChar(char c);
void USART5_SendByte(uint16_t b);
void USART5_SendString(const char* str);
int init_HM19(void);
