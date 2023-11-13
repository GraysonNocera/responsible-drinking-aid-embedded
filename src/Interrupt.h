#include "stm32f0xx.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

void PA0_interuptSetup(void);
void PC13_interuptSetup(void);
void EXTI0_1_IRQHandler(void);
void USART3_4_5_6_7_8_IRQHandler(void);
void EXTI4_15_IRQHandler(void);
void TIM7_IRQHandler();
void TIM7_ChangeLen(int);
void I2C1_IRQHandler(void);
void TIM14_IRQHandler();
void TIM17_IRQHandler();

