#include "stm32f0xx.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "USART.h"

void init_tim14(void);
void init_tim15(void);
void init_tim16(void);
void init_tim3(void);
void init_tim7(void);
void init_tim2(void);
void init_tim3(void);
void init_tim6(void);
void TIM2_delayOneSecond(void);
void TIM2_delayMiliSecond(int ms);
