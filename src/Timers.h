#include "stm32f0xx.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "USART.h"

void init_tim14(void); //bottom button time
void init_tim15(void); //top button time
void init_tim16(void);
void init_tim17(void); //stop display battery after 5s
void init_tim3(void);  //pwm timer
void init_tim7(void);  //20min/ hour timer
void init_tim2(void);  //delay timer
void init_tim3(void);  //pwm timer
void init_tim6(void); //read BAC every one second (could maybe used TIM2 here)
void TIM2_delayOneSecond(void);
void TIM2_delayMiliSecond(int ms);
