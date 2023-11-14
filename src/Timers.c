
#include "Timers.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

//I2C timer
void init_tim2(void){
    // Initialize the timer
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;   // Enable TIM2 clock
    TIM2->PSC = 47999;                      // Set prescaler to get 1ms timebase
    TIM2->ARR = 999;                        // Auto-reload value to get 3s (3000ms)
    TIM2->CR1 |= TIM_CR1_ARPE;             // Enable auto-reload preload
}

//timer for PWM
void init_tim3(void) {
    RCC->AHBENR|=RCC_AHBENR_GPIOCEN;
    GPIOC->AFR[0] &=~0xFF000000;
    GPIOC->AFR[1] &=~0xFF;
    GPIOC->AFR[0] &=~0xFF000000;
    GPIOC->AFR[1] &=~0xFF;

    GPIOC->MODER &= ~0xAA000;
    GPIOC->MODER |= 0xAA000;

    // TODO: Enable TIM3 with 1 Hz timer
    RCC->APB1ENR|=0x2;

    TIM3->PSC=479;  //not sure this right
    TIM3->ARR=999;

    // Set to PWM mode 1 for all channels
    // Can use the following code to set a channel to PWM mode 1 (110)
    // This line set Timer x's channel 1 to be PWM mode 1 (OC1M bits with 110)
    // TIMx->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1;
    TIM3->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1;
    TIM3->CCMR1 |= TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1;
    TIM3->CCMR2 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1;
    TIM3->CCMR2 |= TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1;


    //Enable all 4 channel outputs in `TIM3_CCER` using `CC1E` bit
    //TIM3->CCER|=0x1;
    TIM3->CCER|=0x1111; //set all?

    //Enable TIM3 counter
    TIM3->CR1|=TIM_CR1_CEN;

//    //Set CCR values
//    TIM3->CCR1=800;
//    TIM3->CCR2=400;
//    TIM3->CCR3=200;
//    TIM3->CCR4=100;

}

//Used for read BAC every one second
void init_tim6(void){
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
    TIM6->DIER|=TIM_DIER_UIE;
    TIM6->PSC = 47999;
    TIM6->ARR = 999;
    TIM6->CR1 |= TIM_CR1_ARPE;
    NVIC->ISER[0]|=(0x1<<(TIM6_DAC_IRQn));//
}

//global timer to turn on sensor every ~30 mins
//each time is 89.473 seconds, this times 20=1789.46s. 29min 49.46s
void init_tim7() { //used for 30 minute timer

    RCC->APB1ENR|=0x20;
    // Calculate the prescaler and auto-reload value for a 30-minute timer
    // SystemCoreClock is the current system clock frequency (in Hz)
    // Assuming SystemCoreClock is 8 MHz
    uint32_t prescaler = (SystemCoreClock / 1000) - 1;  // 1 kHz frequency
    uint32_t timer_period_seconds = 2 * 60;  // 30 minutes in seconds
    uint32_t auto_reload = (timer_period_seconds * 1000) - 1;
    ///May be wrong below
    TIM7->PSC=65534;  //not sure this right

//    TIM7->ARR=10000;
    TIM7->ARR=65534;
    TIM7->ARR=65534/60; //change back after demo
    TIM7->CNT=0;


    TIM7->DIER|=TIM_DIER_UIE;

    NVIC->ISER[0]|=(0x1<<(TIM7_IRQn));//

    //USART5_SendString("Drink Timer Start");

    TIM7->CR1|=TIM_CR1_CEN;

}

//timer used to detect button press length PA0
void init_tim14() {

    RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;
    //TIM14->DIER|=TIM_DIER_UIE;
    TIM14->PSC = 47999;
    TIM14->ARR = 65534;
    TIM14->CR1 |= TIM_CR1_ARPE;
    NVIC->ISER[0]|=(0x1<<(TIM14_IRQn));//
    //TIM14->CR1|=TIM_CR1_CEN;
}

//timer used to detect button press length PC13
void init_tim15() {
    RCC->APB2ENR |= RCC_APB2ENR_TIM15EN;
    //TIM15->DIER|=TIM_DIER_UIE;
    TIM15->PSC = 47999;
    TIM15->ARR = 65534;
    TIM15->CR1 |= TIM_CR1_ARPE;
    NVIC->ISER[0]|=(0x1<<(TIM15_IRQn));
    //TIM15->CR1|=TIM_CR1_CEN;
}

void init_tim16() {
    RCC->APB2ENR |= RCC_APB2ENR_TIM16EN;
    TIM16->PSC = 47999;
    TIM16->ARR = 999;
    TIM16->CR1 |= TIM_CR1_ARPE;
    //TIM16->CR1|=TIM_CR1_CEN;
}

void init_tim17() {
    RCC->APB2ENR |= RCC_APB2ENR_TIM17EN;
    TIM17->DIER|=TIM_DIER_UIE;
    TIM17->PSC = 47999;
    TIM17->ARR = 5000;
    TIM17->CNT=0;
    TIM17->CR1 |= TIM_CR1_ARPE;
    NVIC->ISER[0]|=(0x1<<(TIM17_IRQn));//
    TIM17->CR1 |= TIM_CR1_CEN;
}

//delay one second
void TIM2_delayOneSecond(void) {
    TIM2->CNT = 0;                  // Reset the timer counter
    TIM2->CR1 |= TIM_CR1_CEN;       // Start the timer
    while (!(TIM2->SR & TIM_SR_UIF)) {
        // Wait for the update interrupt flag to be set
    }
    //TIM2->SR &= ~TIM_SR_UIF;        // Clear the update interrupt flag
    TIM2->SR=0x00000000;
    TIM2->CR1 &= ~TIM_CR1_CEN;      // Stop the timer
}

//delay milliSecond used for I2C messaging only
void TIM2_delayMiliSecond(int ms) {
    TIM2->PSC = 47999;                      // Set prescaler to get 1ms timebase
    TIM2->ARR = ms-1;                        // Auto-reload value to get 3s (3000ms)

//    TIM2->CNT = 0;                  // Reset the timer counter
    TIM2->CR1 |= TIM_CR1_CEN;       // Start the timer

    while (!(TIM2->SR & TIM_SR_UIF)) {
        // Wait for the update interrupt flag to be set
    }
    //TIM2->SR &= ~TIM_SR_UIF;        // Clear the update interrupt flag
    TIM2->SR=0x00000000;
    TIM2->CR1 &= ~TIM_CR1_CEN;      // Stop the timer
    TIM2->ARR = 999;                        // Auto-reload value to get 3s (3000ms)
    TIM2->CNT = 0;                  // Reset the timer counter

}

void TIM16_delayMiliSecond(int ms) {
    TIM16->PSC = 47999;                      // Set prescaler to get 1ms timebase
    TIM16->ARR = ms-1;                        // Auto-reload value to get 3s (3000ms)

//    TIM16->CNT = 0;                  // Reset the timer counter
    TIM16->CR1 |= TIM_CR1_CEN;       // Start the timer

    while (!(TIM16->SR & TIM_SR_UIF)) {
        // Wait for the update interrupt flag to be set
    }
    //TIM16->SR &= ~TIM_SR_UIF;        // Clear the update interrupt flag
    TIM16->SR=0x00000000;
    TIM16->CR1 &= ~TIM_CR1_CEN;      // Stop the timer
    TIM16->ARR = 999;                        // Auto-reload value to get 3s (3000ms)
    TIM16->CNT = 0;                  // Reset the timer counter

}
