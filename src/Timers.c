#include "Timers.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>



void init_tim3(void) {
    // TODO: Enable GPIO C
    RCC->AHBENR|=RCC_AHBENR_GPIOCEN;

    // TODO: Configure the PC6-9 to be the outputs of TIM3 Ch 1-4
    // TODO: First we clear their MODER bits
    // TODO: Then we set them to AF mode
    // TODO: Set PC6-9 to use AF0 since this corresponds to the TIM3 Ch1-4
    // AFR[0] -> AFRL
    // AFR[1] -> AFRH

    //GPIOC->AFR[0] =0; working but may be wrong
    //GPIOC->AFR[1] =0;
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

    // TODO: Set to PWM mode 1 for all channels
    // Can use the following code to set a channel to PWM mode 1 (110)
    // This line set Timer x's channel 1 to be PWM mode 1 (OC1M bits with 110)
    // TIMx->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1;
    TIM3->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1;
    TIM3->CCMR1 |= TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1;
    TIM3->CCMR2 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1;
    TIM3->CCMR2 |= TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1;


    // TODO: Enable all 4 channel outputs in `TIM3_CCER` using `CC1E` bit
    //TIM3->CCER|=0x1;
    TIM3->CCER|=0x1111; //set all?

    // TODO: Enable TIM3 counter
    TIM3->CR1|=TIM_CR1_CEN;

    // TODO: Set CCR values
    TIM3->CCR1=800;
    TIM3->CCR2=400;
    TIM3->CCR3=200;
    TIM3->CCR4=100;

}


void init_tim2(void){
    // Initialize the timer
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;   // Enable TIM2 clock
    TIM2->PSC = 47999;                      // Set prescaler to get 1ms timebase
    TIM2->ARR = 999;                        // Auto-reload value to get 3s (3000ms)
    TIM2->CR1 |= TIM_CR1_ARPE;             // Enable auto-reload preload
}

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

void TIM2_delayMiliSecond(int ms) {
    TIM2->PSC = 47999;                      // Set prescaler to get 1ms timebase
    TIM2->ARR = ms-1;                        // Auto-reload value to get 3s (3000ms)

    TIM2->CNT = 0;                  // Reset the timer counter
    TIM2->CR1 |= TIM_CR1_CEN;       // Start the timer
    while (!(TIM2->SR & TIM_SR_UIF)) {
        // Wait for the update interrupt flag to be set
    }
    //TIM2->SR &= ~TIM_SR_UIF;        // Clear the update interrupt flag
    TIM2->SR=0x00000000;
    TIM2->CR1 &= ~TIM_CR1_CEN;      // Stop the timer
    TIM2->ARR = 999;                        // Auto-reload value to get 3s (3000ms)

}

void init_tim7() { //used for 3 minute timer

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
    TIM7->CNT=0;


    TIM7->DIER|=TIM_DIER_UIE;

    NVIC->ISER[0]|=(0x1<<(TIM7_IRQn));//

    TIM7->CR1|=TIM_CR1_CEN;
}

int timerCount=0;
int timerSet=20; //each set is 90 seconds
void TIM7_IRQHandler(){
    TIM7->SR=0x00000000;
    timerCount++;
    if(timerCount==timerSet){
        USART5_SendString("Timer ON");
        timerCount=0;
    }
}
