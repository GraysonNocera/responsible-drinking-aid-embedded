#include "stm32f0xx.h"

void LED_Init(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
    GPIOC->MODER |= GPIO_MODER_MODER9_0;
}

void LED_Toggle(void) {
    GPIOC->ODR ^= GPIO_ODR_9;
}

void Delay(uint32_t delay) {
    while (delay--) {
        for (volatile int i = 0; i < 1000; i++);
    }
}

int main(void) {
    HAL_Init();

    LED_Init();

    while (1) {
        LED_Toggle();
        Delay(1000);
    }
}
