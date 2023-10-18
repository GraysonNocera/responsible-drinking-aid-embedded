
#include "ADC.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>



void PWM_update(){
    TIM3->CCR1=100;
    TIM3->CCR2=100;
    TIM3->CCR3=100;
    TIM3->CCR4=100;
}

void ADC_initBatLEDs(void) {
    RCC->AHBENR|=RCC_AHBENR_GPIOCEN;

    GPIOC->MODER &= ~0x000FF000;

    GPIOC->MODER|= 0x00055000; //set C pins 4-7 as outputs

    GPIOC->OTYPER&= ~0x03C0;
    GPIOC->OTYPER|= 0x03C0; //set C pins 4-7 as open drain


//    GPIOC->MODER &=0xFFFFFF00; //set C pins 0-3 as inputs
    //GPIOC->MODER &=0xFFFF0000; //set C pins 0-3 as inputs

//    GPIOC->PUPDR |=0x00000055;//set C pins 0-3 to pull-up resistors

}

void init_ADC(void) {
    //PB0 input 8

    RCC->AHBENR|=RCC_AHBENR_GPIOBEN;
    GPIOB->MODER&=~0x3;
    GPIOB->MODER|=0x3<<6; //set PB0 to Analog

    RCC->APB2ENR|=RCC_APB2ENR_ADC1EN; //enable adc
    RCC->CR2&=~RCC_CR2_HSI14ON;
    RCC->CR2|=RCC_CR2_HSI14ON; //set high speed internal clock

    while(!(RCC->CR2 & RCC_CR2_HSI14RDY)); //waiting

    ADC1->CR&=~ADC_CR_ADEN; //
    ADC1->CR|=ADC_CR_ADEN;//enable adc
    while(!(ADC1->ISR & ADC_ISR_ADRDY));

    while(ADC1->CR & ADC_CR_ADSTART); //may or may not need

    //Must wait again
    ADC1->CHSELR&=~ADC_CHSELR_CHSEL8; //set IN8 in CHSELR
    ADC1->CHSELR|=ADC_CHSELR_CHSEL8; //set IN8 in CHSELR
    while(!(ADC1->ISR & ADC_ISR_ADRDY));

    //Must wait
}

void BAT_changeLEDs(int batLevel){
    if(batLevel==1){
        TIM3->CCR1=100;
        TIM3->CCR2=0;
        TIM3->CCR3=0;
        TIM3->CCR4=0;
    }
    else if(batLevel==2){
        TIM3->CCR1=100;
        TIM3->CCR2=100;
        TIM3->CCR3=0;
        TIM3->CCR4=0;
    }
    else if(batLevel==3){
        TIM3->CCR1=100;
        TIM3->CCR2=100;
        TIM3->CCR3=100;
        TIM3->CCR4=0;
    }
    else{
        TIM3->CCR1=100;
        TIM3->CCR2=100;
        TIM3->CCR3=100;
        TIM3->CCR4=100;
    }

}

//Voltage (V)  |  State of Charge (SOC)
//-----------------------------------
//4.20         |  100%
//4.00         |  75%
//3.80         |  50%
//3.60         |  25%
//3.40         |  10%
//3.20         |   0%

int BAT_PercLookup(uint16_t batVal){
    int percRange;

    if(batVal>=0 && batVal<25){
        percRange=1;
    }
    else if(batVal>=25 && batVal<50){
        percRange=2;
    }
    else if(batVal>=50 && batVal<75){
        percRange=3;
    }
    else{
        percRange=4;
    }
    return percRange;
}

void ADC_read(){

    ADC1->CR&=~ADC_CR_ADSTART; //start adc
    ADC1->CR|=ADC_CR_ADSTART; //
    while(!(ADC1->ISR & ADC_ISR_EOC)); //wait for EOC

    uint16_t batVal = ADC1->DR*100/4095;

    int percRange=BAT_PercLookup(batVal);
    char ascii_string[20]; // Create a buffer to store the ASCII representation
    // Use sprintf to convert the integer to ASCII
    sprintf(ascii_string, "%d", percRange);
    USART5_SendString("Bat: ");
    USART5_SendString(ascii_string);
    BAT_changeLEDs(percRange);

}
