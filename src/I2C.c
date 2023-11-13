#include "I2C.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>




void init_I2C1(void){
    // Enable the I2C peripheral clock PA9-SCL  PA10-SDA
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    // Configure GPIO pins for I2C SCL and SDA
//    RCC->AHBENR |= RCC_AHBENR_GPIOAEN; // Enable GPIOA clock
//    GPIOA->MODER |= (GPIO_MODER_MODER9_1 | GPIO_MODER_MODER10_1); // Alternate function mode
//    GPIOA->AFR[1] |= (0x1 << 6) | (0x1 << 10); // Select AF4 (I2C) for PA9 and PA10

    // Enable the I2C peripheral clock PB6-SCL  PB7-SDA
    // Configure GPIO pins for I2C SCL and SDA
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN; // Enable GPIOA clock
    GPIOB->MODER |= (GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1); // Alternate function mode
    GPIOB->AFR[0] |= (0x1 << 24) | (0x1 << 28); // Select AF1 (I2C) for PB6 and PB7

    // Configure the I2C peripheral
    I2C1->CR1 &= ~I2C_CR1_PE; // Disable I2C1 while configuring
    I2C1->TIMINGR =  0x10420F13; //0x00200B27; // Standard mode with 100 kHz speed in 8mhz mode
    I2C1->TIMINGR =  0xB0420F13; //0x00200B27; // Standard mode with 100 kHz speed in 48mhz mode

    // Configure own address and enable I2C1
    //I2C1->CR2 = (0xH6 << 1);
//    I2C2->CR2 = I2C_CR2_AUTOEND | (1 << 16) ; /* (3) */
    I2C1->CR2 &=~(0x1<<11);//set slave address as 7 bit
    I2C1->CR2 |=0xAA<<0; //slave address of heart rate monitor 7 bit
    I2C1->CR2&=~(0xFF<<16);//clear NBYTES
    I2C1->CR2|=(0x2<<16);//Set N Bytes to one byte

    I2C1->CR1 |= I2C_CR1_ERRIE | I2C_CR1_NACKIE;//enable interupt on error
    I2C1->CR1 |= I2C_CR1_PE;
    NVIC->ISER[0]|=(0x1<<(I2C1_IRQn));

}



void I2C_Start(void) {
    // Generate a start condition
    //I2C1->CR2 &= ~I2C_CR2_RD_WRN; //clear read bit
    I2C1->CR2 |= I2C_CR2_START; //start transfer
    int count=0;
    //TIM2_delayMiliSecond(5);
    while (!(I2C1->ISR & I2C_ISR_TXE)){};
    //while (!(I2C1->ISR & I2C_ISR_TXE) && (count<=4)){TIM2_delayMiliSecond(40); count++;}; // Wait for TXE (transmit data register empty)
}

void I2C_Stop(void) {
    // Generate a stop condition
    I2C1->CR2 |= I2C_CR2_STOP;
    int count=0;
    while ((I2C1->CR2 & I2C_CR2_STOP)){};// && (count<=4)){TIM2_delayMiliSecond(4); count++;}; // Wait until stop condition is cleared
    //while ((I2C1->CR2 & I2C_CR2_STOP) && (count<=4)){TIM2_delayMiliSecond(40); count++;}; // Wait for TXE (transmit data register empty)

}

void I2C_SendByte(uint8_t data) {
    // Write data to the data register
    I2C1->TXDR = data;
    int count=0;
    while (!(I2C1->ISR & I2C_ISR_TXE)){};//&&(count<=4)){TIM2_delayMiliSecond(4); count++;}; // Wait for TXE
    //while (!(I2C1->ISR & I2C_ISR_TXE) && (count<=4)){TIM2_delayMiliSecond(40); count++;}; // Wait for TXE (transmit data register empty)
}

uint8_t I2C_ReadByte(void) {
    // Generate a start condition followed by a read request

    //I2C1->CR2 |= (I2C_CR2_RD_WRN);

    // Wait for data to be received
    int count=0;
    while (!(I2C1->ISR & I2C_ISR_RXNE)){};//&&(count<=4)){TIM2_delayMiliSecond(4); count++;};
    //while (!(I2C1->ISR & I2C_ISR_RXNE) && (count<=4)){TIM2_delayMiliSecond(40); count++;}; // Wait for TXE (transmit data register empty)

    TIM2_delayMiliSecond(2);

    // Read data from the data register
    uint8_t data = I2C1->RXDR;

    return data;
}

