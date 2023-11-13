

/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/

#include "stm32f0xx.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "ADC.h"
#include "BAC.h"
#include "HR.h"
#include "I2C.h"
#include "Interrupt.h"
#include "Timers.h"
#include "USART.h"


void testAPPSim(){
    //set up usart to communicate with HM-19
    init_tim2(); //timer two used for delays
    init_usart5(); //usart used for bluetooth
    int count=0;
    char ascii_string[20]; // Create a buffer to store the ASCII representation
    TIM2_delayMiliSecond(1000);

    USART5_SendString("Ethanol Sensor On");
    TIM2_delayMiliSecond(1000);
    USART5_SendString("Clear Drinks");
    TIM2_delayMiliSecond(1000);

    while(count<26){
        USART5_SendString("Add Drink");
        TIM2_delayMiliSecond(500);
        sprintf(ascii_string, "%d", count);
        USART5_SendString("BAC: ");
        USART5_SendString(ascii_string);
        TIM2_delayMiliSecond(500);
        USART5_SendString("Heart Rate: ");
        USART5_SendString(ascii_string);
        TIM2_delayMiliSecond(500);
        count++;
    }

    while(count>0){
        USART5_SendString("Subtract Drink");
        TIM2_delayMiliSecond(500);
        sprintf(ascii_string, "%d", count);
        USART5_SendString("BAC: ");
        USART5_SendString(ascii_string);
        TIM2_delayMiliSecond(500);
        USART5_SendString("Heart Rate: ");
        USART5_SendString(ascii_string);
        TIM2_delayMiliSecond(500);
        count--;
    }
    USART5_SendString("Ethanol Sensor Off");


}


void testAPP(){
    init_tim2(); //timer two used for delays
    init_usart5(); //usart used for bluetooth
    init_I2C1(); //I2C to communiciate with heart rate and ethanol
    PA0_interuptSetup(); //drink counter interrupt
    PC13_interuptSetup(); //battery led button
    //need a PC13 Interrupt
    init_HR(); //start heart rate monitor
    HR_readMode(); //read mode of heart rate monitor
    init_tim3(); //timer 3 used for PWM
    //init_HM19(); //program bluetooth module
    init_ADC(); //start ADC
    init_tim7();//timer used for 30 minute take breathalyer timer(maybe set when drink counted)?
    init_tim14();
    init_tim15();
    init_tim6();

    USART5_SendString("Start: ");
    while(1){
      HR_readBPM();
      BAC_read();
//     ADC_read();
      TIM2_delayMiliSecond(100);
    }
}

void testSendOnButtonOrEthanol(){
    init_tim2(); //timer two used for delays
    init_usart5(); //usart used for bluetooth
    init_I2C1(); //I2C to communiciate with heart rate and ethanol
    init_tim3(); //timer 3 used for PWM
    PA0_interuptSetup(); //drink counter interrupt
    PC13_interuptSetup(); //battery led button
    init_tim7();//timer used for 30 minute take breathalyer timer(maybe set when drink counted)?
    init_tim14();
    init_tim15();
    init_tim6();
}

void testEthanol(){
    init_tim2(); //timer two used for delays
    init_usart5(); //usart used for bluetooth
    init_I2C1(); //I2C to communiciate with heart rate and ethanol
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;

    GPIOA->MODER&= ~(0x3<<8);
    GPIOA->MODER |= (0x1<<8);  // Clear the MODER bits for PA0
    GPIOA->PUPDR&= ~(0x3<<8);
    GPIOA->PUPDR|= (0x1<<8);
    GPIOA->BSRR|=(0x1<<4);
    GPIOA->MODER&= ~(0x3<<6);
    GPIOA->MODER |= (0x1<<6);  // Clear the MODER bits for PA0
    GPIOA->PUPDR&= ~(0x3<<6);
    GPIOA->PUPDR|= (0x1<<6);
    GPIOA->BSRR|=(0x1<<3);
//    PA0_interuptSetup(); //drink counter interrupt
//    PC13_interuptSetup(); //battery led button
    //GPIOA->BSRR|=(0x1<<3);
    GPIOC->MODER|=0x1<<12;
    GPIOC->PUPDR|=0x1<<12;

    while(1){
        BAC_read();
        GPIOC->BSRR|=(0x1<<6);
        TIM2_delayMiliSecond(2);
        GPIOC->BSRR|=(0x1<<22);

    }
}

void testBluetooth(){
    //set up usart to communicate with HM-19
    init_tim2(); //timer two used for delays
    init_usart5(); //usart used for bluetooth
    //init_HM19();

    while(1){
        //USART5_SendString("Bluetooth connect");
        TIM2_delayMiliSecond(1000);

    }
}

void testDebugLEDs(){
    init_tim2(); //timer two used for delays
    init_tim3();

    while(1){
        TIM3->CCR1=800;
        TIM3->CCR2=0;
        TIM3->CCR3=0;
        TIM3->CCR4=0;
        TIM2_delayMiliSecond(100);
        TIM3->CCR1=0;
        TIM3->CCR2=800;
        TIM3->CCR3=0;
        TIM3->CCR4=0;
        TIM2_delayMiliSecond(100);
        TIM3->CCR1=0;
        TIM3->CCR2=0;
        TIM3->CCR3=800;
        TIM3->CCR4=0;
        TIM2_delayMiliSecond(100);
        TIM3->CCR1=0;
        TIM3->CCR2=0;
        TIM3->CCR3=0;
        TIM3->CCR4=800;
        TIM2_delayMiliSecond(100);

    }

}

int main(void)
{
    //code below should use (8Mhz*6)/presc
    //RCC->CFGR|=(0x1<<20);
    //RCC->CFGR|=(0x1<<16); //this bit breaks when trying to select external

    //code below should use (8Mhz/2)*12
//    RCC->CFGR|=(0x1<<21);
//    RCC->CFGR|=(0x1<<19);
//
//
//    RCC->CR|=(0x1<<24);
//    while (!(RCC->CR & RCC_CR_PLLRDY));
//    RCC->CFGR|=0x2;
//    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);





    //testDebugLEDs();
    //testBluetooth();

    //testEthanol();
    //testSendOnButtonOrEthanol();
    //testAPPSim();


    //Initialize functions

    //set up GPIO for power to BT

    //set up usart to communicate with HM-19
    init_tim2(); //timer two used for delays
    init_usart5(); //usart used for bluetooth
    init_I2C1(); //I2C to communiciate with heart rate and ethanol
    PA0_interuptSetup(); //drink counter interrupt
    PC13_interuptSetup(); //battery led button
    //need a PC13 Interrupt
    //init_HR(); //start heart rate monitor
   // HR_readMode(); //read mode of heart rate monitor
    init_tim3(); //timer 3 used for PWM
    //init_HM19(); //program bluetooth module
    init_ADC(); //start ADC
    init_tim7();//timer used for 30 minute take breathalyer timer(maybe set when drink counted)?
    TIM7_ChangeLen(40);//40 is 59.6486 mins
    init_tim14(); //used for time between bottom button press
    init_tim15(); //used for time between top button press
    init_tim6(); //used to take BAC every second
    init_tim17(); //used for
    //init_tim16(); //wait in ethanol data send


    //HR_setMFIOInt();
//
//    HR_readBPM();
    //uint16_t pBAC=BAC_getData();
    //GPIOA->MODER&= ~(0x3<<8);
   // GPIOA->MODER |= (0x1<<8);  // Clear the MODER bits for PA0
   // GPIOA->PUPDR&= ~(0x3<<8);
   // GPIOA->PUPDR|= (0x1<<8);
    //GPIOA->BSRR|=(0x1<<4);
//    GPIOA->MODER&= ~(0x3<<6);
//    GPIOA->MODER |= (0x1<<6);  // Clear the MODER bits for PA0
//    GPIOA->PUPDR&= ~(0x3<<6);
//    GPIOA->PUPDR|= (0x1<<6);
//    GPIOA->BSRR|=(0x1<<3);
//    USART5_SendString("Start: ");
//    while(1){
//
////
////        HR_readBPM();
//        BAC_read();
////        //ADC_read();
//        TIM2_delayMiliSecond(100);
//
////not sure need this even since everything is interrupts
////     pBAC=BAC_getData();
////
////     char ascii_string[20]; // Create a buffer to store the ASCII representation
////     // Use sprintf to convert the integer to ASCII
////     sprintf(ascii_string, "%d", pBAC);
////     USART5_SendString(ascii_string);
////     USART5_SendChar('\n');
////     USART5_SendChar('\r');
////     TIM2_delayMiliSecond(1000);
//    }


}

