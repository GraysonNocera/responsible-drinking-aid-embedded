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



int main(void)
{

    //Initialize functions

    //set up GPIO for power to BT

    //set up usart to communicate with HM-19
    init_tim2();
    init_usart5();
    init_I2C1();
    PA0_interuptSetup();

    init_HR();
    HR_readMode();
    init_tim3();
    //init_HM19();
    init_ADC();
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
    init_tim7();
    USART5_SendString("Start: ");
    while(1){


        HR_readBPM();
        BAC_read();
        ADC_read();
        TIM2_delayMiliSecond(100);

//not sure need this even since everything is interrupts
//     pBAC=BAC_getData();
//
//     char ascii_string[20]; // Create a buffer to store the ASCII representation
//     // Use sprintf to convert the integer to ASCII
//     sprintf(ascii_string, "%d", pBAC);
//     USART5_SendString(ascii_string);
//     USART5_SendChar('\n');
//     USART5_SendChar('\r');
//     TIM2_delayMiliSecond(1000);
    }

}
