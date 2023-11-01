#include "Interrupt.h"
#include "BAC.h"

int timerCount=0;
int timerSet=20; //each set is 90 seconds
int BACReadtimerCount=0;
int BACReadtimerSet=20; //each set is 1 second

void I2C1_IRQHandler(void) {
    // Check error flags
    USART5_SendString("Nack Failure\n");
    if (I2C1->ISR & I2C_ISR_NACKF) {
        USART5_SendString("Nack Failure\n");
        // Acknowledge failure
        // Handle the NACK (Not Acknowledged) error
    }
    if (I2C1->ISR & I2C_ISR_BERR) {
        // Bus error
        // Handle the bus error
        USART5_SendString("Bus Error\n");

    }
    if (I2C1->ISR & I2C_ISR_ARLO) {
        // Bus error
        // Handle the bus error
        USART5_SendString("Arbitration Lost Error\n");

    }
    // Add handling for other error flags as needed
    //I2C_Stop();
    // Clear the error flags
    I2C1->ISR &= ~I2C_ISR_NACKF;
    I2C1->ISR &= ~I2C_ISR_BERR;
    I2C1->ISR &= ~I2C_ISR_ARLO;
    I2C1->ICR |= (0x1<<4);//~I2C_ISR_NACKF;
    I2C1->ICR |= (0x1<<8);//~I2C_ISR_BERR;
    I2C1->ICR |= (0x1<<3);//~I2C_ISR_ARLO;
    I2C1->ICR |= (0x1<<5);//~I2C_ISR_BERR;

    // Clear other error flags as needed
}

void USART3_4_5_6_7_8_IRQHandler(void) {

    if (USART5->ISR & USART_ISR_RXNE) {
        char receivedChar = USART5->RDR; // Read the received data
        USART5_SendChar(receivedChar);
        //handleInput(receivedChar);
       }
}

void EXTI0_1_IRQHandler(void) {
    // Check if EXTI0 (PA0) triggered the interrupt
    if ((EXTI->PR & EXTI_PR_PR0) != 0) {
        EXTI->PR |= EXTI_PR_PR0;

        if (GPIOA->IDR & GPIO_IDR_0) {
            TIM14->CR1|=TIM_CR1_CEN;
        } else {
            if((TIM14->CNT)>=0x1388){
                USART5_SendString("Bat Button\n");
                ADC_read();
            }
            else if((TIM14->CNT)>=0x7D0){
                USART5_SendString("Clear Drinks");
            }
            else{
                USART5_SendString("Subtract Drink");
            }
            TIM14->CR1&=~(TIM_CR1_CEN);
            TIM14->CNT=0;
        }
    }

}


void EXTI4_15_IRQHandler(void){

    if(EXTI->PR & EXTI_PR_PR9){
        USART5_SendString("Heart Touch\n");
        EXTI->PR |= EXTI_PR_PR9;
    }
    else if(EXTI->PR & EXTI_PR_PR13)
    {

        EXTI->PR |= EXTI_PR_PR13;
        if (GPIOC->IDR & GPIO_IDR_13) {
            TIM15->CR1|=TIM_CR1_CEN;
        } else {
            if((TIM15->CNT)>=0x1388){
                //reset global timer for turning on ethanol sensor
                timerCount=0;
                TIM7->CNT=0; //added, need to test
                //need to turn on sensor
                GPIOA->BSRR|=(0x1<<3);
                //turn on ~1 second timer interrupt to read ADC value
                TIM6->CR1|=TIM_CR1_CEN;
                USART5_SendString("Drink Timer Reset");

                USART5_SendString("Ethanol Sensor On");
                //count number of reads
                //on ~20th read, reset read count, disable interrupt, turn off ethanol sensor

            }
            else{
                USART5_SendString("Add Drink");
            }
            TIM15->CR1&=~(TIM_CR1_CEN);
            TIM15->CNT=0;
        }
    }
    EXTI->PR |= EXTI_PR_PR13;

}

//drink count interrupt
void PA0_interuptSetup(void){
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;


    GPIOA->MODER &= ~GPIO_MODER_MODER0;  // Clear the MODER bits for PA0
    // No need to set anything; it's already in input mode by default
    GPIOA->PUPDR|= 0x6;

    // Configure PA0 to trigger an interrupt on the falling edge (button press)

   // EXTI->FTSR |= EXTI_FTSR_TR0; // Trigger on falling edge
    EXTI->RTSR|=EXTI_RTSR_TR0;
    EXTI->FTSR|=EXTI_FTSR_TR0; //set for falling edge also.

    EXTI->IMR |= EXTI_IMR_MR0;   // Enable EXTI0 (PA0)


    // Enable EXTI0 interrupt in NVIC
    NVIC->ISER[0]|=(0x1<<(EXTI0_1_IRQn));
}

//battery display interrupt
void PC13_interuptSetup(void){
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;
    GPIOA->MODER&= ~(0x3<<6);
    GPIOA->MODER |= (0x1<<6);  // Clear the MODER bits for PA0
    GPIOA->PUPDR&= ~(0x3<<6);
    GPIOA->PUPDR|= (0x1<<7);

    GPIOC->MODER &= ~(0x3<<26);  // Clear the MODER bits for PC13
    // No need to set anything; it's already in input mode by default
    GPIOC->PUPDR|= (0x1<<27);

    // Configure PC13 to trigger an interrupt on the falling edge (button press)
    SYSCFG->EXTICR[3] &= ~SYSCFG_EXTICR4_EXTI13;
    SYSCFG->EXTICR[3] |= (0x1<<5);

    EXTI->RTSR|=EXTI_RTSR_TR13;
    EXTI->FTSR|=EXTI_FTSR_TR13; //set for falling edge also.
    EXTI->IMR |= EXTI_IMR_MR13;   // Enable EXTI13 (PC0)


    // Enable EXTI0 interrupt in NVIC
    NVIC->ISER[0]|=(0x1<<(EXTI4_15_IRQn));
}


void TIM7_IRQHandler(){
    TIM7->SR=0x00000000;
    timerCount++;
    if(timerCount==timerSet){
        USART5_SendString("Timer ON");
        timerCount=0;
    }
}


void TIM6_DAC_IRQHandler(){
    TIM6->SR=0x00000000;
    BACReadtimerCount++;
    BAC_read();
    //on ~20th read, reset read count, disable interrupt, turn off ethanol sensor
    if(BACReadtimerCount==BACReadtimerSet){
        GPIOA->BSRR|=(0x1<<19);
        USART5_SendString("Ethanol Sensor OFF");
        USART5_SendString("Drink Timer Reset");
        timerCount=0; //added, need to test
        TIM7->CNT=0; //added, need to test
        BACReadtimerCount=0;
        TIM6->CR1&=~(TIM_CR1_CEN);
    }
}


void TIM3_IRQHandler(){
    TIM3->SR=0x00000000;
    USART5_SendString("Timer 3");
}

void TIM14_IRQHandler(){
    TIM14->SR=0x00000000;
    USART5_SendString("Timer 14");
}

void TIM15_IRQHandler(){
    TIM15->SR=0x00000000;
    USART5_SendString("Timer 15");
}

void TIM16_IRQHandler(){
    TIM16->SR=0x00000000;
    USART5_SendString("Timer 16");
}
