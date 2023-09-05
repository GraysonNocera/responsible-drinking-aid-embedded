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

void init_usart5() {
    // TODO
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN; //enable GPIOC
    RCC->AHBENR |= RCC_AHBENR_GPIODEN; //enable GPIOD

    GPIOC->MODER&= ~0x03000000; //clear PC12
    GPIOC->MODER|=0x02000000; //set pc12 alt func

    GPIOD->MODER&= ~0x0000000C; //clear PD2
    GPIOD->MODER|=0x00000020; //set PD2 alt func

    GPIOC->AFR[1]&=~(0xF<<16); //set alt funct PC12 to USART5_TX //alf2
    GPIOC->AFR[1]|=(0x2<<16); //set alt funct PC12 to USART5_TX //alf2

    GPIOD->AFR[0]&=~((0xF<<8));//set alt funct PD2 to USART5_RX // alf2
    GPIOD->AFR[0]|=((0x2<<8));//set alt funct PD2 to USART5_RX // alf2

    RCC->APB1ENR|=RCC_APB1ENR_USART5EN; //enable USART5 rcc clock

    USART5->CR1&=~USART_CR1_UE; //turn off usart5 enable bit

    USART5->CR1&=~(0x1<<28); //word length 8 but m1=0
    USART5->CR1&=~USART_CR1_M; //word length 8 but m0=0

    USART5->CR1&=~(USART_CR1_PCE); //turn off parity
    USART5->CR2&=~(0x3<<12); //stop bit set to 1 reg=00
    USART5->CR1&=~(USART_CR1_OVER8); //oversample bit set to 16
    USART5->BRR=0x1388; //set baud rate, the clock speed divided by this num(115200)

    USART5->CR1|=USART_CR1_RE|USART_CR1_TE; //enable RE and TE

    USART5->CR1|=USART_CR1_UE; //turn on usart5 enable bit

    while((!(USART5->ISR& USART_ISR_REACK)) && (!(USART5->ISR& USART_ISR_TEACK)));


    //wait until ISR RXE and TXE are set
}

//const char mes[] = "AT";
//char recMes[25];
//int recIndex=0;
//
//volatile char buffer[1000];
//volatile int buffer_index = 0;
//char target_string[] = "OK+CONN";
//
//
//void handleInput(char character){
//
//    buffer[buffer_index++]=character;
//    buffer[buffer_index] = '\0';
//
//    if(strstr(buffer, target_string) != NULL){
//        // Do something when the target string is received
//        // You can reset the buffer_index here if needed
//        USART5_SendString("connected to bluetooth");
//        buffer_index = 0;
//    }
//    else if(buffer_index>=100){
//        buffer_index=0;
//    }
//
//}

void USART3_4_5_6_7_8_IRQHandler(void) {

    if (USART5->ISR & USART_ISR_RXNE) {
        char receivedChar = USART5->RDR; // Read the received data
        USART5_SendChar(receivedChar);
        //handleInput(receivedChar);
       }
}



void Set_Clock(void){
    // Initialize the timer
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;   // Enable TIM2 clock
    TIM2->PSC = 47999;                      // Set prescaler to get 1ms timebase
    TIM2->ARR = 999;                        // Auto-reload value to get 3s (3000ms)
    TIM2->CR1 |= TIM_CR1_ARPE;             // Enable auto-reload preload
}

void Delay_OneSecond(void) {
    TIM2->CNT = 0;                  // Reset the timer counter
    TIM2->CR1 |= TIM_CR1_CEN;       // Start the timer
    while (!(TIM2->SR & TIM_SR_UIF)) {
        // Wait for the update interrupt flag to be set
    }
    //TIM2->SR &= ~TIM_SR_UIF;        // Clear the update interrupt flag
    TIM2->SR=0x00000000;
    TIM2->CR1 &= ~TIM_CR1_CEN;      // Stop the timer
}
//
//void init_GPIO{
//
//
//}
//How do I wire up the usart connection betweeen the STm32 and the hm-19?


void USART5_SendChar(char c) {
    while (!(USART5->ISR & USART_ISR_TXE)); // Wait until TX buffer is empty
    USART5->TDR = c;
}

void USART5_SendString(const char* str) {
    while (*str) {
        USART5_SendChar(*str++);
    }
}

void enable_tty_interrupt(void) {
    // TODO

    USART5->CR1|=USART_CR1_RXNEIE; //when receive data interupt happens
    //USART5->CR1 |= USART_CR1_TCIE; /* Enable TC interrupt */

    NVIC->ISER[0]|=(0x1<<(USART3_8_IRQn)); //enable interupt
}

void pa0Interupt(void){
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;


    GPIOA->MODER &= ~GPIO_MODER_MODER0;  // Clear the MODER bits for PA0
    // No need to set anything; it's already in input mode by default
    GPIOA->PUPDR|= 0x6;

    // Configure PA0 to trigger an interrupt on the falling edge (button press)

   // EXTI->FTSR |= EXTI_FTSR_TR0; // Trigger on falling edge
    EXTI->RTSR|=EXTI_RTSR_TR0;
    EXTI->IMR |= EXTI_IMR_MR0;   // Enable EXTI0 (PA0)


    // Enable EXTI0 interrupt in NVIC
    NVIC->ISER[0]|=(0x1<<(EXTI0_1_IRQn));
}

void EXTI0_1_IRQHandler(void) {
    // Check if EXTI0 (PA0) triggered the interrupt
    if ((EXTI->PR & EXTI_PR_PR0) != 0) {
        //USART5_SendString("btton");
        USART5_SendString("Drink Consumed");

        // Clear the EXTI0 (PA0) pending interrupt
        EXTI->PR |= EXTI_PR_PR0;
    }
}

int main(void)
{
    //set up GPIO for power to BT

    //set up usart
    init_usart5();
    enable_tty_interrupt();
    Set_Clock();
    pa0Interupt();
    char b;
    int send=0;
    volatile int isCon;
    USART5_SendString("AT");
    Delay_OneSecond();
    Delay_OneSecond();

    USART5_SendString("AT+NAME?");
    Delay_OneSecond();
    USART5_SendString("AT+ADDR?");
    Delay_OneSecond();
    USART5_SendString("AT+ADTY0");
    Delay_OneSecond();
    USART5_SendString("AT+MODE?");
    Delay_OneSecond();
    USART5_SendString("AT+ROLE0");
    Delay_OneSecond();
    USART5_SendString("AT+IMME0");
    Delay_OneSecond();
    USART5_SendString("AT+IBEA0");
    Delay_OneSecond();
    USART5_SendString("AT+IBEA1");
    Delay_OneSecond();
    USART5_SendString("AT+RADD?");
    Delay_OneSecond();
    USART5_SendString("AT+START");






}
