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

void Delay_MiliSecond(int ms) {
    TIM2->PSC = 47999;                      // Set prescaler to get 1ms timebase
    TIM2->ARR = ms;                        // Auto-reload value to get 3s (3000ms)

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

void startHeartRate(){
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

    //set pin to analog mode
    //set pin to not pull up or pull down

    //set pin high using bsrr

//     Set the RSTN pin low for 10ms.
//     While RSTN is low, set the MFIO pin to high.
//     After the 10ms has elapsed, set the RSTN pin high.
//     After an additional 50ms has elapsed, the MAX32664 is in application mode.
    //https://github.com/sparkfun/SparkFun_Bio_Sensor_Hub_Library/blob/f91b5fa12f391f9889919a20c8709849fb4c79fb/src/SparkFun_Bio_Sensor_Hub_Library.cpp#L43

   // GPIOA->BSRR = GPIO_PIN_0; // Set the pin high (3.3V)

}

void I2CEnable(){
    /* (1) Timing register value is computed with the AN4235 xls file,
     fast Mode @400kHz with I2CCLK = 48MHz, rise time = 140ns,
     fall time = 40ns */
    /* (2) Periph enable, receive interrupt enable */
    /* (3) Slave address = 0x5A, read transfer, 1 byte to receive, autoend */

    I2C2->TIMINGR = (uint32_t)0x00B01A4B; /* (1) */
    I2C2->CR1 = I2C_CR1_PE | I2C_CR1_RXIE; /* (2) */
    I2C2->CR2 = I2C_CR2_AUTOEND | (1<<16) | I2C_CR2_RD_WRN
     | (I2C1_OWN_ADDRESS << 1); /* (3) */
}

int main(void)
{
    //set up GPIO for power to BT

    //set up usart to communicate with HM-19
    init_usart5();
    //Interrupt for when we receive usart data
    enable_tty_interrupt();
    //create clock for delay function
    Set_Clock();
    //push button interrupt to send drink data
    pa0Interupt();

//    char b;
//    int send=0;
//    volatile int isCon;
    //Bluetooth commands below with delay to receive return data
    //command to start communication
    USART5_SendString("AT");
    Delay_OneSecond();
    Delay_OneSecond();
    //get device name
    USART5_SendString("AT+NAME?");
    Delay_OneSecond();
    //get device address
    USART5_SendString("AT+ADDR?");
    Delay_OneSecond();
    //set advertising type
    USART5_SendString("AT+ADTY0");
    Delay_OneSecond();
    //get device mode
    USART5_SendString("AT+MODE?");
    Delay_OneSecond();

    //set role as peripheral
    USART5_SendString("AT+ROLE0");
    Delay_OneSecond();
    //set so device starts working as soon as boot
    USART5_SendString("AT+IMME0");
    Delay_OneSecond();

    //Reset advertising beacon
    USART5_SendString("AT+IBEA0");
    Delay_OneSecond();

    //Turn on advertising beacon
    USART5_SendString("AT+IBEA1");
    Delay_OneSecond();

    //get last connected device address
    USART5_SendString("AT+RADD?");
    Delay_OneSecond();

    //unneeded when device in IMME0, if in IMME1 this command will tell module to start working.
    USART5_SendString("AT+START");

    startHeartRate();

    while(1){
//not sure need this even since everything is interrupts
    }




}
