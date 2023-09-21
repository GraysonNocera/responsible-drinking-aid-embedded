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


void setup_tim3(void) {
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

void updatePWM(){
    TIM3->CCR1=100;
    TIM3->CCR2=100;
    TIM3->CCR3=100;
    TIM3->CCR4=100;
}

void enable_batLEDSports(void) {
    RCC->AHBENR|=RCC_AHBENR_GPIOCEN;

    GPIOC->MODER &= ~0x000FF000;

    GPIOC->MODER|= 0x00055000; //set C pins 4-7 as outputs

    GPIOC->OTYPER&= ~0x03C0;
    GPIOC->OTYPER|= 0x03C0; //set C pins 4-7 as open drain


//    GPIOC->MODER &=0xFFFFFF00; //set C pins 0-3 as inputs
    //GPIOC->MODER &=0xFFFF0000; //set C pins 0-3 as inputs

//    GPIOC->PUPDR |=0x00000055;//set C pins 0-3 to pull-up resistors

}

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
    USART5->BRR=0x1388; //set baud rate, the clock speed divided by this num(9600)

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

void USART5_SendByte(uint16_t b) {
    // Send the upper byte
    USART5_SendChar((char)(b >> 8)+48);
    USART5_SendChar((char)(b & 0xFF)+48);
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
        Delay_OneSecond();

//        USART5_SendString("AT+RADD?");
//        Delay_OneSecond();
//        USART5_SendString("AT+CONNL");
//        Delay_OneSecond();

        readBpm();
        Delay_OneSecond();

        readBAC();

        // Clear the EXTI0 (PA0) pending interrupt
        EXTI->PR |= EXTI_PR_PR0;
    }
    updatePWM();
}

void startHeartRate(){
    //Pin 8 reset, pin 9 mfio
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

    GPIOB->MODER&= ~0x00030000; //clear PB8
    GPIOB->MODER|=0x00010000; //set analog

    GPIOB->MODER&= ~0x000C0000; //clear PB9
    GPIOB->MODER|=0x00040000; //set analog



    //set pin to analog mode
    //set pin to not pull up or pull down


    GPIOB->BSRR|=(0x1<<9);
    Delay_MiliSecond(1000);
    //set pin high using bsrr

//     Set the RSTN pin low for 10ms.
    GPIOB->BSRR|=(0x1<<25);
    Delay_MiliSecond(10);
//     While RSTN is low, set the MFIO pin to high.
    GPIOB->BSRR|=(0x1<<8);
//     After the 10ms has elapsed, set the RSTN pin high.
    Delay_MiliSecond(10);
    GPIOB->BSRR|=(0x1<<9);
    Delay_MiliSecond(50);
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
    //I2C2->CR2 = I2C_CR2_AUTOEND | (1<<16) | I2C_CR2_RD_WRN | (I2C1_OWN_ADDRESS << 1); /* (3) */
}


#define I2C_SLAVE_ADDRESS 0x50

void I2C_in(void){
    // Enable the I2C peripheral clock PA9-SCL  PA10-SDA
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    // Configure GPIO pins for I2C SCL and SDA
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN; // Enable GPIOA clock
    GPIOA->MODER |= (GPIO_MODER_MODER9_1 | GPIO_MODER_MODER10_1); // Alternate function mode
    GPIOA->AFR[1] |= (0x1 << 6) | (0x1 << 10); // Select AF1 (I2C) for PA9 and PA10

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
    }
    // Add handling for other error flags as needed

    // Clear the error flags
    I2C1->ISR &= ~I2C_ISR_NACKF;
    I2C1->ISR &= ~I2C_ISR_BERR;
    // Clear other error flags as needed
}

void I2C_Start(void) {
    // Generate a start condition
    //I2C1->CR2 &= ~I2C_CR2_RD_WRN; //clear read bit
    I2C1->CR2 |= I2C_CR2_START; //start transfer

    while (!(I2C1->ISR & I2C_ISR_TXE)); // Wait for TXE (transmit data register empty)
}

void I2C_Stop(void) {
    // Generate a stop condition
    I2C1->CR2 |= I2C_CR2_STOP;
    while (I2C1->CR2 & I2C_CR2_STOP); // Wait until stop condition is cleared
}

void I2C_SendByte(uint8_t data) {
    // Write data to the data register
    I2C1->TXDR = data;
    while (!(I2C1->ISR & I2C_ISR_TXE)); // Wait for TXE
}

uint8_t I2C_ReadByte(void) {
    // Generate a start condition followed by a read request

    //I2C1->CR2 |= (I2C_CR2_RD_WRN);

    // Wait for data to be received
    while (!(I2C1->ISR & I2C_ISR_RXNE));
    Delay_MiliSecond(45);

    // Read data from the data register
    uint8_t data = I2C1->RXDR;

    return data;
}

void I2Cstuff(void) {
    // Initialize the system clock and GPIO

    // Send a message over I2C
    I2C1->CR2&=~(0xFF<<16);//clear NBYTES
    I2C1->CR2|=(0x2<<16);//Set N Bytes to one byte
    I2C1->CR2 &= ~I2C_CR2_RD_WRN;
    I2C_Start();
    I2C_SendByte(0x02); // Send the data byte (change to your actual data)
    I2C_SendByte(0x00); // Send the data byte (change to your actual data)
    //I2C_SendByte(0x00); // Send the data byte (change to your actual data)

    I2C_Stop();
    Delay_MiliSecond(2);


    // Read data from I2C
    I2C1->CR2 |= (I2C_CR2_RD_WRN);

//    I2C1->CR2&=~(0xFF<<16);//clear NBYTES
//    I2C1->CR2|=(0x1<<16);//Set N Bytes to one byte
    I2C_Start();
    uint8_t receivedData = I2C_ReadByte();
    receivedData = I2C_ReadByte();
//    receivedData = I2C_ReadByte();

    I2C_Stop();
    Delay_MiliSecond(2);

//    I2C1->CR2 &= ~I2C_CR2_RD_WRN;
//    I2C_Start();
//    I2C_SendByte(receivedData);
//    I2C_Stop();
}

void I2C_SetOutputMode(void){
    I2C1->CR2 &= ~I2C_CR2_RD_WRN;
    I2C1->CR2&=~(0xFF<<16);//clear NBYTES
    I2C1->CR2|=(0x3<<16);//Set N Bytes to one byte
    I2C_Start();
    I2C_SendByte(0x10); // Send the data byte (change to your actual data)
    I2C_SendByte(0x00); // Send the data byte (change to your actual data)
    I2C_SendByte(0x02); // Send the data byte (change to your actual data)
    I2C_Stop();
    Delay_MiliSecond(2);

    I2C1->CR2 |= (I2C_CR2_RD_WRN);

    I2C1->CR2&=~(0xFF<<16);//clear NBYTES
    I2C1->CR2|=(0x1<<16);//Set N Bytes to one byte
    I2C_Start();
    uint8_t receivedData = I2C_ReadByte();
    I2C_Stop();

}

void I2C_setFifoThreshold(void){
    I2C1->CR2 &= ~I2C_CR2_RD_WRN;
    I2C1->CR2&=~(0xFF<<16);//clear NBYTES
    I2C1->CR2|=(0x3<<16);//Set N Bytes to one byte
    I2C_Start();
    I2C_SendByte(0x10); // Send the data byte (change to your actual data)
    I2C_SendByte(0x01); // Send the data byte (change to your actual data)
    I2C_SendByte(0x0F); // Send the data byte (change to your actual data)
    I2C_Stop();
    Delay_MiliSecond(3);
    I2C1->CR2 |= (I2C_CR2_RD_WRN);

    I2C1->CR2&=~(0xFF<<16);//clear NBYTES
    I2C1->CR2|=(0x1<<16);//Set N Bytes to one byte
    I2C_Start();
    uint8_t receivedData = I2C_ReadByte();
    I2C_Stop();

}

void agcAlgoControl(void){
    I2C1->CR2 &= ~I2C_CR2_RD_WRN;
    I2C1->CR2&=~(0xFF<<16);//clear NBYTES
    I2C1->CR2|=(0x3<<16);//Set N Bytes to one byte
    I2C_Start();
    I2C_SendByte(0x52); // Send the data byte (change to your actual data)
    I2C_SendByte(0x00); // Send the data byte (change to your actual data)
    I2C_SendByte(0x01); // Send the data byte (change to your actual data)
    I2C_Stop();
    Delay_MiliSecond(50);
    I2C1->CR2 |= (I2C_CR2_RD_WRN);

    I2C1->CR2&=~(0xFF<<16);//clear NBYTES
    I2C1->CR2|=(0x1<<16);//Set N Bytes to one byte
    I2C_Start();
    uint8_t receivedData = I2C_ReadByte();
    I2C_Stop();

}

void max30101Control(void){
    I2C1->CR2 &= ~I2C_CR2_RD_WRN;
    I2C1->CR2&=~(0xFF<<16);//clear NBYTES
    I2C1->CR2|=(0x3<<16);//Set N Bytes to one byte
    I2C_Start();
    I2C_SendByte(0x44); // Send the data byte (change to your actual data)
    I2C_SendByte(0x03); // Send the data byte (change to your actual data)
    I2C_SendByte(0x01); // Send the data byte (change to your actual data)
    I2C_Stop();
    Delay_MiliSecond(50);
    I2C1->CR2 |= (I2C_CR2_RD_WRN);

    I2C1->CR2&=~(0xFF<<16);//clear NBYTES
    I2C1->CR2|=(0x1<<16);//Set N Bytes to one byte
    I2C_Start();
    uint8_t receivedData = I2C_ReadByte();
    I2C_Stop();

}

void maximFastAlgoControl(void){
    I2C1->CR2 &= ~I2C_CR2_RD_WRN;
    I2C1->CR2&=~(0xFF<<16);//clear NBYTES
    I2C1->CR2|=(0x3<<16);//Set N Bytes to one byte
    I2C_Start();
    I2C_SendByte(0x52); // Send the data byte (change to your actual data)
    I2C_SendByte(0x02); // Send the data byte (change to your actual data)
    I2C_SendByte(0x01); // Send the data byte (change to your actual data)
    I2C_Stop();
    Delay_MiliSecond(50);
    I2C1->CR2 |= (I2C_CR2_RD_WRN);

    I2C1->CR2&=~(0xFF<<16);//clear NBYTES
    I2C1->CR2|=(0x1<<16);//Set N Bytes to one byte
    I2C_Start();
    uint8_t receivedData = I2C_ReadByte();
    I2C_Stop();

}

void readAlgoSamples(void){
    I2C1->CR2 &= ~I2C_CR2_RD_WRN;
    I2C1->CR2&=~(0xFF<<16);//clear NBYTES
    I2C1->CR2|=(0x3<<16);//Set N Bytes to one byte
    I2C_Start();
    I2C_SendByte(0x51); // Send the data byte (change to your actual data)
    I2C_SendByte(0x00); // Send the data byte (change to your actual data)
    I2C_SendByte(0x03); // Send the data byte (change to your actual data)
    I2C_Stop();

    Delay_MiliSecond(2);
    I2C1->CR2 |= (I2C_CR2_RD_WRN);

    I2C1->CR2&=~(0xFF<<16);//clear NBYTES
    I2C1->CR2|=(0x2<<16);//Set N Bytes to one byte
    I2C_Start();
    uint8_t receivedData = I2C_ReadByte();
    receivedData = I2C_ReadByte();
    I2C_Stop();

}



int initBlue(void){
    USART5_SendString("AT");
    //USART5_SendString("AT+RENEW");
    //USART5_SendString("fhjsdhjkhfjewhfjkhewjkdhjewkhdjkshadkjhjdhsjkhnnnnnnnndjsadhfjkdshfewui    hsdjhew dewhjd edj wedhjwehd wedkj ewdj ewdhwejhdjew dewhd wehd jkewhd ewdhwe dhewj dhwejdhlakjhcuihcue cu cuih cuhecuihechdiuch dsuich uidsch uidhciusdahciuhewiuwe iweibewbdiewbdwebdbewdiubweduihew idh we chsd jchjsdjkcjdshchjds c hdchjdshcjhds c hjdsch whedh ewhdgewhdgh ew gdghewdgh gewhdg hwegh dhgewgh dgh ewdgh egwhdghewdgewghdghewgh dgewghdgh ewdghewgdweghdgh ewhdhg  gdhwegh d ghewghdweghdgh weghdgewdgwehdsdjwejdioewdjoiewo djddajf jewf ewfe foiewfioefijewfjiojioef fe jjwe jkewjf jew jkf jwe jfwe fwenfnewndndnwejkdewkhwlfhjefkjwe fhew fhuqewohfewhifweui fdwjkfhsdhfiuwhf eurgfeufbyeci bcuhebcyubcyreiucberucberucberybcyuer c jercued bcuy erbce cyer c ercbe cwecgedcjhsgd cdjh hejg fjhergf jehgwfhgdcb cyer cbuyecyuer uycbue bcuye bcuyer cbuyberucyb eurcb uycb ue cbue cbuye bcure cuyer cbu ercyub eruc ber cbuer cbuercbuyecbuyecbue cubj hdsjcd hcjb hcbreu bcuyer cbkjsedbchjs buyfr ekaufewi fyuewrufe fhbdsmhbcj hebe fuyerfuyer fvdjhfe erfguefer gffhrefhjsdhjkhfjewhfjkhewjkdhjewkhdjkshadkjhjdhsjkhdjsadhfjkdshfewui    hsdjhew dewhjd edj wedhjwehd wedkj ewdj ewdhwejhdjew dewhd wehd jkewhd ewdhwe dhewj dhwejdhlakjhcuihcue cu cuih cuhecuihechdiuch dsuich uidsch uidhciusdahciuhewiuwe iweibewbdiewbdwebdbewdiubweduihew idh we chsd jchjsdjkcjdshchjds c hdchjdshcjhds c hjdsch whedh ewhdgewhdgh ew gdghewdgh gewhdg hwegh dhgewgh dgh ewdgh egwhdghewdgewghdghewgh dgewghdgh ewdghewgdweghdgh ewhdhg  gdhwegh d ghewghdweghdgh weghdgewdgwehdsdjwejdioewdjoiewo djddajf jewf ewfe foiewfioefijewfjiojioef fe jjwe jkewjf jew jkf jwe jfwe fwenfnewndndnwejkdewkhwlfhjefkjwe fhew fhuqewohfewhifweui fdwjkfhsdhfiuwhf eurgfeufbyeci bcuhebcyubcyreiucberucberucberybcyuer c jercued bcuy erbce cyer c ercbe cwecgedcjhsgd cdjh hejg fjhergf jehgwfhgdcb cyer cbuyecyuer uycbue bcuye bcuyer cbuyberucyb eurcb uycb ue cbue cbuye bcure cuyer cbu ercyub eruc ber cbuer cbuercbuyecbuyecbue cubj hdsjcd hcjb hcbreu bcuyer cbkjsedbchjs buyfr ekaufewi fyuewrufe fhbdsmhbcj hebe fuyerfuyer fvdjhfe erfguefer gffhreghjg jyg hfhgfgh f ghfffdgffgdfg ddfg fg df");

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
////
//    //unneeded when device in IMME0, if in IMME1 this command will tell module to start working.
    USART5_SendString("AT+START");
    Delay_OneSecond();
    USART5_SendString("AT+NAME?");
    Delay_OneSecond();

    USART5_SendString("AT+MODE1");
    Delay_OneSecond();
    USART5_SendString("AT+PWRM1");
    Delay_OneSecond();
    USART5_SendString("AT+PARI?");
    Delay_OneSecond();
    USART5_SendString("AT+POWE?");
    Delay_OneSecond();
    USART5_SendString("AT+STOP?");
    Delay_OneSecond();
    USART5_SendString("AT+SAVE?");
    Delay_OneSecond();


}

void setUpHeart(void){
    startHeartRate();
    Delay_MiliSecond(1000);
    I2C_in();
    I2Cstuff();
    I2C_SetOutputMode();
    I2C_setFifoThreshold();
    agcAlgoControl();
    max30101Control();
    maximFastAlgoControl();
    readAlgoSamples();
    Delay_MiliSecond(1000);
}




void checkStatus(void){
    I2C1->CR2 &= ~I2C_CR2_RD_WRN;
    I2C1->CR2&=~(0xFF<<16);//clear NBYTES
    I2C1->CR2|=(0x2<<16);//Set N Bytes to one byte
    I2C_Start();
    I2C_SendByte(0x00); // Send the data byte (change to your actual data)
    I2C_SendByte(0x00); // Send the data byte (change to your actual data)
    I2C_Stop();

    Delay_MiliSecond(45);
    I2C1->CR2 |= (I2C_CR2_RD_WRN);

    I2C1->CR2&=~(0xFF<<16);//clear NBYTES
    I2C1->CR2|=(0x2<<16);//Set N Bytes to one byte
    I2C_Start();
    uint8_t receivedData = I2C_ReadByte();
    receivedData = I2C_ReadByte();
    I2C_Stop();
}

void numSamplesOutFifo(void){
    I2C1->CR2 &= ~I2C_CR2_RD_WRN;
    I2C1->CR2&=~(0xFF<<16);//clear NBYTES
    I2C1->CR2|=(0x2<<16);//Set N Bytes to one byte
    I2C_Start();
    I2C_SendByte(0x12); // Send the data byte (change to your actual data)
    I2C_SendByte(0x00); // Send the data byte (change to your actual data)
    I2C_Stop();

    Delay_MiliSecond(45);
    I2C1->CR2 |= (I2C_CR2_RD_WRN);

    I2C1->CR2&=~(0xFF<<16);//clear NBYTES
    I2C1->CR2|=(0x2<<16);//Set N Bytes to one byte
    I2C_Start();
    uint8_t receivedData = I2C_ReadByte();
    receivedData = I2C_ReadByte();
    I2C_Stop();

}

void readFillArray(uint8_t bpmArr[]){
    I2C1->CR2 &= ~I2C_CR2_RD_WRN;
    I2C1->CR2&=~(0xFF<<16);//clear NBYTES
    I2C1->CR2|=(0x2<<16);//Set N Bytes to one byte
    I2C_Start();
    I2C_SendByte(0x12); // Send the data byte (change to your actual data)
    I2C_SendByte(0x01); // Send the data byte (change to your actual data)
    I2C_Stop();
    Delay_MiliSecond(2);


    int sizeRead=6;
    //uint8_t bpmArr[sizeRead];

    for(size_t i = 0; i < sizeRead; i++){
        bpmArr[i] = 0;
    }

    I2C1->CR2 |= (I2C_CR2_RD_WRN);
    I2C1->CR2&=~(0xFF<<16);//clear NBYTES
    I2C1->CR2|=(0x6<<16);//Set N Bytes to one byte
    I2C_Start();
    for(size_t i = 0; i < sizeRead; i++){
        bpmArr[i] = I2C_ReadByte();
    }
    I2C_Stop();
}

void readBpm(void){
    uint8_t bpmRetArr[6];
    //while(1){
        checkStatus();
        numSamplesOutFifo();
        readFillArray(bpmRetArr);
        // Heart Rate formatting
        uint16_t heartRate = (uint16_t)(bpmRetArr[0] << 8);
        heartRate |= (bpmRetArr[1]);
        heartRate /= 10;

        // Confidence formatting
        uint8_t confidence = bpmRetArr[2];

        //Blood oxygen level formatting
        uint16_t oxygen = (uint16_t)(bpmRetArr[3] << 8);
        oxygen |= bpmRetArr[4];
        oxygen /= 10;

        //"Machine State" - has a finger been detected?
        uint8_t status = bpmRetArr[5];
        USART5_SendString("Heart Rate: ");
        char ascii_string[20]; // Create a buffer to store the ASCII representation
        // Use sprintf to convert the integer to ASCII
        sprintf(ascii_string, "%d", heartRate);
        USART5_SendString(ascii_string);


        Delay_MiliSecond(250);
   // }
}



void initializeEthanol(uint8_t adcData[]){
    I2C1->CR2 &=~0xFF<<0; //slave address of ethanol sensor
    I2C1->CR2 |=0x9A<<0; //slave address of ethanol sensor
    I2C1->CR2 &= ~I2C_CR2_RD_WRN;
    I2C1->CR2&=~(0xFF<<16);//clear NBYTES
    I2C1->CR2|=(0x1<<16);//Set N Bytes to one byte
    I2C_Start();
    I2C_SendByte(0x00); // Send the data byte (change to your actual data)
    I2C_Stop();
    Delay_MiliSecond(5);


    I2C1->CR2 |= (I2C_CR2_RD_WRN);

    I2C1->CR2&=~(0xFF<<16);//clear NBYTES
    I2C1->CR2|=(0x2<<16);//Set N Bytes to one byte
    I2C_Start();
    adcData[0] = I2C_ReadByte();
    adcData[1] = I2C_ReadByte();
    I2C_Stop();


}

void readMode(void){
    I2C1->CR2 &= ~I2C_CR2_RD_WRN;
    I2C1->CR2&=~(0xFF<<16);//clear NBYTES
    I2C1->CR2|=(0x2<<16);//Set N Bytes to one byte
    I2C_Start();
    I2C_SendByte(0x02); // Send the data byte (change to your actual data)
    I2C_SendByte(0x00); // Send the data byte (change to your actual data)
    I2C_Stop();

    Delay_MiliSecond(45);
    I2C1->CR2 |= (I2C_CR2_RD_WRN);

    I2C1->CR2&=~(0xFF<<16);//clear NBYTES
    I2C1->CR2|=(0x2<<16);//Set N Bytes to one byte
    I2C_Start();
    uint8_t receivedData = I2C_ReadByte();
    receivedData = I2C_ReadByte();
    I2C_Stop();

}


uint16_t alcohol3_getADCdata(void){

    uint8_t writeData;
    uint8_t readData[ 2 ];
    uint16_t ADC_value;

    initializeEthanol(readData);

    ADC_value = readData[ 0 ];
    ADC_value <<= 8;
    ADC_value |= readData[ 1 ];

    ADC_value = ADC_value & 0x0FFF;

    return ADC_value;
}

/* Etanol in CO data
 *
 * Table:  CO [ppm] | Equivalent C2H5OH
            0       |    0
           10       |    1
           50       |    6
          100       |    18
          500       |    274
 */
static float ethanolInCo(uint16_t co)
{
    float etanol;

    if (co == 0)
    {
        etanol = 0;
    }
    else if (co <= 10)
    {
        etanol = co / 10.0;
    }
    else if (co > 10 && co <= 50)
    {
        etanol = (float)(6 / 50.0) * co;
    }
    else if (co > 50 && co <= 100)
    {
        etanol = (0.18 * co);
    }
    else if (co > 100)
    {
        etanol = (float)(274 / 500.0) * co;
    }
    return etanol;
}

/* Convert ppm to mg/L
 *
 * Table:  mg/l | air ppm
 *         1.82 | 1000
 *         0.91 | 500
 *         0.18 | 100
 *         0.09 | 50
 */
static float ppmTomgl(uint16_t ppm)
{
    float mgL;

    mgL = (1.82 * ppm) / 1000.0;
    return mgL;
}

float getBAC(void){

    float Alcohol_mgL;
    uint16_t Ethanol_ppm;
    uint16_t readData;

    readData = alcohol3_getADCdata();
    Ethanol_ppm = ethanolInCo(readData);
    Alcohol_mgL = ppmTomgl(Ethanol_ppm);

    return Alcohol_mgL*1000;
}

void readBAC(){

    uint16_t pBAC=getBAC();

    char ascii_string[20]; // Create a buffer to store the ASCII representation
    // Use sprintf to convert the integer to ASCII
    sprintf(ascii_string, "%d", pBAC);
    USART5_SendString("BAC: ");
    USART5_SendString(ascii_string);
//    USART5_SendChar('\n');
//    USART5_SendChar('\r');
    Delay_MiliSecond(1000);
}

int main(void)
{


    //Initialize functions

    //set up GPIO for power to BT

    //set up usart to communicate with HM-19
    Set_Clock();
    init_usart5();
    I2C_in();
    pa0Interupt();

    //enable_tty_interrupt();
    //setUpHeart();
//    readMode();
    //setup_tim3();
    initBlue();
//
//    readBpm();
//    uint16_t pBAC=getBAC();


    while(1){




        //readBpm();
        readBAC();
//not sure need this even since everything is interrupts
//     pBAC=getBAC();
//
//     char ascii_string[20]; // Create a buffer to store the ASCII representation
//     // Use sprintf to convert the integer to ASCII
//     sprintf(ascii_string, "%d", pBAC);
//     USART5_SendString(ascii_string);
//     USART5_SendChar('\n');
//     USART5_SendChar('\r');
//     Delay_MiliSecond(1000);
    }

}
