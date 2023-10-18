#include "HR.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

void HR_readMode(void) {
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
    TIM2_delayMiliSecond(2);


    // Read data from I2C
    I2C1->CR2 |= (I2C_CR2_RD_WRN);

//    I2C1->CR2&=~(0xFF<<16);//clear NBYTES
//    I2C1->CR2|=(0x1<<16);//Set N Bytes to one byte
    I2C_Start();
    uint8_t receivedData = I2C_ReadByte();
    receivedData = I2C_ReadByte();
//    receivedData = I2C_ReadByte();

    I2C_Stop();
    TIM2_delayMiliSecond(2);

//    I2C1->CR2 &= ~I2C_CR2_RD_WRN;
//    I2C_Start();
//    I2C_SendByte(receivedData);
//    I2C_Stop();
}

void HR_SetOutputMode(void){
    I2C1->CR2 &= ~I2C_CR2_RD_WRN;
    I2C1->CR2&=~(0xFF<<16);//clear NBYTES
    I2C1->CR2|=(0x3<<16);//Set N Bytes to one byte
    I2C_Start();
    I2C_SendByte(0x10); // Send the data byte (change to your actual data)
    I2C_SendByte(0x00); // Send the data byte (change to your actual data)
    I2C_SendByte(0x02); // Send the data byte (change to your actual data)
    I2C_Stop();
    TIM2_delayMiliSecond(2);

    I2C1->CR2 |= (I2C_CR2_RD_WRN);

    I2C1->CR2&=~(0xFF<<16);//clear NBYTES
    I2C1->CR2|=(0x1<<16);//Set N Bytes to one byte
    I2C_Start();
    uint8_t receivedData = I2C_ReadByte();
    I2C_Stop();

}

void HR_setFifoThreshold(void){
    I2C1->CR2 &= ~I2C_CR2_RD_WRN;
    I2C1->CR2&=~(0xFF<<16);//clear NBYTES
    I2C1->CR2|=(0x3<<16);//Set N Bytes to one byte
    I2C_Start();
    I2C_SendByte(0x10); // Send the data byte (change to your actual data)
    I2C_SendByte(0x01); // Send the data byte (change to your actual data)
    I2C_SendByte(0x0F); // Send the data byte (change to your actual data)
    I2C_Stop();
    TIM2_delayMiliSecond(3);
    I2C1->CR2 |= (I2C_CR2_RD_WRN);

    I2C1->CR2&=~(0xFF<<16);//clear NBYTES
    I2C1->CR2|=(0x1<<16);//Set N Bytes to one byte
    I2C_Start();
    uint8_t receivedData = I2C_ReadByte();
    I2C_Stop();

}

void HR_agcAlgoControl(void){
    I2C1->CR2 &= ~I2C_CR2_RD_WRN;
    I2C1->CR2&=~(0xFF<<16);//clear NBYTES
    I2C1->CR2|=(0x3<<16);//Set N Bytes to one byte
    I2C_Start();
    I2C_SendByte(0x52); // Send the data byte (change to your actual data)
    I2C_SendByte(0x00); // Send the data byte (change to your actual data)
    I2C_SendByte(0x01); // Send the data byte (change to your actual data)
    I2C_Stop();
    TIM2_delayMiliSecond(50);
    I2C1->CR2 |= (I2C_CR2_RD_WRN);

    I2C1->CR2&=~(0xFF<<16);//clear NBYTES
    I2C1->CR2|=(0x1<<16);//Set N Bytes to one byte
    I2C_Start();
    uint8_t receivedData = I2C_ReadByte();
    I2C_Stop();

}

void HR_max30101Control(void){
    I2C1->CR2 &= ~I2C_CR2_RD_WRN;
    I2C1->CR2&=~(0xFF<<16);//clear NBYTES
    I2C1->CR2|=(0x3<<16);//Set N Bytes to one byte
    I2C_Start();
    I2C_SendByte(0x44); // Send the data byte (change to your actual data)
    I2C_SendByte(0x03); // Send the data byte (change to your actual data)
    I2C_SendByte(0x01); // Send the data byte (change to your actual data)
    I2C_Stop();
    TIM2_delayMiliSecond(50);
    I2C1->CR2 |= (I2C_CR2_RD_WRN);

    I2C1->CR2&=~(0xFF<<16);//clear NBYTES
    I2C1->CR2|=(0x1<<16);//Set N Bytes to one byte
    I2C_Start();
    uint8_t receivedData = I2C_ReadByte();
    I2C_Stop();

}

void HR_maximFastAlgoControl(void){
    I2C1->CR2 &= ~I2C_CR2_RD_WRN;
    I2C1->CR2&=~(0xFF<<16);//clear NBYTES
    I2C1->CR2|=(0x3<<16);//Set N Bytes to one byte
    I2C_Start();
    I2C_SendByte(0x52); // Send the data byte (change to your actual data)
    I2C_SendByte(0x02); // Send the data byte (change to your actual data)
    I2C_SendByte(0x01); // Send the data byte (change to your actual data)
    I2C_Stop();
    TIM2_delayMiliSecond(50);
    I2C1->CR2 |= (I2C_CR2_RD_WRN);

    I2C1->CR2&=~(0xFF<<16);//clear NBYTES
    I2C1->CR2|=(0x1<<16);//Set N Bytes to one byte
    I2C_Start();
    uint8_t receivedData = I2C_ReadByte();
    I2C_Stop();

}

void HR_algoSampleAvg(void){
    I2C1->CR2 &= ~I2C_CR2_RD_WRN;
    I2C1->CR2&=~(0xFF<<16);//clear NBYTES
    I2C1->CR2|=(0x4<<16);//Set N Bytes to one byte
    I2C_Start();
    I2C_SendByte(0x50); // Send the data byte (change to your actual data)
    I2C_SendByte(0x00); // Send the data byte (change to your actual data)
    I2C_SendByte(0x03); // Send the data byte (change to your actual data)
    I2C_SendByte(0x10); // Send the data byte (change to your actual data)
    I2C_Stop();
    TIM2_delayMiliSecond(50);
    I2C1->CR2 |= (I2C_CR2_RD_WRN);

    I2C1->CR2&=~(0xFF<<16);//clear NBYTES
    I2C1->CR2|=(0x1<<16);//Set N Bytes to one byte
    I2C_Start();
    uint8_t receivedData = I2C_ReadByte();
    I2C_Stop();

}

void HR_readAlgoSamples(void){
    I2C1->CR2 &= ~I2C_CR2_RD_WRN;
    I2C1->CR2&=~(0xFF<<16);//clear NBYTES
    I2C1->CR2|=(0x3<<16);//Set N Bytes to one byte
    I2C_Start();
    I2C_SendByte(0x51); // Send the data byte (change to your actual data)
    I2C_SendByte(0x00); // Send the data byte (change to your actual data)
    I2C_SendByte(0x03); // Send the data byte (change to your actual data)
    I2C_Stop();

    TIM2_delayMiliSecond(2);
    I2C1->CR2 |= (I2C_CR2_RD_WRN);

    I2C1->CR2&=~(0xFF<<16);//clear NBYTES
    I2C1->CR2|=(0x2<<16);//Set N Bytes to one byte
    I2C_Start();
    uint8_t receivedData = I2C_ReadByte();
    receivedData = I2C_ReadByte();
    I2C_Stop();

}

void HR_readSensorLEDs(void){
    uint8_t senArr[12];
    I2C1->CR2 &= ~I2C_CR2_RD_WRN;
    I2C1->CR2&=~(0xFF<<16);//clear NBYTES
    I2C1->CR2|=(0x2<<16);//Set N Bytes to one byte
    I2C_Start();
    I2C_SendByte(0x12); // Send the data byte (change to your actual data)
    I2C_SendByte(0x01); // Send the data byte (change to your actual data)
    I2C_Stop();
    TIM2_delayMiliSecond(2);


    int sizeRead=12;
    //uint8_t bpmArr[sizeRead];

    for(size_t i = 0; i < sizeRead; i++){
        senArr[i] = 0;
    }

    I2C1->CR2 |= (I2C_CR2_RD_WRN);
    I2C1->CR2&=~(0xFF<<16);//clear NBYTES
    I2C1->CR2|=(sizeRead<<16);//Set N Bytes to one byte
    I2C_Start();
    for(size_t i = 0; i < sizeRead; i++){
        senArr[i] = I2C_ReadByte();
    }
    I2C_Stop();
    uint32_t irLed;
    uint32_t redLed;
    irLed = (uint32_t)senArr[0] << 16;
    irLed |= (uint32_t)senArr[1] << 8;
    irLed |= senArr[2];

    // Value of LED two...
    redLed = (uint32_t)senArr[3] << 16;
    redLed |= (uint32_t)senArr[4] << 8;
    redLed |= senArr[5];
}


void init_HR(void){
    HR_startModule();
    TIM2_delayMiliSecond(1000);
    init_I2C1();
    HR_readMode();
    HR_SetOutputMode();
    HR_setFifoThreshold();
    HR_agcAlgoControl();
    HR_max30101Control();
    HR_maximFastAlgoControl();
    HR_algoSampleAvg();
    HR_readAlgoSamples();
    HR_readSensorLEDs();
    TIM2_delayMiliSecond(1000);
}




void HR_checkStatus(void){
    I2C1->CR2 &= ~I2C_CR2_RD_WRN;
    I2C1->CR2&=~(0xFF<<16);//clear NBYTES
    I2C1->CR2|=(0x2<<16);//Set N Bytes to one byte
    I2C_Start();
    I2C_SendByte(0x00); // Send the data byte (change to your actual data)
    I2C_SendByte(0x00); // Send the data byte (change to your actual data)
    I2C_Stop();

    TIM2_delayMiliSecond(45);
    I2C1->CR2 |= (I2C_CR2_RD_WRN);

    I2C1->CR2&=~(0xFF<<16);//clear NBYTES
    I2C1->CR2|=(0x2<<16);//Set N Bytes to one byte
    I2C_Start();
    uint8_t receivedData = I2C_ReadByte();
    receivedData = I2C_ReadByte();
    I2C_Stop();
}

void HR_numSamplesOutFifo(void){
    I2C1->CR2 &= ~I2C_CR2_RD_WRN;
    I2C1->CR2&=~(0xFF<<16);//clear NBYTES
    I2C1->CR2|=(0x2<<16);//Set N Bytes to one byte
    I2C_Start();
    I2C_SendByte(0x12); // Send the data byte (change to your actual data)
    I2C_SendByte(0x00); // Send the data byte (change to your actual data)
    I2C_Stop();

    TIM2_delayMiliSecond(45);
    I2C1->CR2 |= (I2C_CR2_RD_WRN);

    I2C1->CR2&=~(0xFF<<16);//clear NBYTES
    I2C1->CR2|=(0x2<<16);//Set N Bytes to one byte
    I2C_Start();
    uint8_t receivedData = I2C_ReadByte();
    receivedData = I2C_ReadByte();
    I2C_Stop();

}

void HR_readFillArray(uint8_t bpmArr[]){
    I2C1->CR2 &= ~I2C_CR2_RD_WRN;
    I2C1->CR2&=~(0xFF<<16);//clear NBYTES
    I2C1->CR2|=(0x2<<16);//Set N Bytes to one byte
    I2C_Start();
    I2C_SendByte(0x12); // Send the data byte (change to your actual data)
    I2C_SendByte(0x01); // Send the data byte (change to your actual data)
    I2C_Stop();
    TIM2_delayMiliSecond(2);


    int sizeRead=6;
    //uint8_t bpmArr[sizeRead];

    for(size_t i = 0; i < sizeRead; i++){
        bpmArr[i] = 0;
    }

    I2C1->CR2 |= (I2C_CR2_RD_WRN);
    I2C1->CR2&=~(0xFF<<16);//clear NBYTES
    I2C1->CR2|=(0x6<<16);//Set N Bytes to one byte
    I2C_Start();
//    for(size_t i = (sizeRead-1); i > 0; i--){
//        bpmArr[i] = I2C_ReadByte();
//    }
//    bpmArr[0] = I2C_ReadByte();
    for(size_t i = 0; i <sizeRead; i++){
        bpmArr[i] = I2C_ReadByte();
    }
    I2C_Stop();
}

void HR_readBPM(void){
    uint8_t bpmRetArr[6];
    //while(1){
    HR_checkStatus();
    HR_numSamplesOutFifo();
        HR_readFillArray(bpmRetArr);
        // Heart Rate formatting
        uint16_t heartRate = (uint16_t)bpmRetArr[0] << 8;
        heartRate |= (bpmRetArr[1]);
        heartRate /= 10;

        // Confidence formatting
        uint8_t confidence = bpmRetArr[2];
        //Blood oxygen level formatting
        uint16_t oxygen = (uint16_t)bpmRetArr[3] << 8;
        oxygen |= bpmRetArr[4];
        oxygen /= 10;
        //"Machine State" - has a finger been detected?
        uint8_t status = bpmRetArr[5];
        USART5_SendString("Heart Rate: ");
        char ascii_string[20]; // Create a buffer to store the ASCII representation
        // Use sprintf to convert the integer to ASCII
        sprintf(ascii_string, "%d", heartRate);
        USART5_SendString(ascii_string);
//        USART5_SendString(" Confidence: ");
//        sprintf(ascii_string, "%d", confidence);
//        USART5_SendString(ascii_string);
        USART5_SendString(" Status: ");
        sprintf(ascii_string, "%d", status);
        USART5_SendString(ascii_string);

        TIM2_delayMiliSecond(250);
   // }
}


void HR_setMFIOInt(void){
    //set PB9 to interupt
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

    GPIOB->MODER &= ~GPIO_MODER_MODER9;  // Clear the MODER bits for PA0

    // No need to set anything; it's already in input mode by default
    GPIOB->PUPDR&= ~(0x3<<18); //pull up
    GPIOB->PUPDR|=(0x1<<19); //pull up


    // Enable the SYSCFG peripheral clock
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;

    // Connect EXTI9 line to PB9 pin
    SYSCFG->EXTICR[2] |=SYSCFG_EXTICR3_EXTI9_PB;//(0x1<<4);

    //Configure PB9 to trigger an interrupt on the falling edge (button press)
    EXTI->FTSR |= EXTI_FTSR_TR9; // Trigger on falling edge
    //EXTI->RTSR|=EXTI_RTSR_TR9;
    //EXTI->FTSR &= ~(EXTI_FTSR_TR9);       // Trigger on falling edge (HIGH to LOW)
    EXTI->RTSR &= ~(EXTI_RTSR_TR9);    // Disable rising edge trigger
    EXTI->IMR |= EXTI_IMR_MR9;   // Enable EXTI9 (PB9)


    // Enable EXTI0 interrupt in NVIC
    NVIC->ISER[0]|=(0x1<<(EXTI4_15_IRQn));
}


void HR_startModule(){
    //Pin 8 reset, pin 9 mfio
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

    GPIOB->MODER&= ~0x00030000; //clear PB8
    GPIOB->MODER|=0x00010000; //set analog

    GPIOB->MODER&= ~0x000C0000; //clear PB9
    GPIOB->MODER|=0x00040000; //set analog
    //set pin to analog mode
    //set pin to not pull up or pull down

    GPIOB->BSRR|=(0x1<<9);
    TIM2_delayMiliSecond(1000);
    //set pin high using bsrr

//     Set the RSTN pin low for 10ms.
    GPIOB->BSRR|=(0x1<<25);
    TIM2_delayMiliSecond(10);
//     While RSTN is low, set the MFIO pin to high.
    GPIOB->BSRR|=(0x1<<8);
//     After the 10ms has elapsed, set the RSTN pin high.
    TIM2_delayMiliSecond(10);
    GPIOB->BSRR|=(0x1<<9);
    TIM2_delayMiliSecond(50);
//     After an additional 50ms has elapsed, the MAX32664 is in application mode.
    //https://github.com/sparkfun/SparkFun_Bio_Sensor_Hub_Library/blob/f91b5fa12f391f9889919a20c8709849fb4c79fb/src/SparkFun_Bio_Sensor_Hub_Library.cpp#L43

   // GPIOA->BSRR = GPIO_PIN_0; // Set the pin high (3.3V)
}
