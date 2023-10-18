#include "BAC.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>


void BAC_readADC(uint8_t adcData[]){
    I2C1->CR2 &=~0xFF<<0; //slave address of ethanol sensor
    I2C1->CR2 |=0x9A<<0; //slave address of ethanol sensor
    I2C1->CR2 &= ~I2C_CR2_RD_WRN;
    I2C1->CR2&=~(0xFF<<16);//clear NBYTES
    I2C1->CR2|=(0x1<<16);//Set N Bytes to one byte
    I2C_Start();
    I2C_SendByte(0x00); // Send the data byte (change to your actual data)
    I2C_Stop();
    TIM2_delayMiliSecond(5);


    I2C1->CR2 |= (I2C_CR2_RD_WRN);

    I2C1->CR2&=~(0xFF<<16);//clear NBYTES
    I2C1->CR2|=(0x2<<16);//Set N Bytes to one byte
    I2C_Start();
    adcData[0] = I2C_ReadByte();
    adcData[1] = I2C_ReadByte();
    I2C_Stop();
}


uint16_t BAC_getADCData(void){

    uint8_t writeData;
    uint8_t readData[ 2 ];
    uint16_t ADC_value;

    BAC_readADC(readData);

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
static float BAC_ethanolInCo(uint16_t co)
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
static float BAC_ppmTomgl(uint16_t ppm)
{
    float mgL;

    mgL = (1.82 * ppm) / 1000.0;
    return mgL;
}

float BAC_getData(void){

    float Alcohol_mgL;
    uint16_t Ethanol_ppm;
    uint16_t readData;

    readData = BAC_getADCData();
    Ethanol_ppm = BAC_ethanolInCo(readData);
    Alcohol_mgL = BAC_ppmTomgl(Ethanol_ppm);

    return Alcohol_mgL*1000;
}

void BAC_read(){
    if(!(GPIOA->ODR& ~(0xFFF7))){
        GPIOA->BSRR|=(0x1<<4);
        GPIOA->BSRR|=(0x1<<3);}

    uint16_t pBAC=BAC_getData();

    char ascii_string[20]; // Create a buffer to store the ASCII representation
    // Use sprintf to convert the integer to ASCII
    sprintf(ascii_string, "%d", pBAC);
    USART5_SendString("BAC: ");
    USART5_SendString(ascii_string);
//    USART5_SendChar('\n');
//    USART5_SendChar('\r');
    TIM2_delayMiliSecond(1000);
}
