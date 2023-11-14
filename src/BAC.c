#include "BAC.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>


void BAC_readADC(uint8_t adcData[]){
    I2C1->CR2 &=~0xFF<<0; //slave address of ethanol sensor
    //I2C1->CR2 |=0x9A<<0; //slave address of ethanol sensor
    I2C1->CR2 |=0x48<<1; //slave address of ethanol sensor
 //   I2C1->CR2 |=0x49<<1; //slave address of ethanol sensor
//    I2C1->CR2 |=0x4A<<1; //slave address of ethanol sensor
//    I2C1->CR2 |=0x4B<<1; //slave address of ethanol sensor
//    I2C1->CR2 |=0x4C<<1; //slave address of ethanol sensor
  //  I2C1->CR2 |=0x4D<<1; //slave address of ethanol sensor
//  I2C1->CR2 |=0x4E<<1; //slave address of ethanol sensor
 //   I2C1->CR2 |=0x4F<<1; //slave address of ethanol sensor


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
float BAC_ethanolInCo(uint16_t co)
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

/*  BAC  Percent PPM
 *  0.01   0.1   26
 *  0.05   0.5   131
 *  0.08   0.8   209
 *  0.2    2     522
 *  0.3    3     783
 *  */
float BAC_ppmEthanolToBAC(uint16_t ppmin){
    float BAC=0.0;
    float ppm= (float)ppmin;
    if (ppm == 0)
    {
        BAC = 0;
    }
    else if (ppm <= 26)
    {
        BAC = ppm* (float)(.1/26.0);
    }
    else if (ppm > 26 && ppm <= 131)
    {
        BAC = ppm* (float)(.5/131.0);
    }
    else if (ppm > 131 && ppm <= 209.0)
    {
        BAC = ppm* (float)(.8/209.0);
    }
    else if (ppm > 209 && ppm <= 522)
    {
        BAC = ppm* (float)(2.0/522.0);
    }
    else if (ppm > 522 && ppm <= 783)
    {
        BAC = ppm* (float)(3.0/ 783.0);
    }
    return BAC;

}

/* Convert ppm to mg/L
 *
 * Table:  mg/l | air ppm
 *         1.82 | 1000
 *         0.91 | 500
 *         0.18 | 100
 *         0.09 | 50
 */
float BAC_ppmTomgl(uint16_t ppm)
{
    float mgL;

    mgL = (1.82 * ppm) / 1000.0;
    return mgL;
}

float BAC_getData(void){

    float Alcohol_mgL;
    float BAC;
    uint16_t Ethanol_ppm;
    uint16_t readData;

    readData = BAC_getADCData();
    Ethanol_ppm = BAC_ethanolInCo(readData);
    Alcohol_mgL = BAC_ppmTomgl(Ethanol_ppm);
    // 1000mg/L = .1% BAC

    //return 1000:.1%, 8000:.8%, 10,000:1%, 100:.001
    BAC=BAC_ppmEthanolToBAC(Ethanol_ppm);
    return BAC*1000;
    //return Alcohol_mgL*1000;
}

void BAC_read(){
//    if(!(GPIOA->ODR& ~(0xFFF7))){
//        GPIOA->BSRR|=(0x1<<4);
//        GPIOA->BSRR|=(0x1<<3);}
    if(((GPIOA->ODR)&~(0xFFF7))){
        int pBAC=BAC_getData();

        char ascii_string[20]; // Create a buffer to store the ASCII representation
        // Use sprintf to convert the integer to ASCII
        //sprintf(ascii_string, "%d", pBAC);
        sprintf(ascii_string, "%d", pBAC);

        USART5_SendString("BAC: ");
        USART5_SendString(ascii_string);
        TIM2_delayMiliSecond(1000);
    }
}
